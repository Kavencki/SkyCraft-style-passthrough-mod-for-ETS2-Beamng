// ets2_roads - exports the drivable road surface around a point of the ETS2 map
// for EuroBeamNG's CollisionField (BeamNG builds solid roads from it).
//
//   ets2_roads --game "<ETS2 folder>" --x 43618 --z -25649 [--radius 20000] [--out roads.bin]
//
// roads.bin (little-endian):
//   "ESRD" u32 version(2) u32 stripCount
//   per strip: u32 kind f32 width u32 n, then n x (f32 x, f32 y, f32 z)
//   kind 0 road surface, 1 junction link, 2 barrier (points = top of the barrier)
// Coordinates are ETS2 world (X east, Y up, Z south), metres - same as telemetry.
using System.Numerics;
using System.Text;
using TruckLib;
using TruckLib.HashFs;
using TruckLib.Models.Ppd;
using TruckLib.ScsMap;
using TruckLib.Sii;

static class Program
{
    const float StepMetres = 4f;           // sample spacing along roads (2 m: more kinks, not fewer - lane probe)
    const float DefaultHalfWidth = 7f;     // per side, when the side has no railing to tell us
    const float JunctionWidth = 18f;
    const float BarrierWidth = 0.4f;
    const float BarrierHeight = 1.0f;      // top of the wall above the road surface

    // Outermost railing offset on one side (metres from the road centre), or null.
    static float? RailingOffset(RoadSide side)
    {
        var models = side?.Railings?.Models;
        if (models == null) return null;
        float? best = null;
        foreach (var m in models)
        {
            if (m.Model.ToString() is "" or "0" || m.Offset <= 0.5f) continue;
            best = best is null ? m.Offset : Math.Max(best.Value, m.Offset);
        }
        return best;
    }

    const float LaneWidth = 5f;            // junction lanes (ppd nav curves) overlap into a full surface
    const float EndOverlap = 3f;           // extend strips so joints on curves have no gaps
    const float CompactJunction = 35f;     // node spread (m) up to which a junction is filled from its roads
    const int MaxChainPoints = 300;        // ~1.2 km per surface piece
    const float JunctionOverlap = 3f;     // roads reach this far into junctions with game collision
    const float EdgeMargin = 0f;           // extra width per side beyond the game's cross-section (0: exact)
    const float MaxGap = 28f;              // gaps (medians, lane splits) up to this wide are filled
    const float SideWallHeight = 1.0f;     // walls at kerbs / sidewalks (ETS2 edge models)

    record RoadSeg(ulong StartUid, ulong EndUid, List<Vector3> Pts, float HalfL, float HalfR, float? RailL, float? RailR,
                   List<Vector3> Left, List<Vector3> Right, List<Vector2> Rh);

    // One side edge of a surface: points, outward normals (X/Z), owner (surfaces with the same
    // owner are one continuous piece), and whether an open edge gets a wall.
    sealed class EdgeLine
    {
        public Vector3[] P; public Vector2[] N; public int Owner; public bool Walls;
        public Vector3?[] Partner;   // nearest facing edge of a neighbouring surface across a gap
    }
    // Axis sample of a surface, for "is this point on some other surface" tests.
    record struct AxisPt(Vector3 P, Vector2 R, float L, float W, int Owner);

    static float SmoothStep(float x) { x = Math.Clamp(x, 0f, 1f); return x * x * (3 - 2 * x); }

    static long Cell(float x, float z, float size) => ((long)MathF.Floor(x / size) << 32) ^ (uint)(int)MathF.Floor(z / size);

    static bool InHull(List<Vector3> h, Vector3 p, float tol)
    {
        // Convex polygon (either winding): p is inside when it is on the same side of every edge.
        int sign = 0;
        for (int i = 0; i < h.Count; i++)
        {
            var a = h[i]; var b = h[(i + 1) % h.Count];
            float ex = b.X - a.X, ez = b.Z - a.Z, len = MathF.Sqrt(ex * ex + ez * ez);
            if (len < 1e-3f) continue;
            float c = (ex * (p.Z - a.Z) - ez * (p.X - a.X)) / len;   // signed distance
            int s = c > tol ? 1 : c < -tol ? -1 : 0;
            if (s == 0) continue;
            if (sign == 0) sign = s; else if (s != sign) return false;
        }
        return true;
    }

    const float TileSize = 256f;   // m: smaller tiles keep what BeamNG holds close to the car (its collision rebuild is a hitch)

    // Banking: how much the node's right axis rises per metre sideways (clamped to sane values).
    static float Bank(Quaternion rot) => Environment.GetEnvironmentVariable("ES_NOBANK") != null ? 0f : BankOf(rot);
    static float BankOf(Quaternion rot)
    {
        var r = Vector3.Transform(Vector3.UnitX, rot);
        float h = MathF.Sqrt(r.X * r.X + r.Z * r.Z);
        return h > 1e-3f ? Math.Clamp(r.Y / h, -0.2f, 0.2f) : 0f;
    }

    // Horizontal unit vector to the right of travel at point i (ETS2: Y up, Z south).
    static Vector2 RightAt(List<Vector3> pts, int i)
    {
        var d = pts[Math.Min(i + 1, pts.Count - 1)] - pts[Math.Max(i - 1, 0)];
        var f = Vector2.Normalize(new Vector2(d.X, d.Z));
        if (float.IsNaN(f.X)) f = new Vector2(0, -1);
        return new Vector2(-f.Y, f.X);
    }

    // Convex hull (X/Z) of points, keeping their heights; Andrew's monotone chain.
    static List<Vector3> Hull(List<Vector3> pts)
    {
        var p = pts.OrderBy(v => v.X).ThenBy(v => v.Z).ToList();
        if (p.Count < 3) return p;
        float Cross(Vector3 o, Vector3 a, Vector3 b) => (a.X - o.X) * (b.Z - o.Z) - (a.Z - o.Z) * (b.X - o.X);
        var h = new List<Vector3>();
        foreach (var v in p) { while (h.Count >= 2 && Cross(h[^2], h[^1], v) <= 0) h.RemoveAt(h.Count - 1); h.Add(v); }
        int lower = h.Count + 1;
        for (int i = p.Count - 2; i >= 0; i--)
        {
            var v = p[i];
            while (h.Count >= lower && Cross(h[^2], h[^1], v) <= 0) h.RemoveAt(h.Count - 1);
            h.Add(v);
        }
        h.RemoveAt(h.Count - 1);
        return h;
    }

    // Lengthens a polyline at both ends along its end directions.
    static List<Vector3> Extend(List<Vector3> pts, float by)
    {
        if (pts.Count < 2) return pts;
        var d0 = Vector3.Normalize(pts[0] - pts[1]);
        var d1 = Vector3.Normalize(pts[^1] - pts[^2]);
        var res = new List<Vector3>(pts.Count + 2);
        if (!float.IsNaN(d0.X)) res.Add(pts[0] + d0 * by);
        res.AddRange(pts);
        if (!float.IsNaN(d1.X)) res.Add(pts[^1] + d1 * by);
        return res;
    }

    // Placement of a prefab model in the world: yaw about Y + translation, fitted from the control
    // nodes (known both in the .ppd model space and on the map). Height follows a plane fitted to
    // the nodes' height offsets, so tilted junctions line up with every connecting road.
    record struct Placement(float C, float S, Vector3 T, float H0 = 0, float Hx = 0, float Hz = 0, float Cx = 0, float Cz = 0);

    static Vector3 Apply(Placement p, Vector3 m)
    {
        float x = p.C * m.X - p.S * m.Z + p.T.X, z = p.S * m.X + p.C * m.Z + p.T.Z;
        return new(x, m.Y + p.H0 + p.Hx * (x - p.Cx) + p.Hz * (z - p.Cz), z);
    }

    // Least-squares plane r = h0 + hx*(x-cx) + hz*(z-cz) through the nodes' height offsets.
    static Placement FitHeights(Placement p, Vector3[] m, Vector3[] w)
    {
        int n = m.Length;
        var flat = p with { H0 = 0, Hx = 0, Hz = 0 };
        var xs = new float[n]; var zs = new float[n]; var rs = new float[n];
        for (int i = 0; i < n; i++)
        {
            var a = Apply(flat, m[i]);
            xs[i] = a.X; zs[i] = a.Z; rs[i] = w[i].Y - m[i].Y;
        }
        float cx = xs.Average(), cz = zs.Average(), r0 = rs.Average();
        double sxx = 0, sxz = 0, szz = 0, sxr = 0, szr = 0;
        for (int i = 0; i < n; i++)
        {
            double dx = xs[i] - cx, dz = zs[i] - cz, dr = rs[i] - r0;
            sxx += dx * dx; sxz += dx * dz; szz += dz * dz; sxr += dx * dr; szr += dz * dr;
        }
        double det = sxx * szz - sxz * sxz;
        float hx = 0, hz = 0;
        if (n >= 3 && Math.Abs(det) > 1e-3)
        {
            hx = (float)((sxr * szz - szr * sxz) / det);
            hz = (float)((szr * sxx - sxr * sxz) / det);
        }
        else if (sxx + szz > 1e-3)
        {
            // Two nodes (or collinear): slope only along the line through them.
            double ux = xs[n - 1] - xs[0], uz = zs[n - 1] - zs[0];
            double ul = Math.Sqrt(ux * ux + uz * uz);
            if (ul > 1e-3)
            {
                ux /= ul; uz /= ul;
                double stt = 0, str = 0;
                for (int i = 0; i < n; i++)
                {
                    double t = (xs[i] - cx) * ux + (zs[i] - cz) * uz;
                    stt += t * t; str += t * (rs[i] - r0);
                }
                double g = stt > 1e-6 ? str / stt : 0;
                hx = (float)(g * ux);
                hz = (float)(g * uz);
            }
        }
        // Junctions are near-flat; ignore absurd fits.
        if (Math.Abs(hx) > 0.3f || Math.Abs(hz) > 0.3f) { hx = 0; hz = 0; }
        return p with { H0 = r0, Hx = hx, Hz = hz, Cx = cx, Cz = cz };
    }

    static Vector3 Rotate(Placement p, Vector3 d) => new(p.C * d.X - p.S * d.Z, d.Y, p.S * d.X + p.C * d.Z);

    static bool FitPrefab(PrefabDescriptor ppd, Prefab prefab, out Placement best)
    {
        best = default;
        int n = Math.Min(ppd.Nodes.Count, prefab.Nodes.Count);
        if (n < 2) return false;
        double bestErr = double.MaxValue;
        for (int k = 0; k < n; k++)   // the map may start the node list at a different ppd node
        {
            var m = new Vector3[n];
            var w = new Vector3[n];
            for (int i = 0; i < n; i++) { m[i] = ppd.Nodes[(i + k) % n].Position; w[i] = prefab.Nodes[i].Position; }
            var cm = m.Aggregate(Vector3.Zero, (a, b) => a + b) / n;
            var cw = w.Aggregate(Vector3.Zero, (a, b) => a + b) / n;
            double dot = 0, cross = 0;
            for (int i = 0; i < n; i++)
            {
                var a = m[i] - cm; var b = w[i] - cw;
                dot += a.X * b.X + a.Z * b.Z;
                cross += a.X * b.Z - a.Z * b.X;
            }
            float th = (float)Math.Atan2(cross, dot);
            var p = new Placement(MathF.Cos(th), MathF.Sin(th), Vector3.Zero);
            var rc = Apply(p, cm);
            p = p with { T = new Vector3(cw.X - rc.X, 0, cw.Z - rc.Z) };
            double err = 0;
            for (int i = 0; i < n; i++)
            {
                var d = Apply(p, m[i]) - w[i];
                err += d.X * d.X + d.Z * d.Z;
            }
            err = Math.Sqrt(err / n);
            if (err < bestErr) { bestErr = err; best = FitHeights(p, m, w); }
        }
        return bestErr < 1.5;
    }

    // World-space polyline of a ppd nav curve (cubic Hermite between its ends).
    static List<Vector3> NavCurvePoints(NavCurve c, Placement p)
    {
        var a = Apply(p, c.StartPosition);
        var b = Apply(p, c.EndPosition);
        float len = Vector3.Distance(a, b);
        if (len < 0.5f) return null;
        var chord = (b - a) / len;
        Vector3 Dir(Quaternion q)
        {
            var d = Rotate(p, Vector3.Transform(-Vector3.UnitZ, q));
            d = Vector3.Normalize(d);
            return Vector3.Dot(d, chord) < 0 ? -d : d;   // orientation-agnostic: point along travel
        }
        var m0 = Dir(c.StartRotation) * len;
        var m1 = Dir(c.EndRotation) * len;
        int n = Math.Max(2, (int)Math.Ceiling(len / 3f) + 1);
        var pts = new List<Vector3>(n);
        for (int k = 0; k < n; k++) pts.Add(HermiteSpline.Interpolate(a, b, m0, m1, k / (float)(n - 1)));
        return pts;
    }

    // road look token (e.g. "balt10") -> its definition unit, from /def/world/road_look*.sii.
    static Dictionary<string, Unit> LoadRoadLooks(IFileSystem fs)
    {
        var map = new Dictionary<string, Unit>();
        IList<string> files;
        try { files = fs.GetFiles("/def/world"); } catch { return map; }
        foreach (var f in files.Where(p => p.Contains("road_look") && p.EndsWith(".sii")))
        {
            try
            {
                foreach (var u in SiiFile.Open(f, fs, true).Units)
                    if (u.Class == "road_look") map[u.Name.StartsWith("road.") ? u.Name[5..] : u.Name] = u;
            }
            catch (Exception e) { Console.Error.WriteLine($"skip {f}: {e.Message}"); }
        }
        return map;
    }

    // Railway "roads" (traffic_lane.rail.*) are not driveable surfaces for a car.
    static bool IsRailway(Unit u)
    {
        foreach (var key in new[] { "lanes_left", "lanes_right" })
        {
            if (!u.Attributes.TryGetValue(key, out var lanes) || lanes is not System.Collections.IEnumerable en) continue;
            foreach (var l in en)
                if (Convert.ToString((object)l)?.Contains("traffic_lane.rail") == true) return true;
        }
        return false;
    }

    static float Num(Unit u, string key, float dflt = 0)
    {
        if (!u.Attributes.TryGetValue(key, out var v)) return dflt;
        try { return Convert.ToSingle(v, System.Globalization.CultureInfo.InvariantCulture); } catch { return dflt; }
    }

    // Half widths (left, right of the road axis) of the paved surface for a road look. A value can
    // be negative: then that edge lies on the other side of the axis (one-way roads whose
    // template does not reach the axis).
    //  legacy looks: road_offset + road_size + shoulder_size per side
    //  template looks: the cross-section's collision part ("coll"): x from xmin to xmax measured
    //    from the axis (right of travel = +x), plus road_offset when the look has two sides;
    //    the left template mirrored. (ETS2 roads join junction collision exactly with these.)
    static (float L, float R)? LookHalfWidths(Unit u, IFileSystem fs, Dictionary<string, (float Min, float Max)> templateCache)
    {
        if (u.Attributes.ContainsKey("road_size_left") || u.Attributes.ContainsKey("road_size_right"))
        {
            float off = Num(u, "road_offset");
            float l = off + Num(u, "road_size_left") + Num(u, "shoulder_size_left");
            float r = off + Num(u, "road_size_right") + Num(u, "shoulder_size_right");
            return (l, r);
        }
        (float Min, float Max)? Span(string key)
        {
            if (!u.Attributes.TryGetValue(key, out var path)) return null;
            string p = Convert.ToString((object)path);
            if (templateCache.TryGetValue(p, out var cached)) return float.IsNaN(cached.Min) ? null : cached;
            (float, float) span = (float.NaN, float.NaN);
            try
            {
                var model = TruckLib.Models.Model.Open(p, fs);
                var coll = model.Parts.Where(pt => pt.Name.ToString().StartsWith("coll")).SelectMany(pt => pt.Pieces).ToList();
                if (coll.Count > 0)
                    span = (coll.Min(pc => pc.BoundingBox.Start.X), coll.Max(pc => pc.BoundingBox.End.X));
                else
                    span = (model.BoundingBox.Start.X, model.BoundingBox.End.X);
            }
            catch (Exception e) { Console.Error.WriteLine($"template {p}: {e.Message}"); }
            templateCache[p] = span;
            return float.IsNaN(span.Item1) ? null : span;
        }
        var right = Span("template_right");
        if (right is not { } r2) return null;
        var left = Span("template_left");
        float offset = Num(u, "road_offset");
        if (left is { } l2)
            return (offset + l2.Max, offset + r2.Max);          // two sides; the median between them is filled too
        return (-(offset + r2.Min), offset + r2.Max);           // one template: exactly its span
    }

    // prefab unit token -> .ppd path, from /def/world/prefab*.sii (base + DLC files).
    static Dictionary<string, string> LoadPrefabDefs(IFileSystem fs)
    {
        var map = new Dictionary<string, string>();
        IList<string> files;
        try { files = fs.GetFiles("/def/world"); } catch { return map; }
        foreach (var f in files)
        {
            var name = f[(f.LastIndexOf('/') + 1)..];
            if (!name.StartsWith("prefab") || !name.EndsWith(".sii")) continue;
            try
            {
                var sii = SiiFile.Open(f, fs, true);
                foreach (var u in sii.Units)
                {
                    if (u.Class != "prefab_model" || !u.Attributes.TryGetValue("prefab_desc", out var desc)) continue;
                    var token = u.Name.StartsWith("prefab.") ? u.Name[7..] : u.Name;
                    map[token] = desc.ToString();
                    if (u.Attributes.TryGetValue("model_desc", out var md)) PrefabModelPaths[token] = md.ToString();
                }
            }
            catch (Exception e) { Console.Error.WriteLine($"skip {f}: {e.Message}"); }
        }
        return map;
    }

    // prefab unit token -> its 3D model (.pmd), filled by LoadPrefabDefs.
    static readonly Dictionary<string, string> PrefabModelPaths = new();

    // Offsets every point sideways (+ = right of travel direction) in the X/Z plane.
    static List<Vector3> Offset(List<Vector3> pts, float lateral, float up)
    {
        var res = new List<Vector3>(pts.Count);
        for (int i = 0; i < pts.Count; i++)
        {
            var d = pts[Math.Min(i + 1, pts.Count - 1)] - pts[Math.Max(i - 1, 0)];
            var f = Vector2.Normalize(new Vector2(d.X, d.Z));
            if (float.IsNaN(f.X)) f = new Vector2(0, -1);
            var right = new Vector2(-f.Y, f.X);   // ETS2: Y up, Z south -> right of forward
            res.Add(new Vector3(pts[i].X + right.X * lateral, pts[i].Y + up, pts[i].Z + right.Y * lateral));
        }
        return res;
    }

    // ETS2's folder: ES_ETS2_DIR, else the Steam library that has it (Steam's own list of libraries,
    // libraryfolders.vdf; app 227300).
    static string? FindEts2()
    {
        if (Environment.GetEnvironmentVariable("ES_ETS2_DIR") is string env && File.Exists(Path.Combine(env, "base.scs"))) return env;
        var steams = new List<string>();
        foreach (var key in new[] { @"HKEY_CURRENT_USER\Software\Valve\Steam", @"HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\Valve\Steam" })
            foreach (var name in new[] { "SteamPath", "InstallPath" })
                if (Microsoft.Win32.Registry.GetValue(key, name, null) is string p && p.Length > 0) steams.Add(p.Replace('/', '\\'));
        steams.Add(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86), "Steam"));
        var libraries = new List<string>();
        foreach (var steam in steams.Distinct(StringComparer.OrdinalIgnoreCase))
        {
            libraries.Add(steam);
            var vdf = Path.Combine(steam, "steamapps", "libraryfolders.vdf");
            if (!File.Exists(vdf)) continue;
            foreach (System.Text.RegularExpressions.Match m in System.Text.RegularExpressions.Regex.Matches(File.ReadAllText(vdf), "\"path\"\\s+\"([^\"]+)\""))
                libraries.Add(m.Groups[1].Value.Replace(@"\\", @"\"));
        }
        foreach (var lib in libraries.Distinct(StringComparer.OrdinalIgnoreCase))
        {
            var dir = Path.Combine(lib, "steamapps", "common", "Euro Truck Simulator 2");
            if (File.Exists(Path.Combine(dir, "base.scs"))) return dir;
        }
        return null;
    }

    static int Main(string[] args)
    {
        // A crash is reported on stdout too: the bridge's log (bridge.log) records only stdout.
        AppDomain.CurrentDomain.UnhandledException += (_, e) =>
        {
            Console.WriteLine($"ets2_roads crashed: {e.ExceptionObject}");
            Console.Out.Flush();
        };
        if (args.Length == 1 && args[0] == "--api-dump")
        {
            // Debug: public API of TruckLib.Models (no docs ship with the package).
            foreach (var t in typeof(TruckLib.Models.Model).Assembly.GetExportedTypes().Concat(new[] { typeof(IFileSystem), typeof(AssetLoader), typeof(IHashFsReader) }).OrderBy(t => t.FullName))
            {
                Console.WriteLine(t.FullName);
                foreach (var m in t.GetMembers(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance |
                                               System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.DeclaredOnly))
                    if (m is System.Reflection.PropertyInfo p) Console.WriteLine($"   prop {p.PropertyType.Name} {p.Name}");
                    else if (m is System.Reflection.MethodInfo mi && !mi.IsSpecialName)
                        Console.WriteLine($"   meth {mi.ReturnType.Name} {mi.Name}({string.Join(", ", mi.GetParameters().Select(x => x.ParameterType.Name + " " + x.Name))})");
                    else if (m is System.Reflection.FieldInfo fi) Console.WriteLine($"   field {fi.FieldType.Name} {fi.Name}");
            }
            return 0;
        }
        string game = FindEts2() ?? "";
        double cx = double.NaN, cz = double.NaN, radius = 20000;
        string outPath = "roads.bin";
        string dumpLooks = null;
        string tilesVfs = "/art/eurobeamng/tiles";
        string hfDir = null;   // --heightfield-dir: ground as height grids (BeamNG TerrainBlock), meshes only for the rest
        string tilesDir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            @"BeamNG\BeamNG.drive\current\mods\unpacked\eurobeamng\art\eurobeamng\tiles");
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--game": game = args[++i]; break;
                case "--x": cx = double.Parse(args[++i], System.Globalization.CultureInfo.InvariantCulture); break;
                case "--z": cz = double.Parse(args[++i], System.Globalization.CultureInfo.InvariantCulture); break;
                case "--radius": radius = double.Parse(args[++i], System.Globalization.CultureInfo.InvariantCulture); break;
                case "--out": outPath = args[++i]; break;
                case "--dump-looks": dumpLooks = args[++i]; break;
                case "--tiles-dir": tilesDir = args[++i]; break;
                case "--tiles-vfs": tilesVfs = args[++i]; break;
                case "--heightfield-dir": hfDir = args[++i]; break;
                default: Console.Error.WriteLine($"unknown argument {args[i]}"); return 2;
            }
        }
        if (dumpLooks == null && (double.IsNaN(cx) || double.IsNaN(cz)))
        {
            Console.Error.WriteLine("usage: ets2_roads --game <ETS2 folder> --x <X> --z <Z> [--radius m] [--out roads.bin]");
            return 2;
        }

        if (!File.Exists(Path.Combine(game, "base.scs")))
        {
            Console.Error.WriteLine($"ETS2 not found{(game.Length > 0 ? " in " + game : "")} - give its folder with --game");
            return 2;
        }
        // The game's own archives: base map first, then every DLC (map DLCs add sectors).
        var readers = new List<IHashFsReader>();
        foreach (var name in new[] { "base_map.scs", "base.scs", "def.scs", "base_share.scs", "base_vehicle.scs" }
                     .Concat(Directory.GetFiles(game, "dlc_*.scs").Select(Path.GetFileName).OrderBy(n => n)))
        {
            var path = Path.Combine(game, name);
            if (!File.Exists(path)) continue;
            try { readers.Add(HashFsReader.Open(path)); }
            catch (Exception e) { Console.Error.WriteLine($"skip {name}: {e.Message}"); }
        }
        Console.WriteLine($"opened {readers.Count} archives");
        var fs = new AssetLoader(readers.Cast<IFileSystem>().ToArray());

        // ES_INVISIBLE=<truck id>[;<mod file>]: builds the EuroBeamNG invisible-truck mod for that
        // player truck (e.g. scania.s_2016). Every model the truck's definitions use is copied with
        // all its materials switched to one "eut2.shadowonly" material: the truck is not drawn but
        // still casts its shadow; its lights (separate light sources) still light the road.
        if (Environment.GetEnvironmentVariable("ES_INVISIBLE") is string invisibleArg)
            return InvisibleTruck.Build(readers, fs, invisibleArg);
        // Debug: ES_LS=/dir lists a directory of the game archives; ES_CAT=/file prints a text file.
        if (Environment.GetEnvironmentVariable("ES_LS") is string lsDir)
        {
            foreach (var r in readers)
                try { foreach (var d in r.GetDirectoryListing(lsDir, false, true).Subdirectories) Console.WriteLine(d + "/"); } catch { }
            try { foreach (var f in fs.GetFiles(lsDir)) Console.WriteLine(f); } catch { }
            return 0;
        }
        // ES_TERRAINASSETS=dir: writes the images the mod's ground TerrainBlock needs (a flat 16-bit
        // heightmap and an all-material-0 layer map for its first import, flat base textures).
        if (Environment.GetEnvironmentVariable("ES_TERRAINASSETS") is string assetDir)
        {
            Directory.CreateDirectory(assetDir);
            static void Png(string path, int w, int h, int colorType, int bitDepth, Func<int, int, byte[]> pixel)
            {
                var raw = new MemoryStream();
                for (int y = 0; y < h; y++) { raw.WriteByte(0); for (int x = 0; x < w; x++) raw.Write(pixel(x, y)); }
                var zs = new MemoryStream();
                using (var zl = new System.IO.Compression.ZLibStream(zs, System.IO.Compression.CompressionLevel.Optimal, true)) { raw.Position = 0; raw.CopyTo(zl); }
                using var f = File.Create(path);
                f.Write(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 });
                void Chunk(string type, byte[] data)
                {
                    var len = BitConverter.GetBytes(data.Length); Array.Reverse(len); f.Write(len);
                    var td = Encoding.ASCII.GetBytes(type).Concat(data).ToArray();
                    f.Write(td);
                    uint crc = 0xFFFFFFFF;
                    foreach (var bt in td) { crc ^= bt; for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (uint)-(int)(crc & 1)); }
                    var cb = BitConverter.GetBytes(~crc); Array.Reverse(cb); f.Write(cb);
                }
                var hdr = new byte[13];
                var wb = BitConverter.GetBytes(w); Array.Reverse(wb); Array.Copy(wb, 0, hdr, 0, 4);
                var hb = BitConverter.GetBytes(h); Array.Reverse(hb); Array.Copy(hb, 0, hdr, 4, 4);
                hdr[8] = (byte)bitDepth; hdr[9] = (byte)colorType;
                Chunk("IHDR", hdr);
                Chunk("IDAT", zs.ToArray());
                Chunk("IEND", Array.Empty<byte>());
            }
            Png(Path.Combine(assetDir, "ground_flat_1024.png"), 1024, 1024, 0, 16, (_, _) => new byte[] { 0, 0 });
            Png(Path.Combine(assetDir, "ground_layer_1024.png"), 1024, 1024, 0, 8, (_, _) => new byte[] { 255 });
            Png(Path.Combine(assetDir, "ground_none_1024.png"), 1024, 1024, 0, 8, (_, _) => new byte[] { 0 });   // grass: nowhere
            Png(Path.Combine(assetDir, "ground_base_b.png"), 256, 256, 2, 8, (_, _) => new byte[] { 110, 110, 110 });
            Png(Path.Combine(assetDir, "ground_base_nm.png"), 256, 256, 2, 8, (_, _) => new byte[] { 128, 128, 255 });
            Png(Path.Combine(assetDir, "ground_base_r.png"), 256, 256, 0, 8, (_, _) => new byte[] { 200 });
            Png(Path.Combine(assetDir, "ground_base_ao.png"), 256, 256, 0, 8, (_, _) => new byte[] { 255 });
            Png(Path.Combine(assetDir, "ground_base_h.png"), 256, 256, 0, 8, (_, _) => new byte[] { 128 });
            Console.WriteLine($"terrain assets -> {assetDir}");
            return 0;
        }
        // Debug: ES_TYPES=regex prints the public members of the TruckLib types whose name matches.
        if (Environment.GetEnvironmentVariable("ES_TYPES") is string typesRe)
        {
            var re = new System.Text.RegularExpressions.Regex(typesRe);
            foreach (var t in typeof(TruckLib.ScsMap.Map).Assembly.GetTypes().Where(t => t.IsPublic && re.IsMatch(t.FullName!)))
            {
                Console.WriteLine($"{(t.IsEnum ? "enum" : "type")} {t.FullName}");
                if (t.IsEnum) { Console.WriteLine("   " + string.Join(", ", Enum.GetNames(t))); continue; }
                foreach (var p in t.GetProperties()) Console.WriteLine($"   {p.PropertyType.Name} {p.Name}");
                foreach (var f in t.GetFields()) Console.WriteLine($"   field {f.FieldType.Name} {f.Name}");
                foreach (var m in t.GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.DeclaredOnly).Where(m => !m.IsSpecialName))
                    Console.WriteLine($"   {m.ReturnType.Name} {m.Name}({string.Join(", ", m.GetParameters().Select(p => p.ParameterType.Name + " " + p.Name))})");
            }
            return 0;
        }
        // Debug: ES_GREP=/dir;regex prints the lines of the text files below /dir that match.
        if (Environment.GetEnvironmentVariable("ES_GREP") is string grepArg)
        {
            var gp = grepArg.Split(';', 2);
            var re = new System.Text.RegularExpressions.Regex(gp[1]);
            foreach (var f in InvisibleTruck.Walk(readers, gp[0], new HashSet<string>()))
            {
                if (!(f.EndsWith(".sii") || f.EndsWith(".sui") || f.EndsWith(".mat"))) { if (re.IsMatch(f)) Console.WriteLine(f); continue; }
                string text;
                try { text = fs.ReadAllText(f); } catch { continue; }
                foreach (var line in text.Split('\n'))
                    if (re.IsMatch(line)) Console.WriteLine($"{f}: {line.Trim()}");
            }
            return 0;
        }
        if (Environment.GetEnvironmentVariable("ES_CAT") is string catFile)
        {
            Console.WriteLine(fs.ReadAllText(catFile));
            return 0;
        }
        // Debug: print the road look definitions (to learn lane/width attributes).
        if (dumpLooks != null)
        {
            var wanted = new HashSet<string>(dumpLooks.Split(','));
            foreach (var f in fs.GetFiles("/def/world").Where(p => p.Contains("road_look")))
            {
                SiiFile sii;
                try { sii = SiiFile.Open(f, fs, true); } catch (Exception e) { Console.WriteLine($"{f}: {e.Message}"); continue; }
                foreach (var u in sii.Units)
                {
                    var tok = u.Name.Contains('.') ? u.Name[(u.Name.IndexOf('.') + 1)..] : u.Name;
                    if (!wanted.Contains(tok) && !wanted.Contains(u.Name)) continue;
                    Console.WriteLine($"== {f} : {u.Class} {u.Name}");
                    foreach (var kv in u.Attributes)
                    {
                        string v = kv.Value is System.Collections.IEnumerable en && kv.Value is not string
                            ? "[" + string.Join(", ", en.Cast<object>()) + "]" : Convert.ToString(kv.Value);
                        Console.WriteLine($"   {kv.Key} = {v}");
                    }
                }
            }
            return 0;
        }

        // Map sectors are 4000 m squares.
        int s = Map.SectorSize;
        var sectors = new List<SectorCoordinate>();
        for (int x = (int)Math.Floor((cx - radius) / s); x <= (int)Math.Floor((cx + radius) / s); x++)
            for (int z = (int)Math.Floor((cz - radius) / s); z <= (int)Math.Floor((cz + radius) / s); z++)
                sectors.Add(new SectorCoordinate(x, z));
        Console.WriteLine($"loading {sectors.Count} sectors around ({cx:F0}, {cz:F0}), radius {radius:F0} m ...");

        var sw = System.Diagnostics.Stopwatch.StartNew();
        Map map = Map.Open("/map/europe.mbd", fs, sectors);
        Console.WriteLine($"map loaded: {map.MapItems.Count} items in {sw.Elapsed.TotalSeconds:F1} s");

        // Debug: ES_MODEL=/path/model.pmd lists its parts and pieces.
        if (Environment.GetEnvironmentVariable("ES_MODEL") is string modelPath)
        {
            var m = TruckLib.Models.Model.Open(modelPath, fs);
            foreach (var lk in m.Looks.Take(1)) foreach (var mt in lk.Materials) Console.WriteLine($"material {mt}");
            foreach (var part in m.Parts)
                foreach (var pc in part.Pieces)
                {
                    var bb = pc.BoundingBox;
                    Console.WriteLine($"part {part.Name} mat {pc.Material} verts {pc.Vertices.Count} tris {pc.Triangles.Count} x {bb.Start.X:F2}..{bb.End.X:F2} y {bb.Start.Y:F2}..{bb.End.Y:F2} z {bb.Start.Z:F2}..{bb.End.Z:F2}");
                    if (part.Name.ToString().StartsWith("coll"))
                        foreach (var v in pc.Vertices.Take(40)) Console.WriteLine($"     v ({v.Position.X:F2}, {v.Position.Y:F2}, {v.Position.Z:F2})");
                }
            foreach (var v in m.Variants) Console.WriteLine($"variant {v.Name}: " + string.Join(", ", v.Attributes.Select(a => $"{a.Tag}={a.Value}")));
            foreach (var l in m.Parts.SelectMany(p => p.Locators)) Console.WriteLine($"locator {l.Name} ({l.Position.X:F2}, {l.Position.Y:F2}, {l.Position.Z:F2})");
            return 0;
        }

        // Debug: ES_PREFAB=x,z lists the prefab models near that point: looks, materials, pieces.
        if (Environment.GetEnvironmentVariable("ES_PREFAB") is string prefabAt)
        {
            var pz = prefabAt.Split(',').Select(v => float.Parse(v, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            var defs = LoadPrefabDefs(fs);
            foreach (var pf in map.MapItems.Values.OfType<Prefab>())
            {
                var nd = pf.Nodes?.FirstOrDefault(n => n != null);
                if (nd == null) continue;
                if (!pf.Nodes.Any(n => n != null && MathF.Abs(n.Position.X - pz[0]) < 400 && MathF.Abs(n.Position.Z - pz[1]) < 400)) continue;
                var tok = pf.Model.ToString();
                Console.WriteLine($"== prefab {tok} variant {pf.Variant} look {pf.Look} nodes {pf.Nodes.Count}");
                if (!PrefabModelPaths.TryGetValue(tok, out var mp)) { Console.WriteLine("   no model_desc"); continue; }
                Console.WriteLine($"   model {mp}");
                TruckLib.Models.Model m;
                try { m = TruckLib.Models.Model.Open(mp, fs); } catch (Exception e) { Console.WriteLine($"   open failed: {e.Message}"); continue; }
                for (int li = 0; li < m.Looks.Count; li++)
                    Console.WriteLine($"   look[{li}] {m.Looks[li].Name}: " + string.Join(" | ", m.Looks[li].Materials.Select((x, i) => $"{i}:{x}")));
                foreach (var part in m.Parts)
                    foreach (var pc in part.Pieces)
                    {
                        var bb = pc.BoundingBox;
                        Console.WriteLine($"   part {part.Name} piece mat {pc.Material} verts {pc.Vertices.Count} tris {pc.Triangles.Count} y {bb.Start.Y:F1}..{bb.End.Y:F1} x {bb.Start.X:F0}..{bb.End.X:F0} z {bb.Start.Z:F0}..{bb.End.Z:F0}");
                    }
                foreach (var v in m.Variants) Console.WriteLine($"   variant {v.Name}: " + string.Join(", ", v.Attributes.Select(a => $"{a.Tag}={a.Value}")));
            }
            return 0;
        }

        // Debug: --probe x,z lists the map items with a node within 60 m of that point.
        if (Environment.GetEnvironmentVariable("ES_PROBE") is string probeAt)
        {
            var pz = probeAt.Split(',').Select(v => float.Parse(v, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
            var pp = new Vector3(pz[0], 0, pz[1]);
            float D(Vector3 v) => MathF.Sqrt((v.X - pp.X) * (v.X - pp.X) + (v.Z - pp.Z) * (v.Z - pp.Z));
            var looks = LoadRoadLooks(fs); var tc = new Dictionary<string, (float Min, float Max)>();
            Console.WriteLine($"roads missing a node: {map.MapItems.Values.OfType<Road>().Count(r => r.Node == null || r.ForwardNode == null)}");
            foreach (var nd in map.Nodes.Values.Where(nd => D(nd.Position) < 60))
                Console.WriteLine($"node {nd.Uid:X} ({nd.Position.X:F0},{nd.Position.Y:F0},{nd.Position.Z:F0}) back {nd.BackwardItem?.GetType().Name} fwd {nd.ForwardItem?.GetType().Name}");
            foreach (var it in map.MapItems.Values.Where(i => i is not Road && i is not Prefab).Take(0)) { }
            var kinds = map.MapItems.Values.GroupBy(i => i.GetType().Name).Select(g => $"{g.Key}:{g.Count()}");
            Console.WriteLine("item kinds: " + string.Join(" ", kinds));
            var inner = map.MapItems.Values.OfType<Compound>().SelectMany(c => c.MapItems.Values).ToList();
            Console.WriteLine("inside compounds: " + string.Join(" ", inner.GroupBy(i => i.GetType().Name).Select(g => $"{g.Key}:{g.Count()}")));
            foreach (var rd in inner.OfType<Road>().Where(r => r.Node != null && r.ForwardNode != null && Math.Min(D(r.Node.Position), D(r.ForwardNode.Position)) < 300))
                Console.WriteLine($"compound road {rd.RoadType} ({rd.Node.Position.X:F0},{rd.Node.Position.Z:F0})->({rd.ForwardNode.Position.X:F0},{rd.ForwardNode.Position.Z:F0})");
            foreach (var it in map.MapItems.Values)
            {
                if (it is Terrain tr && tr.Node != null && tr.ForwardNode != null && Math.Min(D(tr.Node.Position), D(tr.ForwardNode.Position)) < 300)
                    Console.WriteLine($"terrain item ({tr.Node.Position.X:F0},{tr.Node.Position.Z:F0})->({tr.ForwardNode.Position.X:F0},{tr.ForwardNode.Position.Z:F0}) len {tr.Length:F0} L {tr.Left?.Terrain?.Size} {tr.Left?.Terrain?.Profile} R {tr.Right?.Terrain?.Size} {tr.Right?.Terrain?.Profile} step {tr.StepSize} cols {tr.Left?.Terrain?.QuadData?.Cols} nodeOff {tr.NodeOffset} fwdOff {tr.ForwardNodeOffset} offs " + string.Join(" ", (tr.Right?.Terrain?.QuadData?.Offsets ?? new()).Take(4).Select(o => $"({o.X},{o.Y}){o.Data}")));
                if (it is Prefab pq && pq.Nodes != null && pq.Nodes.Any(nd => nd != null && D(nd.Position) < 150))
                    Console.WriteLine($"prefab {pq.Model} terrain: " + string.Join(" | ", pq.PrefabNodes?.Select(pn => $"{pn.Terrain?.Size} {pn.Terrain?.Profile} q {pn.Terrain?.QuadData?.Rows}x{pn.Terrain?.QuadData?.Cols} off {pn.Terrain?.QuadData?.Offsets?.Count}") ?? new string[0]));
                if (it is Road rd && rd.Node != null && rd.ForwardNode != null && Math.Min(D(rd.Node.Position), D(rd.ForwardNode.Position)) < 60)
                {
                    var lw = looks.TryGetValue(rd.RoadType.ToString(), out var u) ? LookHalfWidths(u, fs, tc) : null;
                    Console.WriteLine($"road {rd.RoadType} {rd.Node.Uid:X}->{rd.ForwardNode.Uid:X} ({rd.Node.Position.X:F0},{rd.Node.Position.Z:F0})->({rd.ForwardNode.Position.X:F0},{rd.ForwardNode.Position.Z:F0}) look {lw} railL {RailingOffset(rd.Left)} railR {RailingOffset(rd.Right)}");
                    foreach (var (sn, sd) in new[] { ("L", rd.Left), ("R", rd.Right) })
                    {
                        var t = sd?.Terrain;
                        if (t == null) continue;
                        var q = t.QuadData;
                        Console.WriteLine($"   terrain {sn}: size {t.Size} profile {t.Profile} coef {t.Coefficient} noise {t.Noise} transition {t.Transition} len {rd.Length:F1} res {rd.Resolution} quads {q?.Rows}x{q?.Cols} offsets {q?.Offsets?.Count} normals {q?.Normals?.Count}");
                        if (q?.Offsets != null)
                            foreach (var o in q.Offsets.Take(6)) Console.WriteLine($"      offset ({o.X},{o.Y}) {o.Data}");
                    }
                }
                if (it is Prefab pf && pf.Nodes != null && pf.Nodes.Any(nd => nd != null && D(nd.Position) < 60))
                    Console.WriteLine($"prefab {pf.Model} nodes " + string.Join(" ", pf.Nodes.Where(nd => nd != null).Select(nd => $"{nd.Uid:X}({nd.Position.X:F0},{nd.Position.Z:F0})")));
            }
            return 0;
        }

        var strips = new List<(uint kind, float width, List<Vector3> pts)>();
        var roadTypes = new Dictionary<string, int>();
        int barriers = 0, junctionsExact = 0, junctionsApprox = 0, junctionsColl = 0;

        var roadLooks = LoadRoadLooks(fs);
        var lookWidths = new Dictionary<string, (float L, float R)?>();
        var templateCache = new Dictionary<string, (float Min, float Max)>();
        int unknownLooks = 0, railways = 0;
        Console.WriteLine($"road looks: {roadLooks.Count}");

        var prefabDefs = LoadPrefabDefs(fs);
        Console.WriteLine($"prefab definitions: {prefabDefs.Count}");
        var ppdCache = new Dictionary<string, PrefabDescriptor>();
        PrefabDescriptor GetPpd(string token)
        {
            if (!prefabDefs.TryGetValue(token, out var path)) return null;
            if (ppdCache.TryGetValue(path, out var cached)) return cached;
            PrefabDescriptor d = null;
            try { d = PrefabDescriptor.Open(path, fs); }
            catch (Exception e) { Console.Error.WriteLine($"ppd {path}: {e.Message}"); }
            ppdCache[path] = d;
            return d;
        }
        double r2 = radius * radius;
        bool Near(Vector3 p) => (p.X - cx) * (p.X - cx) + (p.Z - cz) * (p.Z - cz) <= r2;

        // ---- prefab collision straight from the game's models ----
        // A prefab model (.pmd/.pmg) has collision parts ("coll", "coll_walls", "coll_bridge", ...):
        // the exact surfaces and walls ETS2's own truck drives on. Each variant lists which parts
        // exist (variant attribute i = part i visible). Placed with the same fit as the nav curves.
        var modelCache = new Dictionary<string, TruckLib.Models.Model>();
        TruckLib.Models.Model GetModel(string path)
        {
            if (modelCache.TryGetValue(path, out var m)) return m;
            try { m = TruckLib.Models.Model.Open(path, fs); }
            catch (Exception e) { Console.Error.WriteLine($"model {path}: {e.Message}"); m = null; }
            modelCache[path] = m;
            return m;
        }
        // World triangles (a, b, c, wall) of a prefab's collision, or null when unavailable.
        List<(Vector3 A, Vector3 B, Vector3 C, bool Wall)> PrefabCollision(Prefab pf)
        {
            var tok = pf.Model.ToString();
            if (!PrefabModelPaths.TryGetValue(tok, out var mp)) return null;
            var model = GetModel(mp);
            var ppd = GetPpd(tok);
            if (model == null || ppd == null || pf.Nodes == null || pf.Nodes.Count == 0 || pf.Nodes.Any(n => n == null)) return null;
            if (!FitPrefab(ppd, pf, out var place)) return null;
            var variant = model.Variants.FirstOrDefault(v => v.Name.ToString() == pf.Variant.ToString()) ?? model.Variants.FirstOrDefault();
            // Only what a car drives on: surfaces at road level and the road walls ("coll_walls"...).
            // Garages, services and companies also carry building collision (walls, roofs up to
            // 10 m) - those made "random walls" across the road.
            float maxNodeY = pf.Nodes.Max(n => n.Position.Y);
            // A surface high above the nodes is a roof - unless traffic drives on it: a bridge or
            // flyover inside a junction carries the prefab's own lanes (AI nav curves).
            var navPts = new List<Vector3>();
            foreach (var nc in ppd.NavCurves ?? new List<NavCurve>())
                if (NavCurvePoints(nc, place) is { } np) navPts.AddRange(np);
            bool CarriesLane(Vector3 a, Vector3 b, Vector3 c)
            {
                const float Reach = 8f, Dy = 3f;
                float x0 = MathF.Min(a.X, MathF.Min(b.X, c.X)) - Reach, x1 = MathF.Max(a.X, MathF.Max(b.X, c.X)) + Reach;
                float z0 = MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) - Reach, z1 = MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) + Reach;
                float y0 = MathF.Min(a.Y, MathF.Min(b.Y, c.Y)) - Dy, y1 = MathF.Max(a.Y, MathF.Max(b.Y, c.Y)) + Dy;
                foreach (var p in navPts)
                    if (p.X >= x0 && p.X <= x1 && p.Z >= z0 && p.Z <= z1 && p.Y >= y0 && p.Y <= y1) return true;
                return false;
            }
            var res = new List<(Vector3, Vector3, Vector3, bool)>();
            for (int pi = 0; pi < model.Parts.Count; pi++)
            {
                var part = model.Parts[pi];
                var partName = part.Name.ToString();
                if (!partName.StartsWith("coll")) continue;
                if (variant != null && pi < variant.Attributes.Count && variant.Attributes[pi].Value == 0) continue;
                bool wallsAllowed = partName.Contains("wall");
                foreach (var pc in part.Pieces)
                    foreach (var t in pc.Triangles)
                    {
                        if (t.A >= pc.Vertices.Count || t.B >= pc.Vertices.Count || t.C >= pc.Vertices.Count) continue;
                        var a = Apply(place, pc.Vertices[t.A].Position);
                        var b = Apply(place, pc.Vertices[t.B].Position);
                        var cc = Apply(place, pc.Vertices[t.C].Position);
                        var n = Vector3.Cross(b - a, cc - a);
                        float len = n.Length();
                        if (len < 1e-6f) continue;
                        bool wall = MathF.Abs(n.Y / len) < 0.5f;   // steeper than 60 deg
                        if (wall && !wallsAllowed) continue;
                        if (!wall && MathF.Max(a.Y, MathF.Max(b.Y, cc.Y)) > maxNodeY + 2.5f && !CarriesLane(a, b, cc)) continue;   // roofs, upper floors
                        res.Add((a, b, cc, wall));
                    }
            }
            if (res.Count == 0) return null;
            FillSmallHoles(res);
            SmoothMicroBumps(res);
            return res;
        }

        // The game's collision has small humps (a corner 2-8 cm off the surface around it): no
        // matter to a truck, a jolt to a car. A surface corner whose neighbours all lie within
        // 10 cm of it goes to their mean height when it is 1.5-8 cm off it (kerbs, with their
        // vertical faces and bigger steps, stay).
        static void SmoothMicroBumps(List<(Vector3 A, Vector3 B, Vector3 C, bool Wall)> tris)
        {
            static (int, int, int) K(Vector3 v) => ((int)MathF.Round(v.X * 20), (int)MathF.Round(v.Z * 20), (int)MathF.Round(v.Y * 100));
            var heights = new Dictionary<(int, int, int), float>();
            var nbrs = new Dictionary<(int, int, int), HashSet<(int, int, int)>>();
            foreach (var (a, b, c, wall) in tris)
            {
                if (wall) continue;
                var vs = new[] { a, b, c };
                for (int i = 0; i < 3; i++)
                {
                    var k = K(vs[i]);
                    heights[k] = vs[i].Y;
                    if (!nbrs.TryGetValue(k, out var s)) nbrs[k] = s = new HashSet<(int, int, int)>();
                    s.Add(K(vs[(i + 1) % 3]));
                    s.Add(K(vs[(i + 2) % 3]));
                }
            }
            var fixedY = new Dictionary<(int, int, int), float>();
            foreach (var (k, s) in nbrs)
            {
                if (s.Count < 3) continue;
                float y = heights[k], sum = 0;
                bool flat = true;
                int above = 0, below = 0;
                foreach (var n in s)
                {
                    float ny = heights[n];
                    if (MathF.Abs(ny - y) > 0.10f) { flat = false; break; }
                    if (ny > y + 0.005f) above++; else if (ny < y - 0.005f) below++;
                    sum += ny;
                }
                // Only a lone hump or pit: every neighbour lower, or every neighbour higher.
                if (!flat || (Environment.GetEnvironmentVariable("ES_LONEHUMP") != null && above != s.Count && below != s.Count)) continue;
                float mean = sum / s.Count, off = MathF.Abs(y - mean);
                if (off > 0.015f && off < 0.08f) fixedY[k] = mean;
            }
            if (fixedY.Count == 0) return;
            Vector3 F(Vector3 v) => fixedY.TryGetValue(K(v), out var y) ? new Vector3(v.X, y, v.Z) : v;
            for (int i = 0; i < tris.Count; i++)
            {
                var (a, b, c, wall) = tris[i];
                if (!wall) tris[i] = (F(a), F(b), F(c), wall);
            }
        }

        // Holes inside a junction's collision (traffic islands keep their kerbs elsewhere) catch a
        // wheel like a wall: fill every inner boundary loop up to 300 m^2 with a fan.
        static void FillSmallHoles(List<(Vector3 A, Vector3 B, Vector3 C, bool Wall)> tris)
        {
            var ids = new Dictionary<(int, int, int), int>();
            var verts = new List<Vector3>();
            int Id(Vector3 v)
            {
                var k = ((int)MathF.Round(v.X * 20), (int)MathF.Round(v.Y * 20), (int)MathF.Round(v.Z * 20));
                if (!ids.TryGetValue(k, out int i)) { i = verts.Count; verts.Add(v); ids[k] = i; }
                return i;
            }
            // Undirected edge -> how many surface triangles use it, and one direction seen.
            var edges = new Dictionary<(int, int), (int Count, int From, int To)>();
            foreach (var t in tris)
            {
                if (t.Wall) continue;
                int a = Id(t.A), b = Id(t.B), c = Id(t.C);
                foreach (var (u, w) in new[] { (a, b), (b, c), (c, a) })
                {
                    if (u == w) continue;
                    var key = u < w ? (u, w) : (w, u);
                    edges[key] = edges.TryGetValue(key, out var e) ? (e.Count + 1, e.From, e.To) : (1, u, w);
                }
            }
            // Boundary edges chained into loops.
            var next = new Dictionary<int, List<int>>();
            foreach (var (_, e) in edges)
            {
                if (e.Count != 1) continue;
                if (!next.TryGetValue(e.From, out var l)) next[e.From] = l = new List<int>();
                l.Add(e.To);
                if (!next.TryGetValue(e.To, out l)) next[e.To] = l = new List<int>();
                l.Add(e.From);
            }
            var used = new HashSet<(int, int)>();
            var loops = new List<List<int>>();
            foreach (var start in next.Keys)
                foreach (var first in next[start])
                {
                    if (used.Contains((start, first))) continue;
                    var loop = new List<int> { start };
                    int prev = start, cur = first;
                    used.Add((start, first)); used.Add((first, start));
                    while (cur != start && loop.Count < 2000)
                    {
                        loop.Add(cur);
                        int nx = -1;
                        foreach (var cand in next[cur]) if (!used.Contains((cur, cand))) { nx = cand; break; }
                        if (nx < 0) break;
                        used.Add((cur, nx)); used.Add((nx, cur));
                        prev = cur; cur = nx;
                    }
                    if (cur == start && loop.Count >= 3) loops.Add(loop);
                }
            if (loops.Count < 2) return;
            float Area(List<int> l)
            {
                double s = 0;
                for (int i = 0; i < l.Count; i++)
                {
                    var p = verts[l[i]]; var q = verts[l[(i + 1) % l.Count]];
                    s += (double)p.X * q.Z - (double)q.X * p.Z;
                }
                return (float)Math.Abs(s / 2);
            }
            var outer = loops.OrderByDescending(Area).First();
            foreach (var l in loops)
            {
                if (ReferenceEquals(l, outer) || Area(l) > 300f) continue;
                var c = l.Aggregate(Vector3.Zero, (acc, i) => acc + verts[i]) / l.Count;
                // Covered by another collision part (a raised traffic island has its own): not a hole.
                bool covered = false;
                foreach (var t in tris)
                {
                    if (t.Wall) continue;
                    float det = (t.B.Z - t.C.Z) * (t.A.X - t.C.X) + (t.C.X - t.B.X) * (t.A.Z - t.C.Z);
                    if (MathF.Abs(det) < 1e-9f) continue;
                    float l1 = ((t.B.Z - t.C.Z) * (c.X - t.C.X) + (t.C.X - t.B.X) * (c.Z - t.C.Z)) / det;
                    float l2 = ((t.C.Z - t.A.Z) * (c.X - t.C.X) + (t.A.X - t.C.X) * (c.Z - t.C.Z)) / det;
                    if (l1 >= 0 && l2 >= 0 && 1 - l1 - l2 >= 0) { covered = true; break; }
                }
                if (covered) continue;
                for (int i = 0; i < l.Count; i++) tris.Add((c, verts[l[i]], verts[l[(i + 1) % l.Count]], false));
            }
        }
        var prefabColl = new Dictionary<Prefab, List<(Vector3 A, Vector3 B, Vector3 C, bool Wall)>>();
        foreach (var item in map.MapItems.Values)
            if (item is Prefab pf && pf.Nodes != null && pf.Nodes.Any(n => n != null && Near(n.Position)))
                if (PrefabCollision(pf) is { } coll) prefabColl[pf] = coll;
        Console.WriteLine($"prefabs with game collision: {prefabColl.Count}");
        var collNodes = new HashSet<ulong>(prefabColl.Keys.SelectMany(p => p.Nodes.Where(n => n != null).Select(n => n.Uid)));

        // Debug ES_WIDTHCHK: where a road meets a junction with game collision, compare the road
        // look's numbers with the junction's collision width at that node (ETS2 joins them exactly).
        if (Environment.GetEnvironmentVariable("ES_WIDTHCHK") != null)
        {
            var nodePrefab = new Dictionary<ulong, Prefab>();
            foreach (var pf in prefabColl.Keys) foreach (var n in pf.Nodes) if (n != null) nodePrefab[n.Uid] = pf;
            int shown = 0;
            foreach (var item in map.MapItems.Values)
            {
                if (item is not Road rd || rd.Node == null || rd.ForwardNode == null || shown >= 40) continue;
                var end = nodePrefab.ContainsKey(rd.ForwardNode.Uid) ? rd.ForwardNode : nodePrefab.ContainsKey(rd.Node.Uid) ? rd.Node : null;
                if (end == null) continue;
                var tok = rd.RoadType.ToString();
                if (!roadLooks.TryGetValue(tok, out var lu)) continue;
                string A(string k) => lu.Attributes.TryGetValue(k, out var v) ? Convert.ToString((object)v) : "-";
                string TemplX(string k)
                {
                    if (!lu.Attributes.TryGetValue(k, out var p)) return "-";
                    try
                    {
                        var m = TruckLib.Models.Model.Open(Convert.ToString((object)p), fs);
                        var cps = m.Parts.Where(pt => pt.Name.ToString().StartsWith("coll")).SelectMany(pt => pt.Pieces).ToList();
                        if (cps.Count == 0) return "no coll";
                        return $"{cps.Min(pc => pc.BoundingBox.Start.X):F2}..{cps.Max(pc => pc.BoundingBox.End.X):F2}";
                    }
                    catch (Exception e) { return "err " + e.Message; }
                }
                // Junction collision extent along the node's right axis, through the node.
                var r3 = Vector3.Transform(Vector3.UnitX, end.Rotation);
                var rh = Vector2.Normalize(new Vector2(r3.X, r3.Z));
                var o = new Vector2(end.Position.X, end.Position.Z);
                float tMin = float.MaxValue, tMax = float.MinValue;
                foreach (var t in prefabColl[nodePrefab[end.Uid]])
                {
                    if (t.Wall) continue;
                    var tri = new[] { new Vector2(t.A.X, t.A.Z), new Vector2(t.B.X, t.B.Z), new Vector2(t.C.X, t.C.Z) };
                    for (float ls = -40; ls <= 40; ls += 0.1f)
                    {
                        var q = o + rh * ls;
                        float d1 = (q.X - tri[1].X) * (tri[0].Y - tri[1].Y) - (tri[0].X - tri[1].X) * (q.Y - tri[1].Y);
                        float d2 = (q.X - tri[2].X) * (tri[1].Y - tri[2].Y) - (tri[1].X - tri[2].X) * (q.Y - tri[2].Y);
                        float d3 = (q.X - tri[0].X) * (tri[2].Y - tri[0].Y) - (tri[2].X - tri[0].X) * (q.Y - tri[0].Y);
                        bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
                        if (neg && pos) continue;
                        tMin = Math.Min(tMin, s); tMax = Math.Max(tMax, s);
                    }
                }
                Console.WriteLine($"{tok,-14} offset {A("road_offset"),-4} R {TemplX("template_right"),-12} L {TemplX("template_left"),-12} size L/R {A("road_size_left")}/{A("road_size_right")} | junction coll at node: {tMin:F1} .. {tMax:F1} (right +) | edges R {rd.Right.LeftEdge}/{rd.Right.RightEdge} L {rd.Left.LeftEdge}/{rd.Left.RightEdge}");
                shown++;
            }
            return 0;
        }

        var terrainProfiles = TerrainBuilder.LoadProfiles(fs);
        bool withTerrain = Environment.GetEnvironmentVariable("ES_NOTERRAIN") == null;
        Console.WriteLine($"terrain profiles: {terrainProfiles.Count}{(withTerrain ? "" : " (terrain off)")}");
        // ---- pass 1: every road segment with its axis points and surface widths ----
        var segs = new List<RoadSeg>();
        var tiles = new TileSet(TileSize);
        var nodeEdges = new Dictionary<ulong, List<Vector3>>();   // node uid -> road edge points ending there
        void AddNodeEdge(ulong uid, Vector3 l, Vector3 r)
        {
            if (!nodeEdges.TryGetValue(uid, out var list)) nodeEdges[uid] = list = new List<Vector3>();
            list.Add(l);
            list.Add(r);
        }
        var nodeWidth = new Dictionary<ulong, float>();   // node uid -> widest surface ending there
        var nodeSegs = new Dictionary<ulong, List<(int Seg, bool AtStart)>>();
        var segBank = new List<(float A, float B, float Len)>();
        var segKerb = new Dictionary<int, (bool L, bool R)>();   // kerb / sidewalk on that edge
        var nodeLR = new Dictionary<ulong, (float L, float R, Vector2 Rh)>();   // road width at a node
        foreach (var item in map.MapItems.Values)
        {
            if (item is not Road road || road.Node == null || road.ForwardNode == null) continue;
            var a = road.Node.Position;
            var b = road.ForwardNode.Position;
            if (!Near(a) && !Near(b)) continue;
            var rtok = road.RoadType.ToString();
            if (roadLooks.TryGetValue(rtok, out var lookUnit) && IsRailway(lookUnit)) { railways++; continue; }
            if (!lookWidths.TryGetValue(rtok, out var lw))
            {
                lw = roadLooks.TryGetValue(rtok, out var unit) ? LookHalfWidths(unit, fs, templateCache) : null;
                lookWidths[rtok] = lw;
            }
            // Surface: exactly the road look's cross-section (railings only add walls).
            float? railL = RailingOffset(road.Left), railR = RailingOffset(road.Right);
            float halfL = Math.Clamp((lw?.L ?? DefaultHalfWidth) + EdgeMargin, -30f, 35f);
            float halfR = Math.Clamp((lw?.R ?? DefaultHalfWidth) + EdgeMargin, -30f, 35f);
            if (halfL + halfR < 1f) { float mid = (halfR - halfL) / 2; halfL = 0.5f - mid; halfR = 0.5f + mid; }
            if (lw == null) unknownLooks++;
            // Kerbs / sidewalks on the outer edges (ETS2 "edge" models): walls there. The outer edge
            // of a side template is its RightEdge; a one-template road's left edge is its LeftEdge.
            static bool Kerb(Token t) { var s = t.ToString(); return s.Contains("curb") || s.Contains("_sw_") || s.Contains("sidewalk"); }
            bool twoSided = lookUnit != null && lookUnit.Attributes.ContainsKey("template_left") || lookUnit != null && lookUnit.Attributes.ContainsKey("road_size_left");
            bool kerbR = road.Right != null && Kerb(road.Right.RightEdge);
            bool kerbL = twoSided ? road.Left != null && Kerb(road.Left.RightEdge) : road.Right != null && Kerb(road.Right.LeftEdge);
            segKerb[segs.Count] = (kerbL, kerbR);

            float len = road.Length > 0 ? road.Length : Vector3.Distance(a, b);
            int n = Math.Max(2, (int)Math.Ceiling(len / StepMetres) + 1);
            var pts = new List<Vector3>(n);
            for (int k = 0; k < n; k++) pts.Add(HermiteSpline.InterpolatePolyline(road.Node, road.ForwardNode, k / (float)(n - 1), len));
            var rhs = new List<Vector2>(n);
            for (int k = 0; k < n; k++) rhs.Add(RightAt(pts, k));
            segs.Add(new RoadSeg(road.Node.Uid, road.ForwardNode.Uid, pts, halfL, halfR, railL, railR,
                                 new List<Vector3>(n), new List<Vector3>(n), rhs));
            segBank.Add((Bank(road.Node.Rotation), Bank(road.ForwardNode.Rotation), len));
            // The ground beside the road (ETS2's terrain strips), so leaving the road is not falling.
            if (withTerrain)
            {
                var (bkA, bkB, _) = segBank[^1];
                TerrainBuilder.Side(tiles, road.Node, road.ForwardNode, road.Left?.Terrain, false, halfL, bkA, bkB, len, terrainProfiles);
                TerrainBuilder.Side(tiles, road.Node, road.ForwardNode, road.Right?.Terrain, true, halfR, bkA, bkB, len, terrainProfiles);
            }
            foreach (var (uid, atStart) in new[] { (road.Node.Uid, true), (road.ForwardNode.Uid, false) })
            {
                if (!nodeSegs.TryGetValue(uid, out var ns)) nodeSegs[uid] = ns = new List<(int, bool)>();
                ns.Add((segs.Count - 1, atStart));
            }
            roadTypes[rtok] = roadTypes.GetValueOrDefault(rtok) + 1;
        }

        // Standalone terrain items (ground the map's authors laid between roads): the same strips,
        // from their own line.
        if (withTerrain)
            foreach (var item in map.MapItems.Values)
            {
                if (item is not Terrain tr || tr.Node == null || tr.ForwardNode == null) continue;
                if (!Near(tr.Node.Position) && !Near(tr.ForwardNode.Position)) continue;
                float len = tr.Length > 0 ? tr.Length : Vector3.Distance(tr.Node.Position, tr.ForwardNode.Position);
                TerrainBuilder.Side(tiles, tr.Node, tr.ForwardNode, tr.Left?.Terrain, false, 0, 0, 0, len, terrainProfiles);
                TerrainBuilder.Side(tiles, tr.Node, tr.ForwardNode, tr.Right?.Terrain, true, 0, 0, 0, len, terrainProfiles);
            }

        // Pass-through prefabs (two nodes, e.g. a lane-count change): the roads on both sides count
        // as meeting, so their widths blend across the prefab.
        var passLink = new Dictionary<ulong, ulong>();
        foreach (var item in map.MapItems.Values)
        {
            if (item is not Prefab pf || pf.Nodes == null || pf.Nodes.Count != 2 || pf.Nodes[0] == null || pf.Nodes[1] == null) continue;
            if (Vector3.Distance(pf.Nodes[0].Position, pf.Nodes[1].Position) > 80f) continue;
            passLink[pf.Nodes[0].Uid] = pf.Nodes[1].Uid;
            passLink[pf.Nodes[1].Uid] = pf.Nodes[0].Uid;
        }

        // ---- pass 1b: paved edges. Where two roads of different width meet, the wider one
        // narrows smoothly to the other's width (no steps / arrow shapes at the joint).
        for (int i = 0; i < segs.Count; i++)
        {
            var sg = segs[i];
            var (bankA, bankB, len) = segBank[i];
            // Target half widths (left, right) at each end, in this segment's direction.
            (float L, float R) EndTarget(ulong uid, bool atStart)
            {
                // Into a junction with the game's exact collision: arrive at the exact width
                // (no extra margin), so there is no step where the two meet.
                if (collNodes.Contains(uid))
                    return (sg.RailL == null ? Math.Max(0.5f, sg.HalfL - EdgeMargin) : sg.HalfL,
                            sg.RailR == null ? Math.Max(0.5f, sg.HalfR - EdgeMargin) : sg.HalfR);
                if (!nodeSegs.TryGetValue(uid, out var ns)) return (sg.HalfL, sg.HalfR);
                int j; bool jStart;
                if (ns.Count == 2)
                    (j, jStart) = ns[0].Item1 == i && ns[0].Item2 == atStart ? ns[1] : ns[0];
                else if (ns.Count == 1 && passLink.TryGetValue(uid, out var far) &&
                         nodeSegs.TryGetValue(far, out var fs) && fs.Count == 1)
                    (j, jStart) = fs[0];
                else return (sg.HalfL, sg.HalfR);
                var o = segs[j];
                bool sameDir = atStart != jStart;   // one ends where the other starts
                float oL = sameDir ? o.HalfL : o.HalfR, oR = sameDir ? o.HalfR : o.HalfL;
                return (Math.Min(sg.HalfL, oL), Math.Min(sg.HalfR, oR));
            }
            var t0 = EndTarget(sg.StartUid, true);
            var t1 = EndTarget(sg.EndUid, false);
            float taper = Math.Min(30f, len * 0.45f);
            int n = sg.Pts.Count;
            for (int k = 0; k < n; k++)
            {
                float f = k / (float)(n - 1), s0 = f * len, s1 = (1 - f) * len;
                float W(float own, float tA, float tB)
                {
                    float w = own;
                    if (s0 < taper) w = Math.Min(w, tA + (own - tA) * SmoothStep(s0 / taper));
                    if (s1 < taper) w = Math.Min(w, tB + (own - tB) * SmoothStep(s1 / taper));
                    return w;
                }
                float hl = W(sg.HalfL, t0.L, t1.L), hr = W(sg.HalfR, t0.R, t1.R);
                var rh = sg.Rh[k];
                if (k == 0 && !nodeLR.ContainsKey(sg.StartUid)) nodeLR[sg.StartUid] = (hl, hr, rh);
                if (k == n - 1 && !nodeLR.ContainsKey(sg.EndUid)) nodeLR[sg.EndUid] = (hl, hr, rh);
                float bank = bankA + (bankB - bankA) * f;
                sg.Left.Add(sg.Pts[k] + new Vector3(-rh.X * hl, -bank * hl, -rh.Y * hl));
                sg.Right.Add(sg.Pts[k] + new Vector3(rh.X * hr, bank * hr, rh.Y * hr));
            }
            AddNodeEdge(sg.StartUid, sg.Left[0], sg.Right[0]);
            AddNodeEdge(sg.EndUid, sg.Left[^1], sg.Right[^1]);
            foreach (var uid in new[] { sg.StartUid, sg.EndUid })
                nodeWidth[uid] = Math.Max(nodeWidth.GetValueOrDefault(uid), sg.HalfL + sg.HalfR);
        }

        // ---- pass 1c: pass-through prefabs become road pieces too, their width carried along
        // prefab chains from the nearest real roads and blended end to end.
        var passPrefabs = new List<Prefab>();
        foreach (var item in map.MapItems.Values)
            if (item is Prefab pf && !prefabColl.ContainsKey(pf) && pf.Nodes is { Count: 2 } && pf.Nodes[0] != null && pf.Nodes[1] != null &&
                passLink.ContainsKey(pf.Nodes[0].Uid) && (Near(pf.Nodes[0].Position) || Near(pf.Nodes[1].Position)))
                passPrefabs.Add(pf);
        for (bool changed = true; changed;)   // widths spread through prefab-to-prefab chains
        {
            changed = false;
            foreach (var pf in passPrefabs)
            {
                ulong a = pf.Nodes[0].Uid, b = pf.Nodes[1].Uid;
                if (nodeLR.ContainsKey(a) && !nodeLR.ContainsKey(b)) { nodeLR[b] = nodeLR[a]; changed = true; }
                else if (nodeLR.ContainsKey(b) && !nodeLR.ContainsKey(a)) { nodeLR[a] = nodeLR[b]; changed = true; }
            }
        }
        var virtualPrefabs = new HashSet<Prefab>();
        foreach (var pf in passPrefabs)
        {
            var na = pf.Nodes[0]; var nb = pf.Nodes[1];
            if (!nodeLR.TryGetValue(na.Uid, out var wa) || !nodeLR.TryGetValue(nb.Uid, out var wb)) continue;
            var pa = na.Position; var pb = nb.Position;
            float len = Vector3.Distance(pa, pb);
            if (len < 1f) continue;
            var chord = (pb - pa) / len;
            Vector3 Tan(Quaternion q)
            {
                var d = Vector3.Normalize(Vector3.Transform(-Vector3.UnitZ, q));
                return (Vector3.Dot(d, chord) < 0 ? -d : d) * len;
            }
            int n = Math.Max(2, (int)MathF.Ceiling(len / StepMetres) + 1);
            var pts = new List<Vector3>(n);
            for (int k = 0; k < n; k++) pts.Add(HermiteSpline.Interpolate(pa, pb, Tan(na.Rotation), Tan(nb.Rotation), k / (float)(n - 1)));
            var rhs = Enumerable.Range(0, n).Select(k => RightAt(pts, k)).ToList();
            // Each end's (left, right) in this piece's direction.
            (float L, float R) Oriented((float L, float R, Vector2 Rh) w, Vector2 rh) =>
                Vector2.Dot(w.Rh, rh) >= 0 ? (w.L, w.R) : (w.R, w.L);
            var ea = Oriented(wa, rhs[0]); var eb = Oriented(wb, rhs[^1]);
            var left = new List<Vector3>(n); var right = new List<Vector3>(n);
            for (int k = 0; k < n; k++)
            {
                float f = SmoothStep(k / (float)(n - 1));
                float hl = ea.L + (eb.L - ea.L) * f, hr = ea.R + (eb.R - ea.R) * f;
                left.Add(pts[k] + new Vector3(-rhs[k].X * hl, 0, -rhs[k].Y * hl));
                right.Add(pts[k] + new Vector3(rhs[k].X * hr, 0, rhs[k].Y * hr));
            }
            segs.Add(new RoadSeg(na.Uid, nb.Uid, pts, Math.Max(ea.L, eb.L), Math.Max(ea.R, eb.R), null, null, left, right, rhs));
            AddNodeEdge(na.Uid, left[0], right[0]);
            AddNodeEdge(nb.Uid, left[^1], right[^1]);
            virtualPrefabs.Add(pf);
        }

        // ---- pass 2: chain connected segments of equal width into one continuous surface ----
        // (no seams at the joints, and far fewer objects for BeamNG to build)
        var byStart = new Dictionary<ulong, List<int>>();
        for (int i = 0; i < segs.Count; i++)
        {
            if (!byStart.TryGetValue(segs[i].StartUid, out var l)) byStart[segs[i].StartUid] = l = new List<int>();
            l.Add(i);
        }
        bool Same(RoadSeg x, RoadSeg y) => Math.Abs(x.HalfL - y.HalfL) < 0.05f && Math.Abs(x.HalfR - y.HalfR) < 0.05f;
        int Next(int i, bool[] used)
        {
            if (!byStart.TryGetValue(segs[i].EndUid, out var cands)) return -1;
            foreach (var j in cands) if (!used[j] && Same(segs[i], segs[j])) return j;
            return -1;
        }
        var hasPred = new bool[segs.Count];
        { var probe = new bool[segs.Count]; for (int i = 0; i < segs.Count; i++) { int j = Next(i, probe); if (j >= 0) hasPred[j] = true; } }
        var usedSeg = new bool[segs.Count];
        var segChain = new int[segs.Count];
        int chains = 0;
        // A thin vertical wall along a line: 0.3 m below to BarrierHeight above it.
        void Wall(List<Vector3> line)
        {
            var dn = new Vector3(0, -0.3f, 0);
            var up = new Vector3(0, BarrierHeight, 0);
            for (int k = 0; k + 1 < line.Count; k++)
                tiles.Quad(line[k] + dn, line[k] + up, line[k + 1] + dn, line[k + 1] + up, doubleSided: true);
        }
        void EmitChain(int first)
        {
            var axis = new List<Vector3>();
            var s0 = segs[first];
            for (int i = first; i >= 0 && !usedSeg[i]; i = Next(i, usedSeg))
            {
                usedSeg[i] = true;
                segChain[i] = chains;
                var p = segs[i].Pts;
                axis.AddRange(axis.Count == 0 ? p : p.Skip(1));
                // Mesh: the paved surface between the exact edges.
                var le = segs[i].Left; var re = segs[i].Right;
                for (int k = 0; k + 1 < le.Count; k++) tiles.RoadQuad(le[k], re[k], le[k + 1], re[k + 1]);
                // Into a junction with the game's collision: overlap it by a few metres, so no
                // thin crack stays between the road's end and the junction's surface. Every other
                // end overlaps by half a metre (a 10 cm seam to the next piece caught wheels).
                if (le.Count >= 2)
                {
                    float startOver = collNodes.Contains(segs[i].StartUid) ? JunctionOverlap : 0.5f;
                    float endOver = collNodes.Contains(segs[i].EndUid) ? JunctionOverlap : 0.5f;
                    {
                        var d = Vector3.Normalize((le[0] + re[0]) - (le[1] + re[1])) * startOver;
                        tiles.SeamQuad(le[0] + d, re[0] + d, le[0], re[0]);
                    }
                    {
                        var d = Vector3.Normalize((le[^1] + re[^1]) - (le[^2] + re[^2])) * endOver;
                        tiles.SeamQuad(le[^1], re[^1], le[^1] + d, re[^1] + d);
                    }
                }
                // Barriers stay per segment (railings change along a road).
                if (segs[i].RailL is float l) { strips.Add((2, BarrierWidth, Offset(p, -l, BarrierHeight))); Wall(Offset(p, -l, 0)); barriers++; }
                if (segs[i].RailR is float r) { strips.Add((2, BarrierWidth, Offset(p, r, BarrierHeight))); Wall(Offset(p, r, 0)); barriers++; }
            }
            float shift = (s0.HalfR - s0.HalfL) / 2f;   // surface centre relative to the road axis
            var surface = Math.Abs(shift) > 0.05f ? Offset(axis, shift, 0) : axis;
            // Split very long chains into ~1.2 km pieces (UDP size, streaming granularity).
            for (int start = 0; start < surface.Count - 1; start += MaxChainPoints - 1)
            {
                var piece = surface.GetRange(start, Math.Min(MaxChainPoints, surface.Count - start));
                strips.Add((0, s0.HalfL + s0.HalfR, Extend(piece, EndOverlap)));
            }
            chains++;
        }
        for (int i = 0; i < segs.Count; i++) if (!hasPred[i] && !usedSeg[i]) EmitChain(i);
        for (int i = 0; i < segs.Count; i++) if (!usedSeg[i]) EmitChain(i);   // loops

        // ---- junctions ----
        var hulls = new List<(List<Vector3> H, float MinY, float MaxY)>();
        var lanes = new List<(List<Vector3> Pts, float Half)>();   // junction surfaces other than hulls
        foreach (var item in map.MapItems.Values)
        {
            if (item is not Prefab prefab || prefab.Nodes == null || prefab.Nodes.Count == 0) continue;
            if (virtualPrefabs.Contains(prefab)) continue;   // already a road piece (pass 1c)
            if (prefabColl.TryGetValue(prefab, out var gameColl))
            {
                // The game's own collision: exact surfaces and walls.
                foreach (var (a, b, cc, wall) in gameColl) tiles.Tri(a, b, cc, doubleSided: wall, prio: TileSet.PrioGame);
                // Some models' collision stops short of a road's end (seen: 5 m before a T arm).
                // For compact junctions also fill between the connecting roads' ends, as before:
                // where the game's surface is there the two coincide, elsewhere it closes the gap.
                var cnodes = prefab.Nodes.Where(nd => nd != null).ToList();
                var cctr = cnodes.Aggregate(Vector3.Zero, (acc, nd) => acc + nd.Position) / cnodes.Count;
                if (cnodes.Count > 1 && cnodes.Max(nd => Vector3.Distance(nd.Position, cctr)) <= CompactJunction)
                {
                    var ends = new List<Vector3>();
                    foreach (var nd in cnodes) if (nodeEdges.TryGetValue(nd.Uid, out var e)) ends.AddRange(e);
                    var fill = Hull(ends);
                    if (fill.Count >= 3)
                    {
                        var fc = fill.Aggregate(Vector3.Zero, (acc, v) => acc + v) / fill.Count;
                        for (int k = 0; k < fill.Count; k++) tiles.Tri(fc, fill[k], fill[(k + 1) % fill.Count], prio: TileSet.PrioFill);
                    }
                }
                // Its outline still takes part in gap filling / wall suppression, like a hull.
                var outline = Hull(gameColl.Where(t => !t.Wall).SelectMany(t => new[] { t.A, t.B, t.C }).ToList());
                if (outline.Count >= 3) hulls.Add((outline, outline.Min(v => v.Y), outline.Max(v => v.Y)));
                junctionsColl++;
                continue;
            }
            var nodes = prefab.Nodes.Where(nd => nd != null).ToList();
            if (nodes.Count == 0 || !nodes.Any(nd => Near(nd.Position))) continue;
            var c = nodes.Aggregate(Vector3.Zero, (acc, nd) => acc + nd.Position) / nodes.Count;
            float spread = nodes.Max(nd => Vector3.Distance(nd.Position, c));

            // Compact junctions (T, X, merges): each connecting road continues at its real width
            // to the junction centre - few wide surfaces, no gaps between lanes.
            // Large junctions use the prefab's real lane curves when its descriptor fits the map;
            // otherwise (and for compact ones) the area between the connecting roads is filled.
            PrefabDescriptor ppdLarge = null;
            Placement placeLarge = default;
            bool useLanes = spread > CompactJunction && nodes.Count > 1 &&
                            (ppdLarge = GetPpd(prefab.Model.ToString())) is { NavCurves.Count: > 0 } &&
                            FitPrefab(ppdLarge, prefab, out placeLarge);
            if (!useLanes)
            {
                // Mesh: fill between the exact edges of every road ending at this junction
                // (convex hull, fanned from the centre) - the roads and the fill share edge points.
                var edgePts = new List<Vector3>();
                // Prefab-to-prefab nodes have no road to take a width from: use this junction's
                // widest road (a fixed wide default made "arrow" overhangs past the roads).
                float fallbackW = nodes.Select(nd => nodeWidth.GetValueOrDefault(nd.Uid)).DefaultIfEmpty(0).Max();
                if (fallbackW < 1) fallbackW = 10f;
                foreach (var nd in nodes)
                {
                    if (nodeEdges.TryGetValue(nd.Uid, out var e)) { edgePts.AddRange(e); continue; }
                    var r3 = Vector3.Transform(Vector3.UnitX, nd.Rotation);
                    var rh = Vector2.Normalize(new Vector2(r3.X, r3.Z));
                    if (float.IsNaN(rh.X)) continue;
                    var off = new Vector3(rh.X, 0, rh.Y) * (fallbackW / 2);
                    edgePts.Add(nd.Position - off);
                    edgePts.Add(nd.Position + off);
                }
                bool compact = spread <= CompactJunction || nodes.Count == 1;
                var hull = Hull(edgePts);
                if (compact && hull.Count >= 3)
                {
                    var ctr = hull.Aggregate(Vector3.Zero, (acc, v) => acc + v) / hull.Count;
                    for (int k = 0; k < hull.Count; k++) tiles.Tri(ctr, hull[k], hull[(k + 1) % hull.Count]);
                    hulls.Add((hull, hull.Min(v => v.Y), hull.Max(v => v.Y)));
                }
                foreach (var nd in nodes)
                {
                    var p = nd.Position;
                    float d = Vector3.Distance(p, c);
                    if (d < 0.5f) continue;
                    float w = nodeWidth.TryGetValue(nd.Uid, out var nw) ? nw : fallbackW;
                    int n = Math.Max(2, (int)Math.Ceiling(d / StepMetres) + 1);
                    var pts = new List<Vector3>(n);
                    for (int k = 0; k < n; k++) pts.Add(Vector3.Lerp(p, c, k / (float)(n - 1)));
                    var ext = Extend(pts, EndOverlap + w / 2);
                    strips.Add((1, w, ext));
                    // Large junction without a usable descriptor: each road continues to the centre
                    // at its own width (a hull over a big interchange would be a wrong flat plate).
                    if (!compact)
                    {
                        var le = Offset(ext, -w / 2, 0);
                        var re = Offset(ext, w / 2, 0);
                        for (int k = 0; k + 1 < le.Count; k++) tiles.RoadQuad(le[k], re[k], le[k + 1], re[k + 1]);
                        lanes.Add((ext, w / 2));
                    }
                }
                junctionsApprox++;
                continue;
            }

            // Large junctions (roundabouts, interchanges): the real lane curves of the prefab model.
            {
                var ppd = ppdLarge;
                var place = placeLarge;
                foreach (var nc in ppd.NavCurves)
                {
                    var pts = NavCurvePoints(nc, place);
                    if (pts == null) continue;
                    strips.Add((1, LaneWidth, Extend(pts, EndOverlap)));
                    var le = Offset(pts, -LaneWidth / 2, 0);
                    var re = Offset(pts, LaneWidth / 2, 0);
                    for (int k = 0; k + 1 < le.Count; k++) tiles.RoadQuad(le[k], re[k], le[k + 1], re[k + 1]);
                    lanes.Add((pts, LaneWidth / 2));
                }
                junctionsExact++;
            }
        }

        // ---- gap filling and side walls ----
        // Every surface edge looks outward for the facing edge of a neighbouring surface; gaps up
        // to MaxGap (medians, splits, between junction lanes) are filled. Road edges that stay
        // open (nothing beside them, no ETS2 railing) get a wall so the car cannot leave the roads.
        var lines = new List<EdgeLine>();
        var axisPts = new List<AxisPt>();
        for (int i = 0; i < segs.Count; i++)
        {
            var sg = segs[i];
            int owner = segChain[i];
            var nl = sg.Rh.Select(r => -r).ToArray();
            lines.Add(new EdgeLine { P = sg.Left.ToArray(), N = nl, Owner = owner, Walls = segKerb.TryGetValue(i, out var kb1) && kb1.L });
            lines.Add(new EdgeLine { P = sg.Right.ToArray(), N = sg.Rh.ToArray(), Owner = owner, Walls = segKerb.TryGetValue(i, out var kb2) && kb2.R });
            for (int k = 0; k < sg.Pts.Count; k++) axisPts.Add(new AxisPt(sg.Pts[k], sg.Rh[k], sg.HalfL, sg.HalfR, owner));
        }
        int laneOwner = chains + 1;
        foreach (var (pts, half) in lanes)
        {
            var rh = Enumerable.Range(0, pts.Count).Select(k => RightAt(pts, k)).ToArray();
            lines.Add(new EdgeLine { P = Offset(pts, -half, 0).ToArray(), N = rh.Select(r => -r).ToArray(), Owner = laneOwner });
            lines.Add(new EdgeLine { P = Offset(pts, half, 0).ToArray(), N = rh, Owner = laneOwner });
            for (int k = 0; k < pts.Count; k++) axisPts.Add(new AxisPt(pts[k], rh[k], half, half, laneOwner));
            laneOwner++;
        }
        // Junction fills: their outline, resampled, so gaps between a junction and nearby roads fill too.
        foreach (var (h, _, _) in hulls)
        {
            var ctr = h.Aggregate(Vector3.Zero, (acc, v) => acc + v) / h.Count;
            var hp = new List<Vector3>();
            var hn = new List<Vector2>();
            for (int k = 0; k < h.Count; k++)
            {
                var a = h[k]; var b = h[(k + 1) % h.Count];
                var e = new Vector2(b.X - a.X, b.Z - a.Z);
                float len = e.Length();
                if (len < 0.1f) continue;
                var nrm = new Vector2(-e.Y, e.X) / len;
                var mid = (a + b) / 2;
                if (Vector2.Dot(nrm, new Vector2(mid.X - ctr.X, mid.Z - ctr.Z)) < 0) nrm = -nrm;
                int steps = Math.Max(1, (int)MathF.Ceiling(len / 3f));
                for (int st = 0; st < steps; st++) { hp.Add(Vector3.Lerp(a, b, st / (float)steps)); hn.Add(nrm); }
            }
            if (hp.Count > 1) lines.Add(new EdgeLine { P = hp.ToArray(), N = hn.ToArray(), Owner = laneOwner++ });
        }
        const float EdgeCell = 6f, AxisCell = 16f;
        var edgeHash = new Dictionary<long, List<(int L, int K)>>();
        for (int li = 0; li < lines.Count; li++)
            for (int k = 0; k < lines[li].P.Length; k++)
            {
                var p = lines[li].P[k];
                long key = Cell(p.X, p.Z, EdgeCell);
                if (!edgeHash.TryGetValue(key, out var l)) edgeHash[key] = l = new List<(int, int)>();
                l.Add((li, k));
            }
        var axisHash = new Dictionary<long, List<int>>();
        for (int a = 0; a < axisPts.Count; a++)
        {
            long key = Cell(axisPts[a].P.X, axisPts[a].P.Z, AxisCell);
            if (!axisHash.TryGetValue(key, out var l)) axisHash[key] = l = new List<int>();
            l.Add(a);
        }
        bool Covered(Vector3 p, int owner)
        {
            int cx0 = (int)MathF.Floor(p.X / AxisCell), cz0 = (int)MathF.Floor(p.Z / AxisCell);
            for (int dx = -1; dx <= 1; dx++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (!axisHash.TryGetValue(((long)(cx0 + dx) << 32) ^ (uint)(cz0 + dz), out var l)) continue;
                    foreach (var a in l)
                    {
                        var ax = axisPts[a];
                        if (ax.Owner == owner) continue;
                        var d = p - ax.P;
                        if (MathF.Abs(d.Y) > 1.5f) continue;
                        float lat = d.X * ax.R.X + d.Z * ax.R.Y;
                        float along = d.X * ax.R.Y - d.Z * ax.R.X;
                        if (MathF.Abs(along) <= 2.6f && lat >= -ax.L - 0.5f && lat <= ax.W + 0.5f) return true;
                    }
                }
            foreach (var (h, minY, maxY) in hulls)
                if (p.Y > minY - 1.5f && p.Y < maxY + 1.5f && InHull(h, p, 0.5f)) return true;
            return false;
        }
        int gapQuads = 0, wallQuads = 0;
        foreach (var line in lines)
        {
            line.Partner = new Vector3?[line.P.Length];
            for (int k = 0; k < line.P.Length; k++)
            {
                var p = line.P[k];
                var nrm = line.N[k];
                int cx0 = (int)MathF.Floor(p.X / EdgeCell), cz0 = (int)MathF.Floor(p.Z / EdgeCell);
                float bestD = MaxGap;
                Vector3? best = null;
                int rc = (int)MathF.Ceiling(MaxGap / EdgeCell);
                for (int dx = -rc; dx <= rc; dx++)
                    for (int dz = -rc; dz <= rc; dz++)
                    {
                        if (!edgeHash.TryGetValue(((long)(cx0 + dx) << 32) ^ (uint)(cz0 + dz), out var l)) continue;
                        foreach (var (lj, kk) in l)
                        {
                            var other = lines[lj];
                            if (other.Owner == line.Owner) continue;
                            var c = other.P[kk];
                            var d = new Vector2(c.X - p.X, c.Z - p.Z);
                            float dist = d.Length();
                            if (dist >= bestD) continue;
                            if (MathF.Abs(c.Y - p.Y) > 0.5f + 0.1f * dist) continue;
                            if (dist > 0.05f)
                            {
                                var u = d / dist;
                                if (Vector2.Dot(u, nrm) < 0.7f) continue;             // across, not along
                                if (Vector2.Dot(other.N[kk], -u) < 0.3f) continue;    // its near edge, facing us
                            }
                            bestD = dist;
                            best = c;
                        }
                    }
                line.Partner[k] = best;
            }
            for (int k = 0; k + 1 < line.P.Length; k++)
            {
                var q0 = line.Partner[k];
                var q1 = line.Partner[k + 1];
                if (q0 is Vector3 a && q1 is Vector3 b)
                {
                    if (Vector3.Distance(a, b) > MaxGap) continue;   // jumped to a far surface: no flap
                    var pd = line.P[k + 1] - line.P[k];
                    var qd = b - a;
                    if (pd.X * qd.X + pd.Z * qd.Z > 0) tiles.Quad(line.P[k], a, line.P[k + 1], b, prio: TileSet.PrioFill);
                    else tiles.Tri(line.P[k], a, line.P[k + 1], prio: TileSet.PrioFill);   // partners run backwards: a quad would twist
                }
                else if (q0 is Vector3 a1) tiles.Tri(line.P[k], a1, line.P[k + 1], prio: TileSet.PrioFill);   // where a gap opens/closes
                else if (q1 is Vector3 b1) tiles.Tri(line.P[k], b1, line.P[k + 1], prio: TileSet.PrioFill);
                else continue;
                gapQuads++;
            }
            if (!line.Walls) continue;
            var open = new bool[line.P.Length];
            for (int k = 0; k < line.P.Length; k++) open[k] = line.Partner[k] == null && !Covered(line.P[k], line.Owner);
            var dn = new Vector3(0, -0.3f, 0);
            var up = new Vector3(0, SideWallHeight, 0);
            for (int k = 0; k + 1 < line.P.Length; k++)
            {
                if (!open[k] || !open[k + 1]) continue;
                tiles.Quad(line.P[k] + dn, line.P[k] + up, line.P[k + 1] + dn, line.P[k + 1] + up, doubleSided: true);
                wallQuads++;
            }
        }
        Console.WriteLine($"gaps filled: {gapQuads} quads, side walls: {wallQuads} quads");

        // Mesh tiles straight into the BeamNG mod (seen by BeamNG as /art/eurobeamng/tiles/...).
        tiles.HeightfieldDir = hfDir;
        tiles.Area = (cx, cz, radius);
        tiles.Write(tilesDir, tilesVfs);
        if (hfDir != null)
            Console.WriteLine($"heightfields: {tiles.HfTiles} tiles ({tiles.HfWritten} new) -> {hfDir}; meshes keep {tiles.TrianglesKept} triangles, {tiles.TrianglesDropped} shown by the grids");
        // ES_QUALITY: drive every lane once over what was built: holes and bumps per km (the measure for
        // road quality). Roads: a line a quarter in from each edge; junctions: their AI lanes.
        if (Environment.GetEnvironmentVariable("ES_QUALITY") != null)
        {
            var probe = new List<List<Vector3>>();
            foreach (var sg in segs)
                if (sg.Left.Count == sg.Pts.Count && sg.Right.Count == sg.Pts.Count)
                    foreach (var f in new[] { 0.25f, 0.75f })
                        probe.Add(sg.Left.Select((l, k) => Vector3.Lerp(l, sg.Right[k], f)).ToList());
            int junctionLanes = 0;
            foreach (var pf in prefabColl.Keys)
            {
                // Road junctions only: company yards, garages and services have real kerbs and steps.
                if (!PrefabModelPaths.TryGetValue(pf.Model.ToString(), out var mp) ||
                    !new[] { "cross", "fork", "corner", "/hw", "road", "turn", "round", "ramp", "exit", "merge" }.Any(mp.Contains) ||
                    new[] { "_co", "company", "garage", "service", "gas_", "parking", "depot", "dealer" }.Any(mp.Contains)) continue;
                var ppd = GetPpd(pf.Model.ToString());
                if (ppd?.NavCurves == null || !FitPrefab(ppd, pf, out var place)) continue;
                foreach (var nc in ppd.NavCurves)
                    if (NavCurvePoints(nc, place) is { } np) { probe.Add(np); junctionLanes++; }
            }
            Console.WriteLine(tiles.ProbeLines(probe) + $" ({segs.Count} road pieces, {junctionLanes} junction lanes)");
            Console.WriteLine(tiles.ProbeBand());
        }
        long tris = tiles.Tiles.Values.Sum(t => (long)t.Tris.Count / 3);
        Console.WriteLine($"mesh: {tiles.Tiles.Count} tiles of {TileSize} m, {tris} triangles -> {tilesDir} ({tiles.TilesWritten} new, {tiles.TilesKept} unchanged, {tiles.TilesRemoved} old removed)");
        Console.WriteLine($"walls: {tiles.WallsKept} kept, {tiles.WallsDropped} dropped (road surface on both sides)");
        for (int kk = 0; kk < 2; kk++) Console.WriteLine($"   offsets {(kk == 0 ? "roads" : "terrain items")}: {TerrainBuilder.StatN[kk]}, mean horizontal {TerrainBuilder.StatXZ[kk] / Math.Max(1, TerrainBuilder.StatN[kk]):F1} m, mean vertical {TerrainBuilder.StatY[kk] / Math.Max(1, TerrainBuilder.StatN[kk]):F2} m, vertical > 5 m: {TerrainBuilder.StatBig[kk]}");
        Console.WriteLine($"terrain: {TerrainBuilder.Strips} strips, {TerrainBuilder.Offsets} hand-edited vertices (mean shift {TerrainBuilder.OffsetDeltaSum / Math.Max(1, TerrainBuilder.Offsets):F1} m, {TerrainBuilder.OffsetsIgnored} ignored), {tiles.TerrainVertsLowered} vertices put under roads, {tiles.TerrainSplits} triangles split over roads, {tiles.TerrainHidden} under roads dropped, {tiles.TerrainClipped} bits above roads left out, {tiles.FillsClipped} fill bits above roads left out, {tiles.TerrainFar} far from roads dropped; safety net: {tiles.NetCells} cells of 6 m, {tiles.CracksFilled} crack cells of 1 m filled, {tiles.GapCells} gap cells of 1 m filled");
Console.WriteLine($"overlaps levelled: {tiles.VertsSnapped} vertices put on the game's surface, {tiles.FillsDropped} redundant fill triangles dropped; steps (2-60 cm) {tiles.StepsBefore} -> {tiles.StepsAfter}");

        using (var w = new BinaryWriter(File.Create(outPath)))
        {
            w.Write(Encoding.ASCII.GetBytes("ESRD"));
            w.Write(2u);
            w.Write((uint)strips.Count);
            foreach (var (kind, width, pts) in strips)
            {
                w.Write(kind);
                w.Write(width);
                w.Write((uint)pts.Count);
                foreach (var p in pts) { w.Write(p.X); w.Write(p.Y); w.Write(p.Z); }
            }
        }
        long points = strips.Sum(st => (long)st.pts.Count);
        Console.WriteLine($"wrote {outPath}: {strips.Count} strips ({strips.Count(st => st.kind == 0)} roads, {barriers} barriers), {points} points");
        Console.WriteLine($"road segments: {segs.Count} -> {chains} continuous chains");
        Console.WriteLine($"junctions: {junctionsColl} from game collision, {junctionsApprox} compact (filled from roads), {junctionsExact} large (lane curves)");
        Console.WriteLine($"roads with unknown look width (default used): {unknownLooks}, railway segments skipped: {railways}");
        Console.WriteLine("most common road types (half widths left/right):");
        foreach (var kv in roadTypes.OrderByDescending(kv => kv.Value).Take(15))
        {
            var w = lookWidths.GetValueOrDefault(kv.Key);
            Console.WriteLine($"  {kv.Key,-16} {kv.Value,6}   {(w is { } v ? $"{v.L:F2} / {v.R:F2} m" : "?")}");
        }

        foreach (var rd in readers) rd.Dispose();
        return 0;
    }
}
