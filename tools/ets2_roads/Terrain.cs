// The ground beside the roads, as ETS2 builds it: every road side has a terrain strip (Size m
// wide) whose cross-section follows a terrain profile (/def/world/terrain_profile*.sii: heights at
// distances given by "step" metres, times the side's coefficient), on a grid of QuadData.Cols
// quads along the road. Vertices the map's authors moved by hand are stored as offsets
// (position relative to the road's start node). Terrain noise is not reproduced.
//
// The strips go into the tiles as low-priority ground: they never rise above a road (see
// TileSet.AddTerrain), so they only catch the car once it leaves the road.
using System.Numerics;
using TruckLib;
using TruckLib.ScsMap;
using TruckLib.Sii;

static class TerrainBuilder
{
    const float MaxAcross = 16f;     // m: wide profile rows are split into pieces this wide
    const float MaxAlong = 12f;      // m: and the quads along the road (splits near roads refine it)
    const float MinAcross = 8f;      // m: ETS2 rows closer than this to the last one kept are skipped
    const float MinAlong = 16f;      // m: and its columns
    const float MaxWidth = 150f;     // m: strips end at the first profile row this far out (TileSet keeps only what is near roads)
    const float StartInside = 0f;    // m: the strip starts at the road's edge, at its height (cracks: FillCracks / FillGaps)
    const float MinSize = 1f;        // narrower strips (0.5 m "no terrain") are skipped

    public record Profile(float[] Height, float[] Step);
    static readonly Profile Flat = new(new float[] { 0 }, new float[] { 1000 });

    public static Dictionary<string, Profile> LoadProfiles(IFileSystem fs)
    {
        var map = new Dictionary<string, Profile>();
        IList<string> files;
        try { files = fs.GetFiles("/def/world"); } catch { return map; }
        foreach (var f in files.Where(p => Path.GetFileName(p).StartsWith("terrain_profile") && p.EndsWith(".sii")))
        {
            try
            {
                foreach (var u in SiiFile.Open(f, fs, true).Units)
                {
                    if (u.Class != "terrain_profile") continue;
                    var h = Floats(u, "height"); var s = Floats(u, "step");
                    int n = Math.Min(h.Length, s.Length);
                    if (n == 0) continue;
                    map[u.Name.StartsWith("terrain.") ? u.Name[8..] : u.Name] = new Profile(h[..n], s[..n]);
                }
            }
            catch (Exception e) { Console.Error.WriteLine($"skip {f}: {e.Message}"); }
        }
        return map;
    }

    static float[] Floats(Unit u, string key)
    {
        if (!u.Attributes.TryGetValue(key, out var v) || v is not System.Collections.IEnumerable en || v is string) return Array.Empty<float>();
        var list = new List<float>();
        foreach (var x in en)
            try { list.Add(Convert.ToSingle(x, System.Globalization.CultureInfo.InvariantCulture)); } catch { list.Add(0); }
        return list.ToArray();
    }

    // Distances from the road edge of the profile's vertex rows (0 .. size), and their heights.
    static (List<float> D, List<float> H) Rows(Profile p, float size, float coef)
    {
        var d = new List<float> { 0 };
        var h = new List<float> { p.Height[0] * coef };
        float s = 0;
        for (int r = 0; s < size - 1e-3f; r++)
        {
            float step = r < p.Step.Length ? p.Step[r] : p.Step[^1];
            if (step <= 0) step = 1;
            float h0 = (r < p.Height.Length ? p.Height[r] : p.Height[^1]) * coef;
            float h1 = (r + 1 < p.Height.Length ? p.Height[r + 1] : p.Height[^1]) * coef;
            if (s + step >= size) { h.Add(h0 + (h1 - h0) * (size - s) / step); d.Add(size); break; }
            s += step;
            d.Add(s);
            h.Add(h1);
        }
        return (d, h);
    }

    public static int Strips, Offsets, OffsetsIgnored;
    public static double OffsetDeltaSum;
    public static readonly double[] StatXZ = new double[2], StatY = new double[2]; public static readonly int[] StatN = new int[2];
    public static readonly int[] StatBig = new int[2];

    // One side of a road. half: the paved half width on that side; bank: edge height per metre
    // of half width (as the road surface uses).
    public static void Side(TileSet tiles, INode node, INode fwdNode, RoadTerrain t, bool right, float half,
                            float bankA, float bankB, float len, Dictionary<string, Profile> profiles)
    {
        if (t == null || t.Size < MinSize || len < 0.5f) return;
        var prof = profiles.TryGetValue(t.Profile.ToString(), out var pp) ? pp : Flat;
        // Cut at a profile row boundary, so the hand edits of the rows kept still fit.
        float size = t.Size, s = 0;
        for (int r = 0; s < MaxWidth && s < t.Size; r++) { float st = r < prof.Step.Length ? prof.Step[r] : prof.Step[^1]; s += st <= 0 ? 1 : st; }
        size = MathF.Min(t.Size, s);
        var (rowD, rowH) = Rows(prof, size, t.Coefficient);
        int cols = Math.Max(1, (int)(t.QuadData?.Cols ?? 0));
        if (t.QuadData == null || t.QuadData.Cols == 0) cols = Math.Max(1, (int)MathF.Ceiling(len / 20f));
        int rows = rowD.Count - 1;
        float sideSign = right ? 1 : -1;

        // The grid ETS2 computes (before hand edits): column c at c/cols along the road.
        Vector3 Axis(float f) => HermiteSpline.InterpolatePolyline(node, fwdNode, Math.Clamp(f, 0, 1), len);
        Vector3 GridPos(float f, float dist, float hgt)
        {
            var p = Axis(f);
            var q0 = Axis(f - 0.01f); var q1 = Axis(f + 0.01f);
            var fw = new Vector2(q1.X - q0.X, q1.Z - q0.Z);
            if (fw.LengthSquared() < 1e-8f) fw = new Vector2(fwdNode.Position.X - node.Position.X, fwdNode.Position.Z - node.Position.Z);
            fw = Vector2.Normalize(fw);
            var rh = new Vector2(-fw.Y, fw.X);   // right of travel (x east, z south)
            float bank = bankA + (bankB - bankA) * f;
            float edgeY = p.Y + bank * half;      // bank: right edge up for positive values
            if (!right) edgeY = p.Y - bank * half;
            float r = half + dist;
            return new Vector3(p.X + rh.X * r * sideSign, edgeY + hgt, p.Z + rh.Y * r * sideSign);
        }
        float RowDist(int r) => r == 0 ? -StartInside : rowD[r];

        // Hand edits: the moved vertices' displacement from the computed grid.
        var delta = new Vector3[cols + 1, rows + 1];
        if (t.QuadData?.Offsets != null)
            foreach (var o in t.QuadData.Offsets)
            {
                if (o.X > cols || o.Y > rows) continue;
                var target = node.Position + o.Data;
                var d = target - GridPos(o.X / (float)cols, RowDist(o.Y), rowH[o.Y]);
                if (d.Length() > 400f) { OffsetsIgnored++; continue; }   // not what we think it is: ignore
                OffsetDeltaSum += d.Length();
                { int kk = half == 0 ? 1 : 0; StatN[kk]++; StatXZ[kk] += MathF.Sqrt(d.X * d.X + d.Z * d.Z); StatY[kk] += MathF.Abs(d.Y); if (MathF.Abs(d.Y) > 5) StatBig[kk]++; }
                delta[o.X, o.Y] = d;
                Offsets++;
            }

        // Our grid: ETS2's rows and columns thinned out to at least MinAcross x MinAlong (its 1 m
        // rows by the road and 2 m columns cost BeamNG collision for nothing), wide gaps split to
        // at most MaxAcross x MaxAlong. Hand edits on the rows/columns kept are spread bilinearly.
        var keptCols = new List<int> { 0 };
        int kc = Math.Max(1, (int)MathF.Floor(MinAlong / (len / cols)));
        for (int c = kc; c < cols; c += kc) keptCols.Add(c);
        if (keptCols[^1] != cols) keptCols.Add(cols);
        var keptRows = new List<int> { 0 };
        for (int r = 1; r < rows; r++) if (RowDist(r) - RowDist(keptRows[^1]) >= MinAcross) keptRows.Add(r);
        keptRows.Add(rows);
        static List<(int A, int B, float W)> Pieces(List<int> kept, Func<int, float> pos, float max)
        {
            var list = new List<(int, int, float)>();
            for (int i = 0; i + 1 < kept.Count; i++)
            {
                int k = Math.Max(1, (int)MathF.Ceiling((pos(kept[i + 1]) - pos(kept[i])) / max));
                for (int j = 0; j < k; j++) list.Add((kept[i], kept[i + 1], j / (float)k));
            }
            list.Add((kept[^2], kept[^1], 1f));
            return list;
        }
        var along = Pieces(keptCols, c => c * len / cols, MaxAlong);
        var across = Pieces(keptRows, RowDist, MaxAcross);
        int nu = along.Count, nv = across.Count;
        var g = new Vector3[nu, nv];
        for (int u = 0; u < nu; u++)
        {
            var (ca, cb, wu) = along[u];
            float f = (ca + (cb - ca) * wu) / cols;
            for (int v = 0; v < nv; v++)
            {
                var (ra, rb, wv) = across[v];
                float dist = RowDist(ra) + (RowDist(rb) - RowDist(ra)) * wv;
                float hgt = rowH[ra] + (rowH[rb] - rowH[ra]) * wv;
                if (ra == 0 && wv == 0) hgt = 0;
                var dlt = Vector3.Lerp(Vector3.Lerp(delta[ca, ra], delta[cb, ra], wu),
                                       Vector3.Lerp(delta[ca, rb], delta[cb, rb], wu), wv);
                g[u, v] = GridPos(f, dist, hgt) + dlt;
            }
        }
        for (int u = 0; u + 1 < nu; u++)
            for (int v = 0; v + 1 < nv; v++)
            {
                tiles.TerrainTri(g[u, v], g[u + 1, v], g[u + 1, v + 1]);
                tiles.TerrainTri(g[u, v], g[u + 1, v + 1], g[u, v + 1]);
            }
        Strips++;
    }
}
