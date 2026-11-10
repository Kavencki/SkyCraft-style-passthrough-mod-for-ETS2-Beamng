// Continuous road surface as triangle meshes, cut into square tiles and written as
// Collada (.dae) files that BeamNG loads as TSStatic objects with collision.
//
// Tile mesh coordinates are local to the tile origin (ETS2 x0, 0, z0) in a BeamNG-style
// frame: X = ETS2 east, Y = ETS2 north (-Z), Z = up (absolute ETS2 height). The bridge places
// each tile with the same anchor transform it uses for the car.
using System.Globalization;
using System.Numerics;
using System.Text;

sealed class MeshBuilder
{
    public readonly List<Vector3> Verts = new();
    public readonly List<int> Tris = new();
    readonly Dictionary<(int, int, int), int> index = new();

    // Vertices are welded on a 1 cm grid so neighbouring pieces share them (no cracks).
    public int Vertex(Vector3 local)
    {
        var key = ((int)MathF.Round(local.X * 100), (int)MathF.Round(local.Y * 100), (int)MathF.Round(local.Z * 100));
        if (index.TryGetValue(key, out int i)) return i;
        Verts.Add(local);
        index[key] = Verts.Count - 1;
        return Verts.Count - 1;
    }

    // Road surfaces are one-sided with the front face up (counter-clockwise seen from above):
    // a back-to-back duplicate would make BeamNG's physics pull wheels down into the road.
    // Walls (vertical) are the exception: both sides, so the car can hit them from either side.
    public void Tri(Vector3 a, Vector3 b, Vector3 c, bool doubleSided)
    {
        int ia = Vertex(a), ib = Vertex(b), ic = Vertex(c);
        if (ia == ib || ib == ic || ia == ic) return;
        if (doubleSided)
        {
            Tris.Add(ia); Tris.Add(ib); Tris.Add(ic);
            Tris.Add(ia); Tris.Add(ic); Tris.Add(ib);
            return;
        }
        var n = Vector3.Cross(b - a, c - a);
        if (n.Z >= 0) { Tris.Add(ia); Tris.Add(ib); Tris.Add(ic); }
        else { Tris.Add(ia); Tris.Add(ic); Tris.Add(ib); }
    }
}

sealed partial class TileSet
{
    public readonly float Size;
    public readonly Dictionary<(int, int), MeshBuilder> Tiles = new();
    public TileSet(float size) { Size = size; }

    public (int, int) TileOf(float x, float z) => ((int)MathF.Floor(x / Size), (int)MathF.Floor(z / Size));
    public Vector2 Origin((int x, int z) t) => new((t.x + 0.5f) * Size, (t.z + 0.5f) * Size);

    // Walls are held back until Write: a wall with drivable surface on BOTH sides stands in the
    // middle of a road (an approximated barrier, an edge wall where another road continues, a
    // prefab wall across a lane) and is dropped. Real walls have surface on one side only.
    readonly List<(Vector3 A, Vector3 B, Vector3 C)> walls = new();
    readonly List<(Vector3 A, Vector3 B, Vector3 C)> surfaces = new();
    readonly List<(Vector3 A, Vector3 B, Vector3 C, int Prio)> pending = new();
    public int WallsDropped { get; private set; }
    public int WallsKept { get; private set; }
    public int VertsSnapped { get; private set; }
    public int FillsDropped { get; private set; }

    // Surface priorities: the game's own collision wins over road surfaces, which win over
    // helper fills (gap fills, junction fans, road ends overlapping a junction).
    public const int PrioFill = 0, PrioRoad = 1, PrioGame = 2;

    // World (ETS2) triangle; surfaces are held until Write so overlaps can be made level.
    public void Tri(Vector3 a, Vector3 b, Vector3 c, bool doubleSided = false, int prio = PrioRoad)
    {
        if (!Finite(a) || !Finite(b) || !Finite(c)) return;
        if (doubleSided) { walls.Add((a, b, c)); return; }
        pending.Add((a, b, c, prio));
    }

    // Where surfaces overlap at nearly the same height, their corners are put on one common
    // height: a few cm between two overlapping surfaces is an edge a wheel hits (the "random
    // bumps"). The game's own collision wins; otherwise the highest overlapping surface of equal
    // or higher priority. Helper triangles lying entirely on the game's surface are dropped.
    void LevelOverlaps()
    {
        const float GridCell = 4f, MaxDy = 0.6f;
        float RoadToGameDy = float.TryParse(Environment.GetEnvironmentVariable("ES_GAMEDY"), System.Globalization.NumberStyles.Float, CultureInfo.InvariantCulture, out var gdy) ? gdy : 0.12f;
        float RoadToRoadDy = float.TryParse(Environment.GetEnvironmentVariable("ES_ROADDY"), System.Globalization.NumberStyles.Float, CultureInfo.InvariantCulture, out var rdy) ? rdy : 0.25f;
        var grid = new Dictionary<long, List<int>>();
        for (int i = 0; i < pending.Count; i++)
        {
            var (a, b, c, _) = pending[i];
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / GridCell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / GridCell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / GridCell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / GridCell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 40000) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long k = CellKey(x, z);
                    if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>();
                    l.Add(i);
                }
        }
        var original = pending.ToArray();   // heights are read from the unmodified surfaces
        // Accepted so far: the game's and the roads' surfaces always; fills one by one, a fill whose
        // middle already lies on an accepted surface is a duplicate (gaps were filled from both
        // sides, fills over roads) and is dropped.
        var accepted = new bool[original.Length];
        for (int i = 0; i < original.Length; i++) accepted[i] = original[i].Prio != PrioFill;
        // The height another surface (not `self`, priority >= minPrio) has at p, within MaxDy:
        // the game's surface if there is one, else the highest. gameOnly: only the game's.
        bool HeightAt(Vector3 p, int self, int minPrio, out float h, out bool isGame)
        {
            h = float.MinValue;
            isGame = false;
            if (!grid.TryGetValue(CellKey((int)MathF.Floor(p.X / GridCell), (int)MathF.Floor(p.Z / GridCell)), out var l)) return false;
            bool found = false;
            float gameDy = MaxDy;
            foreach (var i in l)
            {
                if (i == self || !accepted[i]) continue;
                var (a, b, c, prio) = original[i];
                if (prio < minPrio) continue;
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                float l3 = 1 - l1 - l2;
                const float e = -0.01f;
                if (l1 < e || l2 < e || l3 < e) continue;
                float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                float dy = MathF.Abs(y - p.Y);
                if (dy >= MaxDy) continue;
                // A road corner only joins a surface at nearly its own height: a kerb or sidewalk
                // top (15+ cm) next to it is not the same surface (it made humps on the road).
                if (minPrio == PrioRoad && dy >= (prio == PrioGame ? RoadToGameDy : RoadToRoadDy)) continue;
                if (prio == PrioGame)
                {
                    if (!isGame || dy < gameDy) { h = y; gameDy = dy; }
                    isGame = true;
                    found = true;
                }
                else if (!isGame && y > h) { h = y; found = true; }
            }
            return found;
        }
        for (int i = 0; i < pending.Count; i++)
        {
            var (a, b, c, prio) = pending[i];
            if (prio == PrioGame)
            {
                // A piece of the game's collision lying on another one, a little higher (a raised strip
                // or island inside a junction, crossed by its lanes): its corners go down onto the
                // surface under them, so there is no 5-30 cm edge to hit. Kerbs beside the road are
                // not affected (they do not lie on the road surface).
                if (Environment.GetEnvironmentVariable("ES_NOGAMELEVEL") != null) continue;
                bool changed = false;
                Vector3 Down(Vector3 v)
                {
                    float best = float.MinValue;
                    if (grid.TryGetValue(CellKey((int)MathF.Floor(v.X / GridCell), (int)MathF.Floor(v.Z / GridCell)), out var gl))
                        foreach (var j in gl)
                        {
                            if (j == i || original[j].Prio != PrioGame) continue;
                            var (ga, gb, gc, _) = original[j];
                            float det = (gb.Z - gc.Z) * (ga.X - gc.X) + (gc.X - gb.X) * (ga.Z - gc.Z);
                            if (MathF.Abs(det) < 1e-9f) continue;
                            float l1 = ((gb.Z - gc.Z) * (v.X - gc.X) + (gc.X - gb.X) * (v.Z - gc.Z)) / det;
                            float l2 = ((gc.Z - ga.Z) * (v.X - gc.X) + (ga.X - gc.X) * (v.Z - gc.Z)) / det;
                            float l3 = 1 - l1 - l2;
                            if (l1 < -0.01f || l2 < -0.01f || l3 < -0.01f) continue;
                            float y = l1 * ga.Y + l2 * gb.Y + l3 * gc.Y;
                            if (y < v.Y - 0.02f && y > v.Y - 0.3f && y > best) best = y;
                        }
                    if (best > float.MinValue) { changed = true; VertsSnapped++; return new Vector3(v.X, best, v.Z); }
                    return v;
                }
                var (na, nb, nc) = (Down(a), Down(b), Down(c));
                if (changed) pending[i] = (na, nb, nc, prio);
                continue;
            }
            bool ia = HeightAt(a, i, prio, out float ha, out bool ga);
            bool ib = HeightAt(b, i, prio, out float hb, out bool gb);
            bool ic = HeightAt(c, i, prio, out float hc, out bool gc);
            if (prio == PrioFill)
            {
                if (!seams.Contains(i) && HeightAt((a + b + c) / 3f, i, PrioFill, out _, out _))
                {
                    pending[i] = (a, b, c, -1);   // redundant: another surface is already there
                    FillsDropped++;
                    continue;
                }
                accepted[i] = true;
            }
            // Only ever raise towards a non-game surface (both sides meet at the higher one);
            // the game's surface is exact either way.
            if (ia && (ga || ha > a.Y)) { a.Y = ha; VertsSnapped++; }
            if (ib && (gb || hb > b.Y)) { b.Y = hb; VertsSnapped++; }
            if (ic && (gc || hc > c.Y)) { c.Y = hc; VertsSnapped++; }
            pending[i] = (a, b, c, prio);
        }
    }

    // Helper fills (gap fills, junction overlaps) must never be above a road or junction surface:
    // where one rises above it (by more than 2 cm), that part is split down to ~0.5 m and left
    // out - the real surface is there. Fills more than 3 m above are a different level (kept).
    void ClipFills()
    {
        const float Cell = 4f;
        var solid = pending.Where(p => p.Prio >= PrioRoad).Select(p => (p.A, p.B, p.C)).ToArray();
        var grid = new Dictionary<long, List<int>>();
        for (int i = 0; i < solid.Length; i++)
        {
            var (a, b, c) = solid[i];
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / Cell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / Cell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / Cell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / Cell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 40000) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long k = CellKey(x, z);
                    if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>();
                    l.Add(i);
                }
        }
        bool Above(Vector3 p)
        {
            if (!grid.TryGetValue(CellKey((int)MathF.Floor(p.X / Cell), (int)MathF.Floor(p.Z / Cell)), out var l)) return false;
            foreach (var i in l)
            {
                var (a, b, c) = solid[i];
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                float l3 = 1 - l1 - l2;
                const float e = 0.001f;
                if (l1 < e || l2 < e || l3 < e) continue;
                float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                if (p.Y > y + 0.02f && p.Y < y + 3f) return true;
            }
            return false;
        }
        bool Rises(Vector3 a, Vector3 b, Vector3 c)
        {
            float edge = MathF.Max(Vector3.Distance(a, b), MathF.Max(Vector3.Distance(b, c), Vector3.Distance(c, a)));
            int n = Math.Max(1, (int)MathF.Ceiling(edge));
            for (int i = 0; i <= n; i++)
                for (int j = 0; i + j <= n; j++)
                    if (Above(a + (b - a) * (i / (float)n) + (c - a) * (j / (float)n))) return true;
            return false;
        }
        var result = new List<(Vector3 A, Vector3 B, Vector3 C, int Prio)>(pending.Count);
        void Clip(Vector3 a, Vector3 b, Vector3 c, int depth)
        {
            if (!Rises(a, b, c)) { result.Add((a, b, c, PrioFill)); return; }
            float edge = MathF.Max(Vector3.Distance(a, b), MathF.Max(Vector3.Distance(b, c), Vector3.Distance(c, a)));
            if (edge <= 0.6f || depth >= 8) { FillsClipped++; return; }
            var ab = (a + b) / 2f; var bc = (b + c) / 2f; var ca = (c + a) / 2f;
            Clip(a, ab, ca, depth + 1); Clip(ab, b, bc, depth + 1); Clip(ca, bc, c, depth + 1); Clip(ab, bc, ca, depth + 1);
        }
        foreach (var p in pending)
        {
            if (p.Prio == PrioFill) Clip(p.A, p.B, p.C, 0);
            else result.Add(p);
        }
        pending.Clear();
        pending.AddRange(result);
    }

    // Steps: places where two surface corners meet on the ground plane (within 5 cm) at heights
    // 2-60 cm apart - an edge a wheel hits.
    // Every drivable triangle as built, with where it came from (for ProbeLines).
    public const byte KindFill = 0, KindRoad = 1, KindGame = 2, KindTerrain = 3, KindNet = 4, KindCrack = 5, KindGap = 6;
    readonly List<(Vector3 A, Vector3 B, Vector3 C, byte Kind)> built = new();
    readonly List<(Vector3 A, Vector3 B, Vector3 C)> builtWalls = new();
    void AddToMesh(Vector3 a, Vector3 b, Vector3 c, bool doubleSided, byte kind = KindRoad)
    {
        if (!doubleSided) built.Add((a, b, c, kind));
        else builtWalls.Add((a, b, c));
        var ctr = (a + b + c) / 3f;
        var t = TileOf(ctr.X, ctr.Z);
        if (!Tiles.TryGetValue(t, out var mb)) Tiles[t] = mb = new MeshBuilder();
        var o = Origin(t);
        Vector3 L(Vector3 p) => new(p.X - o.X, -(p.Z - o.Y), p.Y);
        mb.Tri(L(a), L(b), L(c), doubleSided);
    }

    // Ground beside the roads (TerrainBuilder): held apart from the drivable surfaces, it is put
    // into the mesh last and kept under them.
    readonly List<(Vector3 A, Vector3 B, Vector3 C)> terrain = new();
    public int TerrainTris => terrain.Count;
    public int TerrainVertsLowered { get; private set; }
    public int TerrainSplits { get; private set; }
    public int TerrainHidden { get; private set; }
    public int TerrainFar { get; private set; }
    public int TerrainClipped { get; private set; }
    public int FillsClipped { get; private set; }
    public int NetCells { get; private set; }
    readonly HashSet<long> bandCells = new();   // 8 m cells within NearRoad of a road (AddTerrain)
    public int CracksFilled { get; private set; }
    public int GapCells { get; private set; }
    public const float NearRoad = 50f;   // m: ground is built this far from any road or junction surface
    public void TerrainTri(Vector3 a, Vector3 b, Vector3 c)
    {
        if (!Finite(a) || !Finite(b) || !Finite(c)) return;
        terrain.Add((a, b, c));
    }

    // Wherever the ground is over a road or junction surface (another road's terrain reaching
    // across it, the strip's start under its own road edge), or less than Below under it, it is
    // put Below under that surface: ground never pokes through a road, so it adds no bumps.
    // Ground far above a surface (a hill over a tunnel) stays.
    void AddTerrain()
    {
        const float GridCell = 4f, Below = 0.3f, Above = 3f;
        var grid = new Dictionary<long, List<int>>();
        var surf = pending.Where(p => p.Prio >= 0).Select(p => (p.A, p.B, p.C)).ToArray();
        for (int i = 0; i < surf.Length; i++)
        {
            var (a, b, c) = surf[i];
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / GridCell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / GridCell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / GridCell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / GridCell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 40000) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long k = CellKey(x, z);
                    if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>();
                    l.Add(i);
                }
        }
        var cache = new Dictionary<(int, int, int), Vector3>();
        Vector3 Fix(Vector3 p)
        {
            var key = ((int)MathF.Round(p.X * 100), (int)MathF.Round(p.Z * 100), (int)MathF.Round(p.Y * 100));
            if (cache.TryGetValue(key, out var done)) return done;
            var res = p;
            if (grid.TryGetValue(CellKey((int)MathF.Floor(p.X / GridCell), (int)MathF.Floor(p.Z / GridCell)), out var l))
            {
                float lowest = float.MaxValue;
                foreach (var i in l)
                {
                    var (a, b, c) = surf[i];
                    float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                    if (MathF.Abs(det) < 1e-9f) continue;
                    float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                    float l3 = 1 - l1 - l2;
                    const float e = 0.001f;   // strictly inside: the ground may meet a road's edge
                    if (l1 < e || l2 < e || l3 < e) continue;
                    float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                    if (p.Y > y - Below && p.Y < y + Above) lowest = MathF.Min(lowest, y - Below);
                }
                if (lowest < float.MaxValue && lowest < res.Y) { res.Y = lowest; TerrainVertsLowered++; }
            }
            cache[key] = res;
            return res;
        }
        // Ground must never be above a road, junction or fill surface anywhere (a large ground
        // triangle can span a road with all corners beside it): every ground triangle is checked
        // on a 1 m grid; one that rises above a surface somewhere is split in four, down to
        // ~0.5 m, and pieces still above it are left out (the surface covers them).
        bool netting = false;
        var emitted = new List<(Vector3 A, Vector3 B, Vector3 C)>();
        bool AboveSurface(Vector3 p)
        {
            if (!grid.TryGetValue(CellKey((int)MathF.Floor(p.X / GridCell), (int)MathF.Floor(p.Z / GridCell)), out var l)) return false;
            foreach (var i in l)
            {
                var (a, b, c) = surf[i];
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                float l3 = 1 - l1 - l2;
                const float e = 0.001f;
                if (l1 < e || l2 < e || l3 < e) continue;
                float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                if (p.Y > y + 0.01f && p.Y < y + Above) return true;
            }
            return false;
        }
        bool RisesAbove(Vector3 a, Vector3 b, Vector3 c)
        {
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / GridCell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / GridCell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / GridCell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / GridCell);
            bool any = false;
            for (int x = x0; x <= x1 && !any; x++)
                for (int z = z0; z <= z1 && !any; z++) any = grid.ContainsKey(CellKey(x, z));
            if (!any) return false;
            float edge = MathF.Max(Vector3.Distance(a, b), MathF.Max(Vector3.Distance(b, c), Vector3.Distance(c, a)));
            int n = Math.Max(1, (int)MathF.Ceiling(edge));
            for (int i = 0; i <= n; i++)
                for (int j = 0; i + j <= n; j++)
                    if (AboveSurface(a + (b - a) * (i / (float)n) + (c - a) * (j / (float)n))) return true;
            return false;
        }
        void Emit(Vector3 a, Vector3 b, Vector3 c, int depth)
        {
            var (fa, fb, fc) = (Fix(a), Fix(b), Fix(c));
            // Entirely under a road (corners and middle): hidden, never reached - not built.
            if (fa.Y < a.Y - 1e-4f && fb.Y < b.Y - 1e-4f && fc.Y < c.Y - 1e-4f)
            {
                var m = (a + b + c) / 3f;
                if (Fix(m).Y < m.Y - 1e-4f) { TerrainHidden++; return; }
            }
            if (RisesAbove(fa, fb, fc))
            {
                float edge = MathF.Max(Vector3.Distance(a, b), MathF.Max(Vector3.Distance(b, c), Vector3.Distance(c, a)));
                if (edge > 0.6f && depth < 8)
                {
                    var ab = (a + b) / 2f; var bc = (b + c) / 2f; var ca = (c + a) / 2f;
                    Emit(a, ab, ca, depth + 1); Emit(ab, b, bc, depth + 1); Emit(ca, bc, c, depth + 1); Emit(ab, bc, ca, depth + 1);
                    TerrainSplits++;
                }
                else TerrainClipped++;
                return;
            }
            AddToMesh(fa, fb, fc, false, netting ? KindNet : KindTerrain);
            MapTri(fa, fb, fc, netting ? 3 : 2);
            emitted.Add((fa, fb, fc));
        }
        // Only the ground near the roads is built (NearRoad m, 8 m cells): far hills cost BeamNG
        // collision and are never driven on.
        const float NearCell = 8f;
        int nearCells = (int)MathF.Ceiling(NearRoad / NearCell);
        var roadCells = new HashSet<long>();
        foreach (var (a, b, c) in surf)
        {
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / NearCell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / NearCell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / NearCell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / NearCell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 2500) continue;
            for (int x = x0; x <= x1; x++) for (int z = z0; z <= z1; z++) roadCells.Add(CellKey(x, z));
        }
        var near = bandCells;
        near.Clear();
        foreach (var k in roadCells)
        {
            int cx = (int)(k >> 32), cz = (int)(uint)k;
            for (int dx = -nearCells; dx <= nearCells; dx++)
                for (int dz = -nearCells; dz <= nearCells; dz++)
                    if (dx * dx + dz * dz <= nearCells * nearCells) near.Add(CellKey(cx + dx, cz + dz));
        }
        foreach (var (a, b, c) in terrain)
        {
            var m = (a + b + c) / 3f;
            if (!near.Contains(CellKey((int)MathF.Floor(m.X / NearCell), (int)MathF.Floor(m.Z / NearCell)))) { TerrainFar++; continue; }
            Emit(a, b, c, 0);
        }
        terrain.Clear();

        // Safety net: near the roads, wherever there is neither road nor ground (road shoulders
        // and sidewalks the game draws as edge models, junction prefabs' own ground, gaps between
        // strips), flat-ish ground at the height of the nearest surface (roads: Below under them).
        // It goes through Emit, so it never rises above a road either.
        const float NetCell = 8f;
        int netReach = (int)MathF.Ceiling(NearRoad / NetCell);
        var cellH = new Dictionary<long, float>();
        void Raster(Vector3 a, Vector3 b, Vector3 c, float dy)
        {
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / NetCell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / NetCell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / NetCell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / NetCell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 2500) return;
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) return;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    float px = (x + 0.5f) * NetCell, pz = (z + 0.5f) * NetCell;
                    float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                    float l3 = 1 - l1 - l2;
                    if (l1 < 0 || l2 < 0 || l3 < 0) continue;
                    float y = l1 * a.Y + l2 * b.Y + l3 * c.Y + dy;
                    long k = CellKey(x, z);
                    cellH[k] = cellH.TryGetValue(k, out var old) ? MathF.Min(old, y) : y;
                }
        }
        foreach (var (a, b, c) in surf) Raster(a, b, c, 0);   // next to a road: flush with it (Emit puts what is under it lower)
        foreach (var (a, b, c) in emitted) Raster(a, b, c, 0);
        bool InBand(int x, int z) => near.Contains(CellKey((int)MathF.Floor((x + 0.5f) * NetCell / NearCell), (int)MathF.Floor((z + 0.5f) * NetCell / NearCell)));
        // Outwards from every covered cell, the nearest surface's height (breadth first).
        var net = new Dictionary<long, (float H, int Steps)>();
        var queue = new Queue<(int X, int Z, float H, int Steps)>();
        foreach (var (k, h) in cellH) queue.Enqueue(((int)(k >> 32), (int)(uint)k, h, 0));
        while (queue.Count > 0)
        {
            var (x, z, h, steps) = queue.Dequeue();
            if (steps >= netReach) continue;
            foreach (var (nx, nz) in new[] { (x + 1, z), (x - 1, z), (x, z + 1), (x, z - 1) })
            {
                long nk = CellKey(nx, nz);
                if (cellH.ContainsKey(nk) || net.ContainsKey(nk) || !InBand(nx, nz)) continue;
                net[nk] = (h, steps + 1);
                queue.Enqueue((nx, nz, h, steps + 1));
            }
        }
        // Corner heights: the mean of the cells around (covered ones too, so the net meets them).
        float CornerY(int x, int z)
        {
            float sum = 0; int n = 0;
            for (int dx = -1; dx <= 0; dx++)
                for (int dz = -1; dz <= 0; dz++)
                {
                    long k = CellKey(x + dx, z + dz);
                    if (net.TryGetValue(k, out var v)) { sum += v.H; n++; }
                    else if (cellH.TryGetValue(k, out var h)) { sum += h; n++; }
                }
            return n > 0 ? sum / n : 0;
        }
        netting = true;
        foreach (var k in net.Keys)
        {
            int x = (int)(k >> 32), z = (int)(uint)k;
            Vector3 P(int cx, int cz) => new(cx * NetCell, CornerY(cx, cz), cz * NetCell);
            var p00 = P(x, z); var p10 = P(x + 1, z); var p01 = P(x, z + 1); var p11 = P(x + 1, z + 1);
            Emit(p00, p10, p11, 0);
            Emit(p00, p11, p01, 0);
            NetCells++;
        }
        netting = false;
        Stage("terrain + net");
        FillCracks(surf);
        Stage("cracks");
        FillGaps();
        Stage("gaps");
    }

    // Last pass over the band along the roads, on a 1 m grid of everything built: an empty cell
    // up to GapReach m from a surface gets that surface's height (ground strips cut short beside
    // a road, slivers between ground and net, saw teeth). Cells of one row at about the same
    // height become one strip, to keep the triangle count down.
    void FillGaps()
    {
        int GapReach = int.TryParse(Environment.GetEnvironmentVariable("ES_GAPREACH"), out var gr) ? gr : 6, Pad = 21;   // pad: RoadReach + 1
        const float BandCell = 8f, Flat = 0.08f;
        int size = (int)Size, w = size + 2 * Pad;
        // Built triangles by tile (with the padding around it).
        var byTile = new Dictionary<(int, int), List<int>>();
        for (int i = 0; i < built.Count; i++)
        {
            var (a, b, c, _) = built[i];
            float minX = MathF.Min(a.X, MathF.Min(b.X, c.X)) - Pad, maxX = MathF.Max(a.X, MathF.Max(b.X, c.X)) + Pad;
            float minZ = MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) - Pad, maxZ = MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) + Pad;
            var (tx0, tz0) = TileOf(minX, minZ); var (tx1, tz1) = TileOf(maxX, maxZ);
            if ((long)(tx1 - tx0 + 1) * (tz1 - tz0 + 1) > 16) continue;
            for (int tx = tx0; tx <= tx1; tx++)
                for (int tz = tz0; tz <= tz1; tz++)
                {
                    if (!byTile.TryGetValue((tx, tz), out var l)) byTile[(tx, tz)] = l = new List<int>();
                    l.Add(i);
                }
        }
        var top = new float[w * w];
        var roadDist = new byte[w * w];   // metres (4-neighbour steps) to a road / junction cell, 255 = far
        var rq = new int[w * w];
        const int RoadReach = 20;   // 4-neighbour steps: ~14-20 m
        var fill = new float[w * w];
        var dist = new byte[w * w];
        var queue = new int[w * w];
        var newTris = new List<(Vector3, Vector3, Vector3)>();
        foreach (var ((tx, tz), list) in byTile)
        {
            int ox = tx * size - Pad, oz = tz * size - Pad;   // world metre of cell (0, 0)
            // Anything of the band in this tile?
            bool anyBand = false;
            for (int bx = (int)MathF.Floor(ox / BandCell); bx <= (int)MathF.Floor((ox + w) / BandCell) && !anyBand; bx++)
                for (int bz = (int)MathF.Floor(oz / BandCell); bz <= (int)MathF.Floor((oz + w) / BandCell); bz++)
                    if (bandCells.Contains(CellKey(bx, bz))) { anyBand = true; break; }
            if (!anyBand) continue;
            Array.Fill(top, float.NaN);
            Array.Fill(fill, float.NaN);
            Array.Clear(dist);
            Array.Fill(roadDist, (byte)255);
            foreach (var i in list)
            {
                var (a, b, c, _) = built[i];
                int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))) - ox), x1 = Math.Min(w - 1, (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X))) - ox);
                int z0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))) - oz), z1 = Math.Min(w - 1, (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z))) - oz);
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                for (int x = x0; x <= x1; x++)
                    for (int z = z0; z <= z1; z++)
                    {
                        float px = ox + x + 0.5f, pz = oz + z + 0.5f;
                        float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                        float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                        float l3 = 1 - l1 - l2;
                        if (l1 < 0 || l2 < 0 || l3 < 0) continue;
                        float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                        int k = z * w + x;
                        if (float.IsNaN(top[k]) || y > top[k]) top[k] = y;
                        if (built[i].Kind <= KindGame) roadDist[k] = 0;
                    }
            }
            // Only near the roads (RoadReach m): a car leaving the road lands there; the saw-toothed
            // outer edge of the band is not worth the triangles.
            {
                int rh = 0, rt = 0;
                for (int k = 0; k < w * w; k++) if (roadDist[k] == 0) rq[rt++] = k;
                while (rh < rt)
                {
                    int k = rq[rh++];
                    if (roadDist[k] >= RoadReach) continue;
                    int x = k % w, z = k / w;
                    if (x > 0 && roadDist[k - 1] == 255) { roadDist[k - 1] = (byte)(roadDist[k] + 1); rq[rt++] = k - 1; }
                    if (x < w - 1 && roadDist[k + 1] == 255) { roadDist[k + 1] = (byte)(roadDist[k] + 1); rq[rt++] = k + 1; }
                    if (z > 0 && roadDist[k - w] == 255) { roadDist[k - w] = (byte)(roadDist[k] + 1); rq[rt++] = k - w; }
                    if (z < w - 1 && roadDist[k + w] == 255) { roadDist[k + w] = (byte)(roadDist[k] + 1); rq[rt++] = k + w; }
                }
            }
            // Breadth first from the edges of what exists into empty band cells.
            int qh = 0, qt = 0;
            for (int z = 0; z < w; z++)
                for (int x = 0; x < w; x++)
                {
                    int k = z * w + x;
                    if (float.IsNaN(top[k])) continue;
                    if ((x > 0 && float.IsNaN(top[k - 1])) || (x < w - 1 && float.IsNaN(top[k + 1])) ||
                        (z > 0 && float.IsNaN(top[k - w])) || (z < w - 1 && float.IsNaN(top[k + w])))
                    { fill[k] = top[k]; queue[qt++] = k; }
                }
            while (qh < qt)
            {
                int k = queue[qh++];
                if (dist[k] >= GapReach) continue;
                int x = k % w, z = k / w;
                for (int d = 0; d < 4; d++)
                {
                    int nx = x + (d == 0 ? 1 : d == 1 ? -1 : 0), nz = z + (d == 2 ? 1 : d == 3 ? -1 : 0);
                    if (nx < 0 || nz < 0 || nx >= w || nz >= w) continue;
                    int nk = nz * w + nx;
                    if (!float.IsNaN(top[nk]) || !float.IsNaN(fill[nk]) || roadDist[nk] == 255) continue;
                    if (!bandCells.Contains(CellKey((int)MathF.Floor((ox + nx + 0.5f) / BandCell), (int)MathF.Floor((oz + nz + 0.5f) / BandCell)))) continue;
                    fill[nk] = fill[k];
                    dist[nk] = (byte)(dist[k] + 1);
                    queue[qt++] = nk;
                }
            }
            // Filled cells of this tile proper, merged into strips along each row.
            for (int z = Pad; z < Pad + size; z++)
            {
                int x = Pad;
                while (x < Pad + size)
                {
                    int k = z * w + x;
                    if (!float.IsNaN(top[k]) || float.IsNaN(fill[k])) { x++; continue; }
                    int x1 = x;
                    float h0 = fill[k], sum = h0;
                    while (x1 + 1 < Pad + size)
                    {
                        int nk = z * w + x1 + 1;
                        if (!float.IsNaN(top[nk]) || float.IsNaN(fill[nk]) || MathF.Abs(fill[nk] - h0) > Flat) break;
                        sum += fill[nk];
                        x1++;
                    }
                    float h = sum / (x1 - x + 1);
                    var p00 = new Vector3(ox + x, h, oz + z); var p10 = new Vector3(ox + x1 + 1, h, oz + z);
                    var p01 = new Vector3(ox + x, h, oz + z + 1); var p11 = new Vector3(ox + x1 + 1, h, oz + z + 1);
                    newTris.Add((p00, p10, p11));
                    newTris.Add((p00, p11, p01));
                    GapCells += x1 - x + 1;
                    x = x1 + 1;
                }
            }
        }
        foreach (var (a, b, c) in newTris)
        {
            AddToMesh(a, b, c, false, KindGap);
            MapTri(a, b, c, 3);
        }
    }

    // Cracks: gaps in the drivable surface narrower than the safety net's 4 m cells (between
    // strips, a road and a junction's collision). On a 1 m grid, an empty cell with surface on both
    // sides within CrackMax m (left and right, or front and back) is a crack: filled flush with
    // the surface around it.
    void FillCracks((Vector3 A, Vector3 B, Vector3 C)[] surf)
    {
        // Tile by tile on arrays (a dictionary of every covered square metre took ~15 s for a
        // 5 km area). Each tile's array has a margin wide enough for every look-up of its own cells
        // (a crack cell is up to CrackMax - 1 m from a surface, and looks CrackMax m to each side);
        // a tile only fills its own cells.
        const int CrackMax = 3, Pad = CrackMax + 1;
        int size = (int)Size, w = size + 2 * Pad;
        var byTile = new Dictionary<(int, int), List<int>>();
        for (int i = 0; i < surf.Length; i++)
        {
            var (a, b, c) = surf[i];
            float minX = MathF.Min(a.X, MathF.Min(b.X, c.X)), maxX = MathF.Max(a.X, MathF.Max(b.X, c.X));
            float minZ = MathF.Min(a.Z, MathF.Min(b.Z, c.Z)), maxZ = MathF.Max(a.Z, MathF.Max(b.Z, c.Z));
            if ((long)((int)maxX - (int)minX + 1) * ((int)maxZ - (int)minZ + 1) > 200000) continue;
            var (tx0, tz0) = TileOf(minX - Pad, minZ - Pad); var (tx1, tz1) = TileOf(maxX + Pad, maxZ + Pad);
            for (int tx = tx0; tx <= tx1; tx++)
                for (int tz = tz0; tz <= tz1; tz++)
                {
                    if (!byTile.TryGetValue((tx, tz), out var l)) byTile[(tx, tz)] = l = new List<int>();
                    l.Add(i);
                }
        }
        var cov = new float[w * w];
        var done = new bool[w * w];
        var crackCells = new Dictionary<long, float>();
        var dirs = new[] { (1, 0), (-1, 0), (0, 1), (0, -1) };
        foreach (var ((tx, tz), list) in byTile)
        {
            int ox = tx * size - Pad, oz = tz * size - Pad;   // world metre of cell (0, 0)
            Array.Fill(cov, float.NaN);
            Array.Clear(done);
            foreach (var i in list)
            {
                var (a, b, c) = surf[i];
                int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))) - ox), x1 = Math.Min(w - 1, (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X))) - ox);
                int z0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))) - oz), z1 = Math.Min(w - 1, (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z))) - oz);
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                for (int x = x0; x <= x1; x++)
                    for (int z = z0; z <= z1; z++)
                    {
                        float px = ox + x + 0.5f, pz = oz + z + 0.5f;
                        float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                        float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                        float l3 = 1 - l1 - l2;
                        if (l1 < 0 || l2 < 0 || l3 < 0) continue;
                        float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                        int k = z * w + x;
                        if (float.IsNaN(cov[k]) || y > cov[k]) cov[k] = y;
                    }
            }
            bool Side(int x, int z, int dx, int dz, float near, out float h)
            {
                h = 0;
                for (int s = 1; s <= CrackMax; s++)
                {
                    int sx = x + dx * s, sz = z + dz * s;
                    if (sx < 0 || sx >= w || sz < 0 || sz >= w) return false;
                    h = cov[sz * w + sx];
                    if (!float.IsNaN(h)) return MathF.Abs(h - near) < 1.0f || float.IsNaN(near);
                }
                return false;
            }
            for (int k = 0; k < w * w; k++)
            {
                if (float.IsNaN(cov[k])) continue;
                int cx = k % w, cz = k / w;
                foreach (var (dx, dz) in dirs)
                    for (int step = 1; step < CrackMax; step++)
                    {
                        int x = cx + dx * step, z = cz + dz * step;
                        if (x < 0 || x >= w || z < 0 || z >= w) break;
                        int nk = z * w + x;
                        if (!float.IsNaN(cov[nk])) break;
                        // Only this tile's own cells (the margin is its neighbours').
                        if (x < Pad || x >= Pad + size || z < Pad || z >= Pad + size) continue;
                        if (done[nk]) continue;
                        done[nk] = true;
                        // Surface across the gap at about the same height (not a road over another one).
                        float h;
                        if (Side(x, z, 1, 0, float.NaN, out float hx1) && Side(x, z, -1, 0, hx1, out float hx2)) h = (hx1 + hx2) / 2;
                        else if (Side(x, z, 0, 1, float.NaN, out float hz1) && Side(x, z, 0, -1, hz1, out float hz2)) h = (hz1 + hz2) / 2;
                        else continue;
                        crackCells[CellKey(ox + x, oz + z)] = h;
                        CracksFilled++;
                    }
            }
        }
        // Crack cells of a row at about one height (2 cm) -> one strip.
        foreach (var row in crackCells.GroupBy(kv => (int)(uint)kv.Key))
        {
            int z = row.Key;
            var cells = row.Select(kv => ((int)(kv.Key >> 32), kv.Value)).OrderBy(c => c.Item1).ToList();
            for (int i = 0; i < cells.Count;)
            {
                int j = i;
                float sum = cells[i].Item2;
                while (j + 1 < cells.Count && cells[j + 1].Item1 == cells[j].Item1 + 1 && MathF.Abs(cells[j + 1].Item2 - cells[i].Item2) < 0.02f)
                    sum += cells[++j].Item2;
                float h = sum / (j - i + 1);
                var p00 = new Vector3(cells[i].Item1, h, z); var p10 = new Vector3(cells[j].Item1 + 1, h, z);
                var p01 = new Vector3(cells[i].Item1, h, z + 1); var p11 = new Vector3(cells[j].Item1 + 1, h, z + 1);
                AddToMesh(p00, p10, p11, false, KindCrack); AddToMesh(p00, p11, p01, false, KindCrack);
                MapTri(p00, p10, p11, 3); MapTri(p00, p11, p01, 3);
                i = j + 1;
            }
        }
    }

    // Debug (ES_MAP=file.png;x;z;size): a top view of what was built, 1 px per metre:
    // grey = road / junction surfaces, green = terrain (darker: lowered under a road), black = nothing.
    byte[] mapPx; float mapX0, mapZ0; int mapN;
    void MapTri(Vector3 a, Vector3 b, Vector3 c, int kind)
    {
        if (mapPx == null) return;
        int x0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) - mapX0)), x1 = Math.Min(mapN - 1, (int)MathF.Ceiling(MathF.Max(a.X, MathF.Max(b.X, c.X)) - mapX0));
        int z0 = Math.Max(0, (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) - mapZ0)), z1 = Math.Min(mapN - 1, (int)MathF.Ceiling(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) - mapZ0));
        float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
        if (MathF.Abs(det) < 1e-9f) return;
        for (int x = x0; x <= x1; x++)
            for (int z = z0; z <= z1; z++)
            {
                float px = mapX0 + x + 0.5f, pz = mapZ0 + z + 0.5f;
                float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                if (l1 < 0 || l2 < 0 || 1 - l1 - l2 < 0) continue;
                int i = (z * mapN + x) * 3;
                if (kind == 1) { mapPx[i] = 150; mapPx[i + 1] = 150; mapPx[i + 2] = 150; }
                else if (kind == 3 && mapPx[i] != 150) { mapPx[i] = 40; mapPx[i + 1] = 90; mapPx[i + 2] = 220; }
                else if (mapPx[i] != 150) { mapPx[i] = 30; mapPx[i + 1] = 140; mapPx[i + 2] = 40; }
            }
    }
    void MapBegin()
    {
        if (Environment.GetEnvironmentVariable("ES_MAP") is not string arg) return;
        var p = arg.Split(';');
        mapN = int.Parse(p[3]);
        mapX0 = float.Parse(p[1], CultureInfo.InvariantCulture) - mapN / 2f;
        mapZ0 = float.Parse(p[2], CultureInfo.InvariantCulture) - mapN / 2f;
        mapPx = new byte[mapN * mapN * 3];
    }
    void MapWrite()
    {
        if (mapPx == null) return;
        var path = Environment.GetEnvironmentVariable("ES_MAP")!.Split(';')[0];
        var raw = new MemoryStream();
        for (int z = 0; z < mapN; z++) { raw.WriteByte(0); raw.Write(mapPx, z * mapN * 3, mapN * 3); }
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
        var nb = BitConverter.GetBytes(mapN); Array.Reverse(nb);
        Array.Copy(nb, 0, hdr, 0, 4); Array.Copy(nb, 0, hdr, 4, 4);
        hdr[8] = 8; hdr[9] = 2;
        Chunk("IHDR", hdr);
        Chunk("IDAT", zs.ToArray());
        Chunk("IEND", Array.Empty<byte>());
        Console.WriteLine($"map image -> {path}");
    }

    // A road end's overlap onto what follows (a fill that is never dropped as redundant: it
    // is there to close the seam between the two, however thin).
    readonly HashSet<int> seams = new();
    public void SeamQuad(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1)
    {
        int n = pending.Count;
        Quad(a0, b0, a1, b1, false, PrioFill);
        for (int i = n; i < pending.Count; i++) seams.Add(i);
    }

    public void Quad(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1, bool doubleSided = false, int prio = PrioRoad)
    {
        Tri(a0, b0, b1, doubleSided, prio);
        Tri(a0, b1, a1, doubleSided, prio);
    }

    // A road surface quad (a0-b0 across, a1-b1 one step on). Where the bank changes along the
    // road, the quad is twisted and its two triangles fold along the diagonal - a ridge a wheel
    // feels. Twisted quads are split bilinearly (up to 4 x 4), so the fold is small.
    public void RoadQuad(Vector3 a0, Vector3 b0, Vector3 a1, Vector3 b1, int prio = PrioRoad)
    {
        float twist = MathF.Abs((a0.Y + b1.Y) - (b0.Y + a1.Y));
        int n = Math.Clamp((int)MathF.Ceiling(twist / 0.02f), 1, 4);
        if (n == 1) { Quad(a0, b0, a1, b1, false, prio); return; }
        Vector3 P(int i, int j)   // i along (a0 -> a1), j across (a -> b)
        {
            float u = i / (float)n, v = j / (float)n;
            return Vector3.Lerp(Vector3.Lerp(a0, b0, v), Vector3.Lerp(a1, b1, v), u);
        }
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Quad(P(i, j), P(i, j + 1), P(i + 1, j), P(i + 1, j + 1), false, prio);
        QuadsSmoothed++;
    }
    public int QuadsSmoothed { get; private set; }

    static bool Finite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

    const float Probe = 1.5f;      // m to each side of a wall where we look for road surface
    const float ProbeDy = 1.2f;    // height tolerance against the wall's foot
    const float Cell = 1.0f;

    static long CellKey(int x, int z) => ((long)x << 32) ^ (uint)z;

    // Decides every held-back wall and puts the real ones into the mesh.
    void ResolveWalls()
    {
        // The probe points: per wall triangle, a point on each side at its foot.
        var probes = new List<(Vector3 P, Vector3 Q)>(walls.Count);
        var needed = new Dictionary<long, List<float>>();
        foreach (var (a, b, c) in walls)
        {
            var n = Vector3.Cross(b - a, c - a);
            var h = new Vector2(n.X, n.Z);
            float hl = h.Length();
            var ctr = (a + b + c) / 3f;
            if (hl < 1e-6f) { probes.Add((new Vector3(float.NaN), new Vector3(float.NaN))); continue; }
            h /= hl;
            float foot = MathF.Min(a.Y, MathF.Min(b.Y, c.Y)) + 0.3f;   // walls start 0.3 m below the road
            var p = new Vector3(ctr.X + h.X * Probe, foot, ctr.Z + h.Y * Probe);
            var q = new Vector3(ctr.X - h.X * Probe, foot, ctr.Z - h.Y * Probe);
            probes.Add((p, q));
            foreach (var v in new[] { p, q })
                needed[CellKey((int)MathF.Floor(v.X / Cell), (int)MathF.Floor(v.Z / Cell))] = null;
        }
        // Surface heights at the centres of only those cells.
        foreach (var (a, b, c) in surfaces)
        {
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / Cell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / Cell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / Cell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / Cell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 1_000_000) continue;   // degenerate giant
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long key = CellKey(x, z);
                    if (!needed.TryGetValue(key, out var hs)) continue;
                    float px = (x + 0.5f) * Cell, pz = (z + 0.5f) * Cell;
                    float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                    float l3 = 1 - l1 - l2;
                    const float e = -0.02f;
                    if (l1 < e || l2 < e || l3 < e) continue;
                    float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                    if (hs == null) needed[key] = hs = new List<float>(2);
                    hs.Add(y);
                }
        }
        bool SurfaceAt(Vector3 v)
        {
            if (!needed.TryGetValue(CellKey((int)MathF.Floor(v.X / Cell), (int)MathF.Floor(v.Z / Cell)), out var hs) || hs == null) return false;
            foreach (var y in hs) if (MathF.Abs(y - v.Y) < ProbeDy) return true;
            return false;
        }
        for (int i = 0; i < walls.Count; i++)
        {
            var (a, b, c) = walls[i];
            var (p, q) = probes[i];
            if (float.IsNaN(p.X) || (SurfaceAt(p) && SurfaceAt(q))) { WallsDropped++; continue; }
            AddToMesh(a, b, c, true);
            WallsKept++;
        }
        walls.Clear();
    }

    // Steps: overlapping surfaces that disagree in height - an edge a wheel hits.
    // Measured as area: 1 m cells where overlapping surfaces disagree in height by 2-60 cm.
    public int CountSteps()
    {
        var cells = new Dictionary<long, (float Min, float Max, int MinP, int MaxP)>();
        foreach (var (a, b, c, prio) in pending)
        {
            if (prio < 0) continue;
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)));
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)));
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 200000) continue;
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    float px = x + 0.5f, pz = z + 0.5f;
                    float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                    float l3 = 1 - l1 - l2;
                    if (l1 < 0 || l2 < 0 || l3 < 0) continue;
                    float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                    long k = CellKey(x, z);
                    if (!cells.TryGetValue(k, out var m)) cells[k] = (y, y, prio, prio);
                    else cells[k] = (y < m.Min ? y : m.Min, y > m.Max ? y : m.Max, y < m.Min ? prio : m.MinP, y > m.Max ? prio : m.MaxP);
                }
        }
        int steps = 0;
        var pairs = new int[3, 3];
        foreach (var (mn, mx, pmn, pmx) in cells.Values)
            if (mx - mn > 0.02f && mx - mn < 0.6f) { steps++; pairs[pmn, pmx]++; }
        string[] pn = { "fill", "road", "game" };
        var sb = new StringBuilder("   step cells by (lower, higher) surface:");
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) if (pairs[i, j] > 0) sb.Append($" {pn[i]}<{pn[j]} {pairs[i, j]}");
        Console.WriteLine(sb.ToString());
        return steps;
    }
    public int StepsBefore { get; private set; }
    public int StepsAfter { get; private set; }

    public void Write(string dir, string vfsDir)
    {
        StepsBefore = CountSteps();
        Stage(null);
        if (Environment.GetEnvironmentVariable("ES_NOLEVEL") == null) LevelOverlaps();
        Stage("level");
        ClipFills();
        Stage("clip fills");
        StepsAfter = CountSteps();
        MapBegin();
        foreach (var (a, b, c, prio) in pending)
        {
            if (prio < 0) continue;
            surfaces.Add((a, b, c));
            AddToMesh(a, b, c, false, (byte)prio);
            MapTri(a, b, c, 1);
        }
        Stage("mesh roads");
        AddTerrain();
        MapWrite();
        pending.Clear();
        ResolveWalls();
        Stage("terrain + walls");
        if (HeightfieldDir != null) { ReduceToHeightfields(); Stage("heightfields"); }
        Directory.CreateDirectory(dir);
        // Tile files are named by their content (hash), so a file that exists already is this very
        // tile: only new tiles are written. BeamNG watches its mod folder and stalls on a burst of
        // changed files; and its converted shapes (.cdae cache) stay valid, so tiles driven
        // before load faster. Files unused for 6 hours are removed.
        var idx = new StringBuilder();
        var keep = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        TilesWritten = 0;
        TilesKept = 0;
        foreach (var ((tx, tz), mb) in Tiles)
        {
            if (mb.Tris.Count == 0) continue;
            ulong hash = ContentHash(mb);
            var name = $"t_{tx}_{tz}_{hash:x16}.dae";
            keep.Add(name);
            var file = Path.Combine(dir, name);
            if (File.Exists(file)) TilesKept++;
            else { WriteCollada(file, mb, $"es_{tx}_{tz}"); TilesWritten++; }
            var o = Origin((tx, tz));
            idx.AppendLine(string.Format(CultureInfo.InvariantCulture, "{0} {1} {2:F2} {3:F2} {4}/{5} {6:x16}",
                tx, tz, o.X, o.Y, vfsDir.TrimEnd('/'), name, hash));
        }
        // "#ground": these tiles hold only what the ground block cannot (bridges, walls); the ground
        // is in the height tiles (the bridge must load those with them).
        // "#area x z radius": what was generated (the bridge makes the next area before the car
        // reaches its edge).
        File.WriteAllText(Path.Combine(dir, "tiles.idx"), string.Format(CultureInfo.InvariantCulture, "#tile {0}\n", Size) +
                          (HeightfieldDir != null ? "#ground\n" : "") +
                          (Area is { } ar ? string.Format(CultureInfo.InvariantCulture, "#area {0:F0} {1:F0} {2:F0}\n", ar.X, ar.Z, ar.Radius) : "") +
                          idx.ToString());
        // Old tiles: not in this area and not written for 6 hours (a tile the previous area still
        // has loaded in BeamNG is recent, so it stays).
        var cutoff = DateTime.UtcNow.AddHours(-6);
        foreach (var f in Directory.GetFiles(dir, "t_*.dae"))
            try
            {
                if (!keep.Contains(Path.GetFileName(f)) && File.GetLastWriteTimeUtc(f) < cutoff) { File.Delete(f); TilesRemoved++; }
            }
            catch (IOException) { }
            catch (UnauthorizedAccessException) { }
    }
    // Drives the given lines (lane lines of roads, junction lanes) every metre over what was built:
    // a hole is a spot with no surface near the line's height; a bump or dip is a kink in the
    // surface height (second difference over 1 m above 4 cm), the edge a wheel feels.
    // Problem spots go to ES_PROBEOUT (csv) when set.
    public string ProbeLines(List<List<Vector3>> lines)
    {
        const float Cell = 4f, Kink = 0.04f;
        var grid = new Dictionary<long, List<int>>();
        for (int i = 0; i < built.Count; i++)
        {
            var (a, b, c, _) = built[i];
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X)) / Cell), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)) / Cell);
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z)) / Cell), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)) / Cell);
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 40000) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long k = CellKey(x, z);
                    if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>();
                    l.Add(i);
                }
        }
        // The surface a wheel at p rests on: the highest within reach of the line's height.
        bool Top(Vector3 p, out float h, out byte kind)
        {
            h = float.MinValue; kind = 255;
            // Heightfield mode: the ground grid (as the TerrainBlock gives it) and the meshes left.
            if (hfGround.Count > 0)
            {
                float g = GroundAt(p.X, p.Z);
                if (!float.IsNaN(g) && g >= p.Y - 1.5f && g <= p.Y + 1.0f) { h = g; kind = 7; }
            }
            if (!grid.TryGetValue(CellKey((int)MathF.Floor(p.X / Cell), (int)MathF.Floor(p.Z / Cell)), out var l)) return kind != 255;
            foreach (var i in l)
            {
                if (meshSeen != null && meshSeen[i] && !meshKeep![i]) continue;   // shown by the ground grid
                var (a, b, c, k) = built[i];
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) continue;
                float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                float l3 = 1 - l1 - l2;
                const float e = -0.001f;
                if (l1 < e || l2 < e || l3 < e) continue;
                float y = l1 * a.Y + l2 * b.Y + l3 * c.Y;
                if (y < p.Y - 1.5f || y > p.Y + 1.0f) continue;
                if (y > h) { h = y; kind = k; }
            }
            return kind != 255;
        }
        string[] kn = { "fill", "road", "game", "terrain", "net", "crack", "gap", "grid" };
        // Debug: ES_AT=x,z lists every built surface at that point.
        if (Environment.GetEnvironmentVariable("ES_AT") is string at)
        {
            var xz = at.Split(',').Select(s => float.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            int inBox = 0;
            foreach (var (a, b, c, k) in built)
            {
                float minX = MathF.Min(a.X, MathF.Min(b.X, c.X)), maxX = MathF.Max(a.X, MathF.Max(b.X, c.X));
                float minZ = MathF.Min(a.Z, MathF.Min(b.Z, c.Z)), maxZ = MathF.Max(a.Z, MathF.Max(b.Z, c.Z));
                if (xz[0] < minX - 0.5f || xz[0] > maxX + 0.5f || xz[1] < minZ - 0.5f || xz[1] > maxZ + 0.5f) continue;
                if (++inBox <= 12)
                    Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  near: {0,-7} ({1:F2},{2:F2},{3:F2}) ({4:F2},{5:F2},{6:F2}) ({7:F2},{8:F2},{9:F2})",
                        kn[Math.Min(7, (int)k)], a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z));
            }
            Console.WriteLine($"  {inBox} triangles with a bounding box within 0.5 m");
            if (hfGround.Count > 0)
                for (int dz = -2; dz <= 2; dz++)
                {
                    var row = new StringBuilder($"  grid z {MathF.Floor(xz[1]) + dz,7}:");
                    for (int dx = -3; dx <= 3; dx++) row.Append(string.Format(CultureInfo.InvariantCulture, " {0,7:F3}", GroundAt(MathF.Floor(xz[0]) + dx, MathF.Floor(xz[1]) + dz)));
                    Console.WriteLine(row.ToString());
                }
            for (float dx = -1; dx <= 1; dx += 1)
            {
                var p = new Vector3(xz[0] + dx, 0, xz[1]);
                if (!grid.TryGetValue(CellKey((int)MathF.Floor(p.X / Cell), (int)MathF.Floor(p.Z / Cell)), out var l)) continue;
                foreach (var i in l)
                {
                    var (a, b, c, k) = built[i];
                    float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                    if (MathF.Abs(det) < 1e-9f) continue;
                    float l1 = ((b.Z - c.Z) * (p.X - c.X) + (c.X - b.X) * (p.Z - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (p.X - c.X) + (a.X - c.X) * (p.Z - c.Z)) / det;
                    if (l1 < 0 || l2 < 0 || 1 - l1 - l2 < 0) continue;
                    Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "  at ({0:F1},{1:F1}): {2,-7} y {3:F3}  tri ({4:F1},{5:F2},{6:F1}) ({7:F1},{8:F2},{9:F1}) ({10:F1},{11:F2},{12:F1})",
                        p.X, p.Z, kn[Math.Min(7, (int)k)], l1 * a.Y + l2 * b.Y + (1 - l1 - l2) * c.Y, a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z));
                }
            }
        }
        long samples = 0;
        int holes = 0, kinks = 0;
        var byKind = new int[8];
        var csv = Environment.GetEnvironmentVariable("ES_PROBEOUT") is string path ? new StringBuilder("x,z,y,what,value,kind\n") : null;
        foreach (var line in lines)
        {
            if (line.Count < 2) continue;
            // Resample every metre along the line.
            var pts = new List<Vector3> { line[0] };
            float carry = 0;
            for (int i = 0; i + 1 < line.Count; i++)
            {
                var a = line[i]; var b = line[i + 1];
                float len = Vector3.Distance(a, b);
                for (float s = 1 - carry; s <= len; s += 1) pts.Add(Vector3.Lerp(a, b, s / len));
                carry = (carry + len) % 1;
            }
            var hs = new float[pts.Count];
            var ks = new byte[pts.Count];
            var ok = new bool[pts.Count];
            for (int i = 0; i < pts.Count; i++)
            {
                ok[i] = Top(pts[i], out hs[i], out ks[i]);
                samples++;
                if (!ok[i])
                {
                    holes++;
                    csv?.Append(string.Format(CultureInfo.InvariantCulture, "{0:F1},{1:F1},{2:F1},hole,0,\n", pts[i].X, pts[i].Z, pts[i].Y));
                }
            }
            for (int i = 1; i + 1 < pts.Count; i++)
            {
                if (!ok[i - 1] || !ok[i] || !ok[i + 1]) continue;
                float d2 = hs[i - 1] - 2 * hs[i] + hs[i + 1];
                if (MathF.Abs(d2) <= Kink) continue;
                kinks++;
                byte k = ks[i] != ks[i - 1] ? ks[i - 1] : ks[i + 1] != ks[i] ? ks[i + 1] : ks[i];
                if (ks[i] < 8) byKind[ks[i]]++;
                csv?.Append(string.Format(CultureInfo.InvariantCulture, "{0:F1},{1:F1},{2:F2},{3},{4:F3},{5}>{6}\n",
                    pts[i].X, pts[i].Z, hs[i], d2 > 0 ? "dip" : "bump", d2, kn[Math.Min(7, (int)ks[i])], k < 8 ? kn[k] : "?"));
            }
        }
        if (csv != null) File.WriteAllText(Environment.GetEnvironmentVariable("ES_PROBEOUT")!, csv.ToString());
        float km = samples / 1000f;
        var sb = new StringBuilder(string.Format(CultureInfo.InvariantCulture,
            "lane probe: {0:F0} km driven, {1} holes ({2:F1}/km), {3} bumps/dips > {4:F0} cm ({5:F1}/km); on:",
            km, holes, holes / Math.Max(km, 0.001f), kinks, Kink * 100, kinks / Math.Max(km, 0.001f)));
        for (int i = 0; i < 8; i++) if (byKind[i] > 0) sb.Append($" {kn[i]} {byKind[i]}");
        return sb.ToString();
    }

    // Off the road: the share of the band along the roads (NearRoad) with nothing to drive on,
    // on a 1 m grid (where a car leaving the road would fall).
    public string ProbeBand()
    {
        const float BandCell = 8f;
        var cov = new HashSet<long>();
        foreach (var (a, b, c, _) in built)
        {
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)));
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)));
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 200000) continue;
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    float px = x + 0.5f, pz = z + 0.5f;
                    float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                    if (l1 < 0 || l2 < 0 || 1 - l1 - l2 < 0) continue;
                    cov.Add(CellKey(x, z));
                }
        }
        long band = 0, empty = 0;
        var holes = new List<(int X, int Z)>();
        foreach (var k in bandCells)
        {
            int bx = (int)(k >> 32), bz = (int)(uint)k;
            for (int x = bx * (int)BandCell; x < (bx + 1) * (int)BandCell; x++)
                for (int z = bz * (int)BandCell; z < (bz + 1) * (int)BandCell; z++)
                {
                    band++;
                    if (!cov.Contains(CellKey(x, z))) { empty++; holes.Add((x, z)); }
                }
        }
        if (Environment.GetEnvironmentVariable("ES_BANDOUT") is string path)
            File.WriteAllLines(path, holes.Select(h => $"{h.X},{h.Z}"));
        // How far from the road the empty spots are (what matters is near it).
        var road = new HashSet<long>();   // 4 m cells with road / junction / fill surface (real coverage)
        foreach (var (a, b, c, k) in built)
        {
            if (k > KindGame) continue;
            int x0 = (int)MathF.Floor(MathF.Min(a.X, MathF.Min(b.X, c.X))), x1 = (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X)));
            int z0 = (int)MathF.Floor(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))), z1 = (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z)));
            if ((long)(x1 - x0 + 1) * (z1 - z0 + 1) > 200000) continue;
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) continue;
            for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    float px = x + 0.5f, pz = z + 0.5f;
                    float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                    if (l1 < 0 || l2 < 0 || 1 - l1 - l2 < 0) continue;
                    road.Add(CellKey((int)MathF.Floor(x / 4f), (int)MathF.Floor(z / 4f)));
                }
        }
        int[] byDist = new int[4];   // within 8, 16, 32 m of a road, further
        var nearHoles = new List<(int X, int Z)>();
        foreach (var (hx, hz) in holes)
        {
            int cx = (int)MathF.Floor(hx / 4f), cz = (int)MathF.Floor(hz / 4f), d = 3;
            for (int r = 0; r <= 8 && d == 3; r++)
                for (int dx = -r; dx <= r && d == 3; dx++)
                    for (int dz = -r; dz <= r; dz++)
                        if ((Math.Abs(dx) == r || Math.Abs(dz) == r) && road.Contains(CellKey(cx + dx, cz + dz))) { d = r <= 2 ? 0 : r <= 4 ? 1 : 2; break; }
            byDist[d]++;
            if (d == 0) nearHoles.Add((hx, hz));
        }
        if (Environment.GetEnvironmentVariable("ES_NEAROUT") is string np) File.WriteAllLines(np, nearHoles.Select(h => $"{h.X},{h.Z}"));
        var counts = new int[7];
        foreach (var t in built) counts[Math.Min(6, (int)t.Kind)]++;
        Console.WriteLine($"   triangles by kind: fill {counts[0]}, road {counts[1]}, game {counts[2]}, terrain {counts[3]}, net {counts[4]}, crack {counts[5]}, gap {counts[6]}");
        return string.Format(CultureInfo.InvariantCulture, "band probe: {0:F2} km2 within {1} m of roads, {2:F3} km2 with nothing ({3:F2} %); empty m2 by distance to a road: <8 m {4}, 8-16 m {5}, 16-32 m {6}, further {7}",
            band / 1e6, NearRoad, empty / 1e6, 100.0 * empty / Math.Max(1, band), byDist[0], byDist[1], byDist[2], byDist[3]);
    }

    // ES_TIMING: how long each step of Write takes.
    readonly System.Diagnostics.Stopwatch stageWatch = new();
    void Stage(string? done)
    {
        if (done != null && Environment.GetEnvironmentVariable("ES_TIMING") != null)
            Console.WriteLine($"   time {done}: {stageWatch.Elapsed.TotalSeconds:F1} s");
        stageWatch.Restart();
    }

    public int TilesWritten { get; private set; }
    public int TilesKept { get; private set; }
    public int TilesRemoved { get; private set; }

    // What a tile contains, independent of triangle and vertex order (as written: cm precision):
    // the bridge keeps an already loaded tile when the next area's tile at that spot is the same.
    static ulong ContentHash(MeshBuilder mb)
    {
        static ulong Mix(ulong h)
        {
            h ^= h >> 33; h *= 0xff51afd7ed558ccdUL; h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53UL; h ^= h >> 33;
            return h;
        }
        ulong VertKey(int i)
        {
            var v = mb.Verts[i];
            ulong x = (ulong)(long)MathF.Round(v.X * 100), y = (ulong)(long)MathF.Round(v.Y * 100), z = (ulong)(long)MathF.Round(v.Z * 100);
            return Mix(x * 0x9E3779B97F4A7C15UL ^ Mix(y + 0x632BE59BD9B4E019UL) ^ Mix(z * 0x94D049BB133111EBUL + 1));
        }
        ulong sum = 0;
        for (int t = 0; t + 2 < mb.Tris.Count; t += 3)
        {
            ulong a = VertKey(mb.Tris[t]), b = VertKey(mb.Tris[t + 1]), c = VertKey(mb.Tris[t + 2]);
            // The same triangle whichever corner comes first (winding kept).
            ulong r1 = Mix(a ^ Mix(b + 1) ^ Mix(Mix(c) + 2)), r2 = Mix(b ^ Mix(c + 1) ^ Mix(Mix(a) + 2)), r3 = Mix(c ^ Mix(a + 1) ^ Mix(Mix(b) + 2));
            sum += Math.Min(r1, Math.Min(r2, r3));
        }
        return Mix(sum ^ (ulong)mb.Tris.Count);
    }

    static void WriteCollada(string path, MeshBuilder mb, string id)
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder(mb.Verts.Count * 30 + mb.Tris.Count * 8 + 2048);
        sb.Append("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
        sb.Append("<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n");
        sb.Append(" <asset><unit name=\"meter\" meter=\"1\"/><up_axis>Z_UP</up_axis></asset>\n");
        sb.Append(" <library_effects><effect id=\"es_road-fx\"><profile_COMMON><technique sid=\"common\"><lambert>");
        sb.Append("<diffuse><color>1 0 1 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>\n");
        sb.Append(" <library_materials><material id=\"es_road\" name=\"es_road\"><instance_effect url=\"#es_road-fx\"/></material></library_materials>\n");
        sb.Append($" <library_geometries><geometry id=\"{id}-mesh\" name=\"{id}\"><mesh>\n");
        sb.Append($"  <source id=\"{id}-pos\"><float_array id=\"{id}-pos-array\" count=\"{mb.Verts.Count * 3}\">");
        foreach (var v in mb.Verts)
            sb.Append(v.X.ToString("F2", ci)).Append(' ').Append(v.Y.ToString("F2", ci)).Append(' ').Append(v.Z.ToString("F2", ci)).Append(' ');
        sb.Append("</float_array>\n");
        sb.Append($"   <technique_common><accessor source=\"#{id}-pos-array\" count=\"{mb.Verts.Count}\" stride=\"3\">");
        sb.Append("<param name=\"X\" type=\"float\"/><param name=\"Y\" type=\"float\"/><param name=\"Z\" type=\"float\"/></accessor></technique_common></source>\n");
        sb.Append($"  <vertices id=\"{id}-vtx\"><input semantic=\"POSITION\" source=\"#{id}-pos\"/></vertices>\n");
        sb.Append($"  <triangles material=\"es_road\" count=\"{mb.Tris.Count / 3}\"><input semantic=\"VERTEX\" source=\"#{id}-vtx\" offset=\"0\"/><p>");
        foreach (var i in mb.Tris) sb.Append(i).Append(' ');
        sb.Append("</p></triangles>\n </mesh></geometry></library_geometries>\n");
        sb.Append($" <library_visual_scenes><visual_scene id=\"scene\"><node id=\"{id}\" name=\"{id}\">");
        sb.Append($"<instance_geometry url=\"#{id}-mesh\"><bind_material><technique_common>");
        sb.Append("<instance_material symbol=\"es_road\" target=\"#es_road\"/></technique_common></bind_material></instance_geometry>");
        sb.Append("</node></visual_scene></library_visual_scenes>\n");
        sb.Append(" <scene><instance_visual_scene url=\"#scene\"/></scene>\n</COLLADA>\n");
        File.WriteAllText(path, sb.ToString());
    }
}
