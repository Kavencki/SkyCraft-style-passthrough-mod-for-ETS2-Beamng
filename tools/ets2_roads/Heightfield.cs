// Heightfields: the drivable ground as a height grid (1 m) per tile, for BeamNG's TerrainBlock.
// The bridge stitches the tiles around the car into one block and swaps it in (a few ms, no
// collision rebuild) - the meshes then only keep what a height grid cannot show: walls, and the
// upper level where there are two (a bridge or flyover over a road, a road over a tunnel).
//
// Per sample (tile tx, tz; sample i, j at ETS2 x = tx*S + i, z = tz*S + j): the surfaces there,
// lowest first; the ground is the top of the lowest group (surfaces within LevelGap m of each
// other). A built triangle stays a mesh only where it is not that ground (more than KeepDy off).
//
// File (one per tile, in HeightfieldDir, named by content): "ESHF", int32 version 1, int32 tx,
// int32 tz, int32 size (samples per side), float32 base (m); size*size int16 heights in cm above
// base, row j = z, column i = x (short.MinValue: no ground); size*size bytes surface
// (0 asphalt, 1 grass, 255 none). hf.idx lists "tx tz file" per tile.
using System.Globalization;
using System.Numerics;
using System.Text;

sealed partial class TileSet
{
    public string? HeightfieldDir;
    public (double X, double Z, double Radius)? Area;   // the area generated (tiles.idx "#area")
    const float LevelGap = 0.7f, KeepDy = 0.05f;   // 2.5: a plate 1 m over a road became the ground
    public int HfTiles { get; private set; }
    public int HfWritten { get; private set; }
    public int TrianglesKept { get; private set; }
    public int TrianglesDropped { get; private set; }
    // ground per tile (for ProbeLines): NaN = none
    readonly Dictionary<(int, int), float[]> hfGround = new();

    static byte SurfaceOf(byte kind) => kind is KindTerrain or KindNet or KindGap ? (byte)1 : (byte)0;

    void ReduceToHeightfields()
    {
        int size = (int)Size;
        // Triangles by tile they overlap.
        var byTile = new Dictionary<(int, int), List<int>>();
        for (int i = 0; i < built.Count; i++)
        {
            var (a, b, c, _) = built[i];
            var (tx0, tz0) = TileOf(MathF.Min(a.X, MathF.Min(b.X, c.X)), MathF.Min(a.Z, MathF.Min(b.Z, c.Z)));
            var (tx1, tz1) = TileOf(MathF.Max(a.X, MathF.Max(b.X, c.X)), MathF.Max(a.Z, MathF.Max(b.Z, c.Z)));
            if ((long)(tx1 - tx0 + 1) * (tz1 - tz0 + 1) > 64) continue;
            for (int tx = tx0; tx <= tx1; tx++)
                for (int tz = tz0; tz <= tz1; tz++)
                {
                    if (!byTile.TryGetValue((tx, tz), out var l)) byTile[(tx, tz)] = l = new List<int>();
                    l.Add(i);
                }
        }
        var keep = new bool[built.Count];
        var seen = new bool[built.Count];   // had a sample inside
        int n = size * size;
        Directory.CreateDirectory(HeightfieldDir!);
        var idxLines = new List<(int Tx, int Tz, string Name)>();
        var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        int hfTiles = 0, hfWritten = 0;
        // Tiles in parallel (each is independent; half the CPU threads, the games keep the rest).
        var opts = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, Environment.ProcessorCount / 2) };
        Parallel.ForEach(byTile, opts, () => (Low: new float[n], Top: new float[n], Surf: new byte[n]), (entry, _, arrays) =>
        {
            var ((tx, tz), list) = entry;
            var (low, top, surf) = arrays;
            int ox = tx * size, oz = tz * size;
            Array.Fill(low, float.PositiveInfinity);
            Array.Fill(top, float.NaN);
            Array.Fill(surf, (byte)255);
            // The samples inside a triangle: (sample index, height there).
            IEnumerable<(int K, float Y)> Samples((Vector3 A, Vector3 B, Vector3 C, byte Kind) t)
            {
                var (a, b, c, _) = t;
                float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
                if (MathF.Abs(det) < 1e-9f) yield break;
                int x0 = Math.Max(0, (int)MathF.Ceiling(MathF.Min(a.X, MathF.Min(b.X, c.X))) - ox);
                int x1 = Math.Min(size - 1, (int)MathF.Floor(MathF.Max(a.X, MathF.Max(b.X, c.X))) - ox);
                int z0 = Math.Max(0, (int)MathF.Ceiling(MathF.Min(a.Z, MathF.Min(b.Z, c.Z))) - oz);
                int z1 = Math.Min(size - 1, (int)MathF.Floor(MathF.Max(a.Z, MathF.Max(b.Z, c.Z))) - oz);
                for (int z = z0; z <= z1; z++)
                    for (int x = x0; x <= x1; x++)
                    {
                        float px = ox + x, pz = oz + z;
                        float l1 = ((b.Z - c.Z) * (px - c.X) + (c.X - b.X) * (pz - c.Z)) / det;
                        float l2 = ((c.Z - a.Z) * (px - c.X) + (a.X - c.X) * (pz - c.Z)) / det;
                        float l3 = 1 - l1 - l2;
                        const float e = -0.001f;
                        if (l1 < e || l2 < e || l3 < e) continue;
                        yield return (z * size + x, l1 * a.Y + l2 * b.Y + l3 * c.Y);
                    }
            }
            // 1: the lowest surface at each sample.
            foreach (var i in list)
                foreach (var (k, y) in Samples(built[i]))
                    if (y < low[k]) low[k] = y;
            // 2: the top of the lowest group = the ground.
            foreach (var i in list)
                foreach (var (k, y) in Samples(built[i]))
                    if (y <= low[k] + LevelGap && (float.IsNaN(top[k]) || y > top[k]))
                    {
                        top[k] = y;
                        surf[k] = SurfaceOf(built[i].Kind);
                    }
            // The tile's file.
            float min = float.PositiveInfinity;
            for (int k = 0; k < n; k++) if (!float.IsNaN(top[k]) && top[k] < min) min = top[k];
            if (float.IsInfinity(min)) return arrays;   // no ground in this tile
            float baseH = MathF.Floor(min) - 1;
            var ground = (float[])top.Clone();
            lock (hfGround) hfGround[(tx, tz)] = ground;
            var bytes = new byte[24 + n * 3];
            Encoding.ASCII.GetBytes("ESHF").CopyTo(bytes, 0);
            BitConverter.GetBytes(1).CopyTo(bytes, 4);
            BitConverter.GetBytes(tx).CopyTo(bytes, 8);
            BitConverter.GetBytes(tz).CopyTo(bytes, 12);
            BitConverter.GetBytes(size).CopyTo(bytes, 16);
            BitConverter.GetBytes(baseH).CopyTo(bytes, 20);
            for (int k = 0; k < n; k++)
            {
                short h = float.IsNaN(top[k]) ? short.MinValue : (short)Math.Clamp(MathF.Round((top[k] - baseH) * 100), short.MinValue + 1, short.MaxValue);
                bytes[24 + k * 2] = (byte)(h & 0xFF);
                bytes[24 + k * 2 + 1] = (byte)((h >> 8) & 0xFF);
                bytes[24 + n * 2 + k] = surf[k];
            }
            ulong hash = 1469598103934665603UL;
            foreach (var bt in bytes) { hash ^= bt; hash *= 1099511628211UL; }
            var name = $"hf_{tx}_{tz}_{hash:x16}.bin";
            var file = Path.Combine(HeightfieldDir!, name);
            bool written = false;
            if (!File.Exists(file)) { File.WriteAllBytes(file, bytes); written = true; }
            lock (idxLines)
            {
                names.Add(name);
                idxLines.Add((tx, tz, name));
                hfTiles++;
                if (written) hfWritten++;
            }
            return arrays;
        }, _ => { });
        HfTiles = hfTiles;
        HfWritten = hfWritten;
        var idx = new StringBuilder();
        foreach (var (tx, tz, name) in idxLines.OrderBy(l => l.Tx).ThenBy(l => l.Tz)) idx.AppendLine($"{tx} {tz} {name}");
        File.WriteAllText(Path.Combine(HeightfieldDir!, "hf.idx"), idx.ToString());
        // Old heightfield files (not in this area, untouched for 6 hours).
        var cutoff = DateTime.UtcNow.AddHours(-6);
        foreach (var f in Directory.GetFiles(HeightfieldDir!, "hf_*.bin"))
            try { if (!names.Contains(Path.GetFileName(f)) && File.GetLastWriteTimeUtc(f) < cutoff) File.Delete(f); }
            catch (IOException) { } catch (UnauthorizedAccessException) { }

        // 3: a triangle stays a mesh where the ground grid, as BeamNG interpolates it (bilinear
        // between the 1 m samples), is not on it: checked every 0.5 m over its whole area, not just
        // at the samples (next to a cliff in the grid - an embankment, a bridge end - the grid dips
        // between samples). Under the ground: never reached, dropped. (In parallel: the grid is only
        // read here.)
        Parallel.For(0, built.Count, opts, i =>
        {
            var (a, b, c, _) = built[i];
            float det = (b.Z - c.Z) * (a.X - c.X) + (c.X - b.X) * (a.Z - c.Z);
            if (MathF.Abs(det) < 1e-9f) return;
            float x0 = MathF.Min(a.X, MathF.Min(b.X, c.X)), x1 = MathF.Max(a.X, MathF.Max(b.X, c.X));
            float z0 = MathF.Min(a.Z, MathF.Min(b.Z, c.Z)), z1 = MathF.Max(a.Z, MathF.Max(b.Z, c.Z));
            if ((x1 - x0) * (z1 - z0) > 250000) { keep[i] = true; return; }
            for (float z = MathF.Ceiling(z0 * 2) / 2; z <= z1 && !keep[i]; z += 0.5f)
                for (float x = MathF.Ceiling(x0 * 2) / 2; x <= x1; x += 0.5f)
                {
                    float l1 = ((b.Z - c.Z) * (x - c.X) + (c.X - b.X) * (z - c.Z)) / det;
                    float l2 = ((c.Z - a.Z) * (x - c.X) + (a.X - c.X) * (z - c.Z)) / det;
                    float l3 = 1 - l1 - l2;
                    if (l1 < -0.001f || l2 < -0.001f || l3 < -0.001f) continue;
                    seen[i] = true;
                    float y = l1 * a.Y + l2 * b.Y + l3 * c.Y, g = GroundAt(x, z);
                    if (float.IsNaN(g) || y > g + KeepDy) { keep[i] = true; break; }
                }
        });

        // The meshes: kept triangles (and those too small to hold a sample) plus every wall.
        Tiles.Clear();
        for (int i = 0; i < built.Count; i++)
        {
            if (seen[i] && !keep[i]) { TrianglesDropped++; continue; }
            TrianglesKept++;
            var (a, b, c, _) = built[i];
            AddToMeshOnly(a, b, c, false);
        }
        foreach (var (a, b, c) in builtWalls) AddToMeshOnly(a, b, c, true);
        meshKeep = keep;
        meshSeen = seen;
    }
    bool[]? meshKeep, meshSeen;

    // Into the tile meshes without recording (built stays the full surface for the probes).
    void AddToMeshOnly(Vector3 a, Vector3 b, Vector3 c, bool doubleSided)
    {
        var ctr = (a + b + c) / 3f;
        var t = TileOf(ctr.X, ctr.Z);
        if (!Tiles.TryGetValue(t, out var mb)) Tiles[t] = mb = new MeshBuilder();
        var o = Origin(t);
        Vector3 L(Vector3 p) => new(p.X - o.X, -(p.Z - o.Y), p.Y);
        mb.Tri(L(a), L(b), L(c), doubleSided);
    }

    // The heightfield ground at p (bilinear, as BeamNG's TerrainBlock reports it), NaN if none.
    float GroundAt(float x, float z)
    {
        int size = (int)Size;
        int ix = (int)MathF.Floor(x), iz = (int)MathF.Floor(z);
        float fx = x - ix, fz = z - iz;
        float S(int sx, int sz)
        {
            var (tx, tz) = ((int)MathF.Floor(sx / (float)size), (int)MathF.Floor(sz / (float)size));
            if (!hfGround.TryGetValue((tx, tz), out var g)) return float.NaN;
            return g[(sz - tz * size) * size + (sx - tx * size)];
        }
        float h00 = S(ix, iz), h10 = S(ix + 1, iz), h01 = S(ix, iz + 1), h11 = S(ix + 1, iz + 1);
        if (float.IsNaN(h00) || float.IsNaN(h10) || float.IsNaN(h01) || float.IsNaN(h11)) return float.NaN;
        return (h00 * (1 - fx) + h10 * fx) * (1 - fz) + (h01 * (1 - fx) + h11 * fx) * fz;
    }
}
