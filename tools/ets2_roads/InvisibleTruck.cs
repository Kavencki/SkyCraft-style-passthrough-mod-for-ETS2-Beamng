// Builds the EuroBeamNG invisible-truck mod (.scs) for one player truck.
//
// ETS2 materials are shared, auto-generated files (/automat/xx/hash.mat) used by roads, buildings
// and trucks alike, so they cannot be overridden. Instead the truck's own models (.pmd) are
// overridden: same geometry (.pmg), every look's material list pointing at one material with the
// effect "eut2.shadowonly" - not drawn, still casting shadows. Light sources are separate, so
// the headlights still light the road. Collision (.pmc) is untouched.
using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using TruckLib;
using TruckLib.HashFs;

static class InvisibleTruck
{
    const string ShadowMat = "/eurobeamng/shadow_only.mat";
    // The render-path (".rfx") variant: every real truck material uses one; the plain
    // "eut2.shadowonly" crashed ETS2 while loading the truck (null shader entry).
    static string Effect = "eut2.shadowonly.rfx";

    // Every file below `dir` in any archive (directories walked through the archives' listings).
    public static IEnumerable<string> Walk(List<IHashFsReader> readers, string dir, HashSet<string> seen)
    {
        if (!seen.Add(dir)) yield break;
        var subdirs = new List<string>();
        var files = new List<string>();
        foreach (var r in readers)
        {
            try
            {
                var l = r.GetDirectoryListing(dir, false, true);
                subdirs.AddRange(l.Subdirectories);
                files.AddRange(l.Files);
            }
            catch { }
        }
        foreach (var f in files.Distinct()) yield return f;
        foreach (var s in subdirs.Distinct())
            foreach (var f in Walk(readers, s, seen)) yield return f;
    }

    public static int Build(List<IHashFsReader> readers, IFileSystem fs, string arg)
    {
        var parts = arg.Split(';');
        string truckId = parts[0].Trim();
        if (parts.Length > 2 && parts[2].Length > 0) Effect = parts[2];
        string outFile = parts.Length > 1 ? parts[1] : Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Euro Truck Simulator 2", "mod", "eurobeamng_invisible_truck.scs");

        // 1. The truck's definitions -> the models they use. (Not /def/vehicle/t_wheel: those are the
        // trailers' wheels - taking them made every trailer's wheels invisible.)
        var seen = new HashSet<string>();
        var defs = Walk(readers, $"/def/vehicle/truck/{truckId}", seen)
            .Where(f => f.EndsWith(".sii") || f.EndsWith(".sui")).ToList();
        if (defs.Count == 0) { Console.Error.WriteLine($"no definitions for truck '{truckId}'"); return 2; }
        var pmdRe = new Regex("\"(/vehicle/[^\"]+\\.pmd)\"");
        var models = new HashSet<string>();
        void ModelsOf(IEnumerable<string> files, HashSet<string> into)
        {
            foreach (var d in files)
            {
                if (!d.EndsWith(".sii") && !d.EndsWith(".sui")) continue;
                string text;
                try { text = fs.ReadAllText(d); } catch { continue; }
                foreach (Match m in pmdRe.Matches(text)) into.Add(m.Groups[1].Value);
            }
        }
        ModelsOf(defs, models);
        // 2. Every model in the truck's own model folders too (accessories referenced indirectly).
        var modelDirs = models.Where(m => m.StartsWith("/vehicle/truck/"))
            .Select(m => "/vehicle/truck/" + m.Split('/')[3]).Distinct().ToList();
        foreach (var md in modelDirs)
            foreach (var f in Walk(readers, md, seen))
                if (f.EndsWith(".pmd")) models.Add(f);
        // 3. The player's driver (and the steering wheel in their hands; AI drivers are /vehicle/driver/ai_*)
        // and the trailer cables' plugs (the cables themselves are drawn by the game, not models).
        // Hidden on purpose, even where other vehicles have them too (a trailer's own plugs: tiny).
        var onPurpose = new HashSet<string> { "/vehicle/driver/swheel.pmd" };
        foreach (var dir in new[] { "/vehicle/driver/man", "/vehicle/driver/woman", "/vehicle/share/models/cables_connectors" })
            foreach (var f in Walk(readers, dir, seen))
                if (f.EndsWith(".pmd")) onPurpose.Add(f);
        models.UnionWith(onPurpose);
        // 4. Never a model that trailers or traffic use too: it would vanish from them as well (all
        // trailers' wheels did, from /def/vehicle/t_wheel). Such a shared part of the truck stays
        // visible. (Not buses: they share only cab decorations, which would float where the
        // invisible truck is - hidden on buses instead.) ES_INVISIBLE_DEBUG=1 lists what is shared.
        var others = new HashSet<string>();
        foreach (var dir in new[] { "trailer", "trailer_defs", "trailer_owned", "trailer_dealer", "trailer_desktop", "trailer_wheel",
                                    "t_wheel", "ai", "ai_wheel", "car", "bus" })
        {
            var these = new HashSet<string>();
            ModelsOf(Walk(readers, "/def/vehicle/" + dir, seen), these);
            if (Environment.GetEnvironmentVariable("ES_INVISIBLE_DEBUG") != null)
                foreach (var m in these.Where(models.Contains)) Console.WriteLine($"  shared with {dir}: {m}");
            if (dir != "bus") others.UnionWith(these);
        }
        int shared = models.RemoveWhere(m => others.Contains(m) && !onPurpose.Contains(m));
        Console.WriteLine($"truck {truckId}: {defs.Count} definitions, {models.Count} models (folders: {string.Join(", ", modelDirs)}); " +
                          $"{shared} left visible (other vehicles use them too)");

        // 5. Rewrite the models, pack the mod.
        var tmp = Path.Combine(Path.GetTempPath(), "eurobeamng_invisible_" + Environment.ProcessId);
        if (Directory.Exists(tmp)) Directory.Delete(tmp, true);
        Directory.CreateDirectory(tmp);
        int done = 0, failed = 0;
        foreach (var path in models)
        {
            try
            {
                var model = TruckLib.Models.Model.Open(path, fs);
                foreach (var look in model.Looks)
                    for (int i = 0; i < look.Materials.Count; i++) look.Materials[i] = ShadowMat;
                var dir = Path.Combine(tmp, Path.GetDirectoryName(path.TrimStart('/')) ?? "");
                Directory.CreateDirectory(dir);
                // Only the descriptor (.pmd: looks, materials); the geometry (.pmg) stays the game's own.
                using (var w = new BinaryWriter(File.Create(Path.Combine(dir, Path.GetFileName(path)))))
                    model.SerializePmd(w);
                done++;
            }
            catch (Exception e) { failed++; Console.Error.WriteLine($"skip {path}: {e.Message}"); }
        }
        Directory.CreateDirectory(Path.Combine(tmp, "eurobeamng"));
        File.WriteAllText(Path.Combine(tmp, "eurobeamng", "shadow_only.mat"), $"effect : \"{Effect}\" {{\n}}\n");
        File.WriteAllText(Path.Combine(tmp, "manifest.sii"),
            "SiiNunit\n{\nmod_package : .package_name\n{\n" +
            "\tpackage_version: \"1.0\"\n" +
            $"\tdisplay_name: \"EuroBeamNG invisible truck ({truckId})\"\n" +
            "\tauthor: \"EuroBeamNG\"\n" +
            "\tcategory[]: \"truck\"\n" +
            "\tdescription_file: \"description.txt\"\n" +
            "}\n}\n");
        File.WriteAllText(Path.Combine(tmp, "description.txt"),
            "Makes the player truck invisible (shadow and lights stay) for EuroBeamNG, where a BeamNG car drives in ETS2.\n" +
            "Disable this mod to see your truck again.\n");
        Directory.CreateDirectory(Path.GetDirectoryName(outFile)!);
        if (File.Exists(outFile)) File.Delete(outFile);
        ZipFile.CreateFromDirectory(tmp, outFile, CompressionLevel.Optimal, false, Encoding.UTF8);
        Directory.Delete(tmp, true);
        Console.WriteLine($"models rewritten: {done}, skipped: {failed} -> {outFile}");
        BuildPlates(Path.Combine(Path.GetDirectoryName(outFile)!, "eurobeamng_invisible_plates.scs"));
        return done > 0 ? 0 : 1;
    }

    // A separate mod: the licence plates' material is one for every vehicle, so this hides the
    // plates of all vehicles (traffic too) - kept apart so it can be turned off on its own.
    static void BuildPlates(string outFile)
    {
        var tmp = Path.Combine(Path.GetTempPath(), "eurobeamng_plates_" + Environment.ProcessId);
        if (Directory.Exists(tmp)) Directory.Delete(tmp, true);
        Directory.CreateDirectory(Path.Combine(tmp, "material", "custom"));
        File.WriteAllText(Path.Combine(tmp, "material", "custom", "license_plate.mat"), $"effect : \"{Effect}\" {{\n}}\n");
        File.WriteAllText(Path.Combine(tmp, "manifest.sii"),
            "SiiNunit\n{\nmod_package : .package_name\n{\n" +
            "\tpackage_version: \"1.0\"\n" +
            "\tdisplay_name: \"EuroBeamNG invisible licence plates\"\n" +
            "\tauthor: \"EuroBeamNG\"\n" +
            "\tcategory[]: \"truck\"\n" +
            "\tdescription_file: \"description.txt\"\n" +
            "}\n}\n");
        File.WriteAllText(Path.Combine(tmp, "description.txt"),
            "Hides licence plates (of all vehicles: the plate material is shared) for EuroBeamNG.\n");
        if (File.Exists(outFile)) File.Delete(outFile);
        ZipFile.CreateFromDirectory(tmp, outFile, CompressionLevel.Optimal, false, Encoding.UTF8);
        Directory.Delete(tmp, true);
        Console.WriteLine($"licence plates mod -> {outFile}");
    }
}
