using NUnit.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Corpus
{
    /// <summary>
    /// Robustness tests with external data. The tests are explicit (not part of the regular
    /// test run) and skipped if the data isn't available. Each case is triangulated in a child
    /// process (with timeout), with and without quality refinement.
    /// </summary>
    /// <remarks>
    /// Environment variables:
    ///   TRIANGLE_GEOS_TESTS   directory tests/xmltester/tests of a GEOS checkout
    ///                         (https://github.com/libgeos/geos)
    ///   TRIANGLE_NRW_ALKIS    GeoJSON file with ALKIS parcels (ETRS89/UTM32), e.g. from
    ///                         https://www.opengeodata.nrw.de (Liegenschaftskataster) or the
    ///                         open data portals of NRW cities
    ///   TRIANGLE_NRW_DGM      DGM1 xyz(.gz) tile from https://www.opengeodata.nrw.de (dgm1_xyz)
    ///   TRIANGLE_CORPUS_TILES maximum number of tiles (default 200)
    ///   TRIANGLE_CORPUS_TILE  tile size in meters (default 200)
    ///   TRIANGLE_CORPUS_SNAP  snap tolerance (ConstraintOptions.SnapTolerance, default 0)
    ///   TRIANGLE_CORPUS_GRID  output grid (ConstraintOptions.GridSize, default 0)
    ///   TRIANGLE_CORPUS_WEDGE wedge tolerance (ConstraintOptions.WedgeTolerance, default 0)
    ///   TRIANGLE_CORPUS_ROUND count triangles flipped by rounding to this grid (default 0 = off,
    ///                         the output grid if set)
    ///   TRIANGLE_CORPUS_ANGLES comma separated minimum angles (default 0,20; 0 = no quality)
    ///
    /// Run: dotnet test --filter TestCategory=Corpus
    /// </remarks>
    [Category("Corpus"), Explicit("Requires external data, see remarks.")]
    public class CorpusTest
    {
        private static readonly TimeSpan Timeout = TimeSpan.FromSeconds(120);

        private const int SteinerPoints = 100000;

        [Test]
        public void GeosXmlTestSuite()
        {
            var directory = Environment.GetEnvironmentVariable("TRIANGLE_GEOS_TESTS");

            if (string.IsNullOrEmpty(directory) || !Directory.Exists(directory))
            {
                Assert.Ignore("TRIANGLE_GEOS_TESTS not set.");
            }

            RunCases(CorpusReaders.ReadGeosXml(directory));
        }

        [Test]
        public void NrwParcels()
        {
            var path = Environment.GetEnvironmentVariable("TRIANGLE_NRW_ALKIS");

            if (string.IsNullOrEmpty(path) || !File.Exists(path))
            {
                Assert.Ignore("TRIANGLE_NRW_ALKIS not set.");
            }

            var rings = CorpusReaders.ReadGeoJsonRings(path);

            RunCases(CorpusReaders.Tiles(Path.GetFileName(path), rings, TileSize(), MaxTiles()));
        }

        [Test]
        public void NrwTerrainWithParcelBoundaries()
        {
            var dgm = Environment.GetEnvironmentVariable("TRIANGLE_NRW_DGM");

            if (string.IsNullOrEmpty(dgm) || !File.Exists(dgm))
            {
                Assert.Ignore("TRIANGLE_NRW_DGM not set.");
            }

            var alkis = Environment.GetEnvironmentVariable("TRIANGLE_NRW_ALKIS");
            var rings = !string.IsNullOrEmpty(alkis) && File.Exists(alkis) ? CorpusReaders.ReadGeoJsonRings(alkis) : null;

            // Use a few windows of the tile: grid points (1 m spacing, highly cocircular) and,
            // if available, parcel boundaries as breaklines.
            var all = CorpusReaders.ReadXyz(dgm, double.MinValue, double.MinValue, double.MaxValue, double.MaxValue);
            double left = all.Min(p => p.X), bottom = all.Min(p => p.Y);
            double size = TileSize();

            var cases = new List<CorpusCase>();

            for (int i = 0; i < 4; i++)
            {
                double x = left + i * 2.5 * size, y = bottom + i * 2.5 * size;
                var c = new CorpusCase { Name = $"{Path.GetFileName(dgm)} window {x:F0}/{y:F0}" };

                c.Points.AddRange(all.Where(p => p.X >= x && p.X <= x + size && p.Y >= y && p.Y <= y + size));

                if (rings != null)
                {
                    c.Lines.AddRange(rings.Where(r => r.Any(p => p.X >= x && p.X <= x + size && p.Y >= y && p.Y <= y + size)));
                }

                if (c.Points.Count > 0) cases.Add(c);
            }

            RunCases(cases);
        }

        /// <summary>
        /// Entry point of the child process: corpus-case &lt;file&gt; &lt;minimum angle&gt;
        /// </summary>
        public static int RunCase(string[] args)
        {
            var c = CorpusCase.Load(args[1]);
            double minAngle = double.Parse(args[2], CultureInfo.InvariantCulture);

            double snap = args.Length > 3 ? double.Parse(args[3], CultureInfo.InvariantCulture) : SnapTolerance();

            Console.WriteLine("RESULT " + CorpusCheck.Run(c, minAngle, SteinerPoints, snap, Grid("TRIANGLE_CORPUS_GRID"), Grid("TRIANGLE_CORPUS_ROUND"), Grid("TRIANGLE_CORPUS_WEDGE")));

            return 0;
        }

        private static void RunCases(IEnumerable<CorpusCase> cases)
        {
            var directory = Path.Combine(Path.GetTempPath(), "triangle-corpus-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(directory);

            var files = new List<(string Name, string File)>();

            foreach (var c in cases)
            {
                var file = Path.Combine(directory, files.Count + ".txt");
                c.Save(file);
                files.Add((c.Name, file));
            }

            var results = new ConcurrentBag<(string Name, double MinAngle, string Result)>();

            var jobs = files.SelectMany(f => Angles().Select(a => (f, a)));

            Parallel.ForEach(jobs, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, job =>
            {
                var ((name, file), minAngle) = job;

                var run = IsolatedProcess.Run(new[] { "corpus-case", file, minAngle.ToString(CultureInfo.InvariantCulture) }, Timeout);

                string result = run.TimedOut ? "TIMEOUT"
                    : run.Output.Split('\n').Select(l => l.Trim()).LastOrDefault(l => l.StartsWith("RESULT "))?.Substring(7)
                      ?? $"CRASH (exit code {run.ExitCode})";

                results.Add((name, minAngle, result));
            });

            Directory.Delete(directory, true);

            var failures = results.Where(r => !r.Result.StartsWith("OK") && !r.Result.StartsWith("SKIPPED"))
                .OrderBy(r => r.Name, StringComparer.Ordinal).ThenBy(r => r.MinAngle).ToList();

            TestContext.Out.WriteLine($"{files.Count} cases, {results.Count} runs, {failures.Count} failures");

            foreach (var g in results.GroupBy(r => (r.MinAngle, r.Result.Split(' ')[0])).OrderBy(g => g.Key))
            {
                TestContext.Out.WriteLine($"  min angle {g.Key.MinAngle}: {g.Key.Item2} x {g.Count()}");
            }

            foreach (var g in results.Where(r => r.Result.StartsWith("OK")).GroupBy(r => r.MinAngle).OrderBy(g => g.Key))
            {
                var stats = g.Select(r => (r.Name, Stats: Stats(r.Result))).ToList();

                TestContext.Out.WriteLine($"  min angle {g.Key}: total {stats.Sum(r => r.Stats["ms"]) / 1000:F1} s, " +
                    $"{stats.Sum(r => r.Stats["triangles"])} triangles, Steiner limit reached x {stats.Count(r => r.Stats["limit"] > 0)}, " +
                    $"bad triangles {stats.Sum(r => r.Stats["bad"])} ({100.0 * stats.Sum(r => r.Stats["bad"]) / Math.Max(1, stats.Sum(r => r.Stats["triangles"])):F2} %) in {stats.Count(r => r.Stats["bad"] > 0)} cases, " +
                    $"flipped by rounding {stats.Sum(r => r.Stats["flip"])} in {stats.Count(r => r.Stats["flip"] > 0)} cases, " +
                    $"bad after rounding {stats.Sum(r => r.Stats["badr"])} ({100.0 * stats.Sum(r => r.Stats["badr"]) / Math.Max(1, stats.Sum(r => r.Stats["triangles"])):F2} %), " +
                    $"bad by class: small input angle {stats.Sum(r => r.Stats["ba"])}, grid floor {stats.Sum(r => r.Stats["bf"])}, narrow strip {stats.Sum(r => r.Stats["bn"])}, at segment {stats.Sum(r => r.Stats["bs"])}, interior {stats.Sum(r => r.Stats["bi"])}");

                foreach (var r in stats.OrderByDescending(r => r.Stats["ms"]).Take(5))
                {
                    TestContext.Out.WriteLine($"    slow: {r.Name} {r.Stats["ms"]} ms, {r.Stats["triangles"]} triangles, bad {r.Stats["bad"]}");
                }

                foreach (var r in stats.OrderByDescending(r => r.Stats["bad"]).Take(5))
                {
                    TestContext.Out.WriteLine($"    worst: {r.Name} bad {r.Stats["bad"]} of {r.Stats["triangles"]} triangles, {r.Stats["ms"]} ms");
                }
            }

            foreach (var f in failures)
            {
                TestContext.Out.WriteLine($"  {f.Name} (min angle {f.MinAngle}): {f.Result}");
            }

            Assert.That(failures, Is.Empty, string.Join("\n", failures.Take(50).Select(f => $"{f.Name} (min angle {f.MinAngle}): {f.Result}")));
        }

        private static Dictionary<string, long> Stats(string result) =>
            result.Split(' ').Select(t => t.Split('=')).Where(t => t.Length == 2)
                .ToDictionary(t => t[0], t => long.Parse(t[1], CultureInfo.InvariantCulture));

        private static double[] Angles() =>
            (Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_ANGLES") ?? "0,20").Split(',')
                .Select(a => double.Parse(a, CultureInfo.InvariantCulture)).ToArray();

        private static double SnapTolerance() =>
            double.TryParse(Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_SNAP"), NumberStyles.Float, CultureInfo.InvariantCulture, out var s) ? s : 0.0;

        private static double Grid(string variable) =>
            double.TryParse(Environment.GetEnvironmentVariable(variable), NumberStyles.Float, CultureInfo.InvariantCulture, out var s) ? s : 0.0;

        private static double TileSize() =>
            double.TryParse(Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_TILE"), NumberStyles.Float, CultureInfo.InvariantCulture, out var s) ? s : 200.0;

        private static int MaxTiles() =>
            int.TryParse(Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_TILES"), out var n) ? n : 200;
    }
}
