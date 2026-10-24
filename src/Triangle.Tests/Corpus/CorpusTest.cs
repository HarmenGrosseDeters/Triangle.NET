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

            Console.WriteLine("RESULT " + CorpusCheck.Run(c, minAngle, SteinerPoints));

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

            var jobs = files.SelectMany(f => new[] { (f, 0.0), (f, 20.0) });

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

            foreach (var f in failures)
            {
                TestContext.Out.WriteLine($"  {f.Name} (min angle {f.MinAngle}): {f.Result}");
            }

            Assert.That(failures, Is.Empty, string.Join("\n", failures.Take(50).Select(f => $"{f.Name} (min angle {f.MinAngle}): {f.Result}")));
        }

        private static double TileSize() =>
            double.TryParse(Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_TILE"), NumberStyles.Float, CultureInfo.InvariantCulture, out var s) ? s : 200.0;

        private static int MaxTiles() =>
            int.TryParse(Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_TILES"), out var n) ? n : 200;
    }
}
