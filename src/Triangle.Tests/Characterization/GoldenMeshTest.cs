using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Characterization
{
    /// <summary>
    /// Pins the exact output (geometry of all triangles and subsegments) of every scenario of
    /// <see cref="MeshCases"/>. Performance work that is meant to be behavior-neutral must
    /// keep all fingerprints. If a change intentionally alters the output, check the
    /// <see cref="InvariantTest"/> results and quality statistics, then regenerate the file:
    ///
    ///   TRIANGLE_UPDATE_GOLDEN=1 dotnet test --filter Name~Golden
    /// </summary>
    /// <remarks>
    /// The fingerprints depend on IEEE double arithmetic only (no transcendental functions in
    /// the meshing code paths that decide vertex positions, except the trigonometry of the
    /// aCute refinement), so they are stable across x64 platforms.
    /// </remarks>
    [Category("Characterization")]
    public class GoldenMeshTest
    {
        public static IEnumerable<TestCaseData> Cases() =>
            MeshCases.All().Select(c => new TestCaseData(c).SetName("Golden(" + c.Name + ")"));

        private static readonly object Lock = new object();

        [TestCaseSource(nameof(Cases))]
        public void Golden(MeshCases.Case c)
        {
            var mesh = MeshAssert.Terminates(() => c.Run(c.Input()), TimeSpan.FromSeconds(120));
            var actual = MeshInvariants.Fingerprint(mesh);

            var golden = Load();

            if (Environment.GetEnvironmentVariable("TRIANGLE_UPDATE_GOLDEN") == "1")
            {
                lock (Lock)
                {
                    golden = Load();
                    golden[c.Name] = actual;
                    File.WriteAllLines(GoldenFile(), golden.OrderBy(g => g.Key, StringComparer.Ordinal).Select(g => g.Key + " " + g.Value));
                }

                return;
            }

            Assert.That(golden.ContainsKey(c.Name), Is.True, $"No fingerprint for '{c.Name}' (run with TRIANGLE_UPDATE_GOLDEN=1).");
            Assert.That(actual, Is.EqualTo(golden[c.Name]), "Mesh differs from the pinned result.");
        }

        private static Dictionary<string, string> Load()
        {
            var file = GoldenFile();

            return File.Exists(file)
                ? File.ReadAllLines(file).Where(l => l.Length > 0).ToDictionary(l => l.Substring(0, l.IndexOf(' ')), l => l.Substring(l.IndexOf(' ') + 1))
                : new Dictionary<string, string>();
        }

        private static string GoldenFile([CallerFilePath] string path = "") =>
            Path.Combine(Path.GetDirectoryName(path), "golden.txt");
    }
}
