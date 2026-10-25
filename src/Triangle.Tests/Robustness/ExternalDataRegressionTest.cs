using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using TriangleNet.Tests.Corpus;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Hard cases found with external data (see <see cref="CorpusTest"/>), kept as regression
    /// tests so they run without the external data: the public test fixtures of earcut,
    /// delaunator and poly2tri on which the original Triangle.NET throws, loops or produces an
    /// inconsistent mesh (see Data/README.md), and a nearly coincident back-and-forth border of
    /// the Natural Earth US counties.
    /// </summary>
    public class ExternalDataRegressionTest
    {
        private static readonly string DataDirectory =
            Path.Combine(Path.GetDirectoryName(typeof(ExternalDataRegressionTest).Assembly.Location), "Robustness", "Data", "Public");

        private static readonly TimeSpan Timeout = TimeSpan.FromSeconds(60);

        public static IEnumerable<TestCaseData> PublicFixtures()
        {
            foreach (var path in Directory.GetFiles(DataDirectory, "*.txt").OrderBy(p => p, StringComparer.Ordinal))
            {
                foreach (double minAngle in new[] { 0.0, 20.0, 30.0 })
                {
                    yield return new TestCaseData(Path.GetFileName(path), minAngle)
                        .SetName($"{Path.GetFileNameWithoutExtension(path)} q{minAngle}");
                }
            }
        }

        /// <summary>
        /// All polylines as segments, enclosed by a frame: the mesh is consistent, has no
        /// inverted triangles, covers the frame and contains all input segments.
        /// </summary>
        [TestCaseSource(nameof(PublicFixtures))]
        public void PublicFixtureGivesValidMesh(string file, double minAngle)
        {
            var c = CorpusCase.Load(Path.Combine(DataDirectory, file));

            var result = MeshAssert.Terminates(() => CorpusCheck.Run(c, minAngle, 20000), Timeout);

            Assert.That(result, Does.StartWith("OK"));
        }

        // From the Natural Earth 10m US counties (Chugach, Alaska): the border runs 5 km from A
        // to B and back to C, 5.7e-10 degrees (0.06 mm) from A-B (angle 6.5e-7 degrees at B).
        private static CorpusCase BacktrackingBorder() => new CorpusCase
        {
            Lines =
            {
                new List<(double X, double Y)>
                {
                    (-148.4572647779999, 60.841009833000044),
                    (-148.5234268869999, 60.84039948100008),
                    (-148.47338487789784, 60.84086112405067)
                }
            }
        };

        [Test]
        public void NearlyCoincidentBacktrackingBorderStopsAtTheSteinerLimit()
        {
            // No refinement can fill the sliver with 20 degree triangles: the Steiner point
            // limit has to stop it (the whole counties layer used to run into the corpus timeout).
            var result = MeshAssert.Terminates(() => CorpusCheck.Run(BacktrackingBorder(), 20, 20000), Timeout);

            Assert.That(result, Does.StartWith("OK"));
            Assert.That(result, Does.Contain(" limit=1 "));
        }

        [Test]
        public void NearlyCoincidentBacktrackingBorderIsMergedBySnapTolerance()
        {
            var result = MeshAssert.Terminates(() => CorpusCheck.Run(BacktrackingBorder(), 20, 20000, snapTolerance: 1e-9), Timeout);

            Assert.That(result, Does.StartWith("OK"));
            Assert.That(result, Does.Contain(" bad=0 "));
            Assert.That(result, Does.Contain(" limit=0 "));
        }
    }
}
