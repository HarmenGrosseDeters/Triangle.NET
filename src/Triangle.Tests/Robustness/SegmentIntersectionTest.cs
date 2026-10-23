using NUnit.Framework;
using System.Collections.Generic;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Input segments that cross each other. The mesher inserts a vertex at each intersection,
    /// but the intersection point is rounded to double precision and thus doesn't lie exactly on
    /// either segment. With three or more segments crossing in a small region, this breaks the
    /// topology.
    /// </summary>
    /// <remarks>
    /// All of these failures can be reproduced with the original C code of Triangle 1.6, i.e. they
    /// are not porting bugs. Workaround: compute all intersections before triangulating
    /// (noding), so the mesher never has to split crossing segments.
    /// </remarks>
    public class SegmentIntersectionTest
    {
        [Test, Category(KnownBug.Category)]
        [Description("Severity: high. Three crossing segments throw 'Topological inconsistency after splitting a segment'.")]
        public void ThreeCrossingSegmentsThrow()
        {
            var poly = TestData.SquareWithSegments((9, 5, 5, 7), (7, 7, 4, 1), (6, 5, 3, 10));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: critical. Three crossing segments silently produce a mesh with inverted triangles (no exception).")]
        [TestCase(2, 4, 4, 8, 1, 5, 6, 6, 1, 7, 5, 3)]
        [TestCase(2, 5, 9, 5, 6, 3, 10, 10, 8, 7, 5, 0)]
        [TestCase(5, 0, 1, 10, 7, 7, 0, 7, 3, 3, 8, 8)]
        public void ThreeCrossingSegmentsProduceInvalidMesh(
            double x1, double y1, double x2, double y2,
            double x3, double y3, double x4, double y4,
            double x5, double y5, double x6, double y6)
        {
            var poly = TestData.SquareWithSegments((x1, y1, x2, y2), (x3, y3, x4, y4), (x5, y5, x6, y6));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: critical. Five crossing segments cause an infinite recursion (StackOverflowException kills the process).")]
        public void FiveCrossingSegmentsCrashProcess()
        {
            IsolatedProcess.AssertSucceeds(nameof(IsolatedScenarios.FiveCrossingSegments));
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: high. Fuzzing with random integer segments: about 20% of the inputs fail.")]
        public void RandomIntegerSegments()
        {
            var failures = new List<string>();

            for (int seed = 0; seed < 200; seed++)
            {
                var poly = TestData.SquareWithSegments(TestData.RandomIntegerSegments(seed, 15));

                try
                {
                    var mesh = (Mesh)MeshAssert.Terminates(() => poly.Triangulate());

                    if (!MeshValidator.IsConsistent(mesh))
                    {
                        failures.Add($"seed {seed}: inconsistent mesh");
                    }
                }
                catch (System.Exception e)
                {
                    failures.Add($"seed {seed}: {e.GetBaseException().Message}");
                }
            }

            Assert.That(failures, Is.Empty, $"{failures.Count} of 200 inputs failed:\n" + string.Join("\n", failures));
        }

        [Test]
        public void RandomCrossingSegments([Values(10, 50)] int count)
        {
            // Random (non-integer) coordinates are less degenerate and work.
            var poly = new Polygon();
            poly.Add(TestData.Square(-1, -1, 102));

            var random = new System.Random(42);

            for (int i = 0; i < count; i++)
            {
                poly.Add(new Segment(
                    new Vertex(random.NextDouble() * 100, random.NextDouble() * 100),
                    new Vertex(random.NextDouble() * 100, random.NextDouble() * 100)), true);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments, 1e-7);
            MeshAssert.HasArea(mesh, 102 * 102);
        }

        [Test]
        public void TwoCrossingSegments()
        {
            var poly = TestData.SquareWithSegments((1, 1, 9, 9), (1, 9, 9, 1));

            var mesh = poly.Triangulate(new QualityOptions { MinimumAngle = 25 });

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);
        }
    }
}
