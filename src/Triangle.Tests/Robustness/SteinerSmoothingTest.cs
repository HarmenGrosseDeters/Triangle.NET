using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Tests.Characterization;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Smoothing of the Steiner points after refinement (<see cref="QualityOptions.SmoothingSteps"/>):
    /// free Steiner points move towards the optimal Delaunay triangulation position if that makes
    /// the changed triangles better. Input vertices, segments and points on segments are not changed.
    /// </summary>
    public class SteinerSmoothingTest
    {
        private static void Smooth(QualityOptions q) => q.SmoothingSteps = 3;

        private static void SmoothAndReduce(QualityOptions q)
        {
            q.SmoothingSteps = 3;
            q.ReduceSteinerPoints = true;
        }

        public static IEnumerable<TestCaseData> QualityCases() =>
            SteinerReductionTest.QualityCases();

        public static IEnumerable<TestCaseData> DelaunayCases() =>
            SteinerReductionTest.DelaunayCases();

        [TestCaseSource(nameof(DelaunayCases))]
        public void SmoothedMeshKeepsAllInvariants(MeshCases.Case c)
        {
            AssertInvariants(c.With("smooth", Smooth), c);
        }

        [TestCaseSource(nameof(DelaunayCases))]
        public void SmoothedAndReducedMeshKeepsAllInvariants(MeshCases.Case c)
        {
            AssertInvariants(c.With("smooth-reduce", SmoothAndReduce), c);
        }

        [TestCaseSource(nameof(QualityCases))]
        public void SmoothingOnlyMovesFreeSteinerPointsAndNeverMakesTheMeshWorse(MeshCases.Case c)
        {
            var plain = MeshAssert.Terminates(() => c.Run(c.Input()), TimeSpan.FromSeconds(120));
            var smoothed = MeshAssert.Terminates(() => c.With("smooth", Smooth).Run(c.Input()), TimeSpan.FromSeconds(120));

            Assert.That(smoothed.Vertices.Count, Is.EqualTo(plain.Vertices.Count));
            Assert.That(smoothed.Triangles.Count, Is.EqualTo(plain.Triangles.Count));

            var smoothedFixed = smoothed.Vertices.Where(v => v.Type != VertexType.FreeVertex).Select(v => (v.X, v.Y)).OrderBy(p => p);
            var plainFixed = plain.Vertices.Where(v => v.Type != VertexType.FreeVertex).Select(v => (v.X, v.Y)).OrderBy(p => p);

            Assert.That(smoothedFixed, Is.EqualTo(plainFixed), "input or segment vertex changed");

            // Each move makes the changed triangles better, so the smallest angle can't decrease
            // and no more triangles violate the minimum angle.
            if (c.MinAngle > 0)
            {
                Assert.That(smoothed.Triangles.Min(t => MeshInvariants.MinimumAngle(t)),
                    Is.GreaterThanOrEqualTo(plain.Triangles.Min(t => MeshInvariants.MinimumAngle(t)) - 1e-9));
                Assert.That(Bad(smoothed, c.MinAngle), Is.LessThanOrEqualTo(Bad(plain, c.MinAngle)));
            }

            if (c.MaxArea > 0)
            {
                Assert.That(smoothed.Triangles.Max(t => MeshInvariants.Area(t)), Is.LessThanOrEqualTo(c.MaxArea));
            }
        }

        [TestCase("herne_373600_5709000/q30-snap1cm")]
        [TestCase("herne_373600_5709000/q30-grid1cm-wedge3cm")]
        [TestCase("points-random/q30")]
        public void SmoothingImprovesTheAverageQuality(string name)
        {
            var c = MeshCases.All().Concat(MeshCases.GridCases()).Single(x => x.Name == name);

            var plain = c.Run(c.Input());
            var smoothed = c.With("smooth", Smooth).Run(c.Input());

            double before = plain.Triangles.Average(t => MeshInvariants.MinimumAngle(t));
            double after = smoothed.Triangles.Average(t => MeshInvariants.MinimumAngle(t));

            TestContext.WriteLine($"{name}: mean minimum angle {before:F2} -> {after:F2}, " +
                $"bad {Bad(plain, c.MinAngle)} -> {Bad(smoothed, c.MinAngle)}");

            Assert.That(after, Is.GreaterThan(before));
        }

        [Test]
        public void SmoothingWithReductionRemovesMoreSteinerPoints()
        {
            var c = MeshCases.GridCases().Single(x => x.Name == "herne_373600_5709000/q30-grid1cm-wedge3cm");

            var reduced = c.With("reduce", q => q.ReduceSteinerPoints = true).Run(c.Input());
            var both = c.With("smooth-reduce", SmoothAndReduce).Run(c.Input());

            Assert.That(both.Vertices.Count, Is.LessThan(reduced.Vertices.Count));
            Assert.That(Bad(both, c.MinAngle), Is.LessThanOrEqualTo(Bad(reduced, c.MinAngle)));
        }

        [Test]
        public void GridModeKeepsVerticesOnTheGrid()
        {
            var c = MeshCases.GridCases().First().With("smooth-reduce", SmoothAndReduce);

            var mesh = c.Run(c.Input());

            Assert.That(mesh.Vertices.All(v => v.X == GridSnapper.Round(v.X, 0.01) && v.Y == GridSnapper.Round(v.Y, 0.01)), Is.True);
        }

        [Test]
        public void UserTestIsRespected()
        {
            bool Large(ITriangle t, double area) => area > 0.5;

            var mesh = (Mesh)TestData.SquareWithSegments((1, 1, 9, 2), (2, 8, 8, 7)).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 30, UserTest = Large, SmoothingSteps = 5 });

            MeshAssert.IsValid(mesh);

            Assert.That(mesh.Triangles.Max(t => MeshInvariants.Area(t)), Is.LessThanOrEqualTo(0.5));
        }

        [Test]
        public void SmoothingIsDeterministic()
        {
            var c = MeshCases.All().Single(x => x.Name == "herne_371800_5710600/q30-snap1cm").With("smooth-reduce", SmoothAndReduce);

            Assert.That(MeshInvariants.Fingerprint(c.Run(c.Input())), Is.EqualTo(MeshInvariants.Fingerprint(c.Run(c.Input()))));
        }

        private static void AssertInvariants(MeshCases.Case configured, MeshCases.Case c)
        {
            var input = configured.Input();
            var mesh = MeshAssert.Terminates(() => configured.Run(input), TimeSpan.FromSeconds(120));

            var reference = c.Input();
            var plain = reference.Segments.Count == 0
                ? (Mesh)new GenericMesher().Triangulate(reference.Points)
                : (Mesh)reference.Triangulate(new ConstraintOptions { SnapTolerance = c.Snap });
            double area = plain.Triangles.Sum(t => MeshInvariants.Area(t));

            var errors = MeshInvariants.Check(mesh, input, configured, area);

            Assert.That(errors, Is.Empty, string.Join(", ", errors));
        }

        private static int Bad(Mesh mesh, double minAngle) =>
            mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < minAngle - 1e-9);
    }
}
