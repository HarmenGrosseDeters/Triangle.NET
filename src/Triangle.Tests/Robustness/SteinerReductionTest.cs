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
    /// Removing redundant Steiner points after refinement (<see cref="QualityOptions.ReduceSteinerPoints"/>):
    /// a free Steiner point is deleted if all triangles of the retriangulated cavity still meet the
    /// quality constraints. Input vertices, segments and points on segments are never touched.
    /// </summary>
    public class SteinerReductionTest
    {
        private static void Reduce(QualityOptions q) => q.ReduceSteinerPoints = true;

        public static IEnumerable<TestCaseData> QualityCases() => Select(MeshCases.All().Concat(MeshCases.GridCases()));

        // With an output grid the mesh isn't exactly Delaunay (see MeshCases.GridCases).
        public static IEnumerable<TestCaseData> DelaunayCases() => Select(MeshCases.All());

        private static IEnumerable<TestCaseData> Select(IEnumerable<MeshCases.Case> cases) =>
            cases.Where(c => c.RunWith != null && (c.MinAngle > 0 || c.MaxArea > 0))
                .Select(c => new TestCaseData(c).SetName("{m}(" + c.Name + ")"));

        [TestCaseSource(nameof(DelaunayCases))]
        public void ReducedMeshKeepsAllInvariants(MeshCases.Case c)
        {
            var reduced = c.With("reduce", Reduce);

            var input = reduced.Input();
            var mesh = MeshAssert.Terminates(() => reduced.Run(input), TimeSpan.FromSeconds(120));

            var reference = c.Input();
            var plain = reference.Segments.Count == 0
                ? (Mesh)new GenericMesher().Triangulate(reference.Points)
                : (Mesh)reference.Triangulate(new ConstraintOptions { SnapTolerance = c.Snap });
            double area = plain.Triangles.Sum(t => MeshInvariants.Area(t));

            var errors = MeshInvariants.Check(mesh, input, reduced, area);

            Assert.That(errors, Is.Empty, string.Join(", ", errors));
        }

        [TestCaseSource(nameof(QualityCases))]
        public void ReductionOnlyRemovesFreeSteinerPoints(MeshCases.Case c)
        {
            var plain = MeshAssert.Terminates(() => c.Run(c.Input()), TimeSpan.FromSeconds(120));
            var reduced = MeshAssert.Terminates(() => c.With("reduce", Reduce).Run(c.Input()), TimeSpan.FromSeconds(120));

            Assert.That(reduced.Triangles.Count, Is.LessThanOrEqualTo(plain.Triangles.Count));

            // Only free Steiner points are removed or moved (collapses move a neighbor).
            var reducedFixed = reduced.Vertices.Where(v => v.Type != VertexType.FreeVertex).Select(v => (v.X, v.Y)).OrderBy(p => p);
            var plainFixed = plain.Vertices.Where(v => v.Type != VertexType.FreeVertex).Select(v => (v.X, v.Y)).OrderBy(p => p);

            Assert.That(reducedFixed, Is.EqualTo(plainFixed), "input or segment vertex changed");
            Assert.That(reduced.Vertices.Count, Is.LessThanOrEqualTo(plain.Vertices.Count));

            // Subsegments are unchanged.
            Assert.That(Subsegments(reduced), Is.EqualTo(Subsegments(plain)));

            // Not more bad triangles than before (some can't be avoided near small input angles).
            if (c.MinAngle > 0)
            {
                Assert.That(Bad(reduced, c.MinAngle), Is.LessThanOrEqualTo(Bad(plain, c.MinAngle)));
            }

            if (c.MaxArea > 0)
            {
                Assert.That(reduced.Triangles.Max(t => MeshInvariants.Area(t)), Is.LessThanOrEqualTo(c.MaxArea));
            }
        }

        [TestCase("herne_373600_5709000/q30-snap1cm")]
        [TestCase("herne_373600_5709000/q30-grid1cm-wedge3cm")]
        [TestCase("points-random/q30")]
        [TestCase("square/q20-area")]
        public void ReductionRemovesSteinerPoints(string name)
        {
            var c = MeshCases.All().Concat(MeshCases.GridCases()).Single(x => x.Name == name);

            var plain = c.Run(c.Input());
            var reduced = c.With("reduce", Reduce).Run(c.Input());

            int before = plain.Vertices.Count(v => v.Type == VertexType.FreeVertex);
            int after = reduced.Vertices.Count(v => v.Type == VertexType.FreeVertex);

            TestContext.WriteLine($"{name}: triangles {plain.Triangles.Count} -> {reduced.Triangles.Count}, " +
                $"free Steiner points {before} -> {after}");

            Assert.That(after, Is.LessThan(before));
        }

        [Test]
        public void GridModeKeepsVerticesOnTheGrid()
        {
            var c = MeshCases.GridCases().First().With("reduce", Reduce);

            var mesh = c.Run(c.Input());

            Assert.That(mesh.Vertices.All(v => v.X == GridSnapper.Round(v.X, 0.01) && v.Y == GridSnapper.Round(v.Y, 0.01)), Is.True);
        }

        [Test]
        public void MaximumAngleAndAreaAreRespected()
        {
            var quality = new QualityOptions { MinimumAngle = 25, MaximumAngle = 110, MaximumArea = 0.3, ReduceSteinerPoints = true };

            var mesh = (Mesh)TestData.SquareWithSegments((1, 1, 9, 2), (2, 8, 8, 7)).Triangulate(new ConstraintOptions(), quality);

            MeshAssert.IsValid(mesh);

            foreach (var t in mesh.Triangles)
            {
                Assert.That(MeshInvariants.Area(t), Is.LessThanOrEqualTo(0.3));
                Assert.That(MeshInvariants.MinimumAngle(t), Is.GreaterThanOrEqualTo(25 - 1e-9));
                Assert.That(MaximumAngle(t), Is.LessThanOrEqualTo(110 + 1e-9));
            }
        }

        [Test]
        public void UserTestDisablesReduction()
        {
            // A user test can't be evaluated for triangles that don't exist yet.
            bool Large(ITriangle t, double area) => area > 0.5;

            var plain = (Mesh)TestData.SquareWithSegments((1, 1, 9, 2)).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 30, UserTest = Large });
            var reduced = (Mesh)TestData.SquareWithSegments((1, 1, 9, 2)).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 30, UserTest = Large, ReduceSteinerPoints = true });

            Assert.That(reduced.Triangles.Count, Is.EqualTo(plain.Triangles.Count));
        }

        [Test]
        public void ReductionIsDeterministic()
        {
            var c = MeshCases.All().Single(x => x.Name == "herne_371800_5710600/q30-snap1cm").With("reduce", Reduce);

            Assert.That(MeshInvariants.Fingerprint(c.Run(c.Input())), Is.EqualTo(MeshInvariants.Fingerprint(c.Run(c.Input()))));
        }

        private static int Bad(Mesh mesh, double minAngle) =>
            mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < minAngle - 1e-9);

        private static List<(double, double, double, double)> Subsegments(Mesh mesh)
        {
            return mesh.Segments.Select(s =>
            {
                Point a = s.GetVertex(0), b = s.GetVertex(1);
                return a.X < b.X || (a.X == b.X && a.Y < b.Y) ? (a.X, a.Y, b.X, b.Y) : (b.X, b.Y, a.X, a.Y);
            }).OrderBy(k => k).ToList();
        }

        private static double MaximumAngle(ITriangle t)
        {
            double max = 0;

            for (int i = 0; i < 3; i++)
            {
                Point a = t.GetVertex(i), b = t.GetVertex((i + 1) % 3), c = t.GetVertex((i + 2) % 3);
                double ux = b.X - a.X, uy = b.Y - a.Y, vx = c.X - a.X, vy = c.Y - a.Y;
                max = Math.Max(max, Math.Atan2(Math.Abs(ux * vy - uy * vx), ux * vx + uy * vy) * 180 / Math.PI);
            }

            return max;
        }
    }
}
