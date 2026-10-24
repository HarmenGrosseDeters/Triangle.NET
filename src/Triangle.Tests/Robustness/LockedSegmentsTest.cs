using NUnit.Framework;
using System;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Segments which may not be split during refinement (e.g. breaklines of a terrain model),
    /// see <see cref="ConstraintOptions.CanSplitSegment"/>.
    /// </summary>
    public class LockedSegmentsTest
    {
        private const int Breakline = 100;

        private static readonly TimeSpan LongTimeout = TimeSpan.FromSeconds(30);

        private static ConstraintOptions LockBreaklines() => new ConstraintOptions
        {
            CanSplitSegment = s => s.Label != Breakline
        };

        private static Polygon SquareWithBreakline(double x1, double y1, double x2, double y2)
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Segment(new Vertex(x1, y1), new Vertex(x2, y2), Breakline), true);
            return poly;
        }

        private static int VerticesInSegmentInterior(IMesh mesh, Point a, Point b)
        {
            var predicates = new RobustPredicates();

            return mesh.Vertices.Count(v =>
                !(v.X == a.X && v.Y == a.Y) && !(v.X == b.X && v.Y == b.Y) &&
                predicates.CounterClockwise(a, b, v) == 0.0 &&
                Math.Min(a.X, b.X) <= v.X && v.X <= Math.Max(a.X, b.X) &&
                Math.Min(a.Y, b.Y) <= v.Y && v.Y <= Math.Max(a.Y, b.Y));
        }

        [Test]
        public void LockedBreaklineKeepsItsVertices()
        {
            var poly = SquareWithBreakline(1, 1, 9, 2);
            var seg = poly.Segments.Last();

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(LockBreaklines(), new QualityOptions { MinimumAngle = 30, MaximumArea = 0.05 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);

            Assert.That(VerticesInSegmentInterior(mesh, seg.GetVertex(0), seg.GetVertex(1)), Is.Zero);

            // The breakline is still a single subsegment.
            Assert.That(mesh.Segments.Count(s => s.Label == Breakline), Is.EqualTo(1));
        }

        [Test]
        public void UnlockedBreaklineIsSplit()
        {
            // Counter check: without the filter, refinement inserts vertices on the breakline.
            var poly = SquareWithBreakline(1, 1, 9, 2);
            var seg = poly.Segments.Last();

            var mesh = poly.Triangulate(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.05 });

            Assert.That(mesh.Segments.Count(s => s.Label == Breakline), Is.GreaterThan(1));
        }

        [Test]
        public void OuterBoundaryIsStillSplit()
        {
            var poly = SquareWithBreakline(1, 1, 9, 2);

            var mesh = poly.Triangulate(LockBreaklines(), new QualityOptions { MinimumAngle = 30, MaximumArea = 0.05 });

            Assert.That(mesh.Segments.Count(s => s.Label != Breakline), Is.GreaterThan(4));
        }

        [Test]
        public void CrossingLockedBreaklinesAreSplitOnlyAtIntersection()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Segment(new Vertex(1, 1), new Vertex(9, 9), Breakline), true);
            poly.Add(new Segment(new Vertex(1, 9), new Vertex(9, 1), Breakline), true);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(LockBreaklines(), new QualityOptions { MinimumAngle = 30, MaximumArea = 0.05 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);

            // Each breakline consists of exactly two subsegments (split at the intersection).
            Assert.That(mesh.Segments.Count(s => s.Label == Breakline), Is.EqualTo(4));
        }

        [Test]
        public void ManyLockedCrossingBreaklines([Range(0, 19)] int seed)
        {
            var random = new Random(seed);
            var poly = new Polygon();

            poly.Add(TestData.Square(0, 0, 10));

            for (int i = 0; i < 30; i++)
            {
                poly.Add(new Segment(new Vertex(random.Next(11), random.Next(11)), new Vertex(random.Next(11), random.Next(11)), Breakline), true);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(LockBreaklines(), new QualityOptions { MinimumAngle = 25 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments, 1e-7);
            MeshAssert.HasArea(mesh, 100);
        }
    }
}
