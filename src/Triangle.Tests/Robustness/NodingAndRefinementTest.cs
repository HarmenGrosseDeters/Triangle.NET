using NUnit.Framework;
using System;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Tests for the segment noder and for quality refinement of noded input.
    /// </summary>
    public class NodingAndRefinementTest
    {
        private static readonly TimeSpan LongTimeout = TimeSpan.FromSeconds(30);

        [Test]
        public void NoderReturnsInputIfNothingToDo()
        {
            var poly = TestData.SquareWithSegments((1, 1, 9, 2), (1, 5, 9, 6));

            Assert.That(SegmentNoder.Node(poly, new RobustPredicates()), Is.SameAs(poly));
        }

        [Test]
        public void NoderSplitsCrossingSegments()
        {
            var poly = TestData.SquareWithSegments((1, 1, 9, 9), (1, 9, 9, 1));

            var noded = SegmentNoder.Node(poly, new RobustPredicates());

            Assert.That(noded, Is.Not.SameAs(poly));
            Assert.That(noded.Points.Count, Is.EqualTo(poly.Points.Count + 1));
            Assert.That(noded.Segments.Count, Is.EqualTo(poly.Segments.Count + 2));

            // Both segments are split at the same vertex instance.
            var center = noded.Points.Last();
            Assert.That(center.X, Is.EqualTo(5.0));
            Assert.That(center.Y, Is.EqualTo(5.0));
            Assert.That(noded.Segments.Count(s => s.GetVertex(0) == center || s.GetVertex(1) == center), Is.EqualTo(4));

            // Input isn't modified.
            Assert.That(poly.Segments.Count, Is.EqualTo(6));
        }

        [Test]
        public void NoderSnapsVertexNearSegment()
        {
            var poly = TestData.SquareWithSegments((1, 5, 9, 5));
            poly.Add(new Vertex(5, 5 + 1e-14));

            var noded = SegmentNoder.Node(poly, new RobustPredicates());

            // The segment now runs through the vertex (the vertex isn't moved).
            Assert.That(noded.Segments.Count, Is.EqualTo(poly.Segments.Count + 1));
            Assert.That(noded.Points.Any(p => p.Y == 5 + 1e-14), Is.True);
        }

        [Test]
        public void WithoutNodingFailuresAreReported()
        {
            // Without noding, the mesher must fail loudly (never return an invalid mesh).
            var poly = TestData.SquareWithSegments((2, 4, 4, 8), (1, 5, 6, 6), (1, 7, 5, 3));

            Assert.Throws(Is.InstanceOf<Exception>(), () => poly.Triangulate(new ConstraintOptions { ResolveIntersections = false }));
        }

        [Test]
        [Description("Fixed: quality refinement of noded input produced zero-length edges (IndexOutOfRangeException in BadTriQueue), because vertices lying almost on segments weren't snapped.")]
        public void QualityMeshingOfCrossingSegments([Range(0, 19)] int seed)
        {
            var poly = TestData.SquareWithSegments(TestData.RandomIntegerSegments(seed, 40));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 25 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments, 1e-7);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        [Description("Fixed: aCute refinement started point location from an edge the new vertex isn't left of, inserting it into the wrong triangle (inverted triangles).")]
        public void AcuteLocationOutsideTriangle()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 20));

            var random = new Random(23);

            for (int i = 0; i < 100; i++)
            {
                poly.Add(new Segment(new Vertex(random.Next(21), random.Next(21)), new Vertex(random.Next(21), random.Next(21))), true);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 400);
        }

        [Test]
        [Description("Fixed: aCute location coinciding with an existing vertex threw 'The new vertex is at the circumcenter of triangle' instead of falling back to the circumcenter.")]
        public void AcuteLocationOnExistingVertex([Values(172, 173, 176)] int seed)
        {
            var poly = TestData.SquareWithSegments(TestData.RandomIntegerSegments(seed, 40));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 30 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void AdjacentPolygonsWithMismatchedBoundaryQualityWithoutLimit()
        {
            // The noder snaps the two nearly coincident boundaries together, so the slivers
            // disappear and quality meshing terminates without a Steiner point limit.
            var poly = TestData.AdjacentPolygonsWithMismatchedBoundary(1e-12);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 25 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 200, 1e-6);
        }
    }
}
