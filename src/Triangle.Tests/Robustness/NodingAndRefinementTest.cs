using NUnit.Framework;
using System;
using System.Collections.Generic;
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
        [Test]
        [Description("Fixed: tiny rings with vertices closer than the snap tolerance (coordinate noise) made the noder split segments forever.")]
        public void MicroRingsWithCoordinateNoise()
        {
            var poly = new Polygon();
            var random = new Random(5);

            for (int k = 0; k < 5; k++)
            {
                double x = -92.66 + 0.002 * k, y = 36.59 + 0.001 * k;
                var ring = new List<Vertex>();

                for (int i = 0; i < 5; i++)
                {
                    ring.Add(new Vertex(x + (random.NextDouble() - 0.5) * 1e-13, y + (random.NextDouble() - 0.5) * 1e-13));
                }

                poly.Add(new Contour(ring));
            }

            poly.Add(TestData.Square(-92.67, 36.58, 0.03));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20, SteinerPoints = 20000 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 0.03 * 0.03, 1e-6);
        }

        private static Polygon StackedSquares(double offset)
        {
            var poly = new Polygon();

            for (int k = 0; k < 40; k++)
            {
                double a = 0.8 + k * offset * 0.1, d = k * offset;
                var c = new[] { (0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0) }
                    .Select(p => new Vertex(600 + d + p.Item1 * Math.Cos(a) - p.Item2 * Math.Sin(a), 600 + d + p.Item1 * Math.Sin(a) + p.Item2 * Math.Cos(a)));

                poly.Add(new Contour(c));
            }

            poly.Add(TestData.Square(595, 595, 10));

            return poly;
        }

        [Test]
        [Description("Fixed: many stacked, nearly identical squares made noding explode; now the snap tolerance is increased.")]
        public void StackedNearlyIdenticalSquares([Values(1e-10, 1e-8)] double offset)
        {
            var poly = StackedSquares(offset);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100, 1e-9);
        }

        [Test]
        public void StackedNearlyIdenticalSquaresBeyondTolerance()
        {
            // Offsets larger than the maximum snap tolerance: noding can't converge in
            // reasonable time, which must be reported (instead of running forever).
            var poly = StackedSquares(1e-6);

            var e = Assert.Throws<AggregateException>(() => MeshAssert.Terminates(() => poly.Triangulate(), LongTimeout));
            Assert.That(e.InnerException, Is.InstanceOf<InvalidOperationException>());
        }

        [Test]
        [Description("Fixed: refinement of needle-shaped regions silently produced inverted triangles (segment splits beyond floating point precision).")]
        public void NeedleTriangles()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(101, 99), new Vertex(101, 1000000), new Vertex(102, 1000000) }));
            poly.Add(new Contour(new[] { new Vertex(100, 100), new Vertex(1000000, 110), new Vertex(1000000, 100) }));
            poly.Add(TestData.Square(-100000, -100000, 1200000));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20, SteinerPoints = 10000 }), LongTimeout);

            MeshAssert.IsValid(mesh);
        }
    }
}
