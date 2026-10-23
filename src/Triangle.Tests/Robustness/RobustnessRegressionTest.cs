using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Degenerate and "dirty" input which is handled correctly. These tests guard against
    /// regressions.
    /// </summary>
    public class RobustnessRegressionTest
    {
        private static readonly TimeSpan LongTimeout = TimeSpan.FromSeconds(30);

        [Test]
        public void ClosedRingWithDistinctFirstAndLastVertex()
        {
            // GIS formats usually repeat the first vertex at the end of a ring.
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(1, 0), new Vertex(1, 1), new Vertex(0, 1), new Vertex(0, 0) }));

            var mesh = poly.Triangulate();

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 1.0);
        }

        [Test]
        public void SelfIntersectingContour()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(1, 1), new Vertex(1, 0), new Vertex(0, 1) }));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 0.5);
        }

        [Test]
        public void OverlappingCollinearSegments()
        {
            var poly = TestData.SquareWithSegments((1, 5, 6, 5), (4, 5, 9, 5));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void SelfOverlappingCollinearContour()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(10, 0), new Vertex(10, 10), new Vertex(5, 10), new Vertex(8, 10), new Vertex(0, 10) }));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void DuplicateSegmentReversed()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));

            var a = new Vertex(2, 2);
            var b = new Vertex(8, 7);

            poly.Add(new Segment(a, b), true);
            poly.Add(new Segment(b, a));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void SpikeInContour()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(10, 0), new Vertex(10, 5), new Vertex(15, 5), new Vertex(10, 5), new Vertex(10, 10), new Vertex(0, 10) }));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void SegmentEndpointsDuplicateContourVertices()
        {
            var poly = TestData.SquareWithSegments((0, 0, 10, 10));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
        }

        [Test]
        public void SegmentThroughVertices()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 4));
            poly.Add(new Vertex(2, 2));
            poly.Add(new Vertex(1, 1));
            poly.Add(new Segment(new Vertex(0.5, 0.5), new Vertex(3.5, 3.5)), true);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
        }

        [Test]
        public void CrossingSegmentsNearlyThroughVertex([Values(1e-8, 1e-12, 1e-15)] double eps)
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 4));
            poly.Add(new Vertex(2, 2 + eps));
            poly.Add(new Segment(new Vertex(0.5, 0.5), new Vertex(3.5, 3.5)), true);
            poly.Add(new Segment(new Vertex(0.5, 3.5), new Vertex(3.5, 0.5)), true);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments, 1e-7);
        }

        [Test]
        public void NearlyDuplicateVertices([Values(1e-6, 1e-10, 1e-13, 1e-15)] double eps)
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Vertex(0.5, 0.5));
            poly.Add(new Vertex(0.5 + eps, 0.5));
            poly.Add(new Vertex(0.5, 0.5 + eps));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 1.0);
        }

        [Test]
        public void VertexNearlyOnSegment([Values(1e-6, 1e-10, 1e-14)] double eps)
        {
            var poly = TestData.SquareWithSegments((1, 5, 9, 5));
            poly.Add(new Vertex(5, 5 + eps));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void LargeCoordinatesWithSmallFeatures([Values(1e5, 1e7, 1e9)] double offset)
        {
            // Typical GIS coordinates (UTM / Gauss-Krueger) with millimeter-sized features.
            var poly = new Polygon();
            poly.Add(TestData.Square(offset, offset, 10));

            var random = new Random(1);

            for (int i = 0; i < 30; i++)
            {
                double x = offset + 1 + random.NextDouble() * 8;
                double y = offset + 1 + random.NextDouble() * 8;

                poly.Add(new Segment(new Vertex(x, y), new Vertex(x + 0.001, y + 0.0005)), true);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100, 1e-6);
        }

        [Test]
        public void SmallInputAngle([Values(1.0, 0.1, 0.01)] double angle)
        {
            var poly = new Polygon();
            var a = angle * Math.PI / 180;

            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(1, 0), new Vertex(Math.Cos(a), Math.Sin(a)) }));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 30 }), LongTimeout);

            MeshAssert.IsValid(mesh);
        }

        [Test]
        public void FanOfSegmentsWithSmallAngles()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(-1, -1, 2));

            var center = new Vertex(0, 0);
            poly.Add(center);

            for (int i = 0; i < 20; i++)
            {
                double a = i * 0.001;
                poly.Add(new Segment(center, new Vertex(0.9 * Math.Cos(a), 0.9 * Math.Sin(a))), 1);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 25 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 4);
        }

        [Test]
        public void ThinRectangleWithMaximumAngle()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(100, 0), new Vertex(100, 0.01), new Vertex(0, 0.01) }));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20, MaximumAngle = 100 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 1.0);
        }

        [Test]
        public void ThinHole()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Contour(new[] { new Vertex(2, 2), new Vertex(8, 2), new Vertex(8, 2.00001), new Vertex(2, 2.00001) }), true);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100 - 6 * 0.00001, 1e-9);
        }

        [Test]
        public void HolePointOnBoundary()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(2, 2, 2), new Point(2, 3));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 96);
        }

        [Test]
        public void TouchingHoles()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(2, 2, 2), true);
            poly.Add(TestData.Square(4, 2, 2), true);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 25 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 92);
        }

        [Test]
        public void HoleTouchingOuterBoundary()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Contour(new[] { new Vertex(0, 4), new Vertex(3, 4), new Vertex(3, 6), new Vertex(0, 6) }), new Point(1, 5));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 25 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 94);
        }

        [Test]
        public void RefineTwice()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(3, 3, 2), true);

            var mesh = poly.Triangulate(new QualityOptions { MinimumAngle = 20 });

            mesh.Refine(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.5 }, false);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 96);
        }

        [Test]
        public void AdjacentPolygonsWithMismatchedBoundary([Values(1e-6, 1e-9, 1e-12)] double eps)
        {
            var poly = TestData.AdjacentPolygonsWithMismatchedBoundary(eps);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate());

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 200, 1e-6);
        }

        [Test]
        public void AdjacentPolygonsWithMismatchedBoundaryQuality([Values(1e-6, 1e-9, 1e-12)] double eps)
        {
            // Without a Steiner point limit, quality meshing of the slivers between the two
            // boundaries practically doesn't terminate (a sliver of width eps requires about
            // length/eps triangles to satisfy the minimum angle). With a limit, the result is
            // valid (though the minimum angle can't be satisfied).
            var poly = TestData.AdjacentPolygonsWithMismatchedBoundary(eps);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20, SteinerPoints = 5000 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 200, 1e-6);
        }

        [Test]
        public void GridPoints([ValueSource(nameof(Triangulators))] ITriangulator triangulator)
        {
            var points = new List<Vertex>();

            for (int i = 0; i < 30; i++)
            {
                for (int j = 0; j < 30; j++)
                {
                    points.Add(new Vertex(i * 0.1, j * 0.1));
                }
            }

            var mesh = triangulator.Triangulate(points, new Configuration());

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 2.9 * 2.9);
        }

        [Test]
        public void RandomCrossingSegmentsQuality([Range(0, 49)] int seed)
        {
            var random = new Random(seed);
            var poly = new Polygon();

            poly.Add(TestData.Square(0, 0, 10));

            for (int i = 0; i < 15; i++)
            {
                poly.Add(new Segment(
                    new Vertex(random.NextDouble() * 10, random.NextDouble() * 10),
                    new Vertex(random.NextDouble() * 10, random.NextDouble() * 10)), true);
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 28 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100);
        }

        [Test]
        public void ParallelMeshingWithPrivatePredicates()
        {
            // Workaround for the thread safety issue (see ThreadSafetyTest).
            static IMesh Triangulate()
            {
                var poly = new Polygon();
                poly.Add(TestData.Square(1e6, 1e6, 10));

                var random = new Random(7);

                for (int i = 0; i < 40; i++)
                {
                    poly.Add(new Segment(
                        new Vertex(1e6 + random.NextDouble() * 10, 1e6 + random.NextDouble() * 10),
                        new Vertex(1e6 + random.NextDouble() * 10, 1e6 + random.NextDouble() * 10)), true);
                }

                var mesher = new GenericMesher(new Configuration(() => new RobustPredicates()));

                return mesher.Triangulate(poly, new QualityOptions { MinimumAngle = 25 });
            }

            int expected = Triangulate().Triangles.Count;

            var counts = new int[32];

            Parallel.For(0, counts.Length, i => counts[i] = Triangulate().Triangles.Count);

            Assert.That(counts, Is.All.EqualTo(expected));
        }

        private static IEnumerable<ITriangulator> Triangulators()
        {
            yield return new Dwyer();
            yield return new SweepLine();
            yield return new Incremental();
        }
    }
}
