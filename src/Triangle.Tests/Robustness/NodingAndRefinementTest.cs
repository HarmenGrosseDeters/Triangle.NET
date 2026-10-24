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
        [Description("Fixed: offsets larger than the maximum snap tolerance made noding fail (crossings merged into " +
            "segment endpoints counted as splits and used up the iterations).")]
        public void StackedNearlyIdenticalSquaresBeyondTolerance([Values(1e-6, 1e-5, 1e-4)] double offset)
        {
            var poly = StackedSquares(offset);

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100, 1e-9);
        }

        [Test]
        public void NodingThatDoesNotConvergeIsReported()
        {
            // If noding doesn't converge (here: only one iteration allowed), this must be
            // reported instead of running forever or returning crossing segments.
            var poly = StackedSquares(1e-6);

            Assert.Throws<InvalidOperationException>(() => SegmentNoder.Node(poly, new RobustPredicates(), maxIterations: 1));
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

        [Test]
        [Description("Fixed: in needle-shaped regions, refinement inserted circumcenters with huge cavities, found an " +
            "encroached segment and undid everything (60,000 points: 78 s, now about 2 s in a release build).")]
        public void NeedleRefinementIsNotQuadratic()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(101, 99), new Vertex(101, 1000000), new Vertex(102, 1000000) }));
            poly.Add(new Contour(new[] { new Vertex(100, 100), new Vertex(1000000, 110), new Vertex(1000000, 100) }));
            poly.Add(TestData.Square(-100000, -100000, 1200000));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new QualityOptions { MinimumAngle = 20, SteinerPoints = 60000 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count, Is.GreaterThan(50000));
        }
        private static Polygon ParcelWithBuilding(double gap)
        {
            // A parcel and a building whose wall runs along the parcel boundary at a distance
            // of 'gap' (typical for cadastral data: separately digitized, nearly coincident lines).
            var poly = new Polygon();

            poly.Add(new Contour(new[] { new Vertex(370000, 5710000), new Vertex(370030, 5710000), new Vertex(370030, 5710020), new Vertex(370000, 5710020) }));
            poly.Add(new Contour(new[] { new Vertex(370005, 5710000 + gap), new Vertex(370015, 5710000 + gap), new Vertex(370015, 5710008), new Vertex(370005, 5710008) }));

            return poly;
        }

        [Test]
        public void SnapToleranceRemovesSlivers()
        {
            // Building wall 5 mm inside the parcel boundary: with the automatic tolerance the
            // sliver is kept (and needs many triangles), with 1 cm it is merged.
            var quality = new QualityOptions { MinimumAngle = 20, SteinerPoints = 50000 };

            var automatic = MeshAssert.Terminates(() => ParcelWithBuilding(0.005).Triangulate(new ConstraintOptions(), quality), LongTimeout);
            var snapped = MeshAssert.Terminates(() => ParcelWithBuilding(0.005).Triangulate(new ConstraintOptions { SnapTolerance = 0.01 }, quality), LongTimeout);

            MeshAssert.IsValid(automatic);
            MeshAssert.IsValid(snapped);
            MeshAssert.HasArea(automatic, 600, 1e-9);

            // Snapping routes the parcel boundary through the building corners, i.e. geometry
            // changes by at most the tolerance (area change <= perimeter * tolerance).
            MeshAssert.HasArea(snapped, 600, 100 * 0.01 / 600);

            Assert.That(snapped.Triangles.Count, Is.LessThan(automatic.Triangles.Count / 4));
        }

        [Test]
        public void SnapToleranceKeepsLargerGaps()
        {
            // A gap larger than the snap tolerance is kept.
            var mesh = ParcelWithBuilding(0.05).Triangulate(new ConstraintOptions { SnapTolerance = 0.01 });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count(v => v.Y == 5710000 + 0.05), Is.EqualTo(2));
        }
        [Test]
        [Description("Fixed: three vertices just over the snap tolerance apart (each within the tolerance of the segment between the other two) made snapping go back and forth forever.")]
        public void SnapCycle()
        {
            // Configuration found in ALKIS data (Herne) with a snap tolerance of 1 cm:
            // C is within 1 cm of A-B, and after splitting A-B at C, B is within 1 cm of A-C.
            var a = new Vertex(0, 0);
            var b = new Vertex(0.010, 0.019);
            var c = new Vertex(0, 0.020);

            var poly = new Polygon();
            poly.Add(TestData.Square(-1, -1, 2));
            poly.Add(a);
            poly.Add(b);
            poly.Add(c);
            poly.Add(new Segment(a, b));
            poly.Add(new Segment(a, c));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SnapTolerance = 0.01 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 4, 1e-9);
        }

        [Test]
        [Description("Fixed: a crossing merged into a segment endpoint 'split' the segment at its endpoint and blocked snapping into it. " +
            "The nearly collinear segments (0.0002°) remained and quality refinement spent 97,000 Steiner points in this wedge (ALKIS Herne).")]
        public void CrossingAtEndpointDoesNotBlockSnapping()
        {
            // Land use boundary and building edge (ALKIS Herne), sharing S; B is 0.02 mm from S-A.
            // The third segment passes S at about 1 mm and crosses both close to S.
            var s = new Vertex(377985.407, 5710096.824);
            var a = new Vertex(377981.985, 5710100.365);
            var b = new Vertex(377982.302, 5710100.037);
            var c = new Vertex(377983.573, 5710095.053);
            var d = new Vertex(377997.164, 5710108.182);

            var poly = new Polygon();
            foreach (var v in new[] { s, a, b, c, d }) poly.Add(v);
            poly.Add(new Segment(s, a));
            poly.Add(new Segment(b, s));
            poly.Add(new Segment(c, d));

            var noded = SegmentNoder.Node(poly, new RobustPredicates(), minTolerance: 0.01);

            // S-A is split at B, so S-B is shared and there is no tiny angle at S.
            Assert.That(noded.Segments.Any(x => Connects(x, s, a)), Is.False);
            Assert.That(noded.Segments.Any(x => Connects(x, b, a)), Is.True);

            poly.Add(TestData.Square(377970, 5710085, 40));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SnapTolerance = 0.01 },
                new QualityOptions { MinimumAngle = 20, SteinerPoints = 100000 }), LongTimeout);

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count, Is.LessThan(2000));
        }

        private static bool Connects(ISegment seg, Vertex p, Vertex q)
        {
            Point u = seg.GetVertex(0), v = seg.GetVertex(1);

            return (u.X == p.X && u.Y == p.Y && v.X == q.X && v.Y == q.Y) ||
                (u.X == q.X && u.Y == q.Y && v.X == p.X && v.Y == p.Y);
        }
    }
}
