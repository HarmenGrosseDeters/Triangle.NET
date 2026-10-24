using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Snap rounding to an output grid (e.g. centimeters): all vertices of the mesh lie on the
    /// grid, so rounding the coordinates when writing the mesh doesn't flip triangles.
    /// </summary>
    public class GridSnapRoundingTest
    {
        private const double Grid = 0.01;

        private static bool OnGrid(Point p, double g) =>
            p.X == GridSnapper.Round(p.X, g) && p.Y == GridSnapper.Round(p.Y, g);

        [Test]
        public void RoundGivesNearestDoubleOfDecimalGrid()
        {
            // Same values as rounding to decimal places, as done when writing the output.
            var random = new Random(1);

            for (int i = 0; i < 10000; i++)
            {
                double x = 300000 + 100000 * random.NextDouble();

                Assert.That(GridSnapper.Round(x, 0.01), Is.EqualTo(Math.Round(x, 2)));
                Assert.That(GridSnapper.Round(x, 0.001), Is.EqualTo(Math.Round(x, 3)));
            }

            Assert.That(GridSnapper.Round(7.3, 0.5), Is.EqualTo(7.5));
            Assert.That(GridSnapper.Round(-7.3, 2.0), Is.EqualTo(-8.0));
        }

        [Test]
        public void SnapRoundingPutsAllVerticesOnTheGrid()
        {
            var poly = RandomSegments(1, 40, 10.0);

            var noded = SegmentNoder.SnapRound(poly, new RobustPredicates(), Grid);

            Assert.That(noded.Points.All(p => OnGrid(p, Grid)), Is.True);
            Assert.That(noded.Segments.All(s => OnGrid(s.GetVertex(0), Grid) && OnGrid(s.GetVertex(1), Grid)), Is.True);
        }

        [TestCase(1)]
        [TestCase(2)]
        [TestCase(3)]
        public void SnapRoundingResultIsProperlyNoded(int seed)
        {
            var poly = RandomSegments(seed, 60, 1.0);

            var noded = SegmentNoder.SnapRound(poly, new RobustPredicates(), Grid);

            AssertProperlyNoded(noded, Grid);
        }

        [Test]
        public void SnapRoundingMergesNearlyCoincidentBoundaries()
        {
            // Two adjacent parcels whose common boundary differs by a few millimeters.
            var poly = TestData.AdjacentPolygonsWithMismatchedBoundary(0.003);

            var noded = SegmentNoder.SnapRound(poly, new RobustPredicates(), Grid);

            AssertProperlyNoded(noded, Grid);

            var mesh = (Mesh)new GenericMesher().Triangulate(noded, new ConstraintOptions { GridSize = Grid },
                new QualityOptions { MinimumAngle = 30 });

            MeshAssert.IsValid(mesh);

            // No sliver between the boundaries: the mesh is small.
            Assert.That(mesh.Triangles.Count, Is.LessThan(200));
        }

        [TestCase(1, 25.0)]
        [TestCase(2, 30.0)]
        [TestCase(3, 33.0)]
        public void QualityMeshVerticesAreOnTheGrid(int seed, double angle)
        {
            var poly = RandomSegments(seed, 30, 10.0);

            var mesh = (Mesh)MeshAssert.Terminates(() => new GenericMesher().Triangulate(poly,
                new ConstraintOptions { GridSize = Grid }, new QualityOptions { MinimumAngle = angle }));

            MeshAssert.IsValid(mesh);

            Assert.That(mesh.Vertices.All(v => OnGrid(v, Grid)), Is.True);

            var predicates = new RobustPredicates();

            Assert.That(mesh.Triangles.All(t => predicates.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)) > 0), Is.True);
        }

        [Test]
        public void RoundingAfterMeshingWithoutGridCanFlipTriangles_WithGridItCannot()
        {
            // Nearly coincident lines with millimeter offsets: rounding the output of a mesh
            // computed in full precision to centimeters creates flat or inverted triangles.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10), false);

            for (int i = 0; i < 8; i++)
            {
                double y = 2 + i * 0.7;
                poly.Add(new Segment(new Vertex(0.5, y), new Vertex(9.5, y + 0.004 * (i + 1))), true);
                poly.Add(new Segment(new Vertex(0.5, y + 0.003), new Vertex(9.5, y + 0.002)), true);
            }

            var options = new QualityOptions { MinimumAngle = 30 };

            var plain = (Mesh)new GenericMesher().Triangulate(poly, options);
            Assert.That(CountFlippedAfterRounding(plain, Grid), Is.GreaterThan(0), "test case isn't critical");

            var grid = (Mesh)new GenericMesher().Triangulate(poly, new ConstraintOptions { GridSize = Grid }, options);
            MeshAssert.IsValid(grid);
            Assert.That(CountFlippedAfterRounding(grid, Grid), Is.EqualTo(0));
        }

        [Test]
        public void SnapVerticesKeepsTrianglesValid()
        {
            // Random points: snapping a point to the grid must not invert a triangle.
            var random = new Random(5);
            var points = Enumerable.Range(0, 2000).Select(_ => new Vertex(random.NextDouble(), random.NextDouble())).ToList();

            var mesh = (Mesh)new GenericMesher().Triangulate(points);

            int unsnapped = GridSnapper.SnapVertices(mesh, 0.01);

            MeshAssert.IsValid(mesh);

            var predicates = new RobustPredicates();
            Assert.That(mesh.Triangles.All(t => predicates.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)) > 0), Is.True);
            Assert.That(mesh.Vertices.Count(v => !OnGrid(v, 0.01)), Is.EqualTo(unsnapped));
        }

        [Test]
        public void LockedSegmentsKeepTheirVertices()
        {
            // Locked segments (break lines) on the grid are not changed by grid rounding.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10), false);
            poly.Add(new Segment(new Vertex(1, 1), new Vertex(9, 8.97), 7), true);

            var mesh = (Mesh)new GenericMesher().Triangulate(poly,
                new ConstraintOptions { GridSize = Grid, CanSplitSegment = s => s.Label != 7 },
                new QualityOptions { MinimumAngle = 30 });

            MeshAssert.IsValid(mesh);

            var locked = mesh.Segments.Where(s => s.Label == 7).ToList();
            Assert.That(locked.Count, Is.EqualTo(1));
            Assert.That(locked[0].GetVertex(0).X + locked[0].GetVertex(1).X, Is.EqualTo(10.0));
        }

        [TestCase(1)]
        [TestCase(2)]
        [TestCase(3)]
        public void NoLargeBadTrianglesRemain(int seed)
        {
            // Subsegments where no grid point is a valid split must not stop the refinement of
            // large triangles nearby (their circumcenter encroaches upon the subsegment).
            var poly = RandomSegments(seed, 30, 10.0);

            var mesh = (Mesh)MeshAssert.Terminates(() => new GenericMesher().Triangulate(poly,
                new ConstraintOptions { GridSize = Grid }, new QualityOptions { MinimumAngle = 30 }));

            MeshAssert.IsValid(mesh);

            var segments = new HashSet<(int, int)>(mesh.Segments.Select(s => (Math.Min(s.P0, s.P1), Math.Max(s.P0, s.P1))));

            bool OnSegment(ITriangle t) => Enumerable.Range(0, 3).Any(i =>
            {
                int a = t.GetVertexID(i), b = t.GetVertexID((i + 1) % 3);
                return segments.Contains((Math.Min(a, b), Math.Max(a, b)));
            });

            double Shortest(ITriangle t) => Enumerable.Range(0, 3).Min(i =>
            {
                var a = t.GetVertex(i);
                var b = t.GetVertex((i + 1) % 3);
                return Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
            });

            int large = mesh.Triangles.Count(t => Characterization.MeshInvariants.MinimumAngle(t) < 29.99 &&
                Shortest(t) > 20 * Grid && !OnSegment(t));

            Assert.That(large, Is.EqualTo(0));
        }

        internal static int CountFlippedAfterRounding(Mesh mesh, double g)
        {
            var predicates = new RobustPredicates();
            Vertex R(Vertex v) => new Vertex(GridSnapper.Round(v.X, g), GridSnapper.Round(v.Y, g));

            return mesh.Triangles.Count(t => predicates.CounterClockwise(R(t.GetVertex(0)), R(t.GetVertex(1)), R(t.GetVertex(2))) <= 0);
        }

        private static void AssertProperlyNoded(IPolygon poly, double g)
        {
            var predicates = new RobustPredicates();
            var segments = poly.Segments;

            for (int i = 0; i < segments.Count; i++)
            {
                var a = segments[i].GetVertex(0);
                var b = segments[i].GetVertex(1);

                Assert.That(a.X != b.X || a.Y != b.Y, Is.True, "degenerate segment");

                for (int j = i + 1; j < segments.Count; j++)
                {
                    var c = segments[j].GetVertex(0);
                    var d = segments[j].GetVertex(1);

                    double o1 = predicates.CounterClockwise(a, b, c), o2 = predicates.CounterClockwise(a, b, d);
                    double o3 = predicates.CounterClockwise(c, d, a), o4 = predicates.CounterClockwise(c, d, b);

                    bool cross = o1 * o2 < 0 && o3 * o4 < 0;
                    Assert.That(cross, Is.False, $"segments {i} and {j} cross");
                }

                // No vertex lies in the interior of a segment.
                foreach (var p in poly.Points)
                {
                    if (p.X == a.X && p.Y == a.Y || p.X == b.X && p.Y == b.Y) continue;

                    bool between = Math.Min(a.X, b.X) <= p.X && p.X <= Math.Max(a.X, b.X) &&
                                   Math.Min(a.Y, b.Y) <= p.Y && p.Y <= Math.Max(a.Y, b.Y);

                    Assert.That(between && predicates.CounterClockwise(a, b, p) == 0, Is.False, "vertex on segment");
                }
            }
        }

        private static Polygon RandomSegments(int seed, int count, double size)
        {
            var random = new Random(seed);
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, size), false);

            for (int i = 0; i < count; i++)
            {
                poly.Add(new Segment(
                    new Vertex(size * random.NextDouble(), size * random.NextDouble()),
                    new Vertex(size * random.NextDouble(), size * random.NextDouble())), true);
            }

            return poly;
        }
    }
}
