using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Meshing.Iterators;
using TriangleNet.Smoothing;
using TriangleNet.Tests.Characterization;
using TriangleNet.Voronoi;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Reproductions of the bug reports in the upstream issue tracker
    /// (https://github.com/wo80/Triangle.NET/issues), as far as the reports contain enough
    /// information.
    /// </summary>
    public class UpstreamIssuesTest
    {
        private static readonly TimeSpan Timeout = TimeSpan.FromSeconds(30);

        [Test]
        [Description("#1: 'Topological inconsistency after splitting a segment' on three segments with large coordinates.")]
        public void Issue1_SegmentsWithLargeCoordinates([Values(0, 1, 2)] int splitting, [Values(0, 20, 30)] double minAngle)
        {
            var c = new Vertex(3470.23824070941, 2025.38518075843);
            var c2 = new Vertex(3470.23824070941, 2025.3851807584); // almost the same point
            var a = new Vertex(3470.24446228851, 2026.74647195611);
            var b = new Vertex(3470.23554561524, 2024.79548995642);
            var d = new Vertex(3469.65964599373, 2025.84786418632);

            var poly = new Polygon();
            foreach (var v in new[] { a, c, b, c2, d }) poly.Add(v);
            poly.Add(new Segment(a, c));
            poly.Add(new Segment(c, b));
            poly.Add(new Segment(c2, d));

            var quality = minAngle > 0 ? new QualityOptions { MinimumAngle = minAngle, SteinerPoints = 100000 } : null;
            // No enclosing contour: mesh the convex hull.
            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SegmentSplitting = splitting, Convex = true }, quality), Timeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
        }

        [Test]
        [Description("#3: smoothing of a domain with a segment that isn't connected to the boundary.")]
        public void Issue3_SmoothingWithOrphanSegment()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(100, 0, 1), new Vertex(110, 60, 1), new Vertex(-5, 50, 1) }, 1));
            var a = new Vertex(30, 20, 2);
            var b = new Vertex(70, 30, 2);
            poly.Add(a);
            poly.Add(b);
            poly.Add(new Segment(a, b, 2));

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 25, MaximumArea = 20 });

            MeshAssert.Terminates(() => new SimpleSmoother().Smooth(mesh, 10), Timeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, Area(poly.Points.Take(4).ToList()), 1e-9);
        }

        [Test]
        [Description("#8: bounded Voronoi diagram of a smoothed conforming Delaunay mesh (faces looped forever).")]
        public void Issue8_BoundedVoronoiAfterSmoothing([Range(0, 9)] int seed)
        {
            var random = new Random(seed);
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(10, 0, 1), new Vertex(10, 10, 1), new Vertex(0, 10, 1) }, 1));

            for (int i = 0; i < 20; i++)
            {
                poly.Add(new Vertex(0.5 + 9 * random.NextDouble(), 0.5 + 9 * random.NextDouble(), 2));
            }

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 25, MaximumArea = 1 });

            new SimpleSmoother().Smooth(mesh, 5);

            var voronoi = MeshAssert.Terminates(() => new BoundedVoronoi(mesh), Timeout);

            Assert.That(voronoi.IsConsistent(), Is.True);
            Assert.That(voronoi.Faces.All(f => f.Edge != null), Is.True);
        }

        [Test]
        [Description("#9: maximum area on a rectangle without segment splitting (10x30 worked, 10x20 didn't).")]
        public void Issue9_AreaConstraintOnRectangleWithoutSegmentSplitting([Values(20, 30)] double height)
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(10, 0, 1), new Vertex(10, height, 1), new Vertex(0, height, 1) }, 1));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SegmentSplitting = 2 },
                new QualityOptions { MaximumArea = 1.73 }), Timeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 10 * height, 1e-9);

            // Without segment splitting, the triangles at the boundary can't get smaller than
            // the boundary allows; all others must meet the area constraint.
            var interior = mesh.Triangles.Where(t => Enumerable.Range(0, 3).All(i => t.GetVertex(i).Label == 0));
            Assert.That(interior.Count(t => MeshInvariants.Area(t) > 1.73 * (1 + 1e-9)), Is.Zero);
            Assert.That(mesh.Triangles.Count, Is.GreaterThan(10 * height / 1.73));
        }

        [Test]
        [Description("#13: maximum edge length via UserTest ('Ran out of precision').")]
        public void Issue13_MaximumEdgeLengthUserTest([Range(0, 4)] int seed)
        {
            // Survey points along a winding river.
            var random = new Random(seed);
            var poly = new Polygon();

            for (int i = 0; i < 400; i++)
            {
                double s = i * 5.0;
                double x = s + 30 * Math.Sin(s / 80);
                double y = 200 * Math.Sin(s / 300) + (random.NextDouble() - 0.5) * 40;
                poly.Add(new Vertex(500000 + x, 5700000 + y));
            }

            double maxLength = 200;

            bool TooLong(ITriangle t, double area)
            {
                for (int i = 0; i < 3; i++)
                {
                    Point p = t.GetVertex(i), q = t.GetVertex((i + 1) % 3);
                    if (Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y)) > maxLength) return true;
                }

                return false;
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { UserTest = TooLong, SteinerPoints = 100000 }), Timeout);

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => TooLong(t, 0)), Is.Zero);
        }

        [Test]
        [Description("#14: the capacity of Polygon(int) wasn't used.")]
        public void Issue14_PolygonCapacity()
        {
            Assert.That(new Polygon(1000).Points.Capacity, Is.EqualTo(1000));
        }

        [Test]
        [Description("#18: SimpleSmoother crashed with a duplicate input vertex.")]
        public void Issue18_SmoothingWithDuplicateVertex()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(100, 0, 1), new Vertex(100, 50, 1), new Vertex(0, 50, 1) }, 1));
            poly.Add(new Vertex(60, 10, 2));
            poly.Add(new Vertex(60, 10, 2));
            poly.Add(new Vertex(30, 30, 2));

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 25, MaximumArea = 50 });

            MeshAssert.Terminates(() => new SimpleSmoother().Smooth(mesh, 10), Timeout);

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 5000, 1e-9);
        }

        [Test]
        [Description("#35: example 5 (region area constraints with aCute) threw 'The new vertex is at the circumcenter of triangle'.")]
        public void Issue35_RegionAreaConstraints()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(10, 0, 1), new Vertex(10, 10, 1), new Vertex(0, 10, 1) }, 1));
            poly.Add(new Contour(new[] { new Vertex(2, 2, 2), new Vertex(5, 2, 2), new Vertex(5, 5, 2), new Vertex(2, 5, 2) }, 2), 1);
            poly.Regions.Add(new RegionPointer(3, 3, 1, 0.05));
            poly.Regions.Add(new RegionPointer(8, 8, 2, 0.5));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, VariableArea = true, SteinerPoints = 100000 }), Timeout);

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < 25 - 1e-6), Is.Zero);
        }

        [Test]
        [Description("#48: VertexCirculator threw a NullReferenceException for input vertices.")]
        public void Issue48_VertexCirculatorForInputVertices()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(1, 0, 1), new Vertex(1, 1, 1), new Vertex(0, 1, 1) }, 1));

            var points = new[] { new Vertex(0.4, 0.4), new Vertex(0.9, 0.9) };
            poly.Points.AddRange(points);

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions(), new QualityOptions { MinimumAngle = 25 });
            var circulator = new VertexCirculator(mesh);

            foreach (var v in points)
            {
                var triangles = circulator.EnumerateTriangles(v).ToList();

                Assert.That(triangles.Count, Is.GreaterThan(2));

                foreach (var t in triangles)
                {
                    ((Topology.Triangle)t).Area = 0.002;
                }
            }

            mesh.Refine(new QualityOptions { MinimumAngle = 25, VariableArea = true });

            MeshAssert.IsValid(mesh);
        }

        [Test]
        [Description("#54/#19: RobustPredicates.Default wasn't thread-safe (parallel triangulation with the default configuration).")]
        public void Issue54_ParallelTriangulationWithDefaultPredicates()
        {
            var errors = 0;

            Parallel.For(0, 200, i =>
            {
                var random = new Random(i);
                var poly = new Polygon();

                for (int k = 0; k < 300; k++)
                {
                    // Integer grid: many cocircular points, i.e. exact arithmetic is needed.
                    poly.Add(new Vertex(random.Next(0, 40), random.Next(0, 40)));
                }

                try
                {
                    var mesh = (Mesh)poly.Triangulate(new ConstraintOptions(), new QualityOptions { MinimumAngle = 25 });
                    if (!MeshValidator.IsConsistent(mesh)) System.Threading.Interlocked.Increment(ref errors);
                }
                catch (Exception)
                {
                    System.Threading.Interlocked.Increment(ref errors);
                }
            });

            Assert.That(errors, Is.Zero);
        }

        [Test]
        [Description("#59: SweepLine.RightOfHyperbola: NullReferenceException (no input data in the report; random and degenerate inputs).")]
        public void Issue59_SweepLine([Range(0, 49)] int seed)
        {
            var random = new Random(seed);
            var points = new List<Vertex>();
            int n = 50 + random.Next(500);

            for (int i = 0; i < n; i++)
            {
                switch (seed % 4)
                {
                    case 0: points.Add(new Vertex(random.NextDouble(), random.NextDouble())); break;
                    case 1: points.Add(new Vertex(random.Next(0, 10), random.Next(0, 10))); break; // duplicates, collinear
                    case 2: points.Add(new Vertex(random.Next(0, 30), 5)); break; // all collinear
                    default: points.Add(new Vertex(500000 + random.Next(0, 100) * 0.01, 5700000 + random.Next(0, 100) * 0.01)); break;
                }
            }

            var mesh = MeshAssert.Terminates(() => new GenericMesher(new SweepLine()).Triangulate(points), Timeout);

            if (mesh.Triangles.Count > 0)
            {
                MeshAssert.IsValid(mesh);
            }
        }

        private static double Area(IList<Vertex> ring)
        {
            double a = 0;

            for (int i = 0; i < ring.Count; i++)
            {
                var p = ring[i];
                var q = ring[(i + 1) % ring.Count];
                a += p.X * q.Y - q.X * p.Y;
            }

            return Math.Abs(a) / 2;
        }
    }
}
