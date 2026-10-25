using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Tests.Characterization;
using TriangleNet.Tests.Corpus;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Standard benchmarks for (constrained) Delaunay triangulation: hard predicate cases,
    /// Ruppert's maple leaf, and synthetic edge cases (grid, concentric circles, spiky star).
    /// Larger external data (Natural Earth coastlines, the test inputs of artem-ogre/CDT) are
    /// in <see cref="CorpusTest"/>.
    /// </summary>
    public class BenchmarkDatasetsTest
    {
        private static readonly string DataDirectory =
            Path.Combine(Path.GetDirectoryName(typeof(BenchmarkDatasetsTest).Assembly.Location), "Robustness", "Data");

        #region Predicates

        private static IEnumerable<(int Index, double[] C, int Sign)> Fixtures(string name, int n)
        {
            foreach (var line in File.ReadLines(Path.Combine(DataDirectory, name)))
            {
                var t = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);

                if (t.Length != n + 2) continue;

                var c = t.Skip(1).Take(n).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();

                yield return (int.Parse(t[0], CultureInfo.InvariantCulture), c, int.Parse(t[n + 1], CultureInfo.InvariantCulture));
            }
        }

        [Test]
        public void Orient2dHardCases()
        {
            var predicates = new RobustPredicates();
            var fixtures = Fixtures("orient2d.txt", 6).ToList();

            Assert.That(fixtures.Count, Is.EqualTo(1000));

            var wrong = fixtures.Where(f => Math.Sign(predicates.CounterClockwise(
                new Point(f.C[0], f.C[1]), new Point(f.C[2], f.C[3]), new Point(f.C[4], f.C[5]))) != f.Sign)
                .Select(f => f.Index).ToList();

            Assert.That(wrong, Is.Empty);
        }

        [Test]
        public void InCircleHardCases()
        {
            var predicates = new RobustPredicates();
            var fixtures = Fixtures("incircle.txt", 8).ToList();

            Assert.That(fixtures.Count, Is.EqualTo(1000));

            var wrong = fixtures.Where(f => Math.Sign(predicates.InCircle(
                new Point(f.C[0], f.C[1]), new Point(f.C[2], f.C[3]), new Point(f.C[4], f.C[5]), new Point(f.C[6], f.C[7]))) != f.Sign)
                .Select(f => f.Index).ToList();

            Assert.That(wrong, Is.Empty);
        }

        #endregion

        #region Maple leaf

        private static Polygon MapleLeaf()
        {
            var c = CorpusCase.Load(Path.Combine(DataDirectory, "maple-leaf.txt"));
            var poly = new Polygon();
            var map = new Dictionary<(double, double), Vertex>();

            Vertex V((double, double) p)
            {
                if (!map.TryGetValue(p, out var v))
                {
                    map[p] = v = new Vertex(p.Item1, p.Item2);
                    poly.Add(v);
                }

                return v;
            }

            foreach (var line in c.Lines)
            {
                for (int i = 0; i + 1 < line.Count; i++)
                {
                    poly.Add(new Segment(V(line[i]), V(line[i + 1])));
                }
            }

            return poly;
        }

        [TestCase(0)]
        [TestCase(20)]
        [TestCase(30)]
        [TestCase(33)]
        public void MapleLeaf(double angle)
        {
            var poly = MapleLeaf();

            var mesh = (Mesh)MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                angle > 0 ? new QualityOptions { MinimumAngle = angle } : null));

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);

            if (angle > 0)
            {
                // Bad triangles only at small input angles (the leaf has sharp notches).
                int bad = mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < angle - 0.01);
                Assert.That(bad, Is.LessThanOrEqualTo(Math.Max(10, mesh.Triangles.Count / 100)), $"{bad} of {mesh.Triangles.Count} triangles");
            }
        }

        #endregion

        #region Grid

        private static List<Vertex> Grid(int n, double spacing = 1.0, double x0 = 0, double y0 = 0)
        {
            var points = new List<Vertex>();

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    points.Add(new Vertex(x0 + i * spacing, y0 + j * spacing));
                }
            }

            return points;
        }

        [TestCase(10, 1.0, 0, 0)]
        [TestCase(10, 0.1, 0, 0)]
        [TestCase(10, 1.0, 372000, 5710000)]
        [TestCase(100, 1.0, 372000, 5710000)]
        public void GridIsDeterministicAndDelaunay(int n, double spacing, double x0, double y0)
        {
            // All squares are cocircular: the diagonals are arbitrary but must be chosen
            // consistently (same result for the same input, valid in any order).
            var first = (Mesh)new GenericMesher().Triangulate(Grid(n, spacing, x0, y0));
            var second = (Mesh)new GenericMesher().Triangulate(Grid(n, spacing, x0, y0));

            MeshAssert.IsValid(first);
            Assert.That(MeshInvariants.Fingerprint(first), Is.EqualTo(MeshInvariants.Fingerprint(second)));
            Assert.That(first.Triangles.Count, Is.EqualTo(2 * (n - 1) * (n - 1)));

            var random = new Random(n);
            var shuffled = Grid(n, spacing, x0, y0).OrderBy(_ => random.Next()).ToList();

            foreach (var algorithm in new ITriangulator[] { new TriangleNet.Meshing.Algorithm.Dwyer(),
                new TriangleNet.Meshing.Algorithm.Incremental(), new TriangleNet.Meshing.Algorithm.SweepLine() })
            {
                var mesh = (Mesh)algorithm.Triangulate(shuffled, new Configuration());

                MeshAssert.IsValid(mesh);
                Assert.That(mesh.Triangles.Count, Is.EqualTo(2 * (n - 1) * (n - 1)), algorithm.GetType().Name);
                Assert.That(MeshValidator.IsDelaunay(mesh), Is.True, algorithm.GetType().Name);
            }
        }

        [Test]
        public void GridWithConstraintsAndQuality()
        {
            // Diagonal constraints through cocircular grid points.
            var poly = new Polygon();
            poly.Points.AddRange(Grid(10));
            poly.Add(new Segment(new Vertex(0, 0), new Vertex(9, 9)), true);
            poly.Add(new Segment(new Vertex(0, 9), new Vertex(9, 0)), true);

            var mesh = (Mesh)MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { Convex = true },
                new QualityOptions { MinimumAngle = 30 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            MeshAssert.HasArea(mesh, 81);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < 29.99), Is.EqualTo(0));
        }

        #endregion

        #region Concentric circles

        private static Polygon ConcentricCircles(int inner, int outer, bool hole, double r1 = 1.0, double r2 = 2.0)
        {
            var poly = new Polygon();

            for (int i = 0; i < outer; i++)
            {
                double a = 2 * Math.PI * i / outer;
                poly.Add(new Vertex(r2 * Math.Cos(a), r2 * Math.Sin(a)));
            }

            var ring = Enumerable.Range(0, inner)
                .Select(i => 2 * Math.PI * i / inner)
                .Select(a => new Vertex(r1 * Math.Cos(a), r1 * Math.Sin(a))).ToList();

            poly.Add(new Contour(ring), hole);

            return poly;
        }

        private static double PolygonArea(IList<Vertex> ring)
        {
            double sum = 0;

            for (int i = 0, j = ring.Count - 1; i < ring.Count; j = i++)
            {
                sum += ring[j].X * ring[i].Y - ring[i].X * ring[j].Y;
            }

            return 0.5 * Math.Abs(sum);
        }

        [TestCase(16, 32)]
        [TestCase(64, 64)]
        [TestCase(200, 37)]
        public void ConcentricCirclesWithHole(int inner, int outer)
        {
            var poly = ConcentricCircles(inner, outer, hole: true);

            foreach (var quality in new QualityOptions[] { null, new QualityOptions { MinimumAngle = 30 } })
            {
                var mesh = (Mesh)MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { Convex = true }, quality));

                MeshAssert.IsValid(mesh);
                MeshAssert.ContainsSegments(mesh, poly.Segments);

                // No triangle inside the inner circle (the hole), the annulus is meshed completely.
                var outerRing = poly.Points.Take(outer).ToList();
                var innerRing = poly.Points.Skip(outer).ToList();

                Assert.That(mesh.Triangles.Any(t => Centroid(t).Length() < 1 - 1e-9 && InsidePolygon(innerRing, Centroid(t))), Is.False);
                MeshAssert.HasArea(mesh, PolygonArea(outerRing) - PolygonArea(innerRing));
            }
        }

        [TestCase(16, 32)]
        [TestCase(200, 37)]
        public void ConcentricCirclesWithoutHoleSeparateInsideAndOutside(int inner, int outer)
        {
            var poly = ConcentricCircles(inner, outer, hole: false);

            var mesh = (Mesh)MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { Convex = true },
                new QualityOptions { MinimumAngle = 25 }));

            MeshAssert.IsValid(mesh);
            MeshAssert.ContainsSegments(mesh, poly.Segments);

            // No triangle crosses the inner circle: none has a vertex strictly inside and one
            // strictly outside (vertices on the circle belong to both sides).
            var innerRing = poly.Points.Skip(outer).ToList();

            foreach (var t in mesh.Triangles)
            {
                var vertices = Enumerable.Range(0, 3).Select(i => t.GetVertex(i)).ToList();

                bool strictlyInside = vertices.Any(v => !OnBoundary(innerRing, v) && InsidePolygon(innerRing, v));
                bool strictlyOutside = vertices.Any(v => !OnBoundary(innerRing, v) && !InsidePolygon(innerRing, v));

                Assert.That(strictlyInside && strictlyOutside, Is.False, $"triangle {t.ID} crosses the circle");
            }
        }

        private static Point Centroid(ITriangle t) => new Point(
            (t.GetVertex(0).X + t.GetVertex(1).X + t.GetVertex(2).X) / 3,
            (t.GetVertex(0).Y + t.GetVertex(1).Y + t.GetVertex(2).Y) / 3);

        private static bool InsidePolygon(IList<Vertex> ring, Point p)
        {
            bool inside = false;

            for (int i = 0, j = ring.Count - 1; i < ring.Count; j = i++)
            {
                if ((ring[i].Y > p.Y) != (ring[j].Y > p.Y) &&
                    p.X < (ring[j].X - ring[i].X) * (p.Y - ring[i].Y) / (ring[j].Y - ring[i].Y) + ring[i].X)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        private static bool OnBoundary(IList<Vertex> ring, Point p)
        {
            for (int i = 0, j = ring.Count - 1; i < ring.Count; j = i++)
            {
                if (RobustPredicates.Default.CounterClockwise(ring[j], ring[i], p) == 0 &&
                    Math.Min(ring[i].X, ring[j].X) <= p.X && p.X <= Math.Max(ring[i].X, ring[j].X) &&
                    Math.Min(ring[i].Y, ring[j].Y) <= p.Y && p.Y <= Math.Max(ring[i].Y, ring[j].Y))
                {
                    return true;
                }
            }

            return false;
        }

        #endregion

        #region Spiky star

        private static Polygon Star(int spikes, double tipAngle, double scale = 1.0, double x0 = 0, double y0 = 0)
        {
            // Spikes of length 'scale' around a core of radius 0.2 * scale. Each spike has the
            // given tip angle (radians), so for tiny angles its two base vertices nearly coincide.
            const double core = 0.2;
            double delta = Math.Asin(Math.Min(1.0, (1 - core) * Math.Tan(0.5 * tipAngle) / core));

            var contour = new List<Vertex>();

            Vertex V(double radius, double angle) =>
                new Vertex(x0 + scale * radius * Math.Cos(angle), y0 + scale * radius * Math.Sin(angle));

            for (int i = 0; i < spikes; i++)
            {
                double a = 2 * Math.PI * i / spikes;

                contour.Add(V(core, a - delta));
                contour.Add(V(1.0, a));
                contour.Add(V(core, a + delta));
            }

            var poly = new Polygon();
            poly.Add(new Contour(contour));

            return poly;
        }

        [TestCase(5, 1e-3)]
        [TestCase(5, 1e-6)]
        [TestCase(5, 1e-9)]
        [TestCase(12, 1e-12)]
        [TestCase(7, 1e-15)]
        public void SpikyStar(int spikes, double tipAngle)
        {
            foreach (var (scale, x0, y0) in new[] { (1.0, 0.0, 0.0), (100.0, 372000.0, 5710000.0), (1e-6, 0.0, 0.0) })
            {
                var poly = Star(spikes, tipAngle, scale, x0, y0);
                var ring = poly.Points.ToList();

                // The base vertices of a spike may be closer than the precision of the
                // coordinates allows (merged then, the spike becomes a segment).
                var cdt = (Mesh)MeshAssert.Terminates(() => poly.Triangulate());
                MeshAssert.IsValid(cdt);
                MeshAssert.HasArea(cdt, PolygonArea(ring), 1e-6);

                var quality = (Mesh)MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                    new QualityOptions { MinimumAngle = 30, SteinerPoints = 20000 }));
                MeshAssert.IsValid(quality);
                MeshAssert.HasArea(quality, PolygonArea(ring), 1e-6);
            }
        }

        #endregion
    }

    internal static class PointExtensions
    {
        public static double Length(this Point p) => Math.Sqrt(p.X * p.X + p.Y * p.Y);
    }
}
