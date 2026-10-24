using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Iterators;
using TriangleNet.Topology;
using TriangleNet.Topology.DCEL;
using TriangleNet.Tests.Robustness;

using TVertex = TriangleNet.Geometry.Vertex;
using HVertex = TriangleNet.Topology.DCEL.Vertex;

namespace TriangleNet.Tests.Review
{
    /// <summary>
    /// Review D: Geometry, Topology, DCEL and mesh iterators.
    /// </summary>
    public class ReviewDTest
    {
        #region Helpers

        private static Mesh RandomPointMesh(int n, int seed)
        {
            var rnd = new Random(seed);
            var poly = new Polygon(n);
            for (int i = 0; i < n; i++)
            {
                poly.Add(new TVertex(rnd.NextDouble() * 100, rnd.NextDouble() * 100));
            }
            return (Mesh)poly.Triangulate();
        }

        private static Mesh BoxWithHoleMesh(bool quality)
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[]
            {
                new TVertex(0, 0), new TVertex(10, 0), new TVertex(10, 10), new TVertex(0, 10)
            }, 1));
            poly.Add(new Contour(new[]
            {
                new TVertex(4, 4), new TVertex(6, 4), new TVertex(6, 6), new TVertex(4, 6)
            }, 2), true);

            return quality
                ? (Mesh)poly.Triangulate(new QualityOptions() { MinimumAngle = 25, MaximumArea = 2 })
                : (Mesh)poly.Triangulate();
        }

        private static bool StrictlyInside(Point p, IList<TVertex> poly)
        {
            // Even-odd test plus distance to boundary.
            bool inside = false;
            int n = poly.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var a = poly[i]; var b = poly[j];
                if ((a.Y > p.Y) != (b.Y > p.Y) &&
                    p.X < (b.X - a.X) * (p.Y - a.Y) / (b.Y - a.Y) + a.X)
                {
                    inside = !inside;
                }
                if (RobustPredicates.Default.CounterClockwise(a, b, p) == 0.0 &&
                    Math.Min(a.X, b.X) <= p.X && p.X <= Math.Max(a.X, b.X) &&
                    Math.Min(a.Y, b.Y) <= p.Y && p.Y <= Math.Max(a.Y, b.Y))
                {
                    return false;
                }
            }
            return inside;
        }

        #endregion

        #region Point / Vertex

        [Test]
        public void PointEqualsAndHashCodeAgreeForSignedZero()
        {
            var a = new Point(0.0, 0.0);
            var b = new Point(-0.0, -0.0);

            Assert.That(a.Equals(b), Is.True);
            Assert.That(a.GetHashCode(), Is.EqualTo(b.GetHashCode()));
            Assert.That(new HashSet<Point> { a }.Contains(b), Is.True);
        }

        [Test]
        public void PointAndVertexEqualityIsSymmetric()
        {
            var p = new Point(1, 2);
            var v = new TVertex(1, 2);

            Assert.That(p.Equals(v), Is.EqualTo(v.Equals(p)));
            Assert.That(p == v, Is.EqualTo(v == p));
            Assert.That(p.GetHashCode(), Is.EqualTo(v.GetHashCode()));
        }

        [Test]
        [Description("Point.CompareTo(null) throws NullReferenceException; IComparable<T> requires " +
            "any instance to compare greater than null (return > 0).")]
        public void PointCompareToNullFollowsIComparableContract()
        {
            var p = new Point(1, 2);
            int result = 0;

            Assert.That(() => result = p.CompareTo(null), Throws.Nothing);
            Assert.That(result, Is.GreaterThan(0));
        }

        [Test]
        public void PointCompareToIsConsistentWithEquals()
        {
            var rnd = new Random(7);
            var pts = Enumerable.Range(0, 200)
                .Select(_ => new Point(rnd.Next(5), rnd.Next(5))).ToList();

            foreach (var a in pts)
            {
                foreach (var b in pts)
                {
                    Assert.That(a.CompareTo(b) == 0, Is.EqualTo(a.Equals(b)));
                    Assert.That(Math.Sign(a.CompareTo(b)), Is.EqualTo(-Math.Sign(b.CompareTo(a))));
                }
            }
        }

        #endregion

        #region Rectangle

        [Test]
        [Description("Rectangle(Rectangle other) forwards (Left, Bottom, Right, Top) to the " +
            "(x, y, width, height) constructor, so the copy has xmax = Left + Right and ymax = Bottom + Top.")]
        public void RectangleCopyConstructorCopiesBounds()
        {
            var r = new Rectangle(1, 2, 3, 4); // [1,4] x [2,6]
            var copy = new Rectangle(r);

            Assert.That(copy.Left, Is.EqualTo(r.Left));
            Assert.That(copy.Bottom, Is.EqualTo(r.Bottom));
            Assert.That(copy.Right, Is.EqualTo(r.Right));
            Assert.That(copy.Top, Is.EqualTo(r.Top));
        }

        [Test]
        [Description("Rectangle.Intersects uses strict comparisons, so a degenerate (zero width/height) " +
            "rectangle never intersects anything, not even a rectangle that Contains it.")]
        public void RectangleContainsImpliesIntersects()
        {
            var r = new Rectangle(0, 0, 10, 10);
            var flat = new Rectangle(2, 0, 6, 0); // horizontal line on the bottom edge of r

            Assert.That(r.Contains(flat), Is.True);
            Assert.That(r.Intersects(flat), Is.True, "Contained rectangle must intersect.");
            Assert.That(flat.Intersects(flat), Is.True, "Non-empty rectangle must intersect itself.");
        }

        [Test]
        public void RectangleExpandFromEmpty()
        {
            var r = new Rectangle();
            Assert.That(r.Contains(0, 0), Is.False);

            r.Expand(new Point(1, 2));
            Assert.That(r.Width, Is.EqualTo(0.0));
            Assert.That(r.Height, Is.EqualTo(0.0));
            Assert.That(r.Contains(1, 2), Is.True);

            var e = new Rectangle();
            e.Expand(new Rectangle());
            r.Expand(e);
            Assert.That(r.Left, Is.EqualTo(1.0));
            Assert.That(r.Top, Is.EqualTo(2.0));
        }

        #endregion

        #region Contour / Polygon

        [Test]
        public void ContourRemovesClosingDuplicate()
        {
            var open = new Contour(new[] { new TVertex(0, 0), new TVertex(1, 0), new TVertex(0, 1) });
            var closed = new Contour(new[] { new TVertex(0, 0), new TVertex(1, 0), new TVertex(0, 1), new TVertex(0, 0) });

            Assert.That(open.Points.Count, Is.EqualTo(3));
            Assert.That(closed.Points.Count, Is.EqualTo(3));
            Assert.That(closed.GetSegments().Count, Is.EqualTo(3));
        }

        [Test]
        [Description("Contour.FindInteriorPoint uses an absolute collinearity threshold (eps = 2e-5 on " +
            "the orientation determinant). For a thin but valid contour every corner looks collinear, " +
            "the perpendicular probe steps far outside, and a bare System.Exception is thrown.")]
        public void FindInteriorPointOfThinRectangle()
        {
            // 10 x 1e-6 rectangle (orientation determinant 1e-5 < eps at every corner).
            var contour = new Contour(new[]
            {
                new TVertex(0, 0), new TVertex(10, 0), new TVertex(10, 1e-6), new TVertex(0, 1e-6)
            });

            Point p = null;
            Assert.That(() => p = contour.FindInteriorPoint(), Throws.Nothing);
            Assert.That(StrictlyInside(p, contour.Points), Is.True);
        }

        [Test]
        [Description("Polygon.Add(contour, hole: true) fails with a bare System.Exception for a thin " +
            "hole contour because Contour.FindInteriorPoint uses an absolute eps (see FindInteriorPointOfThinRectangle).")]
        public void PolygonAddThinHoleContour()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[]
            {
                new TVertex(-1, -1), new TVertex(11, -1), new TVertex(11, 1), new TVertex(-1, 1)
            }));
            var hole = new Contour(new[]
            {
                new TVertex(0, 0), new TVertex(10, 0), new TVertex(5, 2e-6)
            });

            Assert.That(() => poly.Add(hole, true), Throws.Nothing);
        }

        [Test]
        public void FindInteriorPointRandomStarPolygonsUnitScale()
        {
            var failures = StarPolygonFailures(1.0, out int tested);
            Assert.That(failures, Is.Empty,
                $"{failures.Count}/{tested} failed: " + string.Join("; ", failures.Take(5)));
        }

        [TestCase(1e-3)]
        [TestCase(1e5)]
        [Description("Contour.FindInteriorPoint is not scale invariant: the collinearity eps (2e-5) and " +
            "IntersectionHelper.IsPointOnSegment's eps (1e-12) are absolute values on cross products. At small " +
            "scale it throws for valid obtuse triangles, at large scale it returns the midpoint of an edge " +
            "(a point on the contour) because the on-segment rejection never fires.")]
        public void FindInteriorPointRandomStarPolygonsOtherScales(double scale)
        {
            var failures = StarPolygonFailures(scale, out int tested);
            Assert.That(failures, Is.Empty,
                $"{failures.Count}/{tested} failed at scale {scale}: " + string.Join("; ", failures.Take(3)));
        }

        private static List<string> StarPolygonFailures(double scale, out int tested)
        {
            var rnd = new Random(42);
            tested = 0;
            var failures = new List<string>();

            for (int k = 0; k < 300; k++)
            {
                int n = 3 + rnd.Next(20);
                var angles = Enumerable.Range(0, n).Select(_ => rnd.NextDouble() * 2 * Math.PI)
                    .OrderBy(a => a).ToList();
                var pts = angles.Select(a =>
                {
                    double r = (0.1 + rnd.NextDouble()) * scale;
                    return new TVertex(1e3 * scale + r * Math.Cos(a), r * Math.Sin(a));
                }).ToList();

                // Skip degenerate (nearly collinear) polygons.
                double area = 0;
                for (int i = 0, j = n - 1; i < n; j = i++)
                    area += (pts[j].X - 1e3 * scale) * pts[i].Y - (pts[i].X - 1e3 * scale) * pts[j].Y;
                if (Math.Abs(area) < 1e-2 * scale * scale) continue;

                foreach (var list in new[] { pts, Enumerable.Reverse(pts).ToList() })
                {
                    tested++;
                    var c = new Contour(list);
                    try
                    {
                        var p = c.FindInteriorPoint();
                        if (!StrictlyInside(p, c.Points))
                            failures.Add($"k={k} n={n}: ({p.X:R}, {p.Y:R}) not inside " +
                                string.Join(" ", c.Points.Select(q => $"({q.X:R},{q.Y:R})")));
                    }
                    catch (Exception e)
                    {
                        failures.Add($"k={k} n={n}: {e.GetType().Name} " +
                            string.Join(" ", c.Points.Select(q => $"({q.X:R},{q.Y:R})")));
                    }
                }
            }

            return failures;
        }

        [Test]
        [Description("With projected (UTM-like) coordinates, Polygon.Add(contour, hole: true) can place the " +
            "hole seed on the hole's own boundary (FindInteriorPoint returns an edge midpoint because the " +
            "absolute eps 1e-12 in IsPointOnSegment never matches), so the hole point is not strictly inside.")]
        public void HoleSeedStrictlyInsideForUtmCoordinates()
        {
            var rnd = new Random(1);
            var failures = new List<string>();
            for (int k = 0; k < 200; k++)
            {
                double x0 = 370000 + rnd.NextDouble() * 1000, y0 = 5710000 + rnd.NextDouble() * 1000;
                var tri = Enumerable.Range(0, 3)
                    .Select(_ => new TVertex(x0 + rnd.NextDouble() * 50, y0 + rnd.NextDouble() * 50)).ToList();
                double det = RobustPredicates.Default.CounterClockwise(tri[0], tri[1], tri[2]);
                if (Math.Abs(det) < 100) continue; // area >= 50 m^2

                var poly = new Polygon();
                poly.Add(new Contour(tri), true);
                var h = poly.Holes[0];

                // Distance of the seed to the closest edge, relative to the inradius scale.
                double minDist = double.MaxValue;
                for (int i = 0; i < 3; i++)
                {
                    var a = tri[i]; var b = tri[(i + 1) % 3];
                    double len = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                    double d = Math.Abs(RobustPredicates.Default.CounterClockwise(a, b, h)) / len;
                    minDist = Math.Min(minDist, d);
                }
                if (minDist < 1e-6)
                    failures.Add($"k={k}: seed ({h.X:R},{h.Y:R}) has distance {minDist:G3} to the hole boundary");
            }
            Assert.That(failures, Is.Empty, $"{failures.Count} seeds on boundary: " + string.Join("; ", failures.Take(3)));
        }

        [Test]
        [Description("End-to-end effect of the hole-seed problem: for a triangular hole in UTM-like " +
            "coordinates the seed lies (numerically) on the hole boundary; when it rounds to the outside, " +
            "the hole virus eats the surrounding domain instead of the hole and the mesh area is wrong.")]
        public void TriangularHoleInUtmCoordinatesIsCarvedCorrectly()
        {
            var rnd = new Random(1);
            var failures = new List<string>();
            int tested = 0;
            for (int k = 0; k < 200; k++)
            {
                double x0 = 370000 + rnd.NextDouble() * 1000, y0 = 5710000 + rnd.NextDouble() * 1000;
                var tri = Enumerable.Range(0, 3)
                    .Select(_ => new TVertex(x0 + rnd.NextDouble() * 50, y0 + rnd.NextDouble() * 50)).ToList();
                double det = RobustPredicates.Default.CounterClockwise(tri[0], tri[1], tri[2]);
                if (Math.Abs(det) < 100) continue;
                tested++;

                var poly = new Polygon();
                poly.Add(new Contour(new[]
                {
                    new TVertex(x0 - 10, y0 - 10), new TVertex(x0 + 60, y0 - 10),
                    new TVertex(x0 + 60, y0 + 60), new TVertex(x0 - 10, y0 + 60)
                }));
                poly.Add(new Contour(tri), true);

                var mesh = poly.Triangulate();
                double area = mesh.Triangles.Sum(t =>
                    0.5 * RobustPredicates.Default.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)));
                double expected = 70 * 70 - 0.5 * Math.Abs(det);
                if (Math.Abs(area - expected) > 1e-6 * expected)
                    failures.Add($"k={k}: area {area:F3} expected {expected:F3}");
            }
            Assert.That(failures, Is.Empty, $"{failures.Count}/{tested} wrong: " + string.Join("; ", failures.Take(3)));
        }

        [Test]
        public void PolygonBoundsAndHoleHandlingWithClockwiseHole()
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[]
            {
                new TVertex(0, 0), new TVertex(10, 0), new TVertex(10, 10), new TVertex(0, 10)
            }));
            // Clockwise, concave hole (L-shape).
            poly.Add(new Contour(new[]
            {
                new TVertex(2, 2), new TVertex(2, 8), new TVertex(4, 8), new TVertex(4, 4),
                new TVertex(8, 4), new TVertex(8, 2)
            }), true);

            var b = poly.Bounds();
            Assert.That((b.Left, b.Bottom, b.Right, b.Top), Is.EqualTo((0.0, 0.0, 10.0, 10.0)));

            var mesh = poly.Triangulate();
            double area = mesh.Triangles.Sum(t =>
                0.5 * RobustPredicates.Default.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)));
            Assert.That(area, Is.EqualTo(100 - (12 + 8)).Within(1e-9));
        }

        #endregion

        #region ITriangle extensions

        [Test]
        public void TriangleContainsAndBounds()
        {
            var mesh = RandomPointMesh(50, 3);
            var rnd = new Random(5);

            foreach (var t in mesh.Triangles)
            {
                var a = t.GetVertex(0); var b = t.GetVertex(1); var c = t.GetVertex(2);
                Assert.That(t.Contains(a) && t.Contains(b) && t.Contains(c), Is.True);
                var centroid = new Point((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3);
                Assert.That(t.Contains(centroid), Is.True);
                Assert.That(t.Bounds().Contains(centroid), Is.True);
            }

            // Every random point in the convex hull is in exactly one or more (edge) triangles,
            // and never in a triangle that does not geometrically contain it.
            for (int i = 0; i < 200; i++)
            {
                var p = new Point(rnd.NextDouble() * 100, rnd.NextDouble() * 100);
                foreach (var t in mesh.Triangles)
                {
                    var o = RobustPredicates.Default;
                    bool inside = o.CounterClockwise(t.GetVertex(0), t.GetVertex(1), p) >= 0
                               && o.CounterClockwise(t.GetVertex(1), t.GetVertex(2), p) >= 0
                               && o.CounterClockwise(t.GetVertex(2), t.GetVertex(0), p) >= 0;
                    Assert.That(t.Contains(p), Is.EqualTo(inside));
                }
            }
        }

        #endregion

        #region Otri / Osub / Triangle

        [Test]
        public void OtriNavigationIdentities()
        {
            var mesh = BoxWithHoleMesh(true);
            Otri e = default, x = default, y = default;

            foreach (var t in mesh.triangles)
            {
                for (int o = 0; o < 3; o++)
                {
                    e.tri = t; e.orient = o;

                    // Lnext^3 = id, Lnext∘Lprev = id.
                    e.Lnext(ref x); x.Lnext(); x.Lnext();
                    Assert.That(x.Equals(e), Is.True);
                    e.Lnext(ref x); x.Lprev();
                    Assert.That(x.Equals(e), Is.True);

                    // Vertex relations.
                    e.Lnext(ref x);
                    Assert.That(x.Org(), Is.SameAs(e.Dest()));
                    Assert.That(x.Dest(), Is.SameAs(e.Apex()));

                    e.Sym(ref x);
                    if (x.tri.id == Mesh.DUMMY) continue;

                    // Sym∘Sym = id, Sym swaps org/dest.
                    Assert.That(x.Org(), Is.SameAs(e.Dest()));
                    Assert.That(x.Dest(), Is.SameAs(e.Org()));
                    x.Sym(ref y);
                    Assert.That(y.Equals(e), Is.True);

                    // Oprev: same origin.
                    e.Oprev(ref x);
                    Assert.That(x.Org(), Is.SameAs(e.Org()));
                    if (x.Lprev_SymExists())
                    {
                        x.Onext(ref y);
                        Assert.That(y.Equals(e), Is.True, "Onext(Oprev(e)) != e");
                    }

                    // Dnext: same destination, Dprev inverse.
                    e.Dnext(ref x);
                    Assert.That(x.Dest(), Is.SameAs(e.Dest()));
                    x.Dprev(ref y);
                    if (y.tri.id != Mesh.DUMMY)
                        Assert.That(y.Equals(e), Is.True, "Dprev(Dnext(e)) != e");

                    // Rnext/Rprev inverse when both steps stay in the mesh.
                    e.Rnext(ref x);
                    if (x.tri.id != Mesh.DUMMY)
                    {
                        x.Rprev(ref y);
                        Assert.That(y.Equals(e), Is.True, "Rprev(Rnext(e)) != e");
                    }
                }

                for (int o = 0; o < 3; o++)
                {
                    e.tri = t; e.orient = o;
                    e.Onext(ref x);
                    if (x.tri.id == Mesh.DUMMY) continue;
                    Assert.That(x.Org(), Is.SameAs(e.Org()));
                    x.Oprev(ref y);
                    Assert.That(y.Equals(e), Is.True, "Oprev(Onext(e)) != e");
                }
            }
        }

        [Test]
        public void TriangleNeighborAndSegmentAccessors()
        {
            var mesh = BoxWithHoleMesh(false);
            mesh.Renumber();

            int boundaryEdges = 0;
            foreach (var t in mesh.Triangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    var n = t.GetNeighbor(i);
                    Assert.That(t.GetVertexID(i), Is.EqualTo(t.GetVertex(i).ID));
                    if (n == null)
                    {
                        boundaryEdges++;
                        Assert.That(t.GetNeighborID(i), Is.EqualTo(-1));
                        // Every boundary edge must be a segment in a PSLG mesh.
                        Assert.That(t.GetSegment(i), Is.Not.Null);
                    }
                    else
                    {
                        Assert.That(t.GetNeighborID(i), Is.EqualTo(n.ID));
                        // Neighbor relation is symmetric.
                        Assert.That(Enumerable.Range(0, 3).Any(j => n.GetNeighbor(j) == t), Is.True);
                    }

                    var s = t.GetSegment(i);
                    if (s != null)
                    {
                        // Segment endpoints are the edge opposite vertex i.
                        var ids = new[] { s.P0, s.P1 }.OrderBy(v => v).ToArray();
                        var exp = new[] { t.GetVertexID((i + 1) % 3), t.GetVertexID((i + 2) % 3) }
                            .OrderBy(v => v).ToArray();
                        Assert.That(ids, Is.EqualTo(exp));
                        var st = new[] { s.GetTriangle(0), s.GetTriangle(1) };
                        Assert.That(st, Does.Contain(t));
                    }
                }
            }
            Assert.That(boundaryEdges, Is.EqualTo(8));
        }

        #endregion

        #region EdgeIterator

        [Test]
        public void EdgeIteratorEnumeratesEachEdgeOnce([Values(10, 100, 1000)] int n)
        {
            var mesh = RandomPointMesh(n, n);
            var edges = mesh.Edges.ToList();

            var keys = edges.Select(e => (Math.Min(e.P0, e.P1), Math.Max(e.P0, e.P1))).ToList();
            Assert.That(keys.Distinct().Count(), Is.EqualTo(keys.Count), "Duplicate edges.");

            var expected = new HashSet<(int, int)>();
            foreach (var t in mesh.Triangles)
                for (int i = 0; i < 3; i++)
                {
                    int a = t.GetVertexID(i), b = t.GetVertexID((i + 1) % 3);
                    expected.Add((Math.Min(a, b), Math.Max(a, b)));
                }
            Assert.That(keys, Is.EquivalentTo(expected));

            // Euler for a triangulated convex hull: E = V + T - 1.
            Assert.That(edges.Count, Is.EqualTo(mesh.Vertices.Count + mesh.Triangles.Count - 1));

            var segs = EdgeIterator.EnumerateEdges(mesh, false).Count();
            Assert.That(segs, Is.EqualTo(edges.Count));
        }

        [Test]
        public void EdgeIteratorLabelsAndHoles()
        {
            var mesh = BoxWithHoleMesh(true);
            var edges = mesh.Edges.ToList();
            var expected = new HashSet<(int, int)>();
            foreach (var t in mesh.Triangles)
                for (int i = 0; i < 3; i++)
                {
                    int a = t.GetVertexID(i), b = t.GetVertexID((i + 1) % 3);
                    expected.Add((Math.Min(a, b), Math.Max(a, b)));
                }
            Assert.That(edges.Select(e => (Math.Min(e.P0, e.P1), Math.Max(e.P0, e.P1))), Is.EquivalentTo(expected));

            int segCount = mesh.Segments.Count;
            Assert.That(edges.Count(e => e.Label != 0), Is.EqualTo(segCount));
            Assert.That(EdgeIterator.EnumerateEdges(mesh, true).Count(), Is.EqualTo(edges.Count - segCount));
        }

        #endregion

        #region VertexCirculator

        private static Dictionary<TVertex, HashSet<TVertex>> NeighborSets(Mesh mesh)
        {
            var map = new Dictionary<TVertex, HashSet<TVertex>>(ReferenceEqualityComparer.Instance);
            foreach (var t in mesh.Triangles)
                for (int i = 0; i < 3; i++)
                {
                    var a = t.GetVertex(i);
                    if (!map.TryGetValue(a, out var set)) map[a] = set = new HashSet<TVertex>(ReferenceEqualityComparer.Instance);
                    set.Add(t.GetVertex((i + 1) % 3));
                    set.Add(t.GetVertex((i + 2) % 3));
                }
            return map;
        }

        [Test]
        public void VertexCirculatorMatchesTopology([Values(false, true)] bool hole)
        {
            var mesh = hole ? BoxWithHoleMesh(true) : RandomPointMesh(200, 11);
            var nb = NeighborSets(mesh);
            var circ = new VertexCirculator(mesh);

            foreach (var v in mesh.Vertices)
            {
                var list = circ.EnumerateVertices(v).ToList();
                Assert.That(list.Distinct().Count(), Is.EqualTo(list.Count), "duplicate neighbors");
                Assert.That(list, Is.EquivalentTo(nb[v]));

                var tris = circ.EnumerateTriangles(v).ToList();
                Assert.That(tris.Distinct().Count(), Is.EqualTo(tris.Count));
                Assert.That(tris.Count, Is.EqualTo(mesh.Triangles.Count(t =>
                    t.GetVertex(0) == v || t.GetVertex(1) == v || t.GetVertex(2) == v)));

                // Counter-clockwise order: consecutive neighbors form left turns around v
                // (except across the boundary gap).
                for (int i = 0; i + 1 < tris.Count; i++)
                {
                    Assert.That(tris[i].GetNeighbor(0) == tris[i + 1] ||
                                tris[i].GetNeighbor(1) == tris[i + 1] ||
                                tris[i].GetNeighbor(2) == tris[i + 1], Is.True,
                        "consecutive triangles are not adjacent");
                }
            }
        }

        [Test]
        [Description("VertexCirculator shares one cache list between all lazy enumerations; enumerating " +
            "the neighbors of a neighbor inside a foreach (2-ring) clears the list being iterated and " +
            "throws InvalidOperationException ('Collection was modified').")]
        public void VertexCirculatorNestedEnumeration()
        {
            var mesh = RandomPointMesh(30, 1);
            var circ = new VertexCirculator(mesh);
            var v = mesh.Vertices.First();

            var ring2 = new HashSet<TVertex>();
            Assert.That(() =>
            {
                foreach (var n in circ.EnumerateVertices(v))
                    foreach (var m in circ.EnumerateVertices(n))
                        ring2.Add(m);
            }, Throws.Nothing);
        }

        [Test]
        [Description("VertexCirculator.EnumerateVertices throws NullReferenceException for a vertex " +
            "that is not part of the triangulation (e.g. a duplicate input point, VertexType.UndeadVertex), " +
            "because vertex.tri.tri is null; it should yield nothing (or throw a meaningful exception).")]
        public void VertexCirculatorOnDuplicateVertex()
        {
            var poly = new Polygon();
            poly.Add(new TVertex(0, 0));
            poly.Add(new TVertex(1, 0));
            poly.Add(new TVertex(0, 1));
            poly.Add(new TVertex(1, 1));
            poly.Add(new TVertex(1, 1)); // duplicate
            var mesh = (Mesh)poly.Triangulate();

            var undead = mesh.Vertices.Single(v => v.Type == VertexType.UndeadVertex);
            var circ = new VertexCirculator(mesh);

            Assert.That(() => circ.EnumerateVertices(undead).ToList(), Throws.Nothing);
        }

        #endregion

        #region RegionIterator

        [Test]
        public void RegionIteratorStopsAtSegments()
        {
            var poly = new Polygon();
            // Two squares sharing an edge, with a hole in the left one.
            poly.Add(new Contour(new[] { new TVertex(0, 0), new TVertex(10, 0), new TVertex(10, 10), new TVertex(0, 10) }, 1));
            poly.Add(new Contour(new[] { new TVertex(10, 0), new TVertex(20, 0), new TVertex(20, 10), new TVertex(10, 10) }, 2));
            poly.Add(new Contour(new[] { new TVertex(4, 4), new TVertex(6, 4), new TVertex(6, 6), new TVertex(4, 6) }, 3), true);
            var mesh = (Mesh)poly.Triangulate(new QualityOptions { MaximumArea = 3 });

            var left = mesh.Triangles.First(t => (t.GetVertex(0).X + t.GetVertex(1).X + t.GetVertex(2).X) / 3 < 10);
            var visited = new List<Triangle>();
            new RegionIterator().Process(left, tri => visited.Add(tri));

            Assert.That(visited.Distinct().Count(), Is.EqualTo(visited.Count));
            var expectedLeft = mesh.Triangles.Where(t => (t.GetVertex(0).X + t.GetVertex(1).X + t.GetVertex(2).X) / 3 < 10).ToList();
            Assert.That(visited, Is.EquivalentTo(expectedLeft));
            Assert.That(mesh.Triangles.All(t => !t.infected), Is.True, "infection flags not cleared");

            // Boundary label 3 stops only at the hole segments; the shared edge
            // (label 1 or 2) is crossed -> whole mesh.
            visited.Clear();
            new RegionIterator().Process(left, tri => visited.Add(tri), 3);
            Assert.That(visited.Count, Is.EqualTo(mesh.Triangles.Count));
        }

        [Test]
        public void RegionIteratorDefaultActionCopiesLabelAndArea()
        {
            var mesh = BoxWithHoleMesh(true);
            var seed = mesh.triangles.First();
            seed.label = 7;
            seed.area = 1.5;
            new RegionIterator().Process(seed);
            Assert.That(mesh.Triangles.All(t => t.Label == 7 && t.Area == 1.5), Is.True);
        }

        #endregion

        #region DCEL

        [Test]
        public void DcelFromMeshIsConsistentAndEdgesUnique()
        {
            var mesh = BoxWithHoleMesh(true);
            var dcel = Converter.Instance.ToDCEL(mesh);

            Assert.That(dcel.IsConsistent(true, 10), Is.True);
            var edges = dcel.Edges.ToList();
            var keys = edges.Select(e => (Math.Min(e.P0, e.P1), Math.Max(e.P0, e.P1))).ToList();
            Assert.That(keys.Distinct().Count(), Is.EqualTo(keys.Count));
            Assert.That(edges.Count, Is.EqualTo(mesh.Edges.Count()));

            foreach (var v in dcel.Vertices)
            {
                foreach (var he in v.EnumerateEdges())
                    Assert.That(he.Origin, Is.SameAs(v));
            }
            foreach (var f in dcel.Faces)
                Assert.That(f.EnumerateEdges().Count(), Is.EqualTo(3));
        }

        private static DcelMesh OpenDcel(params (int, int, int)[] tris)
        {
            var dcel = new DcelMesh();
            var coords = new[] { (1.0, 1.0), (0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (1.0, -1.0) };
            for (int i = 0; i < coords.Length; i++)
                dcel.Vertices.Add(new HVertex(coords[i].Item1, coords[i].Item2) { ID = i });

            var halfEdges = new Dictionary<(int, int), HalfEdge>();
            int fid = 0, eid = 0;
            foreach (var (a, b, c) in tris)
            {
                var face = new Face(null) { ID = fid++ };
                var idx = new[] { a, b, c };
                var hes = new HalfEdge[3];
                for (int i = 0; i < 3; i++)
                {
                    var o = dcel.Vertices[idx[i]];
                    hes[i] = new HalfEdge(o, face) { ID = eid++ };
                    o.Leaving = hes[i];
                    halfEdges[(idx[i], idx[(i + 1) % 3])] = hes[i];
                    dcel.HalfEdges.Add(hes[i]);
                }
                for (int i = 0; i < 3; i++) hes[i].Next = hes[(i + 1) % 3];
                dcel.Faces.Add(face);
            }
            foreach (var kv in halfEdges)
            {
                if (halfEdges.TryGetValue((kv.Key.Item2, kv.Key.Item1), out var twin))
                    kv.Value.Twin = twin;
            }
            // Mark unused vertices as deleted so IsConsistent ignores them.
            foreach (var v in dcel.Vertices)
                if (v.Leaving == null) v.ID = -1;
            return dcel;
        }

        [Test]
        public void DcelResolveBoundaryEdgesSimple()
        {
            // Two CCW triangles sharing edge 0-2... (1,1),(0,0),(0,2) and (1,1),(0,2),(2,2)
            var dcel = OpenDcel((1, 0, 2), (0, 3, 2));
            dcel.ResolveBoundaryEdges();

            Assert.That(dcel.IsConsistent(true, 10), Is.True);
            Assert.That(dcel.HalfEdges.Count, Is.EqualTo(10));
            Assert.That(dcel.Edges.Count(), Is.EqualTo(5));
        }

        [Test]
        [Description("DcelMesh.ResolveBoundaryEdges keys boundary half-edges by origin vertex in a " +
            "Dictionary; a non-manifold boundary vertex (two triangles touching at a vertex) has two " +
            "outgoing boundary edges and map.Add throws ArgumentException.")]
        public void DcelResolveBoundaryEdgesPinchVertex()
        {
            // Bow-tie: triangles (0,1,2) and (0,3,4) touch only at vertex 0 (1,1).
            var dcel = OpenDcel((0, 1, 4), (0, 3, 2));
            Assert.That(() => dcel.ResolveBoundaryEdges(), Throws.Nothing);
            Assert.That(dcel.IsConsistent(true, 10), Is.True);
        }

        [Test]
        [Description("Face.Empty is a process-wide static singleton whose 'edge' is set by the first " +
            "HalfEdge ever created with it (HalfEdge ctor sets face.edge if null) and never reset, so it " +
            "keeps a reference into an unrelated, older DCEL/Voronoi (stale data + memory leak).")]
        public void FaceEmptyDoesNotReferenceForeignMesh()
        {
            var a = Converter.Instance.ToDCEL(RandomPointMesh(20, 1));
            var b = Converter.Instance.ToDCEL(RandomPointMesh(20, 2));

            var e = Face.Empty.Edge;
            Assert.That(e == null || b.HalfEdges.Contains(e), Is.True,
                "Face.Empty.Edge points into a DCEL other than the most recent one.");
            GC.KeepAlive(a);
        }

        #endregion
    }

    internal static class OtriTestExtensions
    {
        /// <summary>
        /// True if Onext can be applied (the Lprev edge has a real neighbor).
        /// </summary>
        public static bool Lprev_SymExists(this Otri o)
        {
            Otri x = default, y = default;
            o.Lprev(ref x);
            x.Sym(ref y);
            return y.tri != null && y.tri.id != Mesh.DUMMY;
        }
    }
}
