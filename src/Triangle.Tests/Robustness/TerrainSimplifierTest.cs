using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Tools;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// TIN simplification (greedy insertion): select a subset of terrain points such that the
    /// triangulation of the subset approximates all points within a vertical error bound.
    /// </summary>
    public class TerrainSimplifierTest
    {
        private static (List<Vertex>, List<double>) Grid(int n, Func<double, double, double> f)
        {
            var points = new List<Vertex>();
            var heights = new List<double>();

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    points.Add(new Vertex(i, j));
                    heights.Add(f(i, j));
                }
            }

            return (points, heights);
        }

        private static double Hills(double x, double y) =>
            3 * Math.Sin(x / 9.0) * Math.Cos(y / 7.0) + 0.02 * x + Math.Exp(-((x - 40) * (x - 40) + (y - 30) * (y - 30)) / 50.0);

        /// <summary>
        /// Maximum vertical error of all points with respect to the (constrained) Delaunay
        /// triangulation of the selected points (brute force point location).
        /// </summary>
        private static double MaxError(List<Vertex> points, List<double> heights, IList<int> selected, IPolygon constraints = null)
        {
            var poly = new Polygon();
            var z = new Dictionary<(double, double), double>();

            foreach (int i in selected)
            {
                poly.Add(new Vertex(points[i].X, points[i].Y));
                z[(points[i].X, points[i].Y)] = heights[i];
            }

            if (constraints != null)
            {
                foreach (var s in constraints.Segments)
                {
                    poly.Add(new Segment(new Vertex(s.GetVertex(0).X, s.GetVertex(0).Y), new Vertex(s.GetVertex(1).X, s.GetVertex(1).Y)), false);
                }
            }

            var mesh = (Mesh)new GenericMesher().Triangulate(poly, new ConstraintOptions { Convex = true });
            var triangles = mesh.Triangles.ToList();

            double max = 0;

            for (int k = 0; k < points.Count; k++)
            {
                var p = points[k];
                double best = double.MaxValue;

                foreach (var t in triangles)
                {
                    var a = t.GetVertex(0); var b = t.GetVertex(1); var c = t.GetVertex(2);
                    double det = (b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y);
                    double l1 = ((b.X - p.X) * (c.Y - p.Y) - (c.X - p.X) * (b.Y - p.Y)) / det;
                    double l2 = ((c.X - p.X) * (a.Y - p.Y) - (a.X - p.X) * (c.Y - p.Y)) / det;
                    double l3 = 1 - l1 - l2;

                    if (l1 >= -1e-12 && l2 >= -1e-12 && l3 >= -1e-12)
                    {
                        double h = l1 * z[(a.X, a.Y)] + l2 * z[(b.X, b.Y)] + l3 * z[(c.X, c.Y)];
                        best = Math.Abs(h - heights[k]);
                        break;
                    }
                }

                Assert.That(best, Is.Not.EqualTo(double.MaxValue), $"point {k} not covered");
                max = Math.Max(max, best);
            }

            return max;
        }

        [Test]
        public void PlaneNeedsOnlyTheCorners()
        {
            var (points, heights) = Grid(30, (x, y) => 2 * x - 0.5 * y + 100);

            var selected = TerrainSimplifier.Select(points, heights, 0.01);

            Assert.That(selected.Count, Is.EqualTo(4));
        }

        [TestCase(0.5)]
        [TestCase(0.1)]
        [TestCase(0.02)]
        public void ErrorBoundHolds(double maxError)
        {
            var (points, heights) = Grid(60, Hills);

            var selected = TerrainSimplifier.Select(points, heights, maxError);

            Assert.That(selected.Distinct().Count(), Is.EqualTo(selected.Count));
            Assert.That(MaxError(points, heights, selected), Is.LessThanOrEqualTo(maxError * (1 + 1e-9)));
            // The test surface is strongly curved: at 0.02, about half of the points are needed.
            Assert.That(selected.Count, Is.LessThan(points.Count * 0.6));
        }

        [Test]
        public void SmallerErrorSelectsMorePoints()
        {
            var (points, heights) = Grid(60, Hills);

            int coarse = TerrainSimplifier.Select(points, heights, 0.2).Count;
            int fine = TerrainSimplifier.Select(points, heights, 0.02).Count;

            Assert.That(fine, Is.GreaterThan(coarse));
        }

        [Test]
        public void BreakLinesAreRespected()
        {
            // A ridge along a break line: the selection is computed for the constrained
            // triangulation, and the break line vertices are part of the selection.
            var (points, heights) = Grid(40, (x, y) => -Math.Abs(x - y) * 0.5);

            var constraints = new Polygon();
            var a = new Vertex(0, 0);
            var b = new Vertex(39, 39);
            constraints.Add(new Segment(a, b), true);

            var selected = TerrainSimplifier.Select(points, heights, 0.01, constraints);

            Assert.That(selected.Contains(0), Is.True);
            Assert.That(selected.Contains(points.Count - 1), Is.True);
            Assert.That(MaxError(points, heights, selected, constraints), Is.LessThanOrEqualTo(0.01 * (1 + 1e-9)));

            // The surface is planar on both sides of the ridge: only the corners are needed.
            Assert.That(selected.Count, Is.EqualTo(4));
        }

        [Test]
        public void DegenerateInputReturnsAllPoints()
        {
            var points = new List<Vertex> { new Vertex(0, 0), new Vertex(1, 1), new Vertex(2, 2) };
            var heights = new List<double> { 0, 1, 5 };

            Assert.That(TerrainSimplifier.Select(points, heights, 0.1), Is.EquivalentTo(new[] { 0, 1, 2 }));
        }

        [Test]
        public void ArgumentsAreValidated()
        {
            var (points, heights) = Grid(5, (x, y) => 0);

            Assert.Throws<ArgumentException>(() => TerrainSimplifier.Select(points, heights.Take(3).ToList(), 0.1));
            Assert.Throws<ArgumentException>(() => TerrainSimplifier.Select(points, heights, -1));
        }

        [Test]
        public void LargeGridIsFast()
        {
            var (points, heights) = Grid(400, Hills);

            var watch = System.Diagnostics.Stopwatch.StartNew();
            var selected = TerrainSimplifier.Select(points, heights, 0.05);
            watch.Stop();

            Assert.That(selected.Count, Is.LessThan(points.Count / 4));
            Assert.That(watch.Elapsed.TotalSeconds, Is.LessThan(60));
        }
    }
}
