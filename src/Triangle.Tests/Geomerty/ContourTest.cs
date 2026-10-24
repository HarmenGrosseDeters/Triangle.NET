using NUnit.Framework;
using TriangleNet.Geometry;
using System;
using System.Collections.Generic;

namespace TriangleNet.Tests.Geometry
{
    public class ContourTest
    {
        [Test]
        public void TestFindInteriorPoint()
        {
            // The vertices  that define the contour (have to be in order, obviously).
            var vertices = new Vertex[]
            {
                new Vertex(0d, 0d),
                new Vertex(1d ,0d),
                new Vertex(1d, 1d),
                new Vertex(0d, 1d),
            };

            var contour = new Contour(vertices);

            var p = contour.FindInteriorPoint();

            Assert.That(p.X > 0d && p.X < 1d && p.Y > 0d && p.Y < 1d, Is.True);
        }

        [Test]
        public void TestFindInteriorPointL()
        {
            // L-shaped contour (FindPointInPolygon() produces a test candidate
            // which lies exactly on a segment where IsPointInPolygon() returns
            // true, so IsPointOnSegment() is actually needed here).
            var points = new List<Vertex>()
            {
                new Vertex(3, 1),
                new Vertex(1, 1),
                new Vertex(1, 3),
                new Vertex(2, 3),
                new Vertex(2, 2),
                new Vertex(3, 2)
            };

            var contour = new Contour(points);

            var poly = new Polygon(6);

            poly.Add(contour, true);

            var h = poly.Holes[0];
            var p = RobustPredicates.Default;

            int count = points.Count;
            int i = count - 1;

            for (int j = 0; j < count; j++)
            {
                // The hole point must not lie on a segment (it may lie on the extension of one).
                double ccw = p.CounterClockwise(points[i], h, points[j]);
                bool inBox = Math.Min(points[i].X, points[j].X) <= h.X && h.X <= Math.Max(points[i].X, points[j].X) &&
                             Math.Min(points[i].Y, points[j].Y) <= h.Y && h.Y <= Math.Max(points[i].Y, points[j].Y);

                Assert.That(ccw != 0.0 || !inBox, Is.True);

                i = j;
            }

            // Strictly inside the L-shape.
            Assert.That(h.X > 1 && h.Y > 1 && (h.X < 2 && h.Y < 3 || h.X < 3 && h.Y < 2), Is.True);
        }

        [Test]
        public void TestFindInteriorPointDup()
        {
            // Rectangle contour with duplicate point.
            var points = new List<Vertex>()
            {
                new Vertex(0.0, 0.0),
                new Vertex(0.0, 1.0),
                new Vertex(2.0, 1.0),
                new Vertex(2.0, 0.5),
                new Vertex(2.0, 0.5), // duplicate
                new Vertex(2.0, 0.0)
            };

            var contour = new Contour(points);

            Assert.DoesNotThrow(() => contour.FindInteriorPoint());
        }
    }
}
