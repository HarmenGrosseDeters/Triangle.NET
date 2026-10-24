using NUnit.Framework;
using System;
using TriangleNet.Geometry;

namespace TriangleNet.Tests.Robustness
{
    public class PredicatesTest
    {
        [Test]
        [Description("Fixed: FastExpansionSumZeroElim read one element past the end of an expansion (harmless in the original C code, IndexOutOfRangeException in C#).")]
        public void NearlyCollinearPointsOfDifferentMagnitude()
        {
            var predicates = new RobustPredicates();
            var random = new Random(1);

            Exception error = null;

            for (int i = 0; i < 200000 && error == null; i++)
            {
                // Points on a line with tiny perturbations and coordinates of very different magnitude.
                double scale = Math.Pow(2, random.Next(-60, 60));
                double ax = (random.NextDouble() - 0.5) * scale, ay = (random.NextDouble() - 0.5) * scale;
                double dx = random.NextDouble() - 0.5, dy = random.NextDouble() - 0.5;
                double t1 = random.NextDouble() * Math.Pow(2, random.Next(-40, 40));
                double t2 = random.NextDouble() * Math.Pow(2, random.Next(-40, 40));

                var a = new Point(ax, ay);
                var b = new Point(ax + t1 * dx, ay + t1 * dy);
                var c = new Point(ax + t2 * dx + Math.Pow(2, random.Next(-80, 0)), ay + t2 * dy);

                try
                {
                    predicates.CounterClockwise(a, b, c);
                    predicates.InCircle(a, b, c, new Point(ax - t2 * dy, ay + t1 * dx));
                }
                catch (Exception e)
                {
                    error = e;
                }
            }

            Assert.That(error, Is.Null);
        }

        [Test]
        [Description("Fixed: infinite coordinates caused an IndexOutOfRangeException in FastExpansionSumZeroElim (the noder runs before input validation).")]
        public void InfiniteSegmentCoordinates()
        {
            double inf = double.PositiveInfinity;

            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(inf, inf), new Vertex(-inf, inf), new Vertex(-inf, -inf), new Vertex(inf, -inf) }));
            poly.Add(new Vertex(1, 1));

            Assert.Throws<ArgumentException>(() => poly.Triangulate());
        }
    }
}
