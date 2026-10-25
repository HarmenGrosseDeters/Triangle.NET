using NUnit.Framework;
using System;
using System.Numerics;
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

        [TestCase(1e-100)]
        [TestCase(1e-160)]
        [TestCase(1e-200)]
        [TestCase(1e-300)]
        [TestCase(1e-310)]
        [TestCase(1e100)]
        [TestCase(1e200)]
        [TestCase(1e300)]
        [Description("Fixed: for very small or very large coordinates the products underflow (subnormal or zero) or overflow, Shewchuk's error bounds don't hold and the sign was wrong (about half of the cases) or NaN.")]
        public void ExtremeCoordinatesGiveTheExactSign(double scale)
        {
            var predicates = new RobustPredicates();
            var random = new Random(1);
            int wrongOrient = 0, wrongInCircle = 0;

            for (int i = 0; i < 20000; i++)
            {
                // Nearly collinear (on a line, last coordinate moved by a few ulps) ...
                double t1 = random.NextDouble(), t2 = random.NextDouble();
                var a = new Point(scale, 2 * scale);
                var b = new Point(scale * (1 + t1), scale * (2 + 2 * t1));
                var c = new Point(Ulps(scale * (1 + t2), random.Next(-3, 4)), scale * (2 + 2 * t2));

                if (Sign(predicates.CounterClockwise(a, b, c)) != Orient(a, b, c)) wrongOrient++;

                // ... and nearly cocircular points.
                var p = new Point[4];

                for (int j = 0; j < 4; j++)
                {
                    double angle = random.NextDouble() * 2 * Math.PI;
                    p[j] = new Point(scale * Math.Cos(angle) + 3 * scale, scale * Math.Sin(angle) - 5 * scale);
                }

                if (Orient(p[0], p[1], p[2]) <= 0) continue;

                if (Sign(predicates.InCircle(p[0], p[1], p[2], p[3])) != InCircle(p[0], p[1], p[2], p[3])) wrongInCircle++;
            }

            Assert.That((wrongOrient, wrongInCircle), Is.EqualTo((0, 0)));
        }

        [Test]
        public void MixedMagnitudesGiveTheExactSign()
        {
            var predicates = new RobustPredicates();
            var random = new Random(2);
            var values = new[] { 0.0, 1e-310, 1e-200, 1e-120, 1e-40, 1.0, 3.0, 1e40, 1e120, 1e200 };
            int wrong = 0;

            double V() => (random.Next(2) == 0 ? -1 : 1) * Ulps(values[random.Next(values.Length)], random.Next(-2, 3));

            for (int i = 0; i < 50000; i++)
            {
                var a = new Point(V(), V());
                var b = new Point(V(), V());
                var c = new Point(V(), V());
                var d = new Point(V(), V());

                if (Sign(predicates.CounterClockwise(a, b, c)) != Orient(a, b, c)) wrong++;
                if (Sign(predicates.InCircle(a, b, c, d)) != InCircle(a, b, c, d)) wrong++;
            }

            Assert.That(wrong, Is.Zero);
        }

        private static int Sign(double x) => x > 0 ? 1 : x < 0 ? -1 : double.IsNaN(x) ? 2 : 0;

        private static double Ulps(double x, int k)
        {
            for (; k > 0; k--) x = Math.BitIncrement(x);
            for (; k < 0; k++) x = Math.BitDecrement(x);
            return x;
        }

        // Exact reference: every double is m * 2^e, with a common power-of-two scale.
        private static BigInteger[] Integers(params double[] values)
        {
            var mantissa = new long[values.Length];
            var exponent = new int[values.Length];
            int emin = int.MaxValue;

            for (int i = 0; i < values.Length; i++)
            {
                long bits = BitConverter.DoubleToInt64Bits(values[i]);
                int e = (int)((bits >> 52) & 0x7FF);
                long f = bits & 0xFFFFFFFFFFFFFL;

                if (e == 0) exponent[i] = -1074;
                else { f |= 1L << 52; exponent[i] = e - 1075; }

                mantissa[i] = bits < 0 ? -f : f;
                if (mantissa[i] != 0) emin = Math.Min(emin, exponent[i]);
            }

            var result = new BigInteger[values.Length];

            for (int i = 0; i < values.Length; i++)
            {
                result[i] = mantissa[i] == 0 ? BigInteger.Zero : new BigInteger(mantissa[i]) << (exponent[i] - emin);
            }

            return result;
        }

        private static int Orient(Point a, Point b, Point c)
        {
            var v = Integers(a.X, a.Y, b.X, b.Y, c.X, c.Y);
            return ((v[2] - v[0]) * (v[5] - v[1]) - (v[3] - v[1]) * (v[4] - v[0])).Sign;
        }

        private static int InCircle(Point a, Point b, Point c, Point d)
        {
            var v = Integers(a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y);
            BigInteger adx = v[0] - v[6], ady = v[1] - v[7], bdx = v[2] - v[6], bdy = v[3] - v[7], cdx = v[4] - v[6], cdy = v[5] - v[7];
            return ((adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
                  + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy)
                  + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady)).Sign;
        }
    }
}
