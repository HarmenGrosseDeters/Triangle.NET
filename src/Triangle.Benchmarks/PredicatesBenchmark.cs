using System;
using System.Runtime.CompilerServices;
using BenchmarkDotNet.Attributes;
using TriangleNet.Geometry;

namespace TriangleNet.Benchmarks
{
    /// <summary>
    /// Micro-benchmarks of the exact predicates (orientation and incircle test). Each operation
    /// evaluates one predicate; the data sets decide how often the exact (adaptive) stages run.
    /// </summary>
    /// <remarks>
    /// dotnet run -c Release --project src/Triangle.Benchmarks -- --filter "*Predicates*"
    ///
    /// Variants: through the interface <see cref="IPredicates"/> (as the mesher calls them), on the
    /// class directly, and an experimental fast path (<see cref="FastPredicates"/>) with the same
    /// arithmetic and error bounds (so the same results), but without statistics counters and with
    /// readonly bounds. The experimental variant is checked against the library in the setup.
    /// </remarks>
    [MemoryDiagnoser]
    public class PredicatesBenchmark
    {
        private const int N = 4096;

        private Point[] a, b, c, d;

        private IPredicates viaInterface;
        private RobustPredicates direct;

        /// <summary>
        /// utm: random points of a 200 m tile in UTM coordinates (well conditioned);
        /// grid: neighboring points of a regular grid (collinear and cocircular, exact stages);
        /// near: points within 1e-9 of a line or circle (mostly the exact stages).
        /// </summary>
        [Params("utm", "grid", "near")]
        public string Data { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            var random = new Random(1);
            a = new Point[N]; b = new Point[N]; c = new Point[N]; d = new Point[N];

            for (int i = 0; i < N; i++)
            {
                switch (Data)
                {
                    case "utm":
                        a[i] = Utm(random); b[i] = Utm(random); c[i] = Utm(random); d[i] = Utm(random);
                        break;

                    case "grid":
                        {
                            // A unit square of a 1 cm grid and a fourth point on the grid.
                            double x = 372000 + 0.01 * random.Next(10000), y = 5710000 + 0.01 * random.Next(10000);
                            a[i] = new Point(x, y);
                            b[i] = new Point(x + 0.01, y);
                            c[i] = new Point(x + 0.01, y + 0.01);
                            d[i] = random.Next(2) == 0 ? new Point(x, y + 0.01) : new Point(x + 0.02, y + 0.02);
                            break;
                        }

                    default:
                        {
                            double x = 372000 + 200 * random.NextDouble(), y = 5710000 + 200 * random.NextDouble();
                            double r = 1 + random.NextDouble();
                            double t1 = random.NextDouble() * 6.28, t2 = t1 + 2, t3 = t1 + 4;
                            a[i] = new Point(x + r * Math.Cos(t1), y + r * Math.Sin(t1));
                            b[i] = new Point(x + r * Math.Cos(t2), y + r * Math.Sin(t2));
                            c[i] = new Point(x + r * Math.Cos(t3), y + r * Math.Sin(t3));
                            double t4 = random.NextDouble() * 6.28, e = (random.NextDouble() - 0.5) * 1e-9;
                            d[i] = new Point(x + (r + e) * Math.Cos(t4), y + (r + e) * Math.Sin(t4));
                            break;
                        }
                }
            }

            direct = new RobustPredicates();
            viaInterface = direct;

            for (int i = 0; i < N; i++)
            {
                if (Math.Sign(FastPredicates.CounterClockwise(a[i], b[i], c[i])) != Math.Sign(direct.CounterClockwise(a[i], b[i], c[i])) ||
                    Math.Sign(FastPredicates.InCircle(a[i], b[i], c[i], d[i])) != Math.Sign(direct.InCircle(a[i], b[i], c[i], d[i])))
                {
                    throw new InvalidOperationException("Experimental predicates differ.");
                }
            }
        }

        private static Point Utm(Random random) =>
            new Point(372000 + 200 * random.NextDouble(), 5710000 + 200 * random.NextDouble());

        [Benchmark(OperationsPerInvoke = N, Baseline = true)]
        public double OrientInterface()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(viaInterface.CounterClockwise(a[i], b[i], c[i]));
            return s;
        }

        [Benchmark(OperationsPerInvoke = N)]
        public double OrientDirect()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(direct.CounterClockwise(a[i], b[i], c[i]));
            return s;
        }

        [Benchmark(OperationsPerInvoke = N)]
        public double OrientExperimental()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(FastPredicates.CounterClockwise(a[i], b[i], c[i]));
            return s;
        }

        [Benchmark(OperationsPerInvoke = N)]
        public double InCircleInterface()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(viaInterface.InCircle(a[i], b[i], c[i], d[i]));
            return s;
        }

        [Benchmark(OperationsPerInvoke = N)]
        public double InCircleDirect()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(direct.InCircle(a[i], b[i], c[i], d[i]));
            return s;
        }

        [Benchmark(OperationsPerInvoke = N)]
        public double InCircleExperimental()
        {
            double s = 0;
            for (int i = 0; i < N; i++) s += Math.Sign(FastPredicates.InCircle(a[i], b[i], c[i], d[i]));
            return s;
        }
    }

    /// <summary>
    /// Experimental fast path of the predicates: same arithmetic and error bounds as
    /// <see cref="RobustPredicates"/>, the exact stages are those of the library.
    /// </summary>
    internal static class FastPredicates
    {
        private static readonly double Epsilon = ComputeEpsilon();
        private static readonly double CcwErrBoundA = (3.0 + 16.0 * Epsilon) * Epsilon;
        private static readonly double IccErrBoundA = (10.0 + 96.0 * Epsilon) * Epsilon;

        private static double ComputeEpsilon()
        {
            double epsilon = 1.0, check = 1.0, lastcheck;
            do
            {
                lastcheck = check;
                epsilon *= 0.5;
                check = 1.0 + epsilon;
            } while ((check != 1.0) && (check != lastcheck));
            return epsilon;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double CounterClockwise(Point pa, Point pb, Point pc)
        {
            double detleft = (pa.X - pc.X) * (pb.Y - pc.Y);
            double detright = (pa.Y - pc.Y) * (pb.X - pc.X);
            double det = detleft - detright;
            double detsum;

            if (detleft > 0.0)
            {
                if (detright <= 0.0) return det;
                detsum = detleft + detright;
            }
            else if (detleft < 0.0)
            {
                if (detright >= 0.0) return det;
                detsum = -detleft - detright;
            }
            else
            {
                return det;
            }

            double errbound = CcwErrBoundA * detsum;
            if ((det >= errbound) || (-det >= errbound)) return det;

            return RobustPredicates.Default.CounterClockwise(pa, pb, pc);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double InCircle(Point pa, Point pb, Point pc, Point pd)
        {
            double adx = pa.X - pd.X, bdx = pb.X - pd.X, cdx = pc.X - pd.X;
            double ady = pa.Y - pd.Y, bdy = pb.Y - pd.Y, cdy = pc.Y - pd.Y;

            double bdxcdy = bdx * cdy, cdxbdy = cdx * bdy, alift = adx * adx + ady * ady;
            double cdxady = cdx * ady, adxcdy = adx * cdy, blift = bdx * bdx + bdy * bdy;
            double adxbdy = adx * bdy, bdxady = bdx * ady, clift = cdx * cdx + cdy * cdy;

            double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

            double permanent = (Math.Abs(bdxcdy) + Math.Abs(cdxbdy)) * alift
                             + (Math.Abs(cdxady) + Math.Abs(adxcdy)) * blift
                             + (Math.Abs(adxbdy) + Math.Abs(bdxady)) * clift;
            double errbound = IccErrBoundA * permanent;
            if ((det > errbound) || (-det > errbound)) return det;

            return RobustPredicates.Default.InCircle(pa, pb, pc, pd);
        }
    }
}
