using NUnit.Framework;
using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using TriangleNet.Geometry;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// <see cref="RobustPredicates.Default"/> is a singleton, but the adaptive (exact) arithmetic
    /// uses workspace arrays stored in instance fields. Concurrent calls overwrite each other's
    /// intermediate results.
    /// </summary>
    /// <remarks>
    /// Since <see cref="Configuration"/> uses the singleton by default, meshing in parallel
    /// (e.g. in a web service) can produce wrong predicate results for nearly degenerate input,
    /// which is exactly the case the exact arithmetic is meant for. The default instance is also
    /// used directly by MeshValidator, Contour.FindInteriorPoint, the Voronoi classes and the
    /// SimpleSmoother.
    /// Workaround: new GenericMesher(new Configuration(() => new RobustPredicates())).
    /// </remarks>
    public class ThreadSafetyTest
    {
        [Test, Category(KnownBug.Category)]
        [Description("Severity: critical (for multi-threaded use). Concurrent use of RobustPredicates.Default returns wrong results.")]
        public void DefaultPredicatesInParallel()
        {
            Assert.That(CountWrongResultsInParallel(() => RobustPredicates.Default), Is.Zero);
        }

        [Test]
        public void PrivatePredicatesInParallel()
        {
            // One instance per thread works.
            var local = new ThreadLocal<RobustPredicates>(() => new RobustPredicates());

            Assert.That(CountWrongResultsInParallel(() => local.Value), Is.Zero);
        }

        private static int CountWrongResultsInParallel(Func<RobustPredicates> predicates)
        {
            const int n = 5000;

            var random = new Random(1);

            // Nearly cocircular point sets, forcing the adaptive (exact) code path.
            var input = new Point[n][];

            for (int i = 0; i < n; i++)
            {
                double cx = random.NextDouble() * 1e3, cy = random.NextDouble() * 1e3;
                double r = 1 + random.NextDouble();

                input[i] = Enumerable.Range(0, 4).Select(_ =>
                {
                    double a = random.NextDouble() * 2 * Math.PI;
                    return new Point(cx + r * Math.Cos(a), cy + r * Math.Sin(a));
                }).ToArray();
            }

            var reference = new RobustPredicates();
            var expected = input.Select(p => Math.Sign(reference.InCircle(p[0], p[1], p[2], p[3]))).ToArray();

            int wrong = 0;

            Parallel.For(0, 4, new ParallelOptions { MaxDegreeOfParallelism = 4 }, _ =>
            {
                var pred = predicates();

                for (int repeat = 0; repeat < 10; repeat++)
                {
                    for (int i = 0; i < n; i++)
                    {
                        var p = input[i];

                        if (Math.Sign(pred.InCircle(p[0], p[1], p[2], p[3])) != expected[i])
                        {
                            Interlocked.Increment(ref wrong);
                        }
                    }
                }
            });

            return wrong;
        }
    }
}
