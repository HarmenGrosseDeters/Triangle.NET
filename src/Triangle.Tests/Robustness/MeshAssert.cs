using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Validation helpers for robustness tests. A mesh is considered valid if it is
    /// topologically consistent, has no flat or inverted triangles, and all input
    /// segments are fully represented by mesh subsegments.
    /// </summary>
    static class MeshAssert
    {
        /// <summary>
        /// Default timeout for meshing operations that might not terminate.
        /// </summary>
        public static readonly TimeSpan DefaultTimeout = TimeSpan.FromSeconds(10);

        /// <summary>
        /// Run the given meshing function on a separate thread and fail if it does
        /// not finish within the timeout (detects infinite loops).
        /// </summary>
        public static T Terminates<T>(Func<T> func, TimeSpan? timeout = null)
        {
            var task = Task.Factory.StartNew(func, CancellationToken.None,
                TaskCreationOptions.LongRunning, TaskScheduler.Default);

            if (!task.Wait(timeout ?? DefaultTimeout))
            {
                Assert.Fail($"Operation did not terminate within {(timeout ?? DefaultTimeout).TotalSeconds}s (infinite loop?).");
            }

            return task.Result;
        }

        /// <summary>
        /// Assert that the mesh is topologically consistent and contains no
        /// flat or inverted triangles.
        /// </summary>
        public static void IsValid(IMesh mesh)
        {
            Assert.That(mesh, Is.Not.Null);
            Assert.That(mesh.Triangles.Count, Is.GreaterThan(0), "Mesh has no triangles.");
            Assert.That(MeshValidator.IsConsistent((Mesh)mesh), Is.True, "Mesh is not consistent.");

            foreach (var t in mesh.Triangles)
            {
                var a = t.GetVertex(0);
                var b = t.GetVertex(1);
                var c = t.GetVertex(2);

                Assert.That(RobustPredicates.Default.CounterClockwise(a, b, c), Is.GreaterThan(0.0),
                    $"Triangle {t.ID} is flat or inverted.");

                foreach (var v in new[] { a, b, c })
                {
                    Assert.That(double.IsFinite(v.X) && double.IsFinite(v.Y), Is.True,
                        $"Triangle {t.ID} has a non-finite vertex.");
                }
            }
        }

        /// <summary>
        /// Assert that every input segment (with non-coincident endpoints) is completely
        /// covered by mesh subsegments lying on it.
        /// </summary>
        public static void ContainsSegments(IMesh mesh, IEnumerable<ISegment> segments, double relTol = 1e-9)
        {
            var subsegs = mesh.Segments.Select(s => (s.GetVertex(0), s.GetVertex(1))).ToList();

            foreach (var seg in segments)
            {
                var p = seg.GetVertex(0);
                var q = seg.GetVertex(1);

                double length = Distance(p, q);

                if (length == 0.0) continue;

                double tol = relTol * Math.Max(length, Scale(p, q));

                double covered = subsegs
                    .Where(s => DistanceToSegment(s.Item1, p, q) <= tol && DistanceToSegment(s.Item2, p, q) <= tol)
                    .Sum(s => Distance(s.Item1, s.Item2));

                Assert.That(covered, Is.EqualTo(length).Within(tol * 10),
                    $"Input segment ({p.X}, {p.Y}) - ({q.X}, {q.Y}) is not fully contained in the mesh.");
            }
        }

        /// <summary>
        /// Assert that the total area of the mesh equals the expected area.
        /// </summary>
        public static void HasArea(IMesh mesh, double expected, double relTol = 1e-9)
        {
            double area = mesh.Triangles.Sum(t => Area(t));

            Assert.That(area, Is.EqualTo(expected).Within(Math.Abs(expected) * relTol),
                "Mesh area doesn't match.");
        }

        /// <summary>
        /// Assert that each input vertex coordinate is present in the mesh.
        /// </summary>
        public static void ContainsPoints(IMesh mesh, IEnumerable<Point> points)
        {
            var set = new HashSet<(double, double)>(mesh.Vertices
                .Where(v => v.Type != VertexType.UndeadVertex)
                .Select(v => (v.X, v.Y)));

            foreach (var p in points)
            {
                Assert.That(set.Contains((p.X, p.Y)), Is.True, $"Input point ({p.X}, {p.Y}) missing in mesh.");
            }
        }

        public static double Area(ITriangle t)
        {
            var a = t.GetVertex(0);
            var b = t.GetVertex(1);
            var c = t.GetVertex(2);

            return 0.5 * ((b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y));
        }

        private static double Scale(Point p, Point q)
        {
            return Math.Max(Math.Max(Math.Abs(p.X), Math.Abs(p.Y)), Math.Max(Math.Abs(q.X), Math.Abs(q.Y)));
        }

        private static double Distance(Point a, Point b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private static double DistanceToSegment(Point v, Point p, Point q)
        {
            double dx = q.X - p.X, dy = q.Y - p.Y;
            double len2 = dx * dx + dy * dy;
            double t = ((v.X - p.X) * dx + (v.Y - p.Y) * dy) / len2;
            t = Math.Max(0.0, Math.Min(1.0, t));
            return Distance(v, new Point(p.X + t * dx, p.Y + t * dy));
        }
    }
}
