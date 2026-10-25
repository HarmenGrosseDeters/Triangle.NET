using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using NUnit.Framework;
using TriangleNet.Geometry;
using TriangleNet.Meshing.Algorithm;

namespace TriangleNet.Tests.Prototype
{
    /// <summary>
    /// Compares the divide-and-conquer triangulation on the object-based mesh with the same
    /// algorithm on an index-based storage (<see cref="TriangleStore"/>).
    /// </summary>
    public static class PrototypeBenchmark
    {
        public static List<Vertex> Grid(int n)
        {
            var list = new List<Vertex>(n * n);
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    list.Add(new Vertex(i / (double)(float)(n - 1), j / (double)(float)(n - 1)) { ID = list.Count });
            return list;
        }

        public static List<Vertex> Random(int count)
        {
            var r = new System.Random(1);
            var list = new List<Vertex>(count);
            for (int i = 0; i < count; i++) list.Add(new Vertex(r.NextDouble(), r.NextDouble()) { ID = i });
            return list;
        }

        /// <summary>
        /// Sorted vertex id triples of the triangles (for comparing the results).
        /// </summary>
        internal static List<(int, int, int)> Triangles(TriangleStore store)
        {
            var list = new List<(int, int, int)>();
            for (int t = 1; t < store.Count; t++)
            {
                if (store.IsDead(t)) continue;
                list.Add(Key(store.Vertices[3 * t].ID, store.Vertices[3 * t + 1].ID, store.Vertices[3 * t + 2].ID));
            }
            list.Sort();
            return list;
        }

        public static List<(int, int, int)> Triangles(Mesh mesh)
        {
            var list = mesh.Triangles.Select(t => Key(t.GetVertexID(0), t.GetVertexID(1), t.GetVertexID(2))).ToList();
            list.Sort();
            return list;
        }

        private static (int, int, int) Key(int a, int b, int c)
        {
            // Rotate so that the smallest id comes first (keeps the orientation).
            if (a < b && a < c) return (a, b, c);
            if (b < c) return (b, c, a);
            return (c, a, b);
        }

        private static double Median(Func<int> run, out int result, double budgetMs)
        {
            result = run();
            var times = new List<double>();
            var total = Stopwatch.StartNew();
            do
            {
                var sw = Stopwatch.StartNew();
                result = run();
                times.Add(sw.Elapsed.TotalMilliseconds);
            } while (total.ElapsedMilliseconds < budgetMs && times.Count < 200);
            times.Sort();
            return times[times.Count / 2];
        }

        public static void Run(Action<string> output, double budgetMs = 2000)
        {
            var config = new Configuration();

            foreach (var n in new[] { 100, 316, 1000 })
            {
                foreach (var (name, points) in new[] { ("grid", Grid(n)), ("random", Random(n * n)) })
                {
                    // Same result?
                    var a = Triangles((Mesh)new Dwyer().Triangulate(points, config));
                    var b = Triangles(new XDwyer().Triangulate(points, config));
                    bool same = a.SequenceEqual(b);

                    long g0 = GC.CollectionCount(0);
                    double objects = Median(() => new Dwyer().Triangulate(points, config).Triangles.Count, out int c1, budgetMs);
                    double indices = Median(() => new XDwyer().Triangulate(points, config).Count, out _, budgetMs);
                    double meshOnly = Median(() => new Mesh(config, points).Vertices.Count, out _, budgetMs / 4);

                    output($"{name,-6} {points.Count,9} points: objects {objects,9:F2} ms (thereof new Mesh {meshOnly,7:F2} ms), " +
                           $"indices {indices,9:F2} ms, factor {objects / indices:F2}, same result {same} ({c1} triangles)");
                }
            }
        }
    }

    [Category("Benchmark"), Explicit("Prototype benchmark")]
    public class PrototypeBenchmarkTest
    {
        [Test]
        public void IndexStorageGivesSameTriangulation()
        {
            foreach (var points in new[] { PrototypeBenchmark.Grid(50), PrototypeBenchmark.Random(5000) })
            {
                var a = PrototypeBenchmark.Triangles((Mesh)new Dwyer().Triangulate(points, new Configuration()));
                var b = PrototypeBenchmark.Triangles(new XDwyer().Triangulate(points, new Configuration()));
                Assert.That(b, Is.EqualTo(a));
            }
        }
    }
}
