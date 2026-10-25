using BenchmarkDotNet.Attributes;
using System.Collections.Generic;
using TriangleNet.Geometry;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Tests.Prototype;

namespace TriangleNet.Benchmarks
{
    /// <summary>
    /// Divide-and-conquer Delaunay triangulation on the object-based mesh (<see cref="Dwyer"/>)
    /// against the same algorithm on the index-based prototype storage (<see cref="XDwyer"/>,
    /// Triangle.Tests/Prototype). Same points as PrototypeBenchmark.
    /// </summary>
    /// <remarks>
    /// dotnet run -c Release -f net48 --project src/Triangle.Benchmarks -- --filter "*IndexStorage*"
    /// </remarks>
    [MemoryDiagnoser]
    public class IndexStorageBenchmark
    {
        private List<Vertex> points;
        private Configuration config;

        /// <summary>
        /// Points per side of the grid; the random point set has as many points.
        /// </summary>
        [Params(100, 316, 1000)]
        public int N { get; set; }

        [Params("grid", "random")]
        public string Points { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            config = new Configuration();
            points = Points == "grid" ? Grid(N) : Random(N * N);
        }

        [Benchmark(Baseline = true)]
        public int Objects()
        {
            return new Dwyer().Triangulate(points, config).Triangles.Count;
        }

        [Benchmark]
        public int Indices()
        {
            return new XDwyer().Triangulate(points, config).Count;
        }

        /// <summary>
        /// Setting up the mesh, part of <see cref="Objects"/>: the triangle pool allocates the
        /// objects for 2n triangles up front, the vertices go into a dictionary.
        /// </summary>
        [Benchmark]
        public int NewMesh()
        {
            return new Mesh(config, points).Vertices.Count;
        }

        private static List<Vertex> Grid(int n)
        {
            var list = new List<Vertex>(n * n);
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    list.Add(new Vertex(i / (double)(float)(n - 1), j / (double)(float)(n - 1)) { ID = list.Count });
            return list;
        }

        private static List<Vertex> Random(int count)
        {
            var r = new System.Random(1);
            var list = new List<Vertex>(count);
            for (int i = 0; i < count; i++) list.Add(new Vertex(r.NextDouble(), r.NextDouble()) { ID = i });
            return list;
        }
    }
}
