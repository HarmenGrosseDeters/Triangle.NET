using BenchmarkDotNet.Attributes;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Tests.Characterization;

namespace TriangleNet.Benchmarks
{
    /// <summary>
    /// Runs scenarios of <see cref="MeshCases"/>. The time includes creating the input
    /// (small compared to meshing), because the mesher modifies its input.
    /// </summary>
    [MemoryDiagnoser]
    public class MeshingBenchmark
    {
        /// <summary>
        /// Representative scenarios: Delaunay of point sets, constrained/quality meshing of
        /// synthetic and real data, refinement with area constraints, smoothing.
        /// </summary>
        public static readonly string[] Representative =
        {
            "points-random/dwyer",
            "points-utm/sweepline",
            "points-grid/dwyer",
            "points-random/q30",
            "crossing-segments/q20",
            "utm-breaklines/q33",
            "parcels-mismatch/q20",
            "regions/q25-variable-area",
            "square/smooth",
            "herne_371800_5710600/q20-snap1cm",
            "herne_373600_5709000/q20-snap1cm",
            "herne_373600_5709000/q30-snap1cm",
        };

        private MeshCases.Case scenario;

        public IEnumerable<string> Scenarios() => Representative;

        [ParamsSource(nameof(Scenarios))]
        public string Scenario { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            scenario = MeshCases.All().First(c => c.Name == Scenario);
        }

        [Benchmark]
        public int Mesh()
        {
            return scenario.Run(scenario.Input()).Triangles.Count;
        }
    }
}
