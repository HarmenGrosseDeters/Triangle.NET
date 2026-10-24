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
            "herne_371800_5710600/q20-grid1cm-wedge3cm",
            "herne_373600_5709000/q30-grid1cm-wedge3cm",
        };

        private MeshCases.Case scenario;

        /// <summary>
        /// The scenarios to run: <see cref="Representative"/>, or the comma separated list in the
        /// environment variable TRIANGLE_BENCH_SCENARIOS.
        /// </summary>
        public IEnumerable<string> Scenarios()
        {
            var selection = System.Environment.GetEnvironmentVariable("TRIANGLE_BENCH_SCENARIOS");

            return string.IsNullOrEmpty(selection) ? Representative : selection.Split(',');
        }

        [ParamsSource(nameof(Scenarios))]
        public string Scenario { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            scenario = MeshCases.All().Concat(MeshCases.GridCases()).First(c => c.Name == Scenario);
        }

        [Benchmark]
        public int Mesh()
        {
            return scenario.Run(scenario.Input()).Triangles.Count;
        }
    }
}
