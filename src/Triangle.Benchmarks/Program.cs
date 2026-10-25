using BenchmarkDotNet.Running;

namespace TriangleNet.Benchmarks
{
    /// <summary>
    /// Benchmarks for performance work. Correctness is guarded by the characterization tests
    /// (Triangle.Tests/Characterization): run them before and after every optimization.
    /// </summary>
    /// <remarks>
    /// dotnet run -c Release --project src/Triangle.Benchmarks -- --filter "*"
    /// dotnet run -c Release --project src/Triangle.Benchmarks -- --filter "*" --job short
    ///
    /// Memory layout of triangles on .NET Framework 4.8 (Windows): compare
    /// dotnet run -c Release -f net48 --project src/Triangle.Benchmarks -p:TriangleLayout=Arrays -- --filter "*"
    /// dotnet run -c Release -f net48 --project src/Triangle.Benchmarks -p:TriangleLayout=Fields -- --filter "*"
    /// (select scenarios with the environment variable TRIANGLE_BENCH_SCENARIOS, see MeshingBenchmark).
    /// </remarks>
    public static class Program
    {
        public static void Main(string[] args)
        {
            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
        }
    }
}
