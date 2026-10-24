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
    /// </remarks>
    public static class Program
    {
        public static void Main(string[] args)
        {
            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
        }
    }
}
