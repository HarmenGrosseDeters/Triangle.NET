namespace TriangleNet.Tests
{
    using TriangleNet.Tests.Robustness;

    /// <summary>
    /// Entry point of the test assembly.
    /// </summary>
    /// <remarks>
    /// The test runner doesn't use this entry point. It is used by <see cref="IsolatedProcess"/>
    /// to execute scenarios that might crash the process (e.g. StackOverflowException) or never
    /// terminate, so they can't take down the whole test run.
    /// </remarks>
    public static class Program
    {
        public static int Main(string[] args)
        {
            return IsolatedProcess.RunScenario(args);
        }
    }
}
