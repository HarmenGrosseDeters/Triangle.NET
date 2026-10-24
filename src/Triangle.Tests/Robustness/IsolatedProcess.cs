using NUnit.Framework;
using System;
using System.Diagnostics;
using System.Reflection;
using System.Text;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Runs a scenario (a public static method of <see cref="IsolatedScenarios"/>) in a child
    /// process. Use this for code that might crash the process (StackOverflowException can't be
    /// caught in .NET) or might never terminate.
    /// </summary>
    static class IsolatedProcess
    {
        public const int ExitSuccess = 0;
        public const int ExitException = 1;
        public const int ExitUnknownScenario = 2;

        public static readonly TimeSpan DefaultTimeout = TimeSpan.FromSeconds(30);

        /// <summary>
        /// Assert that the given scenario completes successfully in a child process.
        /// </summary>
        public static void AssertSucceeds(string scenario, TimeSpan? timeout = null)
        {
            var result = Run(scenario, timeout ?? DefaultTimeout);

            if (result.TimedOut)
            {
                Assert.Fail($"Scenario '{scenario}' did not terminate within {(timeout ?? DefaultTimeout).TotalSeconds}s (infinite loop?).");
            }

            if (result.ExitCode != ExitSuccess)
            {
                string reason = result.ExitCode == ExitException
                    ? "threw an exception"
                    : $"crashed the process (exit code {result.ExitCode}, e.g. StackOverflowException)";

                Assert.Fail($"Scenario '{scenario}' {reason}:\n{Truncate(result.Output, 2000)}");
            }
        }

        public static (bool TimedOut, int ExitCode, string Output) Run(string scenario, TimeSpan timeout)
        {
            return Run(new[] { scenario }, timeout);
        }

        /// <summary>
        /// Run the test assembly in a child process with the given arguments (see <see cref="Program"/>).
        /// </summary>
        public static (bool TimedOut, int ExitCode, string Output) Run(string[] arguments, TimeSpan timeout)
        {
            var assembly = typeof(IsolatedProcess).Assembly.Location;
            var host = Environment.GetEnvironmentVariable("DOTNET_HOST_PATH") ?? "dotnet";

            var psi = new ProcessStartInfo(host)
            {
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false
            };

            psi.ArgumentList.Add(assembly);

            foreach (var argument in arguments)
            {
                psi.ArgumentList.Add(argument);
            }

            var output = new StringBuilder();

            using var process = new Process { StartInfo = psi };

            // Read asynchronously: a stack overflow trace can fill the pipe buffer.
            process.OutputDataReceived += (s, e) => { if (e.Data != null) lock (output) output.AppendLine(e.Data); };
            process.ErrorDataReceived += (s, e) => { if (e.Data != null) lock (output) output.AppendLine(e.Data); };

            process.Start();
            process.BeginOutputReadLine();
            process.BeginErrorReadLine();

            if (!process.WaitForExit((int)timeout.TotalMilliseconds))
            {
                process.Kill(true);
                return (true, -1, output.ToString());
            }

            process.WaitForExit();

            lock (output)
            {
                return (false, process.ExitCode, output.ToString());
            }
        }

        /// <summary>
        /// Executed in the child process (see <see cref="Program"/>).
        /// </summary>
        public static int RunScenario(string[] args)
        {
            if (args.Length != 1)
            {
                Console.Error.WriteLine("Usage: Triangle.Tests <scenario>");
                return ExitUnknownScenario;
            }

            var method = typeof(IsolatedScenarios).GetMethod(args[0], BindingFlags.Public | BindingFlags.Static);

            if (method == null)
            {
                Console.Error.WriteLine($"Unknown scenario '{args[0]}'.");
                return ExitUnknownScenario;
            }

            try
            {
                method.Invoke(null, null);
                return ExitSuccess;
            }
            catch (TargetInvocationException e)
            {
                Console.Error.WriteLine(e.InnerException);
                return ExitException;
            }
        }

        private static string Truncate(string s, int length)
        {
            return s.Length <= length ? s : s.Substring(0, length) + "...";
        }
    }
}
