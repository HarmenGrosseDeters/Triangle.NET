using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace TriangleNet.Tests.Corpus
{
    /// <summary>
    /// A test case of an external corpus: polylines (become segments) and single points.
    /// </summary>
    public class CorpusCase
    {
        public string Name { get; set; }

        public List<List<(double X, double Y)>> Lines { get; } = new List<List<(double X, double Y)>>();

        public List<(double X, double Y)> Points { get; } = new List<(double X, double Y)>();

        /// <summary>
        /// Save the case in a simple text format (one line per polyline / point).
        /// </summary>
        public void Save(string path)
        {
            using var writer = new StreamWriter(path);

            writer.WriteLine("# " + Name);

            foreach (var line in Lines)
            {
                writer.WriteLine("L " + string.Join(" ", line.Select(p => Format(p.X) + "," + Format(p.Y))));
            }

            foreach (var p in Points)
            {
                writer.WriteLine("P " + Format(p.X) + "," + Format(p.Y));
            }
        }

        public static CorpusCase Load(string path)
        {
            var c = new CorpusCase();

            foreach (var row in File.ReadLines(path))
            {
                if (row.StartsWith("# "))
                {
                    c.Name = row.Substring(2);
                }
                else if (row.StartsWith("L "))
                {
                    c.Lines.Add(row.Substring(2).Split(' ').Select(Parse).ToList());
                }
                else if (row.StartsWith("P "))
                {
                    c.Points.Add(Parse(row.Substring(2)));
                }
            }

            return c;
        }

        private static string Format(double x) => x.ToString("R", CultureInfo.InvariantCulture);

        private static (double, double) Parse(string s)
        {
            var parts = s.Split(',');
            return (double.Parse(parts[0], CultureInfo.InvariantCulture), double.Parse(parts[1], CultureInfo.InvariantCulture));
        }
    }
}
