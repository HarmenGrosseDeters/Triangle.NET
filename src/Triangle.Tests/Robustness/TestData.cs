using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;

namespace TriangleNet.Tests.Robustness
{
    static class TestData
    {
        public static Contour Square(double x, double y, double size)
        {
            return new Contour(new[]
            {
                new Vertex(x, y), new Vertex(x + size, y),
                new Vertex(x + size, y + size), new Vertex(x, y + size)
            });
        }

        /// <summary>
        /// Square [0,10]x[0,10] with additional segments given as (x1, y1, x2, y2).
        /// Segment endpoints are separate vertex instances (duplicates are handled by the mesher).
        /// </summary>
        public static Polygon SquareWithSegments(params (double, double, double, double)[] segments)
        {
            var poly = new Polygon();

            poly.Add(Square(0, 0, 10));

            foreach (var (x1, y1, x2, y2) in segments)
            {
                poly.Add(new Segment(new Vertex(x1, y1), new Vertex(x2, y2)), true);
            }

            return poly;
        }

        /// <summary>
        /// Random segments with integer coordinates in [0,10] (many crossings, touching
        /// endpoints and collinear overlaps).
        /// </summary>
        public static (double, double, double, double)[] RandomIntegerSegments(int seed, int count)
        {
            var random = new Random(seed);

            return Enumerable.Range(0, count)
                .Select(_ => ((double)random.Next(11), (double)random.Next(11), (double)random.Next(11), (double)random.Next(11)))
                .ToArray();
        }

        /// <summary>
        /// Two polygons sharing a common boundary which was digitized twice, with the
        /// vertices of the second copy perturbed by up to eps (typical GIS data).
        /// </summary>
        public static Polygon AdjacentPolygonsWithMismatchedBoundary(double eps)
        {
            var poly = new Polygon();
            var random = new Random(3);

            var left = new List<Vertex> { new Vertex(0, 0) };
            var right = new List<Vertex>();

            for (int i = 0; i <= 10; i++)
            {
                double x = 10 + Math.Sin(i), y = i;

                left.Add(new Vertex(x, y));
                right.Add(new Vertex(x + (random.NextDouble() - 0.5) * eps, y + (random.NextDouble() - 0.5) * eps));
            }

            left.Add(new Vertex(0, 10));
            right.Add(new Vertex(20, 10));
            right.Add(new Vertex(20, 0));

            poly.Add(new Contour(left));
            poly.Add(new Contour(right));

            return poly;
        }
    }
}
