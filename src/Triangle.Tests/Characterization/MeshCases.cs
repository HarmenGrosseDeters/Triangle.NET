using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Smoothing;
using TriangleNet.Tests.Corpus;

namespace TriangleNet.Tests.Characterization
{
    /// <summary>
    /// Catalog of deterministic meshing scenarios (input × options). Used by the invariant
    /// tests, the golden fingerprint tests and the benchmarks, so that performance work is
    /// checked against exactly the workloads it is measured on.
    /// </summary>
    public static class MeshCases
    {
        /// <summary>
        /// A meshing scenario.
        /// </summary>
        public class Case
        {
            public string Name { get; set; }

            /// <summary>
            /// Creates a fresh input (inputs are modified by the mesher, e.g. vertex IDs).
            /// </summary>
            public Func<IPolygon> Input { get; set; }

            public Func<IPolygon, Mesh> Run { get; set; }

            /// <summary>
            /// Minimum angle requested (0 = no quality).
            /// </summary>
            public double MinAngle { get; set; }

            /// <summary>
            /// Maximum triangle area requested (0 = none).
            /// </summary>
            public double MaxArea { get; set; }

            /// <summary>
            /// The input has no angles below 60° between segments, so every triangle must
            /// meet the minimum angle.
            /// </summary>
            public bool AngleGuarantee { get; set; }

            /// <summary>
            /// The result must be (conforming) Delaunay, not only constrained Delaunay.
            /// </summary>
            public bool Conforming { get; set; }

            /// <summary>
            /// Segments that must not be split by refinement.
            /// </summary>
            public Func<ISegment, bool> Locked { get; set; }

            /// <summary>
            /// Snap tolerance (<see cref="ConstraintOptions.SnapTolerance"/>).
            /// </summary>
            public double Snap { get; set; }

            public override string ToString() => Name;
        }

        public const int SteinerLimit = 200000;

        private static readonly string DataDirectory =
            Path.Combine(Path.GetDirectoryName(typeof(MeshCases).Assembly.Location), "Characterization", "Data");

        /// <summary>
        /// All scenarios.
        /// </summary>
        public static IEnumerable<Case> All()
        {
            // Point sets: all three triangulation algorithms.
            foreach (var (name, input) in PointSets())
            {
                yield return Points(name + "/dwyer", input, () => new Dwyer());
                yield return Points(name + "/incremental", input, () => new Incremental());
                yield return Points(name + "/sweepline", input, () => new SweepLine());
                yield return Quality(name + "/q30", input, new ConstraintOptions(), 30, angleGuarantee: true);
            }

            // Polygons with segments.
            foreach (var (name, input, nice) in Polygons())
            {
                yield return Quality(name + "/cdt", input, new ConstraintOptions(), 0);
                yield return Quality(name + "/conforming", input, new ConstraintOptions { ConformingDelaunay = true }, 0, conforming: true);
                yield return Quality(name + "/q20", input, new ConstraintOptions(), 20, angleGuarantee: nice);
                yield return Quality(name + "/q28-legacy", input, new ConstraintOptions(), 28, angleGuarantee: nice, legacy: true);
                yield return Quality(name + "/q33", input, new ConstraintOptions(), 33, angleGuarantee: nice);
                yield return Quality(name + "/q25-conforming", input, new ConstraintOptions { ConformingDelaunay = true }, 25, angleGuarantee: nice, conforming: true);
                yield return Quality(name + "/q20-split1", input, new ConstraintOptions { SegmentSplitting = 1 }, 20);
                yield return Quality(name + "/q20-split2", input, new ConstraintOptions { SegmentSplitting = 2 }, 20);
                yield return Quality(name + "/q20-locked", input, new ConstraintOptions { CanSplitSegment = s => s.Label != 7 }, 20,
                    locked: s => s.Label == 7);
            }

            // Maximum area, regions with area constraints, refinement in two steps, smoothing.
            yield return Quality("square/q20-area", () => SquareWithHole(), new ConstraintOptions(), 20, maxArea: 0.5, angleGuarantee: true);
            yield return Quality("regions/q25-variable-area", Regions, new ConstraintOptions(), 25, variableArea: true, angleGuarantee: true);

            yield return new Case
            {
                Name = "square/refine-twice",
                Input = () => SquareWithHole(),
                MinAngle = 30,
                MaxArea = 0.2,
                AngleGuarantee = true,
                Run = p =>
                {
                    var mesh = (Mesh)p.Triangulate(new ConstraintOptions(), new QualityOptions { MinimumAngle = 20, MaximumArea = 2 });
                    mesh.Refine(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.2 });
                    return mesh;
                }
            };

            yield return new Case
            {
                Name = "square/smooth",
                Input = () => SquareWithHole(),
                Run = p =>
                {
                    var mesh = (Mesh)p.Triangulate(new ConstraintOptions(), new QualityOptions { MinimumAngle = 25, MaximumArea = 1 });
                    new SimpleSmoother().Smooth(mesh, 5);
                    return mesh;
                }
            };

            // Real data (ALKIS Herne, parcels and buildings) with and without snapping.
            foreach (var tile in new[] { "herne_371800_5710600", "herne_373600_5709000", "herne_381000_5710400" })
            {
                IPolygon Tile() => LoadTile(tile);

                yield return Quality(tile + "/cdt", Tile, new ConstraintOptions(), 0);
                yield return Quality(tile + "/q20", Tile, new ConstraintOptions(), 20);
                yield return Quality(tile + "/q20-snap1cm", Tile, new ConstraintOptions { SnapTolerance = 0.01 }, 20);
                yield return Quality(tile + "/q30-snap1cm", Tile, new ConstraintOptions { SnapTolerance = 0.01 }, 30);
            }
        }

        private static Case Points(string name, Func<IPolygon> input, Func<ITriangulator> algorithm)
        {
            return new Case
            {
                Name = name,
                Input = input,
                Conforming = true,
                Run = p => (Mesh)new GenericMesher(algorithm()).Triangulate(p.Points)
            };
        }

        private static Case Quality(string name, Func<IPolygon> input, ConstraintOptions options, double minAngle,
            double maxArea = 0, bool variableArea = false, bool angleGuarantee = false, bool conforming = false,
            bool legacy = false, Func<ISegment, bool> locked = null)
        {
            return new Case
            {
                Name = name,
                Input = input,
                MinAngle = minAngle,
                MaxArea = maxArea,
                AngleGuarantee = angleGuarantee && minAngle > 0,
                Conforming = conforming,
                Locked = locked,
                Snap = options.SnapTolerance,
                Run = p =>
                {
                    // Conforming Delaunay without quality constraints has no Steiner point
                    // limit otherwise.
                    var quality = minAngle > 0 || maxArea > 0 || variableArea || options.ConformingDelaunay
                        ? new QualityOptions
                        {
                            MinimumAngle = minAngle,
                            MaximumArea = maxArea,
                            VariableArea = variableArea,
                            SteinerPoints = SteinerLimit,
                            UseLegacyRefinement = legacy
                        }
                        : null;

                    return (Mesh)p.Triangulate(options, quality);
                }
            };
        }

        #region Inputs

        private static IEnumerable<(string, Func<IPolygon>)> PointSets()
        {
            yield return ("points-random", () => RandomPoints(1, 3000, 0, 0, 1000));
            yield return ("points-utm", () => RandomPoints(2, 2000, 372000, 5710000, 200));
            yield return ("points-grid", Grid);
            yield return ("points-clustered", Clustered);
        }

        private static IEnumerable<(string, Func<IPolygon>, bool)> Polygons()
        {
            yield return ("square-hole", () => SquareWithHole(), true);
            yield return ("star", Star, false);
            yield return ("crossing-segments", CrossingSegments, false);
            yield return ("parcels-mismatch", ParcelsWithMismatchedBoundaries, false);
            yield return ("utm-breaklines", UtmBreaklines, false);
        }

        private static IPolygon RandomPoints(int seed, int n, double x0, double y0, double size)
        {
            var random = new Random(seed);
            var poly = new Polygon(n);

            for (int i = 0; i < n; i++)
            {
                poly.Add(new Vertex(x0 + random.NextDouble() * size, y0 + random.NextDouble() * size));
            }

            return poly;
        }

        private static IPolygon Grid()
        {
            // Highly cocircular (worst case for the incircle test).
            var poly = new Polygon();

            for (int i = 0; i < 40; i++)
            {
                for (int j = 0; j < 40; j++)
                {
                    poly.Add(new Vertex(372000 + i, 5710000 + j));
                }
            }

            return poly;
        }

        private static IPolygon Clustered()
        {
            var random = new Random(3);
            var poly = new Polygon();

            for (int c = 0; c < 20; c++)
            {
                double cx = random.NextDouble() * 1000, cy = random.NextDouble() * 1000;
                double r = Math.Pow(10, -random.Next(1, 6));

                for (int i = 0; i < 100; i++)
                {
                    poly.Add(new Vertex(cx + (random.NextDouble() - 0.5) * r, cy + (random.NextDouble() - 0.5) * r));
                }
            }

            return poly;
        }

        private static Polygon SquareWithHole(double size = 10)
        {
            var poly = new Polygon();

            poly.Add(Contour(7, (0, 0), (size, 0), (size, size), (0, size)));
            poly.Add(Contour(2, (3, 3), (3, 6), (6, 6), (6, 3)), true);

            return poly;
        }

        private static IPolygon Regions()
        {
            var poly = new Polygon();

            poly.Add(Contour(1, (0, 0), (10, 0), (10, 10), (0, 10)));
            poly.Add(new Segment(poly.Points[0], poly.Points[2], 3));
            poly.Regions.Add(new RegionPointer(7, 2, 1, 0.1));
            poly.Regions.Add(new RegionPointer(2, 7, 2, 2.0));

            return poly;
        }

        private static IPolygon Star()
        {
            // Small input angles (5°) at the tips and the center.
            var poly = new Polygon();
            var center = new Vertex(0, 0);
            poly.Add(center);

            int n = 12;

            for (int i = 0; i < n; i++)
            {
                double a = 2 * Math.PI * i / n;
                var tip = new Vertex(10 * Math.Cos(a), 10 * Math.Sin(a));
                var side = new Vertex(10 * Math.Cos(a + 0.087), 10 * Math.Sin(a + 0.087));

                poly.Add(tip);
                poly.Add(side);
                poly.Add(new Segment(center, tip, 7));
                poly.Add(new Segment(tip, side, 1));
            }

            poly.Add(Contour(1, (-15, -15), (15, -15), (15, 15), (-15, 15)));

            return poly;
        }

        private static IPolygon CrossingSegments()
        {
            var poly = new Polygon();
            var random = new Random(4);

            poly.Add(Contour(1, (0, 0), (100, 0), (100, 100), (0, 100)));

            for (int i = 0; i < 40; i++)
            {
                var a = new Vertex(random.NextDouble() * 100, random.NextDouble() * 100);
                var b = new Vertex(random.NextDouble() * 100, random.NextDouble() * 100);

                poly.Add(a);
                poly.Add(b);
                poly.Add(new Segment(a, b, i % 2 == 0 ? 7 : 2));
            }

            return poly;
        }

        private static IPolygon ParcelsWithMismatchedBoundaries()
        {
            // 4 × 4 parcels, each digitized separately; shared boundaries differ by up to 1 mm.
            var poly = new Polygon();
            var random = new Random(5);
            double x0 = 372000, y0 = 5710000;

            double Jitter() => (random.NextDouble() - 0.5) * 1e-3;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    double x = x0 + i * 25, y = y0 + j * 20;

                    poly.Add(Contour((i + j) % 2 == 0 ? 7 : 1,
                        (x + Jitter(), y + Jitter()), (x + 25 + Jitter(), y + Jitter()),
                        (x + 25 + Jitter(), y + 20 + Jitter()), (x + Jitter(), y + 20 + Jitter())));
                }
            }

            return poly;
        }

        private static IPolygon UtmBreaklines()
        {
            // Non-crossing polylines in Gauss-Krüger coordinates plus scattered points.
            var poly = new Polygon();
            var random = new Random(6);
            double x0 = 2580000, y0 = 5640000;

            poly.Add(Contour(1, (x0, y0), (x0 + 300, y0), (x0 + 300, y0 + 300), (x0, y0 + 300)));

            for (int k = 0; k < 8; k++)
            {
                Vertex prev = null;

                for (int i = 0; i <= 20; i++)
                {
                    var v = new Vertex(x0 + 10 + i * 14 + random.NextDouble(), y0 + 20 + k * 35 + 5 * Math.Sin(i * 0.7 + k) + random.NextDouble() * 0.01);
                    poly.Add(v);

                    if (prev != null) poly.Add(new Segment(prev, v, k % 2 == 0 ? 7 : 2));

                    prev = v;
                }
            }

            for (int i = 0; i < 300; i++)
            {
                poly.Add(new Vertex(x0 + 1 + random.NextDouble() * 298, y0 + 1 + random.NextDouble() * 298));
            }

            return poly;
        }

        private static IPolygon LoadTile(string name)
        {
            var c = CorpusCase.Load(Path.Combine(DataDirectory, name + ".txt"));
            var poly = new Polygon();
            var map = new Dictionary<(double, double), Vertex>();

            Vertex V((double, double) p)
            {
                if (!map.TryGetValue(p, out var v))
                {
                    map[p] = v = new Vertex(p.Item1, p.Item2);
                    poly.Add(v);
                }

                return v;
            }

            int k = 0;

            foreach (var line in c.Lines)
            {
                for (int i = 0; i + 1 < line.Count; i++)
                {
                    if (line[i] != line[i + 1])
                    {
                        poly.Add(new Segment(V(line[i]), V(line[i + 1]), k % 3 == 0 ? 7 : 1));
                    }
                }

                k++;
            }

            // Frame (5 % margin).
            var b = poly.Bounds();
            double w = Math.Max(b.Width, b.Height) * 0.05;
            poly.Add(Contour(1, (b.Left - w, b.Bottom - w), (b.Right + w, b.Bottom - w), (b.Right + w, b.Top + w), (b.Left - w, b.Top + w)));

            return poly;
        }

        private static Contour Contour(int label, params (double X, double Y)[] points)
        {
            return new Contour(points.Select(p => new Vertex(p.X, p.Y, label)), label);
        }

        #endregion
    }
}
