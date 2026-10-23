// -----------------------------------------------------------------------
// <copyright file="SegmentNoder.cs">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing
{
    using System;
    using System.Collections.Generic;
    using TriangleNet.Geometry;

    /// <summary>
    /// Splits the segments of a polygon at their intersections (noding).
    /// </summary>
    /// <remarks>
    /// The constraint mesher can insert intersecting segments by itself, but it does so one
    /// intersection at a time. The rounded intersection vertex doesn't lie exactly on the
    /// segments, which can break the topology if several segments cross in a small region.
    ///
    /// The noder computes all intersections in advance and splits the segments, so the mesher
    /// never sees crossing segments. Crossings are detected using exact predicates.
    ///
    /// Additionally, vertices closer than a small tolerance to a segment are snapped onto the
    /// segment (the segment is split at the vertex, the vertex isn't moved). Otherwise, vertices
    /// lying almost on a segment (like rounded intersection points) produce nearly degenerate
    /// triangles, which can be inverted by subsequent refinement (inserting rounded segment
    /// midpoints). After snapping, the distance of a vertex to a segment is either zero or
    /// larger than the tolerance.
    ///
    /// Since both steps can create new crossings, they are repeated until no more changes occur.
    /// </remarks>
    public static class SegmentNoder
    {
        /// <summary>
        /// Relative snap tolerance (with respect to the maximum of the coordinate magnitudes
        /// and the extent of the input), about 1e-12.
        /// </summary>
        public const double DefaultRelativeTolerance = 1.0 / (1L << 40);

        /// <summary>
        /// Split all segments of the polygon at their intersections and at vertices lying
        /// (almost) on the segments.
        /// </summary>
        /// <param name="polygon">The input polygon (won't be modified).</param>
        /// <param name="predicates">The predicates used to detect crossings.</param>
        /// <param name="maxIterations">Maximum number of noding iterations.</param>
        /// <returns>The input polygon, if nothing had to be changed, otherwise a new polygon.</returns>
        public static IPolygon Node(IPolygon polygon, IPredicates predicates, int maxIterations = 50)
        {
            if (polygon.Segments.Count == 0)
            {
                return polygon;
            }

            var segments = new List<ISegment>(polygon.Segments);
            var newPoints = new List<Vertex>();

            double tolerance = GetTolerance(polygon);

            var vertices = new VertexIndex(polygon.Points, tolerance);

            bool changed = false;

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                var splits = new Dictionary<int, List<(double, Vertex)>>();

                AddIntersections(segments, FindCrossings(segments, predicates), vertices, newPoints, splits);

                if (splits.Count == 0)
                {
                    AddNearbyVertices(segments, vertices.All, tolerance, splits);
                }

                if (splits.Count == 0)
                {
                    break;
                }

                segments = Split(segments, splits);
                changed = true;
            }

            if (!changed)
            {
                return polygon;
            }

            var result = new Polygon(polygon.Points.Count + newPoints.Count)
            {
                HasPointMarkers = polygon.HasPointMarkers,
                HasSegmentMarkers = polygon.HasSegmentMarkers
            };

            result.Points.AddRange(polygon.Points);
            result.Points.AddRange(newPoints);
            result.Segments.AddRange(segments);
            result.Holes.AddRange(polygon.Holes);
            result.Regions.AddRange(polygon.Regions);

            return result;
        }

        private static double GetTolerance(IPolygon polygon)
        {
            var bounds = new Rectangle();
            double max = 0.0;

            foreach (var v in polygon.Points)
            {
                bounds.Expand(v);
                max = Math.Max(max, Math.Max(Math.Abs(v.x), Math.Abs(v.y)));
            }

            return DefaultRelativeTolerance * Math.Max(max, Math.Max(bounds.Width, bounds.Height));
        }

        /// <summary>
        /// Find all pairs of segments that cross in their interiors (sweep along x-axis).
        /// </summary>
        private static List<(int, int)> FindCrossings(List<ISegment> segments, IPredicates predicates)
        {
            int n = segments.Count;

            var order = new int[n];
            var xmin = new double[n];

            for (int i = 0; i < n; i++)
            {
                order[i] = i;
                xmin[i] = Math.Min(segments[i].GetVertex(0).x, segments[i].GetVertex(1).x);
            }

            Array.Sort((double[])xmin.Clone(), order);

            var crossings = new List<(int, int)>();

            for (int k = 0; k < n; k++)
            {
                var s = segments[order[k]];
                var a = s.GetVertex(0);
                var b = s.GetVertex(1);

                double xmax = Math.Max(a.x, b.x);
                double ymin = Math.Min(a.y, b.y);
                double ymax = Math.Max(a.y, b.y);

                for (int m = k + 1; m < n && xmin[order[m]] <= xmax; m++)
                {
                    var t = segments[order[m]];
                    var c = t.GetVertex(0);
                    var d = t.GetVertex(1);

                    if (Math.Max(c.y, d.y) < ymin || Math.Min(c.y, d.y) > ymax)
                    {
                        continue;
                    }

                    if (Cross(a, b, c, d, predicates))
                    {
                        crossings.Add((order[k], order[m]));
                    }
                }
            }

            return crossings;
        }

        /// <summary>
        /// Returns true, if the segments (a,b) and (c,d) cross in a single point which is
        /// interior to both segments.
        /// </summary>
        private static bool Cross(Vertex a, Vertex b, Vertex c, Vertex d, IPredicates predicates)
        {
            double o1 = predicates.CounterClockwise(a, b, c);
            double o2 = predicates.CounterClockwise(a, b, d);

            if (!((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)))
            {
                return false;
            }

            double o3 = predicates.CounterClockwise(c, d, a);
            double o4 = predicates.CounterClockwise(c, d, b);

            return (o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0);
        }

        private static void AddIntersections(List<ISegment> segments, List<(int, int)> crossings,
            VertexIndex vertices, List<Vertex> newPoints,
            Dictionary<int, List<(double, Vertex)>> splits)
        {
            foreach (var (i, j) in crossings)
            {
                var s = segments[i];
                var t = segments[j];

                var a = s.GetVertex(0);
                var b = s.GetVertex(1);
                var c = t.GetVertex(0);
                var d = t.GetVertex(1);

                double abx = b.x - a.x, aby = b.y - a.y;
                double cdx = d.x - c.x, cdy = d.y - c.y;

                double denom = abx * cdy - aby * cdx;

                if (denom == 0.0)
                {
                    // Nearly parallel, can't compute the intersection.
                    continue;
                }

                double ts = ((c.x - a.x) * cdy - (c.y - a.y) * cdx) / denom;

                ts = Math.Max(0.0, Math.Min(1.0, ts));

                double x = a.x + ts * abx;
                double y = a.y + ts * aby;

                var v = GetOrCreateVertex(x, y, s.Label, vertices, newPoints);

                AddSplit(splits, i, Parameter(a, b, v), v);
                AddSplit(splits, j, Parameter(c, d, v), v);
            }
        }

        /// <summary>
        /// Find vertices which lie in the interior of a segment or closer than the given
        /// tolerance to the segment.
        /// </summary>
        private static void AddNearbyVertices(List<ISegment> segments, ICollection<Vertex> vertices,
            double tolerance, Dictionary<int, List<(double, Vertex)>> splits)
        {
            var grid = new VertexGrid(vertices);

            for (int i = 0; i < segments.Count; i++)
            {
                var s = segments[i];
                var a = s.GetVertex(0);
                var b = s.GetVertex(1);

                double dx = b.x - a.x, dy = b.y - a.y;
                double length = Math.Sqrt(dx * dx + dy * dy);

                if (length == 0.0)
                {
                    continue;
                }

                foreach (var v in grid.Query(
                    Math.Min(a.x, b.x) - tolerance, Math.Min(a.y, b.y) - tolerance,
                    Math.Max(a.x, b.x) + tolerance, Math.Max(a.y, b.y) + tolerance))
                {
                    if (SameLocation(v, a) || SameLocation(v, b))
                    {
                        continue;
                    }

                    // Distance to the line and parameter along the segment.
                    double distance = Math.Abs(dx * (v.y - a.y) - dy * (v.x - a.x)) / length;
                    double t = ((v.x - a.x) * dx + (v.y - a.y) * dy) / (length * length);

                    if (distance <= tolerance && t > 0.0 && t < 1.0)
                    {
                        AddSplit(splits, i, t, v);
                    }
                }
            }
        }

        private static List<ISegment> Split(List<ISegment> segments, Dictionary<int, List<(double, Vertex)>> splits)
        {
            var result = new List<ISegment>(segments.Count + 2 * splits.Count);

            for (int i = 0; i < segments.Count; i++)
            {
                var s = segments[i];

                if (!splits.TryGetValue(i, out var list))
                {
                    result.Add(s);
                    continue;
                }

                list.Sort((p, q) => p.Item1.CompareTo(q.Item1));

                var prev = s.GetVertex(0);

                var last = s.GetVertex(1);

                foreach (var (t, v) in list)
                {
                    if (t <= 0.0 || t >= 1.0 || SameLocation(v, last))
                    {
                        continue;
                    }

                    if (!SameLocation(prev, v))
                    {
                        result.Add(new Segment(prev, v, s.Label));
                        prev = v;
                    }
                }

                if (!SameLocation(prev, last))
                {
                    result.Add(new Segment(prev, last, s.Label));
                }
            }

            return result;
        }

        private static void AddSplit(Dictionary<int, List<(double, Vertex)>> splits, int i, double t, Vertex v)
        {
            if (!splits.TryGetValue(i, out var list))
            {
                list = new List<(double, Vertex)>();
                splits.Add(i, list);
            }

            list.Add((t, v));
        }

        private static double Parameter(Vertex a, Vertex b, Vertex v)
        {
            double dx = b.x - a.x, dy = b.y - a.y;

            return ((v.x - a.x) * dx + (v.y - a.y) * dy) / (dx * dx + dy * dy);
        }

        private static Vertex GetOrCreateVertex(double x, double y, int label,
            VertexIndex vertices, List<Vertex> newPoints)
        {
            // Merging with vertices closer than the tolerance guarantees that new vertices
            // have a minimum distance, so the noding terminates.
            var v = vertices.Find(x, y);

            if (v == null)
            {
                v = new Vertex(x, y, label);
                vertices.Add(v);
                newPoints.Add(v);
            }

            return v;
        }

        private static bool SameLocation(Vertex a, Vertex b)
        {
            return a.x == b.x && a.y == b.y;
        }

        /// <summary>
        /// Spatial hash to find a vertex within the given tolerance.
        /// </summary>
        private class VertexIndex
        {
            readonly Dictionary<(long, long), List<Vertex>> cells = new Dictionary<(long, long), List<Vertex>>();
            readonly double tolerance, size;

            public List<Vertex> All { get; } = new List<Vertex>();

            public VertexIndex(IEnumerable<Vertex> vertices, double tolerance)
            {
                this.tolerance = tolerance;
                this.size = tolerance > 0.0 ? tolerance : 1.0;

                // Input vertices are never merged (only exact duplicates are skipped).
                foreach (var v in vertices)
                {
                    if (!ContainsExact(v))
                    {
                        Add(v);
                    }
                }
            }

            public void Add(Vertex v)
            {
                var key = Key(v.x, v.y);

                if (!cells.TryGetValue(key, out var list))
                {
                    list = new List<Vertex>();
                    cells.Add(key, list);
                }

                list.Add(v);
                All.Add(v);
            }

            /// <summary>
            /// Find the closest vertex within the tolerance (or null).
            /// </summary>
            public Vertex Find(double x, double y)
            {
                var (i, j) = Key(x, y);

                Vertex result = null;
                double best = double.MaxValue;

                for (long di = -1; di <= 1; di++)
                {
                    for (long dj = -1; dj <= 1; dj++)
                    {
                        if (!cells.TryGetValue((i + di, j + dj), out var list))
                        {
                            continue;
                        }

                        foreach (var v in list)
                        {
                            double dx = v.x - x, dy = v.y - y;
                            double d = Math.Sqrt(dx * dx + dy * dy);

                            if (d <= tolerance && d < best)
                            {
                                best = d;
                                result = v;
                            }
                        }
                    }
                }

                return result;
            }

            private bool ContainsExact(Vertex v)
            {
                return cells.TryGetValue(Key(v.x, v.y), out var list) && list.Exists(p => SameLocation(p, v));
            }

            private (long, long) Key(double x, double y)
            {
                return ((long)Math.Floor(x / size), (long)Math.Floor(y / size));
            }
        }

        /// <summary>
        /// Simple uniform grid for range queries.
        /// </summary>
        private class VertexGrid
        {
            readonly Dictionary<(long, long), List<Vertex>> cells = new Dictionary<(long, long), List<Vertex>>();
            readonly ICollection<Vertex> vertices;
            readonly double left, bottom, size;

            public VertexGrid(ICollection<Vertex> vertices)
            {
                this.vertices = vertices;

                var bounds = new Rectangle();

                foreach (var v in vertices)
                {
                    bounds.Expand(v);
                }

                left = bounds.Left;
                bottom = bounds.Bottom;

                // About one vertex per cell.
                size = Math.Max(bounds.Width, bounds.Height) / Math.Max(1.0, Math.Sqrt(vertices.Count));

                if (size <= 0.0)
                {
                    size = 1.0;
                }

                foreach (var v in vertices)
                {
                    var key = (Cell(v.x, left), Cell(v.y, bottom));

                    if (!cells.TryGetValue(key, out var list))
                    {
                        list = new List<Vertex>();
                        cells.Add(key, list);
                    }

                    list.Add(v);
                }
            }

            public IEnumerable<Vertex> Query(double x0, double y0, double x1, double y1)
            {
                long i0 = Cell(x0, left), i1 = Cell(x1, left);
                long j0 = Cell(y0, bottom), j1 = Cell(y1, bottom);

                if ((double)(i1 - i0 + 1) * (j1 - j0 + 1) > cells.Count)
                {
                    // Large query range: scan all vertices.
                    foreach (var v in vertices)
                    {
                        if (v.x >= x0 && v.x <= x1 && v.y >= y0 && v.y <= y1)
                        {
                            yield return v;
                        }
                    }

                    yield break;
                }

                for (long i = i0; i <= i1; i++)
                {
                    for (long j = j0; j <= j1; j++)
                    {
                        if (cells.TryGetValue((i, j), out var list))
                        {
                            foreach (var v in list)
                            {
                                if (v.x >= x0 && v.x <= x1 && v.y >= y0 && v.y <= y1)
                                {
                                    yield return v;
                                }
                            }
                        }
                    }
                }
            }

            private long Cell(double x, double origin)
            {
                return (long)Math.Floor((x - origin) / size);
            }
        }
    }
}
