// -----------------------------------------------------------------------
// <copyright file="SegmentNoder.cs">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
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
    /// Vertices closer than a small tolerance to each other are merged (the first one is kept).
    /// Additionally, vertices closer than the tolerance to a segment are snapped onto the
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
        /// Largest relative snap tolerance used if noding doesn't converge (about 2.3e-10,
        /// i.e. 1.4 mm for UTM coordinates).
        /// </summary>
        public const double MaxRelativeTolerance = 1.0 / (1L << 32);

        /// <summary>
        /// Split all segments of the polygon at their intersections and at vertices lying
        /// (almost) on the segments.
        /// </summary>
        /// <param name="polygon">The input polygon (won't be modified).</param>
        /// <param name="predicates">The predicates used to detect crossings.</param>
        /// <param name="maxIterations">Maximum number of noding iterations per tolerance.</param>
        /// <param name="minTolerance">Minimum absolute snap tolerance (0 = automatic).</param>
        /// <returns>The input polygon, if nothing had to be changed, otherwise a new polygon.</returns>
        public static IPolygon Node(IPolygon polygon, IPredicates predicates, int maxIterations = 10, double minTolerance = 0.0)
        {
            return Node(polygon, predicates, out _, maxIterations, minTolerance);
        }

        /// <summary>
        /// Maximum angle (in degrees) of a wedge between two segments which is merged
        /// (see the wedgeTolerance parameter of <see cref="Node(IPolygon, IPredicates, out double, int, double, double)"/>).
        /// </summary>
        public const double MaxWedgeAngle = 10.0;

        /// <summary>
        /// Split all segments of the polygon at their intersections and at vertices lying
        /// (almost) on the segments.
        /// </summary>
        /// <param name="polygon">The input polygon (won't be modified).</param>
        /// <param name="predicates">The predicates used to detect crossings.</param>
        /// <param name="tolerance">The absolute snap tolerance used.</param>
        /// <param name="maxIterations">Maximum number of noding iterations per tolerance.</param>
        /// <param name="minTolerance">Minimum absolute snap tolerance (0 = automatic). Use this to
        /// merge features below the precision of the data, e.g. 0.01 for coordinates in meters
        /// which are only accurate to a centimeter.</param>
        /// <param name="wedgeTolerance">Distance up to which nearly coincident segments are
        /// merged (0 = off): a vertex v is inserted into a segment (a, b), if its distance to the
        /// segment is at most the wedge tolerance, and v is connected to a or b by a segment which
        /// forms an angle of less than <see cref="MaxWedgeAngle"/> with (a, b). Such thin wedges
        /// (e.g. a wall digitized slightly off a parcel boundary) have tiny input angles which
        /// refinement can't fix.</param>
        /// <returns>The input polygon, if nothing had to be changed, otherwise a new polygon.</returns>
        /// <remarks>
        /// If noding doesn't converge (e.g. for many nearly coincident segments, where rounded
        /// intersection points keep creating new intersections), it is repeated with a larger
        /// tolerance (up to <see cref="MaxRelativeTolerance"/>), like the snapping noder of GEOS.
        /// </remarks>
        /// <exception cref="InvalidOperationException">Noding doesn't converge.</exception>
        public static IPolygon Node(IPolygon polygon, IPredicates predicates, out double tolerance, int maxIterations = 10,
            double minTolerance = 0.0, double wedgeTolerance = 0.0)
        {
            tolerance = 0.0;

            if (polygon.Segments.Count == 0 || !polygon.Points.All(IsFinite))
            {
                // Nothing to do, or invalid input (rejected by input validation later).
                return polygon;
            }

            double scale = GetScale(polygon);

            // Limit for the number of new vertices (to detect non-convergence early).
            int maxNewPoints = 50 * (polygon.Points.Count + polygon.Segments.Count) + 1000;

            var tolerances = new List<double>();

            for (double relative = DefaultRelativeTolerance; relative <= MaxRelativeTolerance; relative *= 16)
            {
                double t = Math.Max(relative * scale, minTolerance);

                if (tolerances.Count == 0 || t > tolerances[tolerances.Count - 1])
                {
                    tolerances.Add(t);
                }
            }

            foreach (var t in tolerances)
            {
                tolerance = t;

                if (TryNode(polygon, predicates, tolerance, wedgeTolerance, maxIterations, maxNewPoints, out var result))
                {
                    return result;
                }
            }

            // Last resort: snap rounding to the largest tolerance (always converges in theory,
            // e.g. for many segments crossing at tiny angles, where the rounded intersection
            // points keep creating new intersections).
            var rounded = SnapRound(polygon, predicates, tolerance, 100, fallback: false);

            if (rounded != null)
            {
                return rounded;
            }

            throw new InvalidOperationException("Segment noding doesn't converge (too many nearly coincident segments). " +
                "Consider cleaning the input (e.g. snapping it to a grid).");
        }

        /// <summary>
        /// Snap rounding: round all vertices to the grid and split all segments at their
        /// intersections (rounded to the grid) and at vertices whose grid cell (hot pixel)
        /// they pass through.
        /// </summary>
        /// <param name="polygon">The input polygon (won't be modified).</param>
        /// <param name="predicates">The predicates used to detect crossings.</param>
        /// <param name="gridSize">The grid size, e.g. 0.01 for coordinates in meters which
        /// are written with two decimal places.</param>
        /// <param name="maxIterations">Maximum number of noding iterations.</param>
        /// <returns>The noded polygon with all vertices on the grid.</returns>
        /// <remarks>
        /// Since the rounded intersection points don't lie exactly on the segments, the
        /// segments are bent slightly (by at most half a grid cell). Afterwards, no two segments
        /// cross and every vertex has a distance of at least half a grid cell to every segment
        /// not ending in it (iterated snap rounding, Hobby 1999, Halperin and Packer 2002).
        ///
        /// If snap rounding doesn't converge, the result of <see cref="Node(IPolygon, IPredicates, int, double)"/>
        /// with the grid size as minimum tolerance is returned (not all vertices are on the grid then).
        /// </remarks>
        public static IPolygon SnapRound(IPolygon polygon, IPredicates predicates, double gridSize, int maxIterations = 100)
        {
            return SnapRound(polygon, predicates, gridSize, maxIterations, fallback: true);
        }

        /// <param name="fallback">If true, the result of <see cref="Node(IPolygon, IPredicates, int, double)"/>
        /// is returned if snap rounding doesn't converge, otherwise null.</param>
        private static IPolygon SnapRound(IPolygon polygon, IPredicates predicates, double gridSize, int maxIterations, bool fallback)
        {
            if (!(gridSize > 0.0) || double.IsInfinity(gridSize))
            {
                throw new ArgumentException("Grid size must be positive.", nameof(gridSize));
            }

            if (!polygon.Points.All(IsFinite))
            {
                return polygon;
            }

            int maxNewPoints = 50 * (polygon.Points.Count + polygon.Segments.Count) + 1000;

            // Vertices by their (rounded) location.
            var index = new Dictionary<(double, double), Vertex>();
            var vertices = new List<Vertex>();

            // Input vertices which were moved or merged.
            var replaced = new Dictionary<Vertex, Vertex>(new ReferenceComparer());

            var points = new List<Vertex>(polygon.Points.Count);
            var newPoints = new List<Vertex>();

            Vertex Find(double x, double y, out (double, double) key)
            {
                key = (GridSnapper.Round(x, gridSize), GridSnapper.Round(y, gridSize));
                return index.TryGetValue(key, out var v) ? v : null;
            }

            Vertex Snap(Vertex p)
            {
                if (replaced.TryGetValue(p, out var r))
                {
                    return r;
                }

                var v = Find(p.x, p.y, out var key);

                if (v == null)
                {
                    if (key.Item1 == p.x && key.Item2 == p.y)
                    {
                        v = p;
                    }
                    else
                    {
                        v = new Vertex(key.Item1, key.Item2, p.label) { id = p.id };
#if USE_ATTRIBS
                        v.attributes = p.attributes;
#endif
                        replaced[p] = v;
                    }

                    index.Add(key, v);
                    vertices.Add(v);
                    points.Add(v);
                }
                else if (!ReferenceEquals(v, p))
                {
                    replaced[p] = v;

                    if (v.label == 0 && p.label != 0)
                    {
                        v.label = p.label;
                    }
                }

                return v;
            }

            foreach (var p in polygon.Points)
            {
                Snap(p);
            }

            var segments = new List<ISegment>(polygon.Segments.Count);
            var vanished = new List<(Vertex, Vertex)>();

            foreach (var s in polygon.Segments)
            {
                var a = s.GetVertex(0);
                var b = s.GetVertex(1);
                var ra = Snap(a);
                var rb = Snap(b);

                if (SameLocation(ra, rb))
                {
                    vanished.Add((a, b));
                }
                else if (ReferenceEquals(ra, a) && ReferenceEquals(rb, b))
                {
                    segments.Add(s);
                }
                else
                {
                    segments.Add(new Segment(ra, rb, s.Label));
                }
            }

            bool changed = replaced.Count > 0 || vanished.Count > 0;
            bool converged = false;

            var retired = new HashSet<(Vertex, Vertex)>(new EdgeComparer());

            double half = 0.5 * gridSize;

            for (int iteration = 0; iteration < maxIterations && newPoints.Count <= maxNewPoints; iteration++)
            {
                var splits = new Dictionary<int, List<(double, Vertex)>>();

                // First route the segments through the hot pixels of the existing vertices: this
                // merges nearly coincident segments without creating new vertices. (Computing
                // new intersections first made the number of vertices explode for segments
                // crossing at tiny angles, whose rounded intersections create new crossings.)
                AddHotPixels(segments, vertices, half, retired, splits);

                foreach (var (i, j) in splits.Count > 0 ? new List<(int, int)>() : FindCrossings(segments, predicates))
                {
                    var a = segments[i].GetVertex(0);
                    var b = segments[i].GetVertex(1);
                    var c = segments[j].GetVertex(0);
                    var d = segments[j].GetVertex(1);

                    double abx = b.x - a.x, aby = b.y - a.y;
                    double cdx = d.x - c.x, cdy = d.y - c.y;

                    double denom = abx * cdy - aby * cdx;

                    if (denom == 0.0)
                    {
                        continue;
                    }

                    double ts = Math.Max(0.0, Math.Min(1.0, ((c.x - a.x) * cdy - (c.y - a.y) * cdx) / denom));

                    var v = Find(a.x + ts * abx, a.y + ts * aby, out var key);

                    if (v == null)
                    {
                        v = new Vertex(key.Item1, key.Item2, segments[i].Label);
                        index.Add(key, v);
                        vertices.Add(v);
                        newPoints.Add(v);
                    }

                    if (!SameLocation(v, a) && !SameLocation(v, b))
                    {
                        AddSplit(splits, i, Parameter(a, b, v), v);
                    }

                    if (!SameLocation(v, c) && !SameLocation(v, d))
                    {
                        AddSplit(splits, j, Parameter(c, d, v), v);
                    }
                }

                if (splits.Count == 0)
                {
                    converged = true;
                    break;
                }

                foreach (var i in splits.Keys)
                {
                    retired.Add((segments[i].GetVertex(0), segments[i].GetVertex(1)));
                }

                segments = Split(segments, splits);
                changed = true;
            }

            if (!converged)
            {
                return fallback ? Node(polygon, predicates, minTolerance: gridSize) : null;
            }

            if (!changed)
            {
                return polygon;
            }

            var noded = new Polygon(points.Count + newPoints.Count)
            {
                HasPointMarkers = polygon.HasPointMarkers,
                HasSegmentMarkers = polygon.HasSegmentMarkers
            };

            int id = polygon.Points.Count == 0 ? 0 : polygon.Points.Max(p => p.id);

            foreach (var v in newPoints)
            {
                v.id = ++id;
            }

            noded.Points.AddRange(points);
            noded.Points.AddRange(newPoints);
            noded.Segments.AddRange(segments);

            bool Collapsed(Point p) => vanished.Any(e => DistanceToSegment(p, e.Item1, e.Item2) <= gridSize);

            noded.Holes.AddRange(polygon.Holes.Where(h => !Collapsed(h)));
            noded.Regions.AddRange(polygon.Regions.Where(r => !Collapsed(r.Point)));

            return noded;
        }

        /// <summary>
        /// Find vertices whose grid cell (square of size 2 * half centered at the vertex) is
        /// intersected by a segment not ending in the vertex.
        /// </summary>
        private static void AddHotPixels(List<ISegment> segments, ICollection<Vertex> vertices,
            double half, HashSet<(Vertex, Vertex)> retired, Dictionary<int, List<(double, Vertex)>> splits)
        {
            var grid = new VertexGrid(vertices);

            for (int i = 0; i < segments.Count; i++)
            {
                var a = segments[i].GetVertex(0);
                var b = segments[i].GetVertex(1);

                double dx = b.x - a.x, dy = b.y - a.y;

                // A square with center v and half size h intersects the line through a and b,
                // if the distance of v to the line, measured along the axes, is at most h.
                double reach = half * (Math.Abs(dx) + Math.Abs(dy));

                foreach (var v in grid.Query(
                    Math.Min(a.x, b.x) - half, Math.Min(a.y, b.y) - half,
                    Math.Max(a.x, b.x) + half, Math.Max(a.y, b.y) + half))
                {
                    if (SameLocation(v, a) || SameLocation(v, b))
                    {
                        continue;
                    }

                    double t = Parameter(a, b, v);

                    if (t > 0.0 && t < 1.0 && Math.Abs(dx * (v.y - a.y) - dy * (v.x - a.x)) <= reach &&
                        !retired.Contains((a, v)) && !retired.Contains((v, b)))
                    {
                        AddSplit(splits, i, t, v);
                    }
                }
            }
        }

        private static bool TryNode(IPolygon polygon, IPredicates predicates, double tolerance, double wedgeTolerance,
            int maxIterations, int maxNewPoints, out IPolygon result)
        {
            result = polygon;

            var newPoints = new List<Vertex>();

            // Merge vertices closer than the tolerance. Afterwards, all vertices have a minimum
            // distance, which is required for the noding to terminate.
            var vertices = new VertexIndex(tolerance);
            var merged = MergeVertices(polygon.Points, vertices);

            var segments = new List<ISegment>(polygon.Segments.Count);

            // Segments which collapsed to a point.
            var vanished = new List<(Vertex, Vertex)>();

            foreach (var s in polygon.Segments)
            {
                var a = s.GetVertex(0);
                var b = s.GetVertex(1);

                if (!merged.TryGetValue(a, out var ma) & !merged.TryGetValue(b, out var mb))
                {
                    segments.Add(s);
                    continue;
                }

                ma = ma ?? a;
                mb = mb ?? b;

                // Segments shorter than the tolerance vanish.
                if (!SameLocation(ma, mb))
                {
                    segments.Add(new Segment(ma, mb, s.Label));
                }
                else
                {
                    vanished.Add((a, b));
                }
            }

            bool changed = merged.Count > 0;
            bool converged = false;

            // Segments which have been split. Snapping must not create them again: for vertices
            // with distances close to the tolerance, snapping could otherwise go back and forth.
            var retired = new HashSet<(Vertex, Vertex)>(new EdgeComparer());

            // The input segment each piece lies on (merging wedges must not move a segment by
            // more than the wedge tolerance, also not by merging a chain of wedges).
            var origin = wedgeTolerance > tolerance ? new Dictionary<ISegment, (Vertex, Vertex)>() : null;

            for (int iteration = 0; iteration < maxIterations && newPoints.Count <= maxNewPoints; iteration++)
            {
                var splits = new Dictionary<int, List<(double, Vertex)>>();

                AddIntersections(segments, FindCrossings(segments, predicates), vertices, newPoints, splits);

                if (splits.Count == 0)
                {
                    AddNearbyVertices(segments, vertices.All, tolerance, retired, splits);
                }

                if (splits.Count == 0 && wedgeTolerance > tolerance)
                {
                    AddWedgeVertices(segments, vertices.All, tolerance, wedgeTolerance, retired, splits, origin);
                }

                if (splits.Count == 0)
                {
                    converged = true;
                    break;
                }

                foreach (var i in splits.Keys)
                {
                    retired.Add((segments[i].GetVertex(0), segments[i].GetVertex(1)));
                }

                segments = Split(segments, splits, origin);
                changed = true;
            }

            if (!converged)
            {
                return false;
            }

            if (changed)
            {
                var noded = new Polygon(polygon.Points.Count + newPoints.Count)
                {
                    HasPointMarkers = polygon.HasPointMarkers,
                    HasSegmentMarkers = polygon.HasSegmentMarkers
                };

                // New vertices get IDs above the user's IDs (otherwise user IDs are no longer
                // unique and the mesh renumbers all vertices).
                int id = polygon.Points.Count == 0 ? 0 : polygon.Points.Max(p => p.id);

                foreach (var v in newPoints)
                {
                    v.id = ++id;
                }

                noded.Points.AddRange(polygon.Points.Where(p => !merged.ContainsKey(p)));
                noded.Points.AddRange(newPoints);
                noded.Segments.AddRange(segments);

                // A hole or region point inside a contour which collapsed (smaller than the
                // tolerance) would now lie in the surrounding region: drop it.
                bool Collapsed(Point p) => vanished.Any(e => DistanceToSegment(p, e.Item1, e.Item2) <= tolerance);

                noded.Holes.AddRange(polygon.Holes.Where(h => !Collapsed(h)));
                noded.Regions.AddRange(polygon.Regions.Where(r => !Collapsed(r.Point)));

                result = noded;
            }

            return true;
        }

        /// <summary>
        /// Add the vertices to the index. Vertices closer than the tolerance to a vertex
        /// already in the index are mapped to that vertex.
        /// </summary>
        private static Dictionary<Vertex, Vertex> MergeVertices(IEnumerable<Vertex> points, VertexIndex index)
        {
            var merged = new Dictionary<Vertex, Vertex>(new ReferenceComparer());

            foreach (var v in points)
            {
                var target = index.Find(v.x, v.y);

                if (target == null)
                {
                    index.Add(v);
                }
                else if (!SameLocation(target, v))
                {
                    // Exact duplicates are kept (handled by the triangulator).
                    merged[v] = target;

                    // Keep the label of the merged vertex.
                    if (target.label == 0 && v.label != 0)
                    {
                        target.label = v.label;
                    }
                }
            }

            return merged;
        }

        private static double GetScale(IPolygon polygon)
        {
            var bounds = new Rectangle();
            double max = 0.0;

            foreach (var v in polygon.Points)
            {
                bounds.Expand(v);
                max = Math.Max(max, Math.Max(Math.Abs(v.x), Math.Abs(v.y)));
            }

            return Math.Max(max, Math.Max(bounds.Width, bounds.Height));
        }

        /// <summary>
        /// Find all pairs of segments that cross in their interiors (sweep along x-axis).
        /// </summary>
        /// <remarks>
        /// If the segments overlap less along the y-axis (e.g. many long horizontal segments,
        /// which makes the x-sweep quadratic), the sweep runs along the y-axis. The pairs are
        /// returned in the order of the x-sweep in any case (the order matters for merging
        /// intersection points).
        /// </remarks>
        private static List<(int, int)> FindCrossings(List<ISegment> segments, IPredicates predicates)
        {
            int n = segments.Count;

            var order = new int[n];
            var xmin = new double[n];

            double extentX = 0.0, extentY = 0.0;

            for (int i = 0; i < n; i++)
            {
                var a = segments[i].GetVertex(0);
                var b = segments[i].GetVertex(1);

                order[i] = i;
                xmin[i] = Math.Min(a.x, b.x);

                extentX += Math.Abs(b.x - a.x);
                extentY += Math.Abs(b.y - a.y);
            }

            Array.Sort((double[])xmin.Clone(), order);

            if (extentY >= extentX)
            {
                return Sweep(segments, order, xmin, true, predicates);
            }

            var yorder = new int[n];
            var ymin = new double[n];

            for (int i = 0; i < n; i++)
            {
                yorder[i] = i;
                ymin[i] = Math.Min(segments[i].GetVertex(0).y, segments[i].GetVertex(1).y);
            }

            Array.Sort((double[])ymin.Clone(), yorder);

            var crossings = Sweep(segments, yorder, ymin, false, predicates);

            // Order of the x-sweep: by rank in the x-order of the first, then of the second segment.
            var rank = new int[n];

            for (int k = 0; k < n; k++)
            {
                rank[order[k]] = k;
            }

            for (int i = 0; i < crossings.Count; i++)
            {
                var (p, q) = crossings[i];

                if (rank[p] > rank[q])
                {
                    crossings[i] = (q, p);
                }
            }

            crossings.Sort((u, v) =>
            {
                int c = rank[u.Item1].CompareTo(rank[v.Item1]);
                return c != 0 ? c : rank[u.Item2].CompareTo(rank[v.Item2]);
            });

            return crossings;
        }

        /// <summary>
        /// Sweep along the x-axis (or y-axis) with the segments sorted by their minimum coordinate.
        /// </summary>
        private static List<(int, int)> Sweep(List<ISegment> segments, int[] order, double[] min,
            bool alongX, IPredicates predicates)
        {
            int n = segments.Count;

            var crossings = new List<(int, int)>();

            for (int k = 0; k < n; k++)
            {
                var s = segments[order[k]];
                var a = s.GetVertex(0);
                var b = s.GetVertex(1);

                double max = alongX ? Math.Max(a.x, b.x) : Math.Max(a.y, b.y);
                double lo = alongX ? Math.Min(a.y, b.y) : Math.Min(a.x, b.x);
                double hi = alongX ? Math.Max(a.y, b.y) : Math.Max(a.x, b.x);

                for (int m = k + 1; m < n && min[order[m]] <= max; m++)
                {
                    var t = segments[order[m]];
                    var c = t.GetVertex(0);
                    var d = t.GetVertex(1);

                    double tlo = alongX ? Math.Min(c.y, d.y) : Math.Min(c.x, d.x);
                    double thi = alongX ? Math.Max(c.y, d.y) : Math.Max(c.x, d.x);

                    if (thi < lo || tlo > hi)
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

                // The intersection may be merged with an endpoint of a segment. That's no split
                // of this segment (and it must not be retired: snapping a nearly collinear
                // vertex into it would be blocked, leaving a tiny angle between the segments).
                if (!SameLocation(v, a) && !SameLocation(v, b))
                {
                    AddSplit(splits, i, Parameter(a, b, v), v);
                }

                if (!SameLocation(v, c) && !SameLocation(v, d))
                {
                    AddSplit(splits, j, Parameter(c, d, v), v);
                }
            }
        }

        /// <summary>
        /// Find vertices which lie in the interior of a segment or closer than the given
        /// tolerance to the segment.
        /// </summary>
        private static void AddNearbyVertices(List<ISegment> segments, ICollection<Vertex> vertices,
            double tolerance, HashSet<(Vertex, Vertex)> retired, Dictionary<int, List<(double, Vertex)>> splits)
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

                    // Vertices within the tolerance of an endpoint are practically coincident
                    // with it. Snapping them would create micro segments, and for clusters of
                    // such vertices (e.g. tiny rings), splitting could go on forever.
                    if (Distance(v, a) <= tolerance || Distance(v, b) <= tolerance)
                    {
                        continue;
                    }

                    // Distance to the line and parameter along the segment.
                    double distance = Math.Abs(dx * (v.y - a.y) - dy * (v.x - a.x)) / length;
                    double t = ((v.x - a.x) * dx + (v.y - a.y) * dy) / (length * length);

                    if (distance <= tolerance && t > 0.0 && t < 1.0 &&
                        !retired.Contains((a, v)) && !retired.Contains((v, b)))
                    {
                        AddSplit(splits, i, t, v);
                    }
                }
            }
        }

        /// <summary>
        /// Find vertices closer than the wedge tolerance to a segment (a, b), which are connected
        /// to a or b by a segment forming a small angle with (a, b).
        /// </summary>
        private static void AddWedgeVertices(List<ISegment> segments, ICollection<Vertex> vertices,
            double tolerance, double wedgeTolerance, HashSet<(Vertex, Vertex)> retired,
            Dictionary<int, List<(double, Vertex)>> splits, Dictionary<ISegment, (Vertex, Vertex)> origin)
        {
            var neighbors = new Dictionary<Vertex, List<Vertex>>(new ReferenceComparer());

            void Connect(Vertex p, Vertex q)
            {
                if (!neighbors.TryGetValue(p, out var list))
                {
                    list = new List<Vertex>(4);
                    neighbors.Add(p, list);
                }

                list.Add(q);
            }

            foreach (var s in segments)
            {
                Connect(s.GetVertex(0), s.GetVertex(1));
                Connect(s.GetVertex(1), s.GetVertex(0));
            }

            double cosMax = Math.Cos(MaxWedgeAngle * Math.PI / 180.0);

            // Is v connected to the apex by a segment forming a small angle with (apex, other)?
            bool IsWedge(Vertex v, Vertex apex, Vertex other)
            {
                if (!neighbors.TryGetValue(v, out var list) || !list.Any(w => SameLocation(w, apex)))
                {
                    return false;
                }

                double ux = v.x - apex.x, uy = v.y - apex.y;
                double vx = other.x - apex.x, vy = other.y - apex.y;

                double dot = ux * vx + uy * vy;

                return dot > 0.0 && dot * dot >= cosMax * cosMax * (ux * ux + uy * uy) * (vx * vx + vy * vy);
            }

            var grid = new VertexGrid(vertices);

            for (int i = 0; i < segments.Count; i++)
            {
                var a = segments[i].GetVertex(0);
                var b = segments[i].GetVertex(1);

                var (oa, ob) = origin.TryGetValue(segments[i], out var o) ? o : (a, b);

                double dx = b.x - a.x, dy = b.y - a.y;
                double length = Math.Sqrt(dx * dx + dy * dy);

                if (length == 0.0)
                {
                    continue;
                }

                foreach (var v in grid.Query(
                    Math.Min(a.x, b.x) - wedgeTolerance, Math.Min(a.y, b.y) - wedgeTolerance,
                    Math.Max(a.x, b.x) + wedgeTolerance, Math.Max(a.y, b.y) + wedgeTolerance))
                {
                    if (SameLocation(v, a) || SameLocation(v, b) ||
                        Distance(v, a) <= tolerance || Distance(v, b) <= tolerance)
                    {
                        continue;
                    }

                    double distance = Math.Abs(dx * (v.y - a.y) - dy * (v.x - a.x)) / length;
                    double t = ((v.x - a.x) * dx + (v.y - a.y) * dy) / (length * length);

                    if (distance <= wedgeTolerance && t > 0.0 && t < 1.0 &&
                        DistanceToSegment(v, oa, ob) <= wedgeTolerance &&
                        (IsWedge(v, a, b) || IsWedge(v, b, a)) &&
                        !retired.Contains((a, v)) && !retired.Contains((v, b)))
                    {
                        AddSplit(splits, i, t, v);
                    }
                }
            }
        }

        private static List<ISegment> Split(List<ISegment> segments, Dictionary<int, List<(double, Vertex)>> splits,
            Dictionary<ISegment, (Vertex, Vertex)> origin = null)
        {
            var result = new List<ISegment>(segments.Count + 2 * splits.Count);

            // Endpoints (unordered) of all segments which are kept unchanged and of all new
            // segments, used to avoid duplicate new segments.
            // For new segments the Segment is stored (null for unchanged input segments).
            var existing = new Dictionary<(Vertex, Vertex), (int Label, Segment Created)>(new EdgeComparer());

            // Pieces inherit the input segment they lie on.
            void AddOrigin(ISegment piece, ISegment parent)
            {
                if (origin != null)
                {
                    origin[piece] = origin.TryGetValue(parent, out var o) ? o : (parent.GetVertex(0), parent.GetVertex(1));
                }
            }

            for (int i = 0; i < segments.Count; i++)
            {
                if (!splits.ContainsKey(i))
                {
                    existing[(segments[i].GetVertex(0), segments[i].GetVertex(1))] = (segments[i].Label, null);
                }
            }

            void AddNew(Vertex a, Vertex b, int label, ISegment parent)
            {
                if (!existing.TryGetValue((a, b), out var entry))
                {
                    var segment = new Segment(a, b, label);
                    existing.Add((a, b), (label, segment));
                    result.Add(segment);
                    AddOrigin(segment, parent);
                }
                else if (entry.Label == 0 && label != 0)
                {
                    // Coinciding pieces: a non-zero label wins (as in the mesher).
                    if (entry.Created != null)
                    {
                        entry.Created.Label = label;
                        existing[(a, b)] = (label, entry.Created);
                    }
                    else
                    {
                        // Unchanged input segment with label 0: add the piece, the mesher merges
                        // duplicate segments and keeps the non-zero label.
                        var segment = new Segment(a, b, label);
                        existing[(a, b)] = (label, segment);
                        result.Add(segment);
                        AddOrigin(segment, parent);
                    }
                }
            }

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
                        AddNew(prev, v, s.Label, s);
                        prev = v;
                    }
                }

                if (!SameLocation(prev, last))
                {
                    AddNew(prev, last, s.Label, s);
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

        private static double DistanceToSegment(Point p, Vertex a, Vertex b)
        {
            double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
            double t = l2 == 0.0 ? 0.0 : Math.Max(0.0, Math.Min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2));
            double ex = a.x + t * dx - p.x, ey = a.y + t * dy - p.y;

            return Math.Sqrt(ex * ex + ey * ey);
        }

        private static bool SameLocation(Vertex a, Vertex b)
        {
            return a.x == b.x && a.y == b.y;
        }

        private static bool IsFinite(Vertex v)
        {
            return !double.IsNaN(v.x) && !double.IsInfinity(v.x) && !double.IsNaN(v.y) && !double.IsInfinity(v.y);
        }

        private static double Distance(Vertex a, Vertex b)
        {
            double dx = a.x - b.x, dy = a.y - b.y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private class ReferenceComparer : IEqualityComparer<Vertex>
        {
            public bool Equals(Vertex a, Vertex b) => ReferenceEquals(a, b);

            public int GetHashCode(Vertex v) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(v);
        }

        /// <summary>
        /// Compares edges by their endpoint locations (ignoring direction).
        /// </summary>
        private class EdgeComparer : IEqualityComparer<(Vertex, Vertex)>
        {
            public bool Equals((Vertex, Vertex) e, (Vertex, Vertex) f)
            {
                return (SameLocation(e.Item1, f.Item1) && SameLocation(e.Item2, f.Item2)) ||
                       (SameLocation(e.Item1, f.Item2) && SameLocation(e.Item2, f.Item1));
            }

            public int GetHashCode((Vertex, Vertex) e)
            {
                // Symmetric in the two endpoints.
                return Hash(e.Item1) ^ Hash(e.Item2);
            }

            private static int Hash(Vertex v)
            {
                return v.x.GetHashCode() * 31 + v.y.GetHashCode();
            }
        }

        /// <summary>
        /// Spatial hash to find a vertex within the given tolerance.
        /// </summary>
        private class VertexIndex
        {
            readonly Dictionary<(long, long), List<Vertex>> cells = new Dictionary<(long, long), List<Vertex>>();
            readonly double tolerance, size;

            public List<Vertex> All { get; } = new List<Vertex>();

            public VertexIndex(double tolerance)
            {
                this.tolerance = tolerance;
                this.size = tolerance > 0.0 ? tolerance : 1.0;
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
