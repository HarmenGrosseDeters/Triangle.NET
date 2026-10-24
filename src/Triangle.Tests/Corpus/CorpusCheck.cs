using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Corpus
{
    /// <summary>
    /// Triangulates a corpus case (all polylines as segments, enclosed by a frame) and
    /// validates the result.
    /// </summary>
    public static class CorpusCheck
    {
        /// <summary>
        /// Returns "OK ..." or a description of the failure.
        /// </summary>
        public static string Run(CorpusCase c, double minAngle, int steinerPoints, double snapTolerance = 0.0,
            double gridSize = 0.0, double outputGrid = 0.0, double wedgeTolerance = 0.0)
        {
            var poly = new Polygon();
            var map = new Dictionary<(double, double), Vertex>();

            Vertex V((double, double) p)
            {
                if (!map.TryGetValue(p, out var v))
                {
                    v = new Vertex(p.Item1, p.Item2);
                    map[p] = v;
                    poly.Add(v);
                }
                return v;
            }

            foreach (var line in c.Lines)
            {
                foreach (var p in line) V(p);

                for (int i = 0; i + 1 < line.Count; i++)
                {
                    if (line[i] != line[i + 1])
                    {
                        poly.Add(new Segment(V(line[i]), V(line[i + 1])));
                    }
                }
            }

            foreach (var p in c.Points) V(p);

            if (poly.Points.Count == 0)
            {
                return "SKIPPED (empty)";
            }

            // Frame enclosing everything.
            var b = poly.Bounds();
            double mag = new[] { b.Left, b.Right, b.Top, b.Bottom }.Max(Math.Abs);
            double w = Math.Max(Math.Max(b.Width, b.Height) * 0.1, mag * 1e-6 + 1e-6);
            var f = new[] { (b.Left - w, b.Bottom - w), (b.Right + w, b.Bottom - w), (b.Right + w, b.Top + w), (b.Left - w, b.Top + w) };

            if (gridSize > 0.0)
            {
                // The frame is snapped to the grid, too.
                f = f.Select(p => (GridSnapper.Round(p.Item1, gridSize), GridSnapper.Round(p.Item2, gridSize))).ToArray();
            }

            for (int i = 0; i < 4; i++)
            {
                poly.Add(new Segment(V(f[i]), V(f[(i + 1) % 4])));
            }

            double area = (f[1].Item1 - f[0].Item1) * (f[2].Item2 - f[1].Item2);
            double scale = Math.Max(mag + w, Math.Max(b.Width, b.Height) + 2 * w);

            Mesh mesh;
            var watch = System.Diagnostics.Stopwatch.StartNew();

            try
            {
                var quality = minAngle > 0 ? new QualityOptions { MinimumAngle = minAngle, SteinerPoints = steinerPoints } : null;
                mesh = (Mesh)poly.Triangulate(new ConstraintOptions { SnapTolerance = snapTolerance, GridSize = gridSize, WedgeTolerance = wedgeTolerance }, quality);
            }
            catch (ArgumentException) when (poly.Points.Any(p => !IsFinite(p.X) || !IsFinite(p.Y)))
            {
                return "SKIPPED (non-finite input rejected)";
            }
            catch (Exception e)
            {
                var ex = e.GetBaseException();
                return "EXCEPTION " + ex.GetType().Name + ": " + ex.Message;
            }

            watch.Stop();

            if (!MeshValidator.IsConsistent(mesh))
            {
                return "INCONSISTENT";
            }

            var predicates = new RobustPredicates();

            if (mesh.Triangles.Any(t => predicates.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)) <= 0.0))
            {
                return "INVERTED";
            }

            double sum = mesh.Triangles.Sum(t =>
            {
                var p = t.GetVertex(0); var q = t.GetVertex(1); var r = t.GetVertex(2);
                return 0.5 * ((q.X - p.X) * (r.Y - p.Y) - (r.X - p.X) * (q.Y - p.Y));
            });

            if (Math.Abs(sum - area) > 1e-9 * area)
            {
                return $"AREA {sum:R} != {area:R}";
            }

            // Every input segment must be covered by mesh subsegments (within the snap tolerance).
            SegmentNoder.Node(poly, predicates, out double noderTolerance, minTolerance: snapTolerance);
            double tol = Math.Max(Math.Max(scale * 1e-11, 4 * noderTolerance), gridSize + wedgeTolerance);

            var subsegs = mesh.Segments.Select(s => ((Point)s.GetVertex(0), (Point)s.GetVertex(1))).ToList();
            int missing = poly.Segments.Count(s => !IsCovered(subsegs, s.GetVertex(0), s.GetVertex(1), tol));

            if (missing > 0 && Environment.GetEnvironmentVariable("TRIANGLE_CORPUS_DEBUG") == "1")
            {
                foreach (var m in poly.Segments.Where(s => !IsCovered(subsegs, s.GetVertex(0), s.GetVertex(1), tol)))
                {
                    Console.WriteLine($"missing {m.GetVertex(0).X:R} {m.GetVertex(0).Y:R} -> {m.GetVertex(1).X:R} {m.GetVertex(1).Y:R} tol={tol}");
                }
            }

            if (missing > 0)
            {
                return $"SEGMENTS_MISSING {missing}";
            }

            // Quality statistics (no failure: small input angles can't be fixed, and the
            // Steiner point limit may stop the refinement).
            int bad = minAngle > 0 ? mesh.Triangles.Count(t => MinimumAngle(t) < minAngle - 0.01) : 0;
            bool limit = minAngle > 0 && mesh.Vertices.Count - poly.Points.Count >= steinerPoints;

            // Triangles which are flat or inverted after rounding the coordinates to the output grid.
            double og = gridSize > 0.0 ? gridSize : outputGrid;
            int flip = 0, badRounded = bad;

            if (og > 0.0)
            {
                Vertex R(Vertex v) => new Vertex(GridSnapper.Round(v.X, og), GridSnapper.Round(v.Y, og));

                flip = 0;
                badRounded = 0;

                foreach (var t in mesh.Triangles)
                {
                    Vertex p = R(t.GetVertex(0)), q = R(t.GetVertex(1)), r = R(t.GetVertex(2));

                    if (predicates.CounterClockwise(p, q, r) <= 0.0)
                    {
                        flip++;
                        badRounded++;
                    }
                    else if (minAngle > 0 && MinimumAngle(p, q, r) < minAngle - 0.01)
                    {
                        badRounded++;
                    }
                }
            }

            var (classAngle, classFloor, classStrip, classSegment, classInterior) = Classify(mesh, minAngle, gridSize);

            return $"OK triangles={mesh.Triangles.Count} ms={watch.ElapsedMilliseconds} bad={bad} limit={(limit ? 1 : 0)} flip={flip} badr={badRounded}" +
                $" ba={classAngle} bf={classFloor} bn={classStrip} bs={classSegment} bi={classInterior}";
        }

        /// <summary>
        /// Classify the bad triangles: at a small input angle (a vertex where two segments form an
        /// angle below the minimum angle), below the grid floor (shortest edge less than two grid
        /// cells), in a narrow strip between a segment and another segment closer than 5 cm,
        /// touching a segment, or in the interior.
        /// </summary>
        private static (int, int, int, int, int) Classify(Mesh mesh, double minAngle, double gridSize)
        {
            if (minAngle <= 0)
            {
                return (0, 0, 0, 0, 0);
            }

            const double strip = 0.05;

            // Directions of the segments at each vertex, and the segment edges.
            var directions = new Dictionary<int, List<double>>();
            var segments = new HashSet<(int, int)>();

            foreach (var s in mesh.Segments)
            {
                Vertex a = s.GetVertex(0), b = s.GetVertex(1);

                void Add(Vertex p, Vertex q)
                {
                    if (!directions.TryGetValue(p.ID, out var list))
                    {
                        directions[p.ID] = list = new List<double>();
                    }

                    list.Add(Math.Atan2(q.Y - p.Y, q.X - p.X));
                }

                Add(a, b);
                Add(b, a);
                segments.Add((Math.Min(a.ID, b.ID), Math.Max(a.ID, b.ID)));
            }

            var smallAngle = new HashSet<int>();

            foreach (var item in directions)
            {
                var d = item.Value;

                if (d.Count < 2) continue;

                d.Sort();

                for (int i = 0; i < d.Count; i++)
                {
                    double gap = (i + 1 < d.Count ? d[i + 1] - d[i] : d[0] + 2 * Math.PI - d[i]) * 180 / Math.PI;

                    if (gap < minAngle - 0.01)
                    {
                        smallAngle.Add(item.Key);
                        break;
                    }
                }
            }

            int angle = 0, floor = 0, narrow = 0, segment = 0, interior = 0;

            var segmentList = mesh.Segments.ToList();

            // Is a segment other than (p, q) (and its collinear continuations) closer than the
            // strip width to the point?
            bool InStrip(Point c, Vertex p, Vertex q)
            {
                double ux = q.X - p.X, uy = q.Y - p.Y;

                foreach (var s in segmentList)
                {
                    Vertex a = s.GetVertex(0), b = s.GetVertex(1);

                    if (Math.Max(Math.Min(a.X, b.X) - c.X, c.X - Math.Max(a.X, b.X)) > strip ||
                        Math.Max(Math.Min(a.Y, b.Y) - c.Y, c.Y - Math.Max(a.Y, b.Y)) > strip)
                    {
                        continue;
                    }

                    if (a.ID == p.ID || a.ID == q.ID || b.ID == p.ID || b.ID == q.ID)
                    {
                        double vx = b.X - a.X, vy = b.Y - a.Y;
                        double sin = Math.Abs(ux * vy - uy * vx) / Math.Sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));

                        if (sin < 0.2 || (a.ID == p.ID || a.ID == q.ID) && (b.ID == p.ID || b.ID == q.ID)) continue;
                    }

                    double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
                    double t = l2 == 0 ? 0 : Math.Max(0, Math.Min(1, ((c.X - a.X) * dx + (c.Y - a.Y) * dy) / l2));
                    double ex = a.X + t * dx - c.X, ey = a.Y + t * dy - c.Y;

                    if (ex * ex + ey * ey < strip * strip) return true;
                }

                return false;
            }

            foreach (var t in mesh.Triangles)
            {
                if (MinimumAngle(t) >= minAngle - 0.01) continue;

                double shortest = double.MaxValue;
                Vertex sp = null, sq = null;

                for (int i = 0; i < 3; i++)
                {
                    Vertex p = t.GetVertex(i), q = t.GetVertex((i + 1) % 3);
                    shortest = Math.Min(shortest, Math.Sqrt((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y)));

                    if (segments.Contains((Math.Min(p.ID, q.ID), Math.Max(p.ID, q.ID))))
                    {
                        sp = p;
                        sq = q;
                    }
                }

                var centroid = new Point((t.GetVertex(0).X + t.GetVertex(1).X + t.GetVertex(2).X) / 3,
                    (t.GetVertex(0).Y + t.GetVertex(1).Y + t.GetVertex(2).Y) / 3);

                if (Enumerable.Range(0, 3).Any(i => smallAngle.Contains(t.GetVertexID(i)))) angle++;
                else if (gridSize > 0 && shortest < 2 * gridSize) floor++;
                else if (sp != null && InStrip(centroid, sp, sq)) narrow++;
                else if (sp != null) segment++;
                else interior++;
            }

            return (angle, floor, narrow, segment, interior);
        }

        private static double MinimumAngle(Topology.Triangle t) =>
            MinimumAngle(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2));

        private static double MinimumAngle(Point p, Point q, Point r)
        {
            double min = 180;
            var v = new[] { p, q, r };

            for (int i = 0; i < 3; i++)
            {
                Point a = v[i], b = v[(i + 1) % 3], c = v[(i + 2) % 3];
                double ux = b.X - a.X, uy = b.Y - a.Y, vx = c.X - a.X, vy = c.Y - a.Y;
                double angle = Math.Atan2(Math.Abs(ux * vy - uy * vx), ux * vx + uy * vy) * 180 / Math.PI;
                min = Math.Min(min, angle);
            }

            return min;
        }

        private static bool IsCovered(List<(Point, Point)> subsegs, Point p0, Point p1, double tol)
        {
            double dx = p1.X - p0.X, dy = p1.Y - p0.Y, len = Math.Sqrt(dx * dx + dy * dy);

            // Union of the subsegments lying on p0-p1, projected onto p0-p1.
            var intervals = new List<(double, double)>();

            foreach (var (a, b) in subsegs)
            {
                if (DistanceToSegment(a, p0, p1) > tol || DistanceToSegment(b, p0, p1) > tol) continue;

                double ta = ((a.X - p0.X) * dx + (a.Y - p0.Y) * dy) / len;
                double tb = ((b.X - p0.X) * dx + (b.Y - p0.Y) * dy) / len;

                double s = Math.Max(0, Math.Min(ta, tb)), e = Math.Min(len, Math.Max(ta, tb));

                // Subsegments beyond an endpoint (within tolerance) don't cover anything.
                if (e > s) intervals.Add((s, e));
            }

            intervals.Sort();

            double covered = 0, end = double.NegativeInfinity;

            foreach (var (s, e) in intervals)
            {
                if (e <= end) continue;
                covered += e - Math.Max(s, end);
                end = e;
            }

            return Math.Abs(covered - len) <= tol * 10 + len * 1e-9;
        }

        private static bool IsFinite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);

        private static double DistanceToSegment(Point v, Point p, Point q)
        {
            double dx = q.X - p.X, dy = q.Y - p.Y, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.Max(0, Math.Min(1, ((v.X - p.X) * dx + (v.Y - p.Y) * dy) / l2));
            double ex = p.X + t * dx - v.X, ey = p.Y + t * dy - v.Y;
            return Math.Sqrt(ex * ex + ey * ey);
        }
    }
}
