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
        public static string Run(CorpusCase c, double minAngle, int steinerPoints, double snapTolerance = 0.0)
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

            for (int i = 0; i < 4; i++)
            {
                poly.Add(new Segment(V(f[i]), V(f[(i + 1) % 4])));
            }

            double area = (f[1].Item1 - f[0].Item1) * (f[2].Item2 - f[1].Item2);
            double scale = Math.Max(mag + w, Math.Max(b.Width, b.Height) + 2 * w);

            Mesh mesh;

            try
            {
                var quality = minAngle > 0 ? new QualityOptions { MinimumAngle = minAngle, SteinerPoints = steinerPoints } : null;
                mesh = (Mesh)poly.Triangulate(new ConstraintOptions { SnapTolerance = snapTolerance }, quality);
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
            double tol = Math.Max(scale * 1e-11, 4 * noderTolerance);

            var subsegs = mesh.Segments.Select(s => ((Point)s.GetVertex(0), (Point)s.GetVertex(1))).ToList();
            int missing = poly.Segments.Count(s => !IsCovered(subsegs, s.GetVertex(0), s.GetVertex(1), tol));

            if (missing > 0)
            {
                return $"SEGMENTS_MISSING {missing}";
            }

            return $"OK triangles={mesh.Triangles.Count}";
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
