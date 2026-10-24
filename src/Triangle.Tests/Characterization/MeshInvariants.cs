using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using TriangleNet.Geometry;
using TriangleNet.Topology;

namespace TriangleNet.Tests.Characterization
{
    /// <summary>
    /// Checks of properties every mesh must have, independent of how it was computed.
    /// </summary>
    public static class MeshInvariants
    {
        /// <summary>
        /// Returns a list of violated invariants (empty if the mesh is fine).
        /// </summary>
        /// <param name="mesh">The mesh.</param>
        /// <param name="input">The input polygon (after meshing).</param>
        /// <param name="c">The scenario.</param>
        /// <param name="area">The expected meshed area (NaN to skip the check).</param>
        public static List<string> Check(Mesh mesh, IPolygon input, MeshCases.Case c, double area)
        {
            var errors = new List<string>();
            var predicates = new RobustPredicates();

            if (mesh.Triangles.Count == 0) errors.Add("no triangles");

            if (!MeshValidator.IsConsistent(mesh)) errors.Add("inconsistent topology");

            int inverted = mesh.Triangles.Count(t => predicates.CounterClockwise(t.GetVertex(0), t.GetVertex(1), t.GetVertex(2)) <= 0);
            if (inverted > 0) errors.Add($"{inverted} flat or inverted triangles");

            double sum = mesh.Triangles.Sum(t => Area(t));
            if (!double.IsNaN(area) && Math.Abs(sum - area) > 1e-9 * Math.Abs(area))
            {
                errors.Add($"area {sum:R}, expected {area:R}");
            }

            // (Constrained) Delaunay: no vertex inside the circumcircle of a neighbor, unless
            // the shared edge is a segment (constrained) - for conforming meshes, never.
            int nonDelaunay = 0;

            // Conforming Delaunay is only guaranteed if the Steiner point limit wasn't reached.
            bool conforming = c.Conforming && mesh.Vertices.Count - input.Points.Count < MeshCases.SteinerLimit;

            foreach (var t in mesh.Triangles)
            {
                var tri = new Otri { tri = t };

                for (tri.orient = 0; tri.orient < 3; tri.orient++)
                {
                    var neighbor = default(Otri);
                    tri.Sym(ref neighbor);

                    if (neighbor.tri.id == Mesh.DUMMY || neighbor.tri.id < t.id) continue;

                    var seg = default(Osub);
                    tri.Pivot(ref seg);

                    if (seg.seg.hash != Mesh.DUMMY && !conforming) continue;

                    if (predicates.InCircle(tri.Org(), tri.Dest(), tri.Apex(), neighbor.Apex()) > 0 &&
                        !IsRoundoffArtifact(tri.Org(), tri.Dest(), tri.Apex(), neighbor.Apex()))
                    {
                        nonDelaunay++;
                    }
                }
            }

            if (nonDelaunay > 0) errors.Add($"{nonDelaunay} non-Delaunay edges");

            // Input segments are present in the mesh (as chains of subsegments).
            var noded = TriangleNet.Meshing.SegmentNoder.Node(input, predicates, out double noderTolerance, minTolerance: c.Snap);
            double tolerance = Math.Max(Scale(input) * 1e-11, 4 * noderTolerance);

            var subsegs = mesh.Segments.Select(s => (s.GetVertex(0), s.GetVertex(1))).ToList();
            int missing = input.Segments.Count(s => !IsCovered(subsegs, s.GetVertex(0), s.GetVertex(1), tolerance));
            if (missing > 0) errors.Add($"{missing} input segments missing");

            // Locked segments are not split by refinement: every piece of a locked segment
            // after noding (crossings are split, that's unavoidable) is a single subsegment.
            if (c.Locked != null)
            {
                var meshEdges = new HashSet<(double, double, double, double)>(mesh.Segments.Select(s => Key(s.GetVertex(0), s.GetVertex(1))));
                int split = noded.Segments.Where(c.Locked).Count(s => !meshEdges.Contains(Key(s.GetVertex(0), s.GetVertex(1))));

                if (split > 0) errors.Add($"{split} locked segments split");
            }

            // Quality.
            if (c.AngleGuarantee && mesh.Vertices.Count - input.Points.Count < MeshCases.SteinerLimit)
            {
                int bad = mesh.Triangles.Count(t => MinimumAngle(t) < c.MinAngle - 1e-6);
                if (bad > 0) errors.Add($"{bad} triangles below {c.MinAngle}°");
            }

            if (c.MaxArea > 0)
            {
                int large = mesh.Triangles.Count(t => Area(t) > c.MaxArea * (1 + 1e-9));
                if (large > 0) errors.Add($"{large} triangles larger than {c.MaxArea}");
            }

            return errors;
        }

        /// <summary>
        /// Fingerprint of the mesh geometry, independent of the order of triangles and vertices.
        /// </summary>
        public static string Fingerprint(Mesh mesh)
        {
            string P(Vertex v) => BitConverter.DoubleToInt64Bits(v.X).ToString("x16") + BitConverter.DoubleToInt64Bits(v.Y).ToString("x16");

            var triangles = mesh.Triangles.Select(t =>
            {
                var v = new[] { P(t.GetVertex(0)), P(t.GetVertex(1)), P(t.GetVertex(2)) };
                int k = Array.IndexOf(v, v.Min(StringComparer.Ordinal));
                return v[k] + v[(k + 1) % 3] + v[(k + 2) % 3] + ":" + t.Label;
            }).OrderBy(s => s, StringComparer.Ordinal);

            var segments = mesh.Segments.Select(s =>
            {
                string a = P(s.GetVertex(0)), b = P(s.GetVertex(1));
                return (string.CompareOrdinal(a, b) < 0 ? a + b : b + a) + ":" + s.Label;
            }).OrderBy(s => s, StringComparer.Ordinal);

            using var sha = SHA256.Create();
            var hash = sha.ComputeHash(Encoding.ASCII.GetBytes(string.Join("\n", triangles) + "\n#\n" + string.Join("\n", segments)));

            return $"V={mesh.Vertices.Count} T={mesh.Triangles.Count} S={mesh.Segments.Count} H={BitConverter.ToString(hash, 0, 8).Replace("-", "").ToLowerInvariant()}";
        }

        /// <summary>
        /// A vertex inserted on a segment is rounded to the nearest representable point, so it
        /// can lie a few ulps off the segment. The edges at such a vertex are not flipped (in
        /// exact arithmetic they'd be Delaunay), so the exact incircle test can report a
        /// violation. Detect this case: the apex, an edge vertex and the far vertex are
        /// collinear up to a few ulps.
        /// </summary>
        private static bool IsRoundoffArtifact(Vertex a, Vertex b, Vertex apex, Vertex far)
        {
            foreach (var x in new[] { a, b })
            {
                double dx = far.X - apex.X, dy = far.Y - apex.Y, len = Math.Sqrt(dx * dx + dy * dy);
                double deviation = Math.Abs((x.X - apex.X) * dy - (x.Y - apex.Y) * dx) / len;
                double ulp = 8 * Math.Max(Ulp(x.X), Ulp(x.Y));

                if (deviation <= ulp) return true;
            }

            return false;
        }

        private static double Ulp(double x) => Math.BitIncrement(Math.Abs(x)) - Math.Abs(x);

        public static double Area(ITriangle t)
        {
            Point p = t.GetVertex(0), q = t.GetVertex(1), r = t.GetVertex(2);
            return 0.5 * ((q.X - p.X) * (r.Y - p.Y) - (r.X - p.X) * (q.Y - p.Y));
        }

        public static double MinimumAngle(ITriangle t)
        {
            double min = 180;

            for (int i = 0; i < 3; i++)
            {
                Point a = t.GetVertex(i), b = t.GetVertex((i + 1) % 3), c = t.GetVertex((i + 2) % 3);
                double ux = b.X - a.X, uy = b.Y - a.Y, vx = c.X - a.X, vy = c.Y - a.Y;
                min = Math.Min(min, Math.Atan2(Math.Abs(ux * vy - uy * vx), ux * vx + uy * vy) * 180 / Math.PI);
            }

            return min;
        }

        private static (double, double, double, double) Key(Point a, Point b)
        {
            return a.X < b.X || (a.X == b.X && a.Y < b.Y) ? (a.X, a.Y, b.X, b.Y) : (b.X, b.Y, a.X, a.Y);
        }

        private static double Scale(IPolygon input)
        {
            var b = input.Bounds();
            return new[] { b.Left, b.Right, b.Top, b.Bottom }.Max(Math.Abs) + Math.Max(b.Width, b.Height);
        }

        private static bool IsCovered(List<(Vertex, Vertex)> subsegs, Point p0, Point p1, double tol)
        {
            double dx = p1.X - p0.X, dy = p1.Y - p0.Y, len = Math.Sqrt(dx * dx + dy * dy);

            if (len == 0) return true;

            var intervals = new List<(double, double)>();

            foreach (var (a, b) in subsegs)
            {
                if (Distance(a, p0, p1) > tol || Distance(b, p0, p1) > tol) continue;

                double ta = ((a.X - p0.X) * dx + (a.Y - p0.Y) * dy) / len;
                double tb = ((b.X - p0.X) * dx + (b.Y - p0.Y) * dy) / len;
                double s = Math.Max(0, Math.Min(ta, tb)), e = Math.Min(len, Math.Max(ta, tb));

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

        private static double Distance(Point v, Point p, Point q)
        {
            double dx = q.X - p.X, dy = q.Y - p.Y, l2 = dx * dx + dy * dy;
            double t = l2 == 0 ? 0 : Math.Max(0, Math.Min(1, ((v.X - p.X) * dx + (v.Y - p.Y) * dy) / l2));
            double ex = p.X + t * dx - v.X, ey = p.Y + t * dy - v.Y;
            return Math.Sqrt(ex * ex + ey * ey);
        }
    }
}
