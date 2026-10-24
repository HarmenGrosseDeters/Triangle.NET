// -----------------------------------------------------------------------
// <copyright file="Contour.cs" company="">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Geometry
{
    using System;
    using System.Collections.Generic;
    using TriangleNet.Tools;

    /// <summary>
    /// Represents a contour of a polygon (outer boundary or internal holes).
    /// </summary>
    public class Contour
    {
        int marker;

        bool convex;

        /// <summary>
        /// Gets or sets the list of points making up the contour.
        /// </summary>
        public List<Vertex> Points { get; set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="Contour" /> class.
        /// </summary>
        /// <param name="points">The points that make up the contour.</param>
        public Contour(IEnumerable<Vertex> points)
            : this(points, 0, false)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Contour" /> class.
        /// </summary>
        /// <param name="points">The points that make up the contour.</param>
        /// <param name="marker">Contour marker.</param>
        public Contour(IEnumerable<Vertex> points, int marker)
            : this(points, marker, false)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Contour" /> class.
        /// </summary>
        /// <param name="points">The points that make up the contour.</param>
        /// <param name="marker">Contour marker.</param>
        /// <param name="convex">The hole is convex.</param>
        public Contour(IEnumerable<Vertex> points, int marker, bool convex)
        {
            AddPoints(points);

            this.marker = marker;
            this.convex = convex;
        }

        /// <summary>
        /// Gets the list of segments of the contour.
        /// </summary>
        /// <returns></returns>
        public List<ISegment> GetSegments()
        {
            var segments = new List<ISegment>();

            var p = Points;

            int count = p.Count - 1;

            for (int i = 0; i < count; i++)
            {
                // Add segments to polygon.
                segments.Add(new Segment(p[i], p[i + 1], marker));
            }

            // Close the contour.
            segments.Add(new Segment(p[count], p[0], marker));

            return segments;
        }

        /// <summary>
        /// Try to find a point inside the contour.
        /// </summary>
        /// <param name="limit">The number of iterations on each segment (default = 5).</param>
        /// <param name="eps">Not used anymore (all directions are tried).</param>
        /// <returns>Point inside the contour</returns>
        /// <exception cref="InvalidOperationException">Throws if no point could be found.</exception>
        /// <remarks>
        /// For each corner (index i) of the contour, the 3 points with indices i-1, i and i+1
        /// are considered and a search on the bisecting line and on the line perpendicular to
        /// the neighbors is started, in both directions. A given number of points will be tested
        /// (limit), while the distance to the corner is halved in each iteration. A point must be
        /// strictly inside (exact predicates) and is accepted, if its distance to the boundary
        /// is at least 10% of its distance to the corner; otherwise the point with the largest
        /// distance to the boundary is returned.
        /// </remarks>
        public Point FindInteriorPoint(int limit = 5, double eps = 2e-5)
        {
            if (convex)
            {
                var p = Points;

                int count = p.Count;

                var point = new Point(0.0, 0.0);

                for (int i = 0; i < count; i++)
                {
                    point.x += p[i].x;
                    point.y += p[i].y;
                }

                // If the contour is convex, use its centroid.
                point.x /= count;
                point.y /= count;

                return point;
            }

            return FindPointInPolygon(this.Points, limit, eps);
        }

        private void AddPoints(IEnumerable<Vertex> points)
        {
            Points = new List<Vertex>(points);

            int count = Points.Count - 1;

            // Check if first vertex equals last vertex.
            if (Points[0] == Points[count])
            {
                Points.RemoveAt(count);
            }
        }

        #region Helper methods

        private static Point FindPointInPolygon(List<Vertex> contour, int limit, double eps)
        {
            // Candidates are searched near the corners: in the direction of the angle bisector
            // and perpendicular to the line through the neighbors (both ways), at decreasing
            // distances. A candidate must lie strictly inside (exact predicates) and should keep
            // a distance to the boundary that is not tiny compared to the search distance
            // (otherwise rounding, or snapping of the contour while meshing, could move it to the
            // other side). If no candidate meets that, the one with the largest clearance is used.
            int length = contour.Count;

            if (length < 3)
            {
                throw new InvalidOperationException("A contour needs at least three points.");
            }

            var predicates = RobustPredicates.Default;

            var test = new Point();
            Point best = null;
            double bestClearance = 0.0;

            Point a = contour[length - 1], b, c;

            for (int i = 0; i < length; i++)
            {
                b = contour[i];
                c = contour[(i + 1) % length];

                for (int direction = 0; direction < 2; direction++)
                {
                    double dx, dy;

                    if (direction == 0)
                    {
                        // Direction [corner point -> midpoint(a-c)].
                        dx = (a.x + c.x) / 2 - b.x;
                        dy = (a.y + c.y) / 2 - b.y;
                    }
                    else
                    {
                        // Perpendicular to a-c.
                        dx = (c.y - a.y) / 2;
                        dy = (a.x - c.x) / 2;
                    }

                    double h = 1.0;

                    for (int j = 0; j <= limit; j++, h /= 2)
                    {
                        for (int sign = -1; sign <= 1; sign += 2)
                        {
                            test.x = b.x + sign * dx * h;
                            test.y = b.y + sign * dy * h;

                            if (!IsStrictlyInside(test, contour, predicates))
                            {
                                continue;
                            }

                            double clearance = DistanceToBoundary(test, contour);
                            double offset = h * Math.Sqrt(dx * dx + dy * dy);

                            if (clearance >= 0.1 * offset)
                            {
                                return new Point(test.x, test.y);
                            }

                            if (clearance > bestClearance)
                            {
                                bestClearance = clearance;
                                best = new Point(test.x, test.y);
                            }
                        }
                    }
                }

                a = b;
            }

            if (best == null)
            {
                throw new InvalidOperationException("Could not find a point inside the contour (degenerate or self-intersecting contour?).");
            }

            return best;
        }

        /// <summary>
        /// Exact point in polygon test (winding number). Points on the boundary are not inside.
        /// </summary>
        private static bool IsStrictlyInside(Point p, List<Vertex> poly, IPredicates predicates)
        {
            int winding = 0;
            int count = poly.Count;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                Point q0 = poly[j], q1 = poly[i];

                double orient = predicates.CounterClockwise(q0, q1, p);

                if (orient == 0.0 &&
                    Math.Min(q0.x, q1.x) <= p.x && p.x <= Math.Max(q0.x, q1.x) &&
                    Math.Min(q0.y, q1.y) <= p.y && p.y <= Math.Max(q0.y, q1.y))
                {
                    // On the boundary.
                    return false;
                }

                if (q0.y <= p.y)
                {
                    if (q1.y > p.y && orient > 0.0) winding++;
                }
                else if (q1.y <= p.y && orient < 0.0)
                {
                    winding--;
                }
            }

            return winding != 0;
        }

        private static double DistanceToBoundary(Point p, List<Vertex> poly)
        {
            double min = double.MaxValue;
            int count = poly.Count;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                Point q0 = poly[j], q1 = poly[i];

                double dx = q1.x - q0.x, dy = q1.y - q0.y, l2 = dx * dx + dy * dy;
                double t = l2 == 0.0 ? 0.0 : Math.Max(0.0, Math.Min(1.0, ((p.x - q0.x) * dx + (p.y - q0.y) * dy) / l2));
                double ex = q0.x + t * dx - p.x, ey = q0.y + t * dy - p.y;

                min = Math.Min(min, ex * ex + ey * ey);
            }

            return Math.Sqrt(min);
        }

        #endregion
    }
}
