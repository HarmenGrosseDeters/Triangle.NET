// -----------------------------------------------------------------------
// <copyright file="TerrainSimplifier.cs">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Tools
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using TriangleNet.Geometry;
    using TriangleNet.Meshing;
    using TriangleNet.Topology;

    /// <summary>
    /// Simplification of terrain models (TIN): select a subset of the points of a dense
    /// terrain model (e.g. a 1 m grid) such that the triangulation of the subset approximates
    /// all points within a given vertical error.
    /// </summary>
    /// <remarks>
    /// Greedy insertion (Garland and Heckbert, "Fast polygonal approximation of terrains and
    /// height fields", 1995): starting with the corners of the convex hull (and the vertices of
    /// the break lines), the point with the largest error in each triangle is added, until all
    /// errors are within the bound. Points are selected where the terrain is curved, flat areas
    /// get large triangles.
    ///
    /// The selected points (together with the break lines) can then be meshed with quality
    /// constraints. Heights of Steiner points should be interpolated from the dense model.
    /// </remarks>
    public static class TerrainSimplifier
    {

        /// <summary>
        /// Select a subset of the points.
        /// </summary>
        /// <param name="points">The terrain points.</param>
        /// <param name="heights">The heights of the points.</param>
        /// <param name="maxError">The maximum vertical error.</param>
        /// <param name="constraints">Optional break lines (segments). The error is computed
        /// for the constrained triangulation, and points at segment vertices are always
        /// selected.</param>
        /// <returns>The indices of the selected points (sorted).</returns>
        public static List<int> Select(IList<Vertex> points, IList<double> heights, double maxError,
            IPolygon constraints = null)
        {
            if (points.Count != heights.Count)
            {
                throw new ArgumentException("The number of heights doesn't match the number of points.", nameof(heights));
            }

            if (!(maxError >= 0.0) || double.IsInfinity(maxError))
            {
                throw new ArgumentException("The maximum error must be non-negative.", nameof(maxError));
            }

            int n = points.Count;

            // Height of each location (the first point wins for duplicates).
            var index = new Dictionary<(double, double), int>(n);

            for (int i = 0; i < n; i++)
            {
                var key = (points[i].x, points[i].y);

                if (!index.ContainsKey(key))
                {
                    index.Add(key, i);
                }
            }

            var predicates = RobustPredicates.Default;

            var selected = new HashSet<int>(ConvexHullCorners(points, predicates));

            if (selected.Count < 3)
            {
                // All points collinear: nothing to simplify.
                return Enumerable.Range(0, n).ToList();
            }

            // Locations of constraint vertices without a point (interpolated lazily).
            var extra = new Dictionary<(double, double), double>();
            Func<double, double, double> dense = null;

            double Height(double x, double y)
            {
                if (index.TryGetValue((x, y), out int i))
                {
                    return heights[i];
                }

                if (!extra.TryGetValue((x, y), out double h))
                {
                    dense = dense ?? CreateInterpolation(points, heights);
                    h = dense(x, y);
                    extra.Add((x, y), h);
                }

                return h;
            }

            if (constraints != null)
            {
                foreach (var s in constraints.Segments)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        var v = s.GetVertex(k);

                        if (index.TryGetValue((v.x, v.y), out int i))
                        {
                            selected.Add(i);
                        }
                    }
                }
            }

            // Points in spatial order, which makes point location by walking fast.
            var order = SpatialOrder(points);

            var rank = new int[n];

            for (int k = 0; k < n; k++)
            {
                rank[order[k]] = k;
            }

            // The unselected points in each triangle of the mesh. After inserting points, only
            // the points in triangles which changed are located again.
            var buckets = new Dictionary<Triangle, Bucket>();
            var isSelected = new bool[n];

            foreach (int i in selected)
            {
                isSelected[i] = true;
            }

            var pending = order.Where(i => !isSelected[i]).ToList();

            // Heights of the mesh vertices by their hash (NaN = not known yet).
            var z = new List<double>();

            double Z(Vertex v)
            {
                while (z.Count <= v.hash)
                {
                    z.Add(double.NaN);
                }

                double h = z[v.hash];

                if (double.IsNaN(h))
                {
                    z[v.hash] = h = Height(v.x, v.y);
                }

                return h;
            }

            var mesh = Triangulate(points, selected, constraints);

            var tri = default(Otri);

            var changed = new HashSet<Triangle>();
            var created = new List<Bucket>();
            var queue = new BucketQueue();

            // Set if the mesh was built from scratch (no insertions since).
            bool fresh = true;

            // Set after the first check with a triangulation built from scratch: the few
            // remaining points are inserted at once (each check is expensive).
            bool polishing = false;

            while (true)
            {
                tri.tri = mesh.triangles.First();
                tri.orient = 0;

                foreach (int i in pending)
                {
                    var p = points[i];

                    var result = Locate(mesh, p, ref tri);

                    if (result == LocateResult.Outside || tri.tri.id == Mesh.DUMMY)
                    {
                        // On the boundary (or outside due to snapping of the break lines).
                        tri.Sym();

                        if (tri.tri.id == Mesh.DUMMY)
                        {
                            tri.tri = mesh.triangles.First();
                            continue;
                        }
                    }

                    double error;

                    if (result == LocateResult.OnVertex)
                    {
                        // Duplicate location: error with respect to the vertex height.
                        var v = tri.Org();
                        error = Math.Abs(Z(v) - heights[i]);
                    }
                    else
                    {
                        error = Math.Abs(Interpolate(tri.tri, p.x, p.y, Z) - heights[i]);
                    }

                    if (!buckets.TryGetValue(tri.tri, out var bucket))
                    {
                        bucket = new Bucket(tri.tri);
                        buckets.Add(tri.tri, bucket);
                        created.Add(bucket);
                    }

                    bucket.Points.Add(i);

                    if (error > bucket.Error)
                    {
                        bucket.Error = error;
                        bucket.Worst = i;
                    }
                }

                pending.Clear();

                foreach (var bucket in created)
                {
                    if (bucket.Error > maxError)
                    {
                        queue.Push(bucket);
                    }
                }

                created.Clear();

                double max = queue.Count > 0 ? queue.Peek().Error : 0.0;

                if (max <= maxError)
                {
                    if (fresh)
                    {
                        break;
                    }

                    // The Delaunay triangulation of grid points isn't unique (cocircular
                    // points), so the triangulation of the selected points built from scratch
                    // may differ from the incremental one: check all points again.
                    // Points in triangles which exist in both triangulations keep their errors.
                    mesh = Triangulate(points, selected, constraints);
                    z.Clear();
                    fresh = true;
                    polishing = true;

                    var unchanged = new Dictionary<TriangleKey, Bucket>(buckets.Count);

                    foreach (var bucket in buckets.Values)
                    {
                        unchanged[bucket.Key] = bucket;
                    }

                    buckets.Clear();

                    foreach (var t in mesh.triangles)
                    {
                        var key = new TriangleKey(t);

                        if (unchanged.TryGetValue(key, out var bucket))
                        {
                            unchanged.Remove(key);
                            bucket.Triangle = t;
                            buckets.Add(t, bucket);
                        }
                    }

                    foreach (var bucket in unchanged.Values)
                    {
                        queue.Remove(bucket);
                        pending.AddRange(bucket.Points.Where(i => !isSelected[i]));
                    }

                    pending.Sort((i, j) => rank[i].CompareTo(rank[j]));
                    continue;
                }

                fresh = false;

                // Insert the worst points first (closer to the result of strictly greedy
                // insertion, which selects fewer points than inserting all at once).
                double limit = polishing ? maxError : Math.Max(maxError, 0.5 * max);

                var insert = new List<(Triangle, int)>();

                while (queue.Count > 0 && queue.Peek().Error >= limit)
                {
                    var bucket = queue.Pop();
                    insert.Add((bucket.Triangle, bucket.Worst));
                }

                insert.Sort((u, v) => rank[u.Item2].CompareTo(rank[v.Item2]));

                // Triangles changed by the insertions (all of them are in the star of a new
                // vertex right after its insertion).
                changed.Clear();

                foreach (var (t, i) in insert)
                {
                    selected.Add(i);
                    isSelected[i] = true;

                    // The bucket triangle is the starting point of the search.
                    changed.Add(t);
                    tri.tri = t;
                    tri.orient = 0;

                    if (Insert(mesh, points[i], ref tri))
                    {
                        AddStar(tri, changed);
                    }
                }

                // Points of triangles which changed are located again.
                foreach (var t in changed)
                {
                    if (buckets.TryGetValue(t, out var bucket) &&
                        (Otri.IsDead(t) || isSelected[bucket.Worst] || !bucket.Key.Equals(new TriangleKey(t))))
                    {
                        buckets.Remove(t);
                        queue.Remove(bucket);
                        pending.AddRange(bucket.Points.Where(i => !isSelected[i]));
                    }
                }

                pending.Sort((i, j) => rank[i].CompareTo(rank[j]));
            }

            var list = selected.ToList();
            list.Sort();

            return list;
        }

        private class Bucket
        {
            public Bucket(Triangle t)
            {
                Triangle = t;
                Key = new TriangleKey(t);
            }

            public Triangle Triangle;
            public readonly TriangleKey Key;

            // Position in the queue (-1 = not queued).
            public int Position = -1;
            public readonly List<int> Points = new List<int>();
            public int Worst = -1;
            public double Error = -1.0;
        }

        /// <summary>
        /// Max-heap of buckets by their worst error.
        /// </summary>
        private class BucketQueue
        {
            readonly List<Bucket> heap = new List<Bucket>();

            public int Count => heap.Count;

            public Bucket Peek() => heap[0];

            public void Clear()
            {
                foreach (var b in heap)
                {
                    b.Position = -1;
                }

                heap.Clear();
            }

            public void Push(Bucket b)
            {
                b.Position = heap.Count;
                heap.Add(b);
                Up(b.Position);
            }

            public Bucket Pop()
            {
                var top = heap[0];
                RemoveAt(0);
                return top;
            }

            public void Remove(Bucket b)
            {
                if (b.Position >= 0)
                {
                    RemoveAt(b.Position);
                }
            }

            private void RemoveAt(int i)
            {
                var b = heap[i];
                int last = heap.Count - 1;

                if (i != last)
                {
                    Move(heap[last], i);
                    heap.RemoveAt(last);
                    Down(i);
                    Up(i);
                }
                else
                {
                    heap.RemoveAt(last);
                }

                b.Position = -1;
            }

            private void Move(Bucket b, int i)
            {
                heap[i] = b;
                b.Position = i;
            }

            private void Up(int i)
            {
                var b = heap[i];

                while (i > 0)
                {
                    int parent = (i - 1) / 2;

                    if (heap[parent].Error >= b.Error)
                    {
                        break;
                    }

                    Move(heap[parent], i);
                    i = parent;
                }

                Move(b, i);
            }

            private void Down(int i)
            {
                if (i >= heap.Count)
                {
                    return;
                }

                var b = heap[i];
                int n = heap.Count;

                while (true)
                {
                    int child = 2 * i + 1;

                    if (child >= n)
                    {
                        break;
                    }

                    if (child + 1 < n && heap[child + 1].Error > heap[child].Error)
                    {
                        child++;
                    }

                    if (heap[child].Error <= b.Error)
                    {
                        break;
                    }

                    Move(heap[child], i);
                    i = child;
                }

                Move(b, i);
            }
        }

        /// <summary>
        /// Identifies a triangle by the locations of its vertices (independent of the mesh).
        /// </summary>
        private readonly struct TriangleKey : IEquatable<TriangleKey>
        {
            readonly double x0, y0, x1, y1, x2, y2;

            public TriangleKey(Triangle t)
            {
                // Rotate the (counterclockwise) vertices such that the smallest comes first.
                var a = t.vertices[0];
                var b = t.vertices[1];
                var c = t.vertices[2];

                if (Less(b, a) && Less(b, c))
                {
                    (a, b, c) = (b, c, a);
                }
                else if (Less(c, a) && Less(c, b))
                {
                    (a, b, c) = (c, a, b);
                }

                x0 = a.x; y0 = a.y; x1 = b.x; y1 = b.y; x2 = c.x; y2 = c.y;
            }

            private static bool Less(Vertex p, Vertex q) => p.x < q.x || (p.x == q.x && p.y < q.y);

            public bool Equals(TriangleKey o) =>
                x0 == o.x0 && y0 == o.y0 && x1 == o.x1 && y1 == o.y1 && x2 == o.x2 && y2 == o.y2;

            public override bool Equals(object obj) => obj is TriangleKey o && Equals(o);

            public override int GetHashCode()
            {
                int hash = x0.GetHashCode();
                hash = hash * 31 + y0.GetHashCode();
                hash = hash * 31 + x1.GetHashCode();
                hash = hash * 31 + y1.GetHashCode();
                hash = hash * 31 + x2.GetHashCode();
                return hash * 31 + y2.GetHashCode();
            }
        }

        /// <summary>
        /// Point location by walking from the given triangle (fast for points in spatial order).
        /// </summary>
        private static LocateResult Locate(Mesh mesh, Point p, ref Otri tri)
        {
            var org = tri.Org();
            var dest = tri.Dest();

            if (org.x == p.x && org.y == p.y)
            {
                return LocateResult.OnVertex;
            }

            if (dest.x == p.x && dest.y == p.y)
            {
                tri.Lnext();
                return LocateResult.OnVertex;
            }

            double ahead = mesh.predicates.CounterClockwise(org, dest, p);

            if (ahead < 0.0)
            {
                tri.Sym();

                if (tri.tri.id == Mesh.DUMMY)
                {
                    tri.Sym();
                    return mesh.locator.Locate(p, ref tri);
                }
            }
            else if (ahead == 0.0 &&
                ((org.x < p.x) == (p.x < dest.x)) && ((org.y < p.y) == (p.y < dest.y)))
            {
                return LocateResult.OnEdge;
            }

            return mesh.locator.PreciseLocate(p, ref tri, false);
        }

        /// <summary>
        /// Insert a point into the (constrained) Delaunay triangulation.
        /// </summary>
        /// <returns>True, if the point was inserted ('tri' has the new vertex as origin then).</returns>
        private static bool Insert(Mesh mesh, Vertex p, ref Otri tri)
        {
            var v = new Vertex(p.x, p.y) { type = VertexType.FreeVertex };

            var result = Locate(mesh, v, ref tri);

            if (result == LocateResult.OnVertex)
            {
                return false;
            }

            if (tri.tri.id == Mesh.DUMMY)
            {
                tri.Sym();
            }

            var seg = default(Osub);

            if (result == LocateResult.OnEdge)
            {
                tri.Pivot(ref seg);

                if (seg.seg.hash == Mesh.DUMMY)
                {
                    seg = default(Osub);
                }
                else
                {
                    // On a break line: split it.
                    v.type = VertexType.SegmentVertex;
                }
            }

            if (mesh.InsertVertex(v, ref tri, ref seg, false, false) == InsertVertexResult.Successful)
            {
                v.hash = mesh.hash_vtx++;
                v.id = v.hash;
                mesh.vertices.Add(v.hash, v);

                return true;
            }

            return false;
        }

        /// <summary>
        /// Add the triangles around the origin of the given triangle.
        /// </summary>
        private static void AddStar(Otri start, HashSet<Triangle> star)
        {
            var t = start;

            // Counterclockwise until back at the start or at the boundary.
            do
            {
                star.Add(t.tri);
                t.Onext();
            }
            while (t.tri.id != Mesh.DUMMY && t.tri != start.tri);

            if (t.tri.id == Mesh.DUMMY)
            {
                // Boundary vertex: the other direction.
                t = start;

                while (true)
                {
                    t.Oprev();

                    if (t.tri.id == Mesh.DUMMY)
                    {
                        break;
                    }

                    star.Add(t.tri);
                }
            }
        }

        private static Mesh Triangulate(IList<Vertex> points, HashSet<int> selected, IPolygon constraints)
        {
            var poly = new Polygon(selected.Count);

            foreach (int i in selected)
            {
                poly.Add(new Vertex(points[i].x, points[i].y));
            }

            if (constraints != null)
            {
                var copies = new Dictionary<Vertex, Vertex>();

                Vertex Copy(Vertex v)
                {
                    if (!copies.TryGetValue(v, out var c))
                    {
                        c = new Vertex(v.x, v.y);
                        copies.Add(v, c);
                    }

                    return c;
                }

                foreach (var s in constraints.Segments)
                {
                    poly.Add(new Segment(Copy(s.GetVertex(0)), Copy(s.GetVertex(1))), true);
                }
            }

            // Keep the convex hull (otherwise triangles outside the break lines are removed).
            return (Mesh)new GenericMesher().Triangulate(poly, new ConstraintOptions { Convex = true });
        }

        /// <summary>
        /// Linear interpolation in the triangle.
        /// </summary>
        private static double Interpolate(Triangle t, double x, double y, Func<Vertex, double> height)
        {
            return Interpolate(t.vertices[0], t.vertices[1], t.vertices[2], x, y, height);
        }

        private static double Interpolate(Vertex a, Vertex b, Vertex c, double x, double y, Func<Vertex, double> height)
        {
            double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);

            double l1 = ((b.x - x) * (c.y - y) - (c.x - x) * (b.y - y)) / det;
            double l2 = ((c.x - x) * (a.y - y) - (a.x - x) * (c.y - y)) / det;
            double l3 = 1.0 - l1 - l2;

            return l1 * height(a) + l2 * height(b) + l3 * height(c);
        }

        /// <summary>
        /// Interpolation in the Delaunay triangulation of all points (for break line vertices
        /// which aren't terrain points).
        /// </summary>
        private static Func<double, double, double> CreateInterpolation(IList<Vertex> points, IList<double> heights)
        {
            var copies = new List<Vertex>(points.Count);
            var z = new Dictionary<(double, double), double>(points.Count);

            for (int i = 0; i < points.Count; i++)
            {
                copies.Add(new Vertex(points[i].x, points[i].y));

                if (!z.ContainsKey((points[i].x, points[i].y)))
                {
                    z.Add((points[i].x, points[i].y), heights[i]);
                }
            }

            var mesh = (Mesh)new GenericMesher().Triangulate(copies);

            return (x, y) =>
            {
                var tri = default(Otri);
                tri.tri = mesh.triangles.First();

                var result = mesh.locator.Locate(new Point(x, y), ref tri);

                if (result == LocateResult.Outside || tri.tri.id == Mesh.DUMMY)
                {
                    tri.Sym();
                }

                if (tri.tri.id == Mesh.DUMMY)
                {
                    // Outside of the terrain model: use the nearest point.
                    var v = copies.OrderBy(p => (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y)).First();
                    return z[(v.x, v.y)];
                }

                return Interpolate(tri.tri, x, y, v => z[(v.x, v.y)]);
            };
        }

        /// <summary>
        /// Indices of the corners of the convex hull (monotone chain, collinear points removed).
        /// </summary>
        private static List<int> ConvexHullCorners(IList<Vertex> points, IPredicates predicates)
        {
            var sorted = Enumerable.Range(0, points.Count)
                .OrderBy(i => points[i].x).ThenBy(i => points[i].y).ToList();

            var hull = new List<int>();

            // Lower and upper hull.
            for (int pass = 0; pass < 2; pass++)
            {
                int start = hull.Count;

                foreach (int i in sorted)
                {
                    while (hull.Count >= start + 2 &&
                        predicates.CounterClockwise(points[hull[hull.Count - 2]], points[hull[hull.Count - 1]], points[i]) <= 0.0)
                    {
                        hull.RemoveAt(hull.Count - 1);
                    }

                    hull.Add(i);
                }

                // The last point is the first point of the other chain.
                hull.RemoveAt(hull.Count - 1);

                sorted.Reverse();
            }

            // Remove duplicates (degenerate input).
            return hull.GroupBy(i => (points[i].x, points[i].y)).Select(g => g.First()).ToList();
        }

        /// <summary>
        /// Order the points along a Z-order curve.
        /// </summary>
        private static int[] SpatialOrder(IList<Vertex> points)
        {
            var bounds = new Rectangle();

            foreach (var p in points)
            {
                bounds.Expand(p);
            }

            double size = Math.Max(bounds.Width, bounds.Height);
            double scale = size > 0.0 ? 65535.0 / size : 0.0;

            var keys = new ulong[points.Count];
            var order = new int[points.Count];

            for (int i = 0; i < points.Count; i++)
            {
                uint x = (uint)((points[i].x - bounds.Left) * scale);
                uint y = (uint)((points[i].y - bounds.Bottom) * scale);

                keys[i] = Interleave(x) | (Interleave(y) << 1);
                order[i] = i;
            }

            Array.Sort(keys, order);

            return order;
        }

        private static ulong Interleave(uint v)
        {
            ulong x = v;
            x = (x | (x << 16)) & 0x0000FFFF0000FFFFUL;
            x = (x | (x << 8)) & 0x00FF00FF00FF00FFUL;
            x = (x | (x << 4)) & 0x0F0F0F0F0F0F0F0FUL;
            x = (x | (x << 2)) & 0x3333333333333333UL;
            x = (x | (x << 1)) & 0x5555555555555555UL;
            return x;
        }
    }
}
