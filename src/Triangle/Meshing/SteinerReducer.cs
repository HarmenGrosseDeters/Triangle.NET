// -----------------------------------------------------------------------
// <copyright file="SteinerReducer.cs">
// Triangle.NET code by Christian Woltering, http://triangle.codeplex.com/
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing
{
    using System.Collections.Generic;
    using System.Threading;
    using TriangleNet.Geometry;
    using TriangleNet.Topology;

    /// <summary>
    /// Removes redundant Steiner points after refinement.
    /// </summary>
    /// <remarks>
    /// Delaunay refinement inserts points greedily, so later points often make earlier ones
    /// unnecessary (cf. "Delaunay sifting", and the local search of Abrahamsen et al.,
    /// "Computing Non-Obtuse Triangulations with Few Steiner Points", SoCG 2025, which also
    /// removes Steiner points of a constrained Delaunay triangulation).
    ///
    /// A free Steiner point (not on a segment) is deleted if all triangles of the Delaunay
    /// triangulation of its cavity meet the quality constraints and don't encroach upon a
    /// subsegment. The cavity is triangulated the same way <see cref="Mesh.DeleteVertex"/>
    /// does it, so the check sees the triangles the deletion creates. Input vertices,
    /// segments and points on segments are not changed, and no coordinates are created
    /// (vertices on an output grid stay on the grid).
    /// </remarks>
    internal class SteinerReducer
    {
        // Larger cavities are skipped (they don't occur in quality meshes).
        private const int MaxDegree = 64;

        private readonly Mesh mesh;
        private readonly Behavior behavior;
        private readonly IPredicates predicates;

        // The cavity polygon (counterclockwise) and whether its edge i -> i + 1 is a subsegment.
        private readonly List<Vertex> polygon = new List<Vertex>();
        private readonly List<bool> constrained = new List<bool>();

        // Polygon indices of the vertices of the retriangulated cavity.
        private readonly List<int> chain = new List<int>();

        private double areaLimit;
        private bool tie;

        // Moves neighbors for collapses.
        private readonly SteinerSmoother smoother;
        private readonly List<Otri> neighbors = new List<Otri>();

        public SteinerReducer(Mesh mesh, IPredicates predicates, double gridSize)
        {
            this.mesh = mesh;
            this.behavior = mesh.behavior;
            this.predicates = predicates;

            smoother = behavior.Exclude == null ? new SteinerSmoother(mesh, predicates, gridSize) : null;
        }

        /// <summary>
        /// Remove redundant Steiner points.
        /// </summary>
        /// <returns>The number of removed points.</returns>
        public int Apply(CancellationToken cancellationToken)
        {
            if (behavior.UserTest != null)
            {
                return 0;
            }

            int removed = 0;

            var handles = new List<Otri>();

            HashSet<Vertex> candidates = null;

            // Deleting a point changes the cavities of its neighbors, so repeat for the
            // neighbors of deleted points until no point can be removed.
            do
            {
                CollectFreeVertices(mesh, handles, candidates);

                candidates = new HashSet<Vertex>();

                foreach (var handle in handles)
                {
                    cancellationToken.ThrowIfCancellationRequested();

                    var deltri = handle;

                    if (!Find(ref deltri))
                    {
                        continue;
                    }

                    if (Removable(deltri))
                    {
                        candidates.UnionWith(polygon);
                        mesh.DeleteVertex(ref deltri);
                        removed++;
                    }
                    else if (smoother != null && Collapse(deltri, candidates))
                    {
                        removed++;
                    }
                }
            }
            while (candidates.Count > 0);

            return removed;
        }

        /// <summary>
        /// Collect one triangle per free Steiner point (with the point as origin).
        /// </summary>
        internal static void CollectFreeVertices(Mesh mesh, List<Otri> handles, HashSet<Vertex> only = null)
        {
            handles.Clear();

            var seen = new HashSet<Vertex>();
            var tri = default(Otri);

            foreach (var t in mesh.triangles)
            {
                tri.tri = t;

                for (tri.orient = 0; tri.orient < 3; tri.orient++)
                {
                    var v = tri.Org();

                    if (v.type == VertexType.FreeVertex && (only == null || only.Contains(v)) && seen.Add(v))
                    {
                        handles.Add(tri);
                    }
                }
            }
        }

        /// <summary>
        /// Check that the handle still has a free Steiner point as origin (earlier deletions
        /// flip edges and free triangles).
        /// </summary>
        private static bool Find(ref Otri tri)
        {
            return !Otri.IsDead(tri.tri) && tri.Org().type == VertexType.FreeVertex;
        }

        /// <summary>
        /// Try to delete the origin of the given triangle after moving a neighbor (a free
        /// Steiner point) to the midpoint of their edge, which collapses the edge.
        /// </summary>
        /// <remarks>
        /// The move is kept only if the point can then be deleted, and the other triangles
        /// changed by the move don't violate more constraints than before (see
        /// <see cref="SteinerSmoother.NotWorse"/>).
        /// </remarks>
        private bool Collapse(Otri deltri, HashSet<Vertex> changed)
        {
            Vertex w = deltri.Org();

            var tri = deltri;
            var nb = default(Otri);

            neighbors.Clear();

            do
            {
                if (tri.tri.id == Mesh.DUMMY)
                {
                    return false;
                }

                if (tri.Dest().type == VertexType.FreeVertex)
                {
                    // The edge from the neighbor to w.
                    tri.Sym(ref nb);
                    neighbors.Add(nb);
                }

                tri.Onext();
            }
            while (!tri.Equals(deltri));

            var found = default(Otri);

            foreach (var start in neighbors)
            {
                Vertex v = start.Org();

                if (smoother.TryMove(start, 0.5 * (v.x + w.x), 0.5 * (v.y + w.y),
                    () => Find(smoother.Touched, w, ref found) && Removable(found) && smoother.NotWorse(w)))
                {
                    changed.UnionWith(polygon);
                    foreach (var t in smoother.Touched)
                    {
                        changed.Add(t.vertices[0]);
                        changed.Add(t.vertices[1]);
                        changed.Add(t.vertices[2]);
                    }

                    mesh.DeleteVertex(ref found);
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Find a triangle with the given origin.
        /// </summary>
        private static bool Find(List<Triangle> triangles, Vertex v, ref Otri tri)
        {
            foreach (var t in triangles)
            {
                tri.tri = t;

                for (tri.orient = 0; tri.orient < 3; tri.orient++)
                {
                    if (tri.Org() == v)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Check whether the origin of the given triangle can be deleted.
        /// </summary>
        private bool Removable(Otri deltri)
        {
            if (!GatherCavity(deltri))
            {
                return false;
            }

            // Triangulate the cavity: polygon vertices 1, 2, ..., n - 1, 0, base edge 0 -> 1.
            int n = polygon.Count;

            chain.Clear();
            for (int i = 1; i < n; i++) chain.Add(i);
            chain.Add(0);

            tie = false;

            if (!Triangulate(0, n - 1))
            {
                return false;
            }

            // In a cocircular configuration, the deletion may choose other diagonals. They
            // have the same angles opposite to each polygon edge (so the same minimum angle
            // and encroachments), but not the same maximum angle and areas.
            return !tie || (behavior.MaxAngle == 0.0 && !behavior.fixedArea && !behavior.VarArea);
        }

        /// <summary>
        /// Gather the polygon around the origin of the given triangle. Returns false if the
        /// point can't be deleted.
        /// </summary>
        private bool GatherCavity(Otri deltri)
        {
            polygon.Clear();
            constrained.Clear();

            var tri = deltri;
            var edge = default(Otri);
            var sub = default(Osub);

            Triangle first = deltri.tri;

            areaLimit = double.MaxValue;

            do
            {
                if (tri.tri.id == Mesh.DUMMY || polygon.Count == MaxDegree)
                {
                    return false;
                }

                // A segment through the point.
                tri.Pivot(ref sub);
                if (sub.seg.hash != Mesh.DUMMY)
                {
                    return false;
                }

                // All triangles of the cavity must belong to the same region.
                if (tri.tri.label != first.label)
                {
                    return false;
                }

                if (behavior.Exclude != null && behavior.Exclude(tri.tri))
                {
                    return false;
                }

                if (behavior.VarArea && tri.tri.area > 0.0 && tri.tri.area < areaLimit)
                {
                    areaLimit = tri.tri.area;
                }

                polygon.Add(tri.Dest());

                // The polygon edge opposite to the point.
                tri.Lnext(ref edge);
                edge.Pivot(ref sub);
                constrained.Add(sub.seg.hash != Mesh.DUMMY);

                tri.Onext();
            }
            while (!tri.Equals(deltri));

            return polygon.Count >= 3;
        }

        /// <summary>
        /// Triangulate the polygon chain[first], ..., chain[last] (counterclockwise, closed by
        /// the edge chain[last] -> chain[first]) like <see cref="Mesh.TriangulatePolygon"/>
        /// and check the triangles.
        /// </summary>
        private bool Triangulate(int first, int last)
        {
            if (last - first < 2)
            {
                return true;
            }

            Vertex a = polygon[chain[first]];
            Vertex c = polygon[chain[last]];

            // Find the vertex whose circle with the base edge contains no other vertex.
            int best = first + 1;

            for (int i = first + 2; i < last; i++)
            {
                double incircle = predicates.InCircle(c, a, polygon[chain[best]], polygon[chain[i]]);

                if (incircle > 0.0)
                {
                    best = i;
                }
                else if (incircle == 0.0)
                {
                    tie = true;
                }
            }

            if (!Good(first, best, last))
            {
                return false;
            }

            return Triangulate(first, best) && Triangulate(best, last);
        }

        /// <summary>
        /// Check the triangle chain[i], chain[j], chain[k] of the retriangulated cavity.
        /// </summary>
        private bool Good(int i, int j, int k)
        {
            Vertex torg = polygon[chain[i]];
            Vertex tdest = polygon[chain[j]];
            Vertex tapex = polygon[chain[k]];

            if (predicates.CounterClockwise(torg, tdest, tapex) <= 0.0)
            {
                return false;
            }

            // Polygon edges which are subsegments must not be encroached. The edges of the
            // triangle are polygon edges if their endpoints are adjacent in the chain.
            if ((j == i + 1 && Encroached(chain[i], chain[j], tapex)) ||
                (k == j + 1 && Encroached(chain[j], chain[k], torg)))
            {
                return false;
            }

            if (i == 0 && k == chain.Count - 1 && Encroached(chain[k], chain[i], tdest))
            {
                return false;
            }

            return TriangleQuality.Check(behavior, torg, tdest, tapex, areaLimit, out _);
        }

        /// <summary>
        /// Check whether the polygon edge from polygon[p] to polygon[q] is a subsegment that
        /// the given apex encroaches upon (see QualityMesher.CheckSeg4Encroach).
        /// </summary>
        private bool Encroached(int p, int q, Vertex apex)
        {
            int n = polygon.Count;

            // Polygon edge p -> p + 1 (counterclockwise) or q -> q + 1.
            bool isConstrained = (q == (p + 1) % n && constrained[p]) || (p == (q + 1) % n && constrained[q]);

            if (!isConstrained)
            {
                return false;
            }

            return TriangleQuality.Encroaches(behavior, polygon[p], polygon[q], apex);
        }
    }
}
