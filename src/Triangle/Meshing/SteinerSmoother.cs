// -----------------------------------------------------------------------
// <copyright file="SteinerSmoother.cs">
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
    /// Moves free Steiner points to improve the mesh quality (optimal Delaunay triangulation
    /// smoothing).
    /// </summary>
    /// <remarks>
    /// Each free Steiner point (not on a segment) is moved to the area-weighted mean of the
    /// circumcenters of its triangles, the update of the optimal Delaunay triangulation
    /// (L. Chen, J. Xu, "Optimal Delaunay triangulations", J. Comput. Math. 22, 2004; also
    /// P. Alliez et al., "Variational tetrahedral meshing", 2005), or halfway there. On an
    /// output grid, the new position is rounded to the grid. The triangulation is made
    /// Delaunay again by edge flips (never across subsegments). The move is kept only if the
    /// changed triangles are better than before: fewer triangles violate the quality
    /// constraints, or the same number and the smallest angle is larger. Otherwise the flips
    /// are undone exactly. So no triangle gets worse than the worst triangle it replaces, and
    /// input vertices, segments and points on segments are not changed.
    /// </remarks>
    internal class SteinerSmoother
    {
        // Points with more neighbors are skipped (they don't occur in quality meshes).
        private const int MaxDegree = 64;

        // Flips per move (a move that needs more is rejected).
        private const int MaxFlips = 256;

        private readonly Mesh mesh;
        private readonly Behavior behavior;
        private readonly IPredicates predicates;
        private readonly double gridSize;

        private readonly List<Otri> star = new List<Otri>();
        private readonly List<Triangle> touched = new List<Triangle>();
        private readonly HashSet<Triangle> touchedSet = new HashSet<Triangle>();
        private readonly Stack<Otri> work = new Stack<Otri>();
        private readonly List<Otri> flips = new List<Otri>();
        private readonly List<Otri> handles = new List<Otri>();

        private readonly Vertex candidate = new Vertex();

        // Quality of the triangles before the move.
        private int oldBad;
        private double oldWorst;

        public SteinerSmoother(Mesh mesh, IPredicates predicates, double gridSize)
        {
            this.mesh = mesh;
            this.behavior = mesh.behavior;
            this.predicates = predicates;
            this.gridSize = gridSize;
        }

        /// <summary>
        /// Move each free Steiner point once (if that improves the mesh).
        /// </summary>
        /// <returns>The number of moved points.</returns>
        public int Apply(CancellationToken cancellationToken)
        {
            if (behavior.Exclude != null)
            {
                return 0;
            }

            int moved = 0;

            SteinerReducer.CollectFreeVertices(mesh, handles);

            foreach (var handle in handles)
            {
                cancellationToken.ThrowIfCancellationRequested();

                var tri = handle;

                if (!Otri.IsDead(tri.tri) && tri.Org().type == VertexType.FreeVertex && Move(tri))
                {
                    moved++;
                }
            }

            return moved;
        }

        /// <summary>
        /// Try to move the origin of the given triangle.
        /// </summary>
        private bool Move(Otri start)
        {
            if (!GatherStar(start))
            {
                return false;
            }

            Vertex v = start.Org();

            double x = v.x;
            double y = v.y;

            // Optimal Delaunay triangulation update: area-weighted mean of the circumcenters.
            double sx = 0.0, sy = 0.0, sa = 0.0;

            foreach (var t in star)
            {
                Vertex b = t.Dest();
                Vertex c = t.Apex();

                double bx = b.x - x, by = b.y - y;
                double cx = c.x - x, cy = c.y - y;

                double d = 2.0 * (bx * cy - by * cx);

                if (d <= 0.0)
                {
                    return false;
                }

                double b2 = bx * bx + by * by;
                double c2 = cx * cx + cy * cy;

                // Circumcenter relative to v, weighted with the area (d / 4).
                double area = 0.25 * d;

                sx += area * (cy * b2 - by * c2) / d;
                sy += area * (bx * c2 - cx * b2) / d;
                sa += area;
            }

            double tx = x + sx / sa;
            double ty = y + sy / sa;

            return TryMove(start, tx, ty, Better) || TryMove(start, 0.5 * (x + tx), 0.5 * (y + ty), Better);
        }

        /// <summary>
        /// Gather the triangles around the origin of the given triangle.
        /// </summary>
        private bool GatherStar(Otri start)
        {
            star.Clear();

            var tri = start;
            var sub = default(Osub);

            do
            {
                if (tri.index == 0 || star.Count == MaxDegree)
                {
                    return false;
                }

                tri.Pivot(ref sub);
                if (sub.seg.hash != Mesh.DUMMY)
                {
                    return false;
                }

                star.Add(tri);

                tri.Onext();
            }
            while (!tri.Equals(start));

            return star.Count >= 3;
        }

        /// <summary>
        /// Move the origin of the given triangle to the given position (rounded to the output
        /// grid) and restore the Delaunay property. The move is undone if the position is
        /// outside the kernel of the star, or if <paramref name="accept"/> returns false.
        /// </summary>
        /// <remarks>
        /// The triangles changed by the move are available as <see cref="Touched"/> when
        /// <paramref name="accept"/> is called.
        /// </remarks>
        public bool TryMove(Otri start, double tx, double ty, System.Func<bool> accept)
        {
            if (!GatherStar(start))
            {
                return false;
            }

            Vertex v = start.Org();

            if (gridSize > 0.0)
            {
                tx = GridSnapper.Round(tx, gridSize);
                ty = GridSnapper.Round(ty, gridSize);
            }

            if (tx == v.x && ty == v.y)
            {
                return false;
            }

            // The new position must lie in the kernel of the star polygon.
            candidate.x = tx;
            candidate.y = ty;

            foreach (var t in star)
            {
                if (predicates.CounterClockwise(candidate, t.Dest(), t.Apex()) <= 0.0)
                {
                    return false;
                }
            }

            touched.Clear();
            touchedSet.Clear();
            flips.Clear();
            work.Clear();

            oldBad = 0;
            oldWorst = 0.0;

            foreach (var t in star)
            {
                Touch(t.tri);
            }

            double x = v.x, y = v.y;

            v.x = tx;
            v.y = ty;

            bool accepted = MakeDelaunay() && accept();

            if (!accepted)
            {
                for (int i = flips.Count - 1; i >= 0; i--)
                {
                    var flip = flips[i];
                    mesh.Unflip(ref flip);
                }

                v.x = x;
                v.y = y;
            }

            return accepted;
        }

        /// <summary>
        /// Restore the Delaunay property with edge flips (Lawson), starting with the edges of
        /// the star of the moved point.
        /// </summary>
        private bool MakeDelaunay()
        {
            var top = default(Otri);
            var sub = default(Osub);

            foreach (var t in star)
            {
                var e = t;
                work.Push(e);
                e.Lnext();
                work.Push(e);
            }

            while (work.Count > 0)
            {
                var e = work.Pop();

                e.Pivot(ref sub);
                if (sub.seg.hash != Mesh.DUMMY)
                {
                    continue;
                }

                e.Sym(ref top);
                if (top.index == 0)
                {
                    continue;
                }

                Vertex a = e.Org();
                Vertex b = e.Dest();
                Vertex c = e.Apex();
                Vertex d = top.Apex();

                if (predicates.InCircle(a, b, c, d) <= 0.0 ||
                    predicates.CounterClockwise(c, d, b) <= 0.0 ||
                    predicates.CounterClockwise(d, c, a) <= 0.0)
                {
                    continue;
                }

                if (flips.Count == MaxFlips)
                {
                    return false;
                }

                Touch(e.tri);
                Touch(top.tri);

                // Triangles abc and bad become dca (handle d -> c) and cdb.
                mesh.Flip(ref e);
                flips.Add(e);

                var next = e;
                next.Lnext();
                work.Push(next);
                next.Lnext();
                work.Push(next);

                e.Sym(ref next);
                next.Lnext();
                work.Push(next);
                next.Lnext();
                work.Push(next);
            }

            return true;
        }

        /// <summary>
        /// Remember a triangle that is changed by the move, with its quality before the move.
        /// </summary>
        private void Touch(Triangle t)
        {
            if (touchedSet.Add(t))
            {
                touched.Add(t);

                Evaluate(t, ref oldBad, ref oldWorst);
            }
        }

        /// <summary>
        /// The triangles changed by the current move.
        /// </summary>
        public List<Triangle> Touched => touched;

        private bool Better()
        {
            Score(null, out int bad, out double worst);

            return bad < oldBad || (bad == oldBad && worst < oldWorst - 1e-12);
        }

        /// <summary>
        /// Check that the changed triangles (except the ones with the given vertex, which will
        /// be deleted) don't violate more constraints than before the move, and that no
        /// triangle is worse than the worst triangle before the move or the minimum angle.
        /// </summary>
        public bool NotWorse(Vertex deleted)
        {
            Score(deleted, out int bad, out double worst);

            return bad <= oldBad && worst <= System.Math.Max(oldWorst, behavior.goodAngle);
        }

        private void Score(Vertex skip, out int bad, out double worst)
        {
            bad = 0;
            worst = 0.0;

            var tri = default(Otri);

            foreach (var t in touched)
            {
                tri.tri = t;

                if (skip != null)
                {
                    tri.orient = 0;
                    if (tri.Org() == skip || tri.Dest() == skip || tri.Apex() == skip)
                    {
                        continue;
                    }
                }

                Evaluate(t, ref bad, ref worst);
            }
        }

        /// <summary>
        /// Count the triangle if it violates the quality constraints, and update the worst
        /// (square of the cosine of the) smallest angle.
        /// </summary>
        private void Evaluate(Triangle t, ref int bad, ref double worst)
        {
            var tri = new Otri { tri = t, orient = 0 };
            var sub = default(Osub);

            Vertex a = tri.Org();
            Vertex b = tri.Dest();
            Vertex c = tri.Apex();

            bool good = TriangleQuality.Check(behavior, a, b, c, t.area, out double cos2);

            if (good && behavior.UserTest != null)
            {
                double area = 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
                good = !behavior.UserTest(t, area);
            }

            // Subsegments must not be encroached.
            for (tri.orient = 0; good && tri.orient < 3; tri.orient++)
            {
                tri.Pivot(ref sub);

                if (sub.seg.hash != Mesh.DUMMY && TriangleQuality.Encroaches(behavior, tri.Org(), tri.Dest(), tri.Apex()))
                {
                    good = false;
                }
            }

            if (!good)
            {
                bad++;
            }

            if (cos2 > worst)
            {
                worst = cos2;
            }
        }
    }
}
