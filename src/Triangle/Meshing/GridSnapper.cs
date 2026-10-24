// -----------------------------------------------------------------------
// <copyright file="GridSnapper.cs">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing
{
    using System;
    using System.Collections.Generic;
    using TriangleNet.Geometry;
    using TriangleNet.Topology;

    /// <summary>
    /// Rounding of mesh vertices to a grid (e.g. the precision of the output format).
    /// </summary>
    /// <remarks>
    /// Rounding the coordinates of a mesh when writing it (e.g. to centimeters) can flip or
    /// flatten triangles near nearly coincident lines. Use <see cref="ConstraintOptions.GridSize"/>
    /// (snap rounding of the input with <see cref="SegmentNoder.SnapRound"/> and rounding of
    /// the Steiner points with <see cref="SnapVertices"/>) to get a mesh with all vertices on
    /// the grid instead.
    /// </remarks>
    public static class GridSnapper
    {
        /// <summary>
        /// Round a coordinate to the grid.
        /// </summary>
        /// <param name="x">The coordinate.</param>
        /// <param name="gridSize">The grid size.</param>
        /// <returns>The nearest grid coordinate.</returns>
        /// <remarks>
        /// For decimal grids (0.1, 0.01, ...) the result is the same as rounding to decimal
        /// places (<see cref="Math.Round(double, int)"/>), i.e. the double nearest to the
        /// decimal value, which isn't always the case for <c>Math.Round(x / g) * g</c>.
        /// </remarks>
        public static double Round(double x, double gridSize)
        {
            double k = 1.0 / gridSize;
            double n = Math.Round(k);

            if (gridSize < 1.0 && Math.Abs(k - n) <= 1e-9 * k)
            {
                return Math.Round(x * n) / n;
            }

            return Math.Round(x / gridSize) * gridSize;
        }

        /// <summary>
        /// Move all vertices of the mesh to the nearest grid point, if this doesn't flip or
        /// flatten a triangle (otherwise try the other corners of the grid cell).
        /// </summary>
        /// <param name="mesh">The mesh.</param>
        /// <param name="gridSize">The grid size.</param>
        /// <returns>The number of vertices which couldn't be moved to the grid.</returns>
        /// <remarks>
        /// Vertices on segments are moved, too (the segments bend slightly). The mesh may no
        /// longer be exactly (constrained) Delaunay, and the angles change slightly.
        /// </remarks>
        public static int SnapVertices(IMesh mesh, double gridSize)
        {
            var m = (Mesh)mesh;

            if (!(gridSize > 0.0) || double.IsInfinity(gridSize))
            {
                throw new ArgumentException("Grid size must be positive.", nameof(gridSize));
            }

            var predicates = m.predicates;

            // Vertices off the grid (usually few: Steiner points are already on the grid).
            var off = new Dictionary<int, Vertex>();

            foreach (var v in m.vertices.Values)
            {
                if (v.type != VertexType.UndeadVertex &&
                    (Round(v.x, gridSize) != v.x || Round(v.y, gridSize) != v.y))
                {
                    off.Add(v.hash, v);
                }
            }

            if (off.Count == 0)
            {
                return 0;
            }

            // The triangles around each of these vertices.
            var star = new Dictionary<int, List<Triangle>>(off.Count);

            foreach (var t in m.triangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    var v = t.vertices[i];

                    if (!off.ContainsKey(v.hash))
                    {
                        continue;
                    }

                    if (!star.TryGetValue(v.hash, out var list))
                    {
                        list = new List<Triangle>(6);
                        star.Add(v.hash, list);
                    }

                    list.Add(t);
                }
            }

            int failed = 0;

            foreach (var v in off.Values)
            {
                if (!star.TryGetValue(v.hash, out var triangles))
                {
                    // Not part of the mesh.
                    continue;
                }

                double rx = Round(v.x, gridSize);
                double ry = Round(v.y, gridSize);

                if (!TryMove(v, rx, ry, triangles, predicates) && !TryCellCorners(v, rx, ry, gridSize, triangles, predicates))
                {
                    failed++;
                }
            }

            return failed;
        }

        private static bool TryCellCorners(Vertex v, double rx, double ry, double gridSize,
            List<Triangle> triangles, IPredicates predicates)
        {
            // The other corners of the grid cell containing the vertex, nearest first.
            double ox = Round(v.x < rx ? rx - gridSize : rx + gridSize, gridSize);
            double oy = Round(v.y < ry ? ry - gridSize : ry + gridSize, gridSize);

            var candidates = new[] { (ox, ry), (rx, oy), (ox, oy) };

            Array.Sort(candidates, (p, q) => Distance2(v, p).CompareTo(Distance2(v, q)));

            foreach (var (x, y) in candidates)
            {
                if (TryMove(v, x, y, triangles, predicates))
                {
                    return true;
                }
            }

            return false;
        }

        private static double Distance2(Vertex v, (double x, double y) p)
        {
            double dx = p.x - v.x, dy = p.y - v.y;
            return dx * dx + dy * dy;
        }

        private static bool TryMove(Vertex v, double x, double y, List<Triangle> triangles, IPredicates predicates)
        {
            double x0 = v.x, y0 = v.y;

            v.x = x;
            v.y = y;

            // If all triangles of the star stay positively oriented, the new location lies in
            // the kernel of the star polygon, so it can't coincide with another vertex.
            foreach (var t in triangles)
            {
                if (predicates.CounterClockwise(t.vertices[0], t.vertices[1], t.vertices[2]) <= 0.0)
                {
                    v.x = x0;
                    v.y = y0;

                    return false;
                }
            }

            return true;
        }
    }
}
