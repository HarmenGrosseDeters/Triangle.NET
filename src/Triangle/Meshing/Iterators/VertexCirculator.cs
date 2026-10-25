// -----------------------------------------------------------------------
// <copyright file="VertexCirculator.cs" company="">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing.Iterators
{
    using System.Collections.Generic;
    using TriangleNet.Geometry;
    using TriangleNet.Topology;

    /// <summary>
    /// Vertex circulator to enumerate the vertices connected to a mesh vertex.
    /// </summary>
    public class VertexCirculator
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="VertexCirculator" /> class.
        /// </summary>
        public VertexCirculator(Mesh mesh)
        {
            mesh.MakeVertexMap();
        }

        /// <summary>
        /// Enumerate all vertices adjacent to given vertex.
        /// </summary>
        /// <param name="vertex">The center vertex (has to be part of the mesh topology).</param>
        /// <returns></returns>
        public IEnumerable<Vertex> EnumerateVertices(Vertex vertex)
        {
            foreach (var item in BuildCache(vertex, true))
            {
                yield return item.Dest();
            }
        }

        /// <summary>
        /// Enumerate all triangles adjacent to given vertex.
        /// </summary>
        /// <param name="vertex">The center vertex (has to be part of the mesh topology).</param>
        /// <returns></returns>
        public IEnumerable<ITriangle> EnumerateTriangles(Vertex vertex)
        {
            foreach (var item in BuildCache(vertex, false))
            {
                yield return item.tri;
            }
        }

        private List<Otri> BuildCache(Vertex vertex, bool vertices)
        {
            // A new list for each call (enumerations may be nested).
            var cache = new List<Otri>();

            if (vertex.tri.tri.IsNull || vertex.type == VertexType.UndeadVertex)
            {
                // Not part of the mesh topology (e.g. a duplicate input vertex).
                return cache;
            }

            Otri init = vertex.tri;
            Otri next = default;
            Otri prev = default;

            init.Copy(ref next);

            // Move counter-clockwise around the vertex.
            while (next.index != 0)
            {
                cache.Add(next);

                next.Copy(ref prev);
                next.Onext();

                if (next.Equals(init))
                {
                    break;
                }
            }

            if (next.index == 0)
            {
                // We reached the boundary. To get all adjacent triangles, start
                // again at init triangle and now move clockwise.
                init.Copy(ref next);

                if (vertices)
                {
                    // Don't forget to add the vertex lying on the boundary.
                    prev.Lnext();
                    cache.Add(prev);
                }

                next.Oprev();

                while (next.index != 0)
                {
                    cache.Insert(0, next);

                    next.Oprev();

                    if (next.Equals(init))
                    {
                        break;
                    }
                }
            }

            return cache;
        }
    }
}
