// -----------------------------------------------------------------------
// <copyright file="Triangle.cs" company="">
// Triangle Copyright (c) 1993, 1995, 1997, 1998, 2002, 2005 Jonathan Richard Shewchuk
// Triangle.NET code by Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Topology
{
    using System;
    using System.Runtime.CompilerServices;
    using TriangleNet.Geometry;

    /// <summary>
    /// A triangle of a mesh: a handle to the triangle's data in the <see cref="TrianglePool"/>.
    /// </summary>
    /// <remarks>
    /// The vertices, neighbors, subsegments and attributes of all triangles of a mesh are stored
    /// in arrays of the pool (index-based storage, like delaunator or BurstTriangulator), not in
    /// an object per triangle. This saves most of the garbage collection work for large meshes.
    /// Two handles are equal, if they refer to the same triangle.
    /// </remarks>
    public readonly struct Triangle : ITriangle, IEquatable<Triangle>
    {
        internal readonly TrianglePool pool;

        // Position in the arrays of the pool (0 is the "outer space" triangle of the mesh).
        internal readonly int index;

        internal Triangle(TrianglePool pool, int index)
        {
            this.pool = pool;
            this.index = index;
        }

        // Hash for dictionary (the position in the pool, -1 for the outer space triangle).
        internal int hash => index - 1;

        // The ID is only used for mesh output.
        internal int id
        {
            get => pool.ids[index];
            set => pool.ids[index] = value;
        }

        internal int label
        {
            get => pool.labels[index];
            set => pool.labels[index] = value;
        }

        internal double area
        {
            get => pool.areas[index];
            set => pool.areas[index] = value;
        }

        internal bool infected
        {
            get => pool.infected[index];
            set => pool.infected[index] = value;
        }

        // Marks the triangle as visited in a search (see Mesh.NextMark).
        internal int mark
        {
            get => pool.marks[index];
            set => pool.marks[index] = value;
        }

        internal VertexList vertices => new VertexList(pool.vertices, 3 * index);
        internal NeighborList neighbors => new NeighborList(pool, 3 * index);
        internal SubsegList subsegs => new SubsegList(pool.subsegs, 3 * index);

        /// <summary>
        /// Set the neighbor opposite to the given corner.
        /// </summary>
        internal void SetNeighbor(int k, Otri ot)
        {
            pool.neighbors[3 * index + k] = (ot.tri.index << 2) | ot.orient;
        }

        /// <summary>
        /// Gets a value indicating whether the handle refers to no triangle.
        /// </summary>
        internal bool IsNull => pool == null;

        internal readonly struct VertexList
        {
            private readonly Vertex[] a;
            private readonly int offset;

            public VertexList(Vertex[] a, int offset) { this.a = a; this.offset = offset; }

            public ref Vertex this[int k]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref a[offset + k];
            }
        }

        internal readonly struct SubsegList
        {
            private readonly Osub[] a;
            private readonly int offset;

            public SubsegList(Osub[] a, int offset) { this.a = a; this.offset = offset; }

            public ref Osub this[int k]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref a[offset + k];
            }
        }

        internal readonly struct NeighborList
        {
            private readonly TrianglePool pool;
            private readonly int offset;

            public NeighborList(TrianglePool pool, int offset) { this.pool = pool; this.offset = offset; }

            public Otri this[int k]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    int code = pool.neighbors[offset + k];
                    return new Otri { tri = new Triangle(pool, code >> 2), orient = code & 3 };
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => pool.neighbors[offset + k] = (value.tri.index << 2) | value.orient;
            }
        }

        #region Public properties

        /// <summary>
        /// Gets or sets the triangle id.
        /// </summary>
        public int ID
        {
            get { return id; }
            set { id = value; }
        }

        /// <summary>
        /// Region ID the triangle belongs to.
        /// </summary>
        public int Label
        {
            get { return label; }
            set { label = value; }
        }

        /// <summary>
        /// Gets the triangle area constraint.
        /// </summary>
        public double Area
        {
            get { return area; }
            set { area = value; }
        }

        /// <summary>
        /// Gets the specified corners vertex.
        /// </summary>
        /// <param name="index">The corner index (0, 1 or 2).</param>
        /// <returns></returns>
        public Vertex GetVertex(int index)
        {
            return vertices[index]; // TODO: Check range?
        }

        /// <summary>
        /// Gets the specified corners vertex id.
        /// </summary>
        /// <param name="index">The corner index (0, 1 or 2).</param>
        /// <returns></returns>
        public int GetVertexID(int index)
        {
            return vertices[index].id;
        }

        /// <summary>
        /// Gets a triangles' neighbor.
        /// </summary>
        /// <param name="index">The neighbor index (0, 1 or 2).</param>
        /// <returns>The neighbor opposite of vertex with given index.</returns>
        public ITriangle GetNeighbor(int index)
        {
            var neighbor = neighbors[index].tri;
            return neighbor.hash == Mesh.DUMMY ? null : (ITriangle)neighbor;
        }

        /// <inheritdoc />
        public int GetNeighborID(int index)
        {
            var neighbor = neighbors[index].tri;
            return neighbor.hash == Mesh.DUMMY ? -1 : neighbor.id;
        }

        /// <summary>
        /// Gets a triangles segment.
        /// </summary>
        /// <param name="index">The vertex index (0, 1 or 2).</param>
        /// <returns>The segment opposite of vertex with given index.</returns>
        public ISegment GetSegment(int index)
        {
            return subsegs[index].seg.hash == Mesh.DUMMY ? null : subsegs[index].seg;
        }

        #endregion

        /// <inheritdoc />
        public bool Equals(Triangle other) => index == other.index && pool == other.pool;

        /// <inheritdoc />
        public override bool Equals(object obj) => obj is Triangle other && Equals(other);

        /// <inheritdoc />
        public override int GetHashCode()
        {
            return hash;
        }

        /// <summary>
        /// Equality of handles (same triangle).
        /// </summary>
        public static bool operator ==(Triangle a, Triangle b) => a.Equals(b);

        /// <summary>
        /// Inequality of handles.
        /// </summary>
        public static bool operator !=(Triangle a, Triangle b) => !a.Equals(b);

        /// <inheritdoc />
        public override string ToString()
        {
            return string.Format("TID {0}", hash);
        }
    }
}
