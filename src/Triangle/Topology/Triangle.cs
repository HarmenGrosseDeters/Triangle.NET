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
    /// The triangle data structure.
    /// </summary>
    public class Triangle : ITriangle
    {
        // Hash for dictionary. Will be set by mesh instance.
        internal int hash;

        // The ID is only used for mesh output.
        internal int id;

        // The vertices, adjoining triangles and subsegments are stored inline (as arrays, a
        // triangle consisted of four objects on the heap, which made up more than half of all
        // allocations of a quality mesh and a large part of the garbage collection time).
        private Vertex vertex0, vertex1, vertex2;
        private Otri neighbor0, neighbor1, neighbor2;
        private Osub subseg0, subseg1, subseg2;

        /// <summary>
        /// The three vertices (indexed like an array).
        /// </summary>
        internal VertexList vertices => new VertexList(this);

        /// <summary>
        /// The three adjoining triangles (indexed like an array).
        /// </summary>
        internal NeighborList neighbors => new NeighborList(this);

        /// <summary>
        /// The three adjoining subsegments (indexed like an array).
        /// </summary>
        internal SubsegList subsegs => new SubsegList(this);
        internal int label;
        internal double area;
        internal bool infected;

        /// <summary>
        /// Initializes a new instance of the <see cref="Triangle" /> class.
        /// </summary>
        public Triangle()
        {
            // The vertices, adjoining subsegments and triangles are null (default).
        }

        internal readonly struct VertexList
        {
            private readonly Triangle t;

            public VertexList(Triangle t) { this.t = t; }

            public ref Vertex this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref t.vertex0;
                        case 1: return ref t.vertex1;
                        case 2: return ref t.vertex2;
                        default: throw new IndexOutOfRangeException();
                    }
                }
            }
        }

        internal readonly struct NeighborList
        {
            private readonly Triangle t;

            public NeighborList(Triangle t) { this.t = t; }

            public ref Otri this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref t.neighbor0;
                        case 1: return ref t.neighbor1;
                        case 2: return ref t.neighbor2;
                        default: throw new IndexOutOfRangeException();
                    }
                }
            }
        }

        internal readonly struct SubsegList
        {
            private readonly Triangle t;

            public SubsegList(Triangle t) { this.t = t; }

            public ref Osub this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref t.subseg0;
                        case 1: return ref t.subseg1;
                        case 2: return ref t.subseg2;
                        default: throw new IndexOutOfRangeException();
                    }
                }
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
            return neighbors[index].tri.hash == Mesh.DUMMY ? null : neighbors[index].tri;
        }

        /// <inheritdoc />
        public int GetNeighborID(int index)
        {
            return neighbors[index].tri.hash == Mesh.DUMMY ? -1 : neighbors[index].tri.id;
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
        public override int GetHashCode()
        {
            return hash;
        }

        /// <inheritdoc />
        public override string ToString()
        {
            return string.Format("TID {0}", hash);
        }
    }
}
