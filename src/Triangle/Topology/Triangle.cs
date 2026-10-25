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

#if FIELD_LAYOUT
        // Fields with ref-returning indexers (one heap object per triangle, also on
        // netstandard2.0, e.g. .NET Framework 4.8). Build with -p:TriangleLayout=Fields.
        private Vertex vertex0, vertex1, vertex2;
        private Otri neighbor0, neighbor1, neighbor2;
        private Osub subseg0, subseg1, subseg2;

        internal VertexList vertices => new VertexList(this);
        internal NeighborList neighbors => new NeighborList(this);
        internal SubsegList subsegs => new SubsegList(this);
#elif NET8_0_OR_GREATER && !ARRAY_LAYOUT
        // The vertices, adjoining triangles and subsegments are stored inline. With arrays, a
        // triangle consists of four objects on the heap, which make up more than half of all
        // allocations of a quality mesh and a large part of the garbage collection time.
        internal Buffer3<Vertex> vertices;
        internal Buffer3<Otri> neighbors;
        internal Buffer3<Osub> subsegs;
#else
        internal Vertex[] vertices = new Vertex[3];
        internal Otri[] neighbors = new Otri[3];
        internal Osub[] subsegs = new Osub[3];
#endif
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

#if FIELD_LAYOUT
        internal readonly struct VertexList
        {
            private readonly Triangle o;

            public VertexList(Triangle o) { this.o = o; }

            public ref Vertex this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.vertex0 : ref index == 1 ? ref o.vertex1 : ref o.vertex2;
            }
        }

        internal readonly struct NeighborList
        {
            private readonly Triangle o;

            public NeighborList(Triangle o) { this.o = o; }

            public ref Otri this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.neighbor0 : ref index == 1 ? ref o.neighbor1 : ref o.neighbor2;
            }
        }

        internal readonly struct SubsegList
        {
            private readonly Triangle o;

            public SubsegList(Triangle o) { this.o = o; }

            public ref Osub this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.subseg0 : ref index == 1 ? ref o.subseg1 : ref o.subseg2;
            }
        }

#endif

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
