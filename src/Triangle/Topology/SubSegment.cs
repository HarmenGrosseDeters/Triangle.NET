// -----------------------------------------------------------------------
// <copyright file="SubSegment.cs" company="">
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
    /// The subsegment data structure.
    /// </summary>
    public class SubSegment : ISegment
    {
        // Hash for dictionary. Will be set by mesh instance.
        internal int hash;

#if FIELD_LAYOUT
        // Fields with ref-returning indexers (see Triangle).
        private Osub subseg0, subseg1;
        private Vertex vertex0, vertex1, vertex2, vertex3;
        private Otri triangle0, triangle1;

        internal SubsegList subsegs => new SubsegList(this);
        internal VertexList vertices => new VertexList(this);
        internal TriangleList triangles => new TriangleList(this);
#elif NET8_0_OR_GREATER && !ARRAY_LAYOUT
        // Stored inline instead of arrays (fewer objects on the heap, see Triangle).
        internal Buffer2<Osub> subsegs;
        internal Buffer4<Vertex> vertices;
        internal Buffer2<Otri> triangles;
#else
        internal Osub[] subsegs = new Osub[2];
        internal Vertex[] vertices = new Vertex[4];
        internal Otri[] triangles = new Otri[2];
#endif
        internal int boundary;

        /// <summary>
        /// Initializes a new instance of the <see cref="SubSegment" /> class.
        /// </summary>
        public SubSegment()
        {
            // The vertices, adjoining subsegments and triangles are null (default), the
            // boundary marker is zero.
        }

#if FIELD_LAYOUT
        internal readonly struct SubsegList
        {
            private readonly SubSegment o;

            public SubsegList(SubSegment o) { this.o = o; }

            public ref Osub this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.subseg0 : ref o.subseg1;
            }
        }

        internal readonly struct VertexList
        {
            private readonly SubSegment o;

            public VertexList(SubSegment o) { this.o = o; }

            public ref Vertex this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.vertex0 : ref index == 1 ? ref o.vertex1 : ref index == 2 ? ref o.vertex2 : ref o.vertex3;
            }
        }

        internal readonly struct TriangleList
        {
            private readonly SubSegment o;

            public TriangleList(SubSegment o) { this.o = o; }

            public ref Otri this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => ref index == 0 ? ref o.triangle0 : ref o.triangle1;
            }
        }

#endif

        #region Public properties

        /// <summary>
        /// Gets the first endpoints vertex id.
        /// </summary>
        public int P0 => vertices[0].id;

        /// <summary>
        /// Gets the seconds endpoints vertex id.
        /// </summary>
        public int P1 => vertices[1].id;

        /// <summary>
        /// Gets the segment boundary mark.
        /// </summary>
        public int Label => boundary;

        #endregion

        /// <summary>
        /// Gets the segments endpoint.
        /// </summary>
        public Vertex GetVertex(int index)
        {
            return vertices[index]; // TODO: Check range?
        }

        /// <summary>
        /// Gets an adjoining triangle.
        /// </summary>
        public ITriangle GetTriangle(int index)
        {
            return triangles[index].tri.hash == Mesh.DUMMY ? null : triangles[index].tri;
        }

        /// <summary>
        /// Gets an adjoining triangle.
        /// </summary>
        public void GetTriangle(int index, ref Otri otri)
        {
            otri = triangles[index];
        }

        /// <inheritdoc />
        public override int GetHashCode()
        {
            return hash;
        }

        /// <inheritdoc />
        public override string ToString()
        {
            return string.Format("SID {0}", hash);
        }
    }
}
