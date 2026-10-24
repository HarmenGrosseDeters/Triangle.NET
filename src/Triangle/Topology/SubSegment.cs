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

        // Stored inline instead of arrays (fewer objects on the heap, see Triangle).
        private Osub subseg0, subseg1;
        private Vertex vertex0, vertex1, vertex2, vertex3;
        private Otri triangle0, triangle1;
        internal int boundary;

        /// <summary>
        /// The two adjoining subsegments (indexed like an array).
        /// </summary>
        internal SubsegList subsegs => new SubsegList(this);

        /// <summary>
        /// The endpoints (0, 1) and the endpoints of the input segment (2, 3).
        /// </summary>
        internal VertexList vertices => new VertexList(this);

        /// <summary>
        /// The two adjoining triangles (indexed like an array).
        /// </summary>
        internal TriangleList triangles => new TriangleList(this);

        /// <summary>
        /// Initializes a new instance of the <see cref="SubSegment" /> class.
        /// </summary>
        public SubSegment()
        {
            // The vertices, adjoining subsegments and triangles are null (default), the
            // boundary marker is zero.
        }

        internal readonly struct SubsegList
        {
            private readonly SubSegment s;

            public SubsegList(SubSegment s) { this.s = s; }

            public ref Osub this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref s.subseg0;
                        case 1: return ref s.subseg1;
                        default: throw new IndexOutOfRangeException();
                    }
                }
            }
        }

        internal readonly struct VertexList
        {
            private readonly SubSegment s;

            public VertexList(SubSegment s) { this.s = s; }

            public ref Vertex this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref s.vertex0;
                        case 1: return ref s.vertex1;
                        case 2: return ref s.vertex2;
                        case 3: return ref s.vertex3;
                        default: throw new IndexOutOfRangeException();
                    }
                }
            }
        }

        internal readonly struct TriangleList
        {
            private readonly SubSegment s;

            public TriangleList(SubSegment s) { this.s = s; }

            public ref Otri this[int index]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    switch (index)
                    {
                        case 0: return ref s.triangle0;
                        case 1: return ref s.triangle1;
                        default: throw new IndexOutOfRangeException();
                    }
                }
            }
        }

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
