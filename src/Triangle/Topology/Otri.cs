// -----------------------------------------------------------------------
// <copyright file="Otri.cs">
// Triangle Copyright (c) 1993, 1995, 1997, 1998, 2002, 2005 Jonathan Richard Shewchuk
// Triangle.NET code by Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Topology
{
    using System;
    using TriangleNet.Geometry;

    /// <summary>
    /// An oriented triangle.
    /// </summary>
    /// <remarks>
    /// Includes a pointer to a triangle and orientation.  The orientation denotes an edge
    /// of the triangle. Hence, there are three possible orientations. By convention, each
    /// edge always points counterclockwise about the corresponding triangle.
    /// </remarks>
    public struct Otri
    {
        internal Triangle tri;
        internal int orient; // Ranges from 0 to 2.

        public int Orient => orient;

        /// <summary>
        /// Gets or sets the triangle.
        /// </summary>
        public Triangle Triangle
        {
            get { return tri; }
            set { tri = value; }
        }

        /// <inheritdoc />
        public override string ToString()
        {
            if (tri.IsNull)
            {
                return "O-TID [null]";
            }
            return string.Format("O-TID {0}", tri.hash);
        }

        #region Otri primitives (public)

        // For fast access
        static readonly int[] plus1Mod3 = { 1, 2, 0 };
        static readonly int[] minus1Mod3 = { 2, 0, 1 };

        // The following primitives are all described by Guibas and Stolfi.
        // However, Guibas and Stolfi use an edge-based data structure,
        // whereas I use a triangle-based data structure.
        //
        // lnext: finds the next edge (counterclockwise) of a triangle.
        //
        // onext: spins counterclockwise around a vertex; that is, it finds 
        // the next edge with the same origin in the counterclockwise direction. This
        // edge is part of a different triangle.
        //
        // oprev: spins clockwise around a vertex; that is, it finds the 
        // next edge with the same origin in the clockwise direction.  This edge is 
        // part of a different triangle.
        //
        // dnext: spins counterclockwise around a vertex; that is, it finds 
        // the next edge with the same destination in the counterclockwise direction.
        // This edge is part of a different triangle.
        //
        // dprev: spins clockwise around a vertex; that is, it finds the 
        // next edge with the same destination in the clockwise direction. This edge 
        // is part of a different triangle.
        //
        // rnext: moves one edge counterclockwise about the adjacent 
        // triangle. (It's best understood by reading Guibas and Stolfi. It 
        // involves changing triangles twice.)
        //
        // rprev: moves one edge clockwise about the adjacent triangle.
        // (It's best understood by reading Guibas and Stolfi.  It involves
        // changing triangles twice.)

        /// <summary>
        /// Find the abutting triangle; same edge. [sym(abc) -> ba*]
        /// </summary>
        /// <remarks>
        /// Note that the edge direction is necessarily reversed, because the handle specified 
        /// by an oriented triangle is directed counterclockwise around the triangle.
        /// </remarks>
        public void Sym(ref Otri ot)
        {
            int code = tri.pool.neighbors[3 * tri.index + orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;
        }

        /// <summary>
        /// Find the abutting triangle; same edge. [sym(abc) -> ba*]
        /// </summary>
        public void Sym()
        {
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;
        }

        /// <summary>
        /// Find the next edge (counterclockwise) of a triangle. [lnext(abc) -> bca]
        /// </summary>
        public void Lnext(ref Otri ot)
        {
            ot.tri = tri;
            ot.orient = plus1Mod3[orient];
        }

        /// <summary>
        /// Find the next edge (counterclockwise) of a triangle. [lnext(abc) -> bca]
        /// </summary>
        public void Lnext()
        {
            orient = plus1Mod3[orient];
        }

        /// <summary>
        /// Find the previous edge (clockwise) of a triangle. [lprev(abc) -> cab]
        /// </summary>
        public void Lprev(ref Otri ot)
        {
            ot.tri = tri;
            ot.orient = minus1Mod3[orient];
        }

        /// <summary>
        /// Find the previous edge (clockwise) of a triangle. [lprev(abc) -> cab]
        /// </summary>
        public void Lprev()
        {
            orient = minus1Mod3[orient];
        }

        /// <summary>
        /// Find the next edge counterclockwise with the same origin. [onext(abc) -> ac*]
        /// </summary>
        public void Onext(ref Otri ot)
        {
            //Lprev(ref ot);
            ot.tri = tri;
            ot.orient = minus1Mod3[orient];

            //ot.SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + ot.orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;
        }

        /// <summary>
        /// Find the next edge counterclockwise with the same origin. [onext(abc) -> ac*]
        /// </summary>
        public void Onext()
        {
            //LprevSelf();
            orient = minus1Mod3[orient];

            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;
        }

        /// <summary>
        /// Find the next edge clockwise with the same origin. [oprev(abc) -> a*b]
        /// </summary>
        public void Oprev(ref Otri ot)
        {
            //Sym(ref ot);
            int code = tri.pool.neighbors[3 * tri.index + orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;

            //ot.LnextSelf();
            ot.orient = plus1Mod3[ot.orient];
        }

        /// <summary>
        /// Find the next edge clockwise with the same origin. [oprev(abc) -> a*b]
        /// </summary>
        public void Oprev()
        {
            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;

            //LnextSelf();
            orient = plus1Mod3[orient];
        }

        /// <summary>
        /// Find the next edge counterclockwise with the same destination. [dnext(abc) -> *ba]
        /// </summary>
        public void Dnext(ref Otri ot)
        {
            //Sym(ref ot);
            int code = tri.pool.neighbors[3 * tri.index + orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;

            //ot.LprevSelf();
            ot.orient = minus1Mod3[ot.orient];
        }

        /// <summary>
        /// Find the next edge counterclockwise with the same destination. [dnext(abc) -> *ba]
        /// </summary>
        public void Dnext()
        {
            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;

            //LprevSelf();
            orient = minus1Mod3[orient];
        }

        /// <summary>
        /// Find the next edge clockwise with the same destination. [dprev(abc) -> cb*]
        /// </summary>
        public void Dprev(ref Otri ot)
        {
            //Lnext(ref ot);
            ot.tri = tri;
            ot.orient = plus1Mod3[orient];

            //ot.SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + ot.orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;
        }

        /// <summary>
        /// Find the next edge clockwise with the same destination. [dprev(abc) -> cb*]
        /// </summary>
        public void Dprev()
        {
            //LnextSelf();
            orient = plus1Mod3[orient];

            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;
        }

        /// <summary>
        /// Find the next edge (counterclockwise) of the adjacent triangle. [rnext(abc) -> *a*]
        /// </summary>
        public void Rnext(ref Otri ot)
        {
            //Sym(ref ot);
            int code = tri.pool.neighbors[3 * tri.index + orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;

            //ot.LnextSelf();
            ot.orient = plus1Mod3[ot.orient];

            //ot.SymSelf();
            code = tri.pool.neighbors[3 * ot.tri.index + ot.orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;
        }

        /// <summary>
        /// Find the next edge (counterclockwise) of the adjacent triangle. [rnext(abc) -> *a*]
        /// </summary>
        public void Rnext()
        {
            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;

            //LnextSelf();
            orient = plus1Mod3[orient];

            //SymSelf();
            code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;
        }

        /// <summary>
        /// Find the previous edge (clockwise) of the adjacent triangle. [rprev(abc) -> b**]
        /// </summary>
        public void Rprev(ref Otri ot)
        {
            //Sym(ref ot);
            int code = tri.pool.neighbors[3 * tri.index + orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;

            //ot.LprevSelf();
            ot.orient = minus1Mod3[ot.orient];

            //ot.SymSelf();
            code = tri.pool.neighbors[3 * ot.tri.index + ot.orient];
            ot.tri = new Triangle(tri.pool, code >> 2);
            ot.orient = code & 3;
        }

        /// <summary>
        /// Find the previous edge (clockwise) of the adjacent triangle. [rprev(abc) -> b**]
        /// </summary>
        public void Rprev()
        {
            //SymSelf();
            int code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;

            //LprevSelf();
            orient = minus1Mod3[orient];

            //SymSelf();
            code = tri.pool.neighbors[3 * tri.index + orient];
            tri = new Triangle(tri.pool, code >> 2);
            orient = code & 3;
        }

        /// <summary>
        /// Origin [org(abc) -> a]
        /// </summary>
        public Vertex Org()
        {
            return tri.vertices[plus1Mod3[orient]];
        }

        /// <summary>
        /// Destination [dest(abc) -> b]
        /// </summary>
        public Vertex Dest()
        {
            return tri.vertices[minus1Mod3[orient]];
        }

        /// <summary>
        /// Apex [apex(abc) -> c]
        /// </summary>
        public Vertex Apex()
        {
            return tri.vertices[orient];
        }

        /// <summary>
        /// Copy an oriented triangle.
        /// </summary>
        public void Copy(ref Otri ot)
        {
            ot.tri = tri;
            ot.orient = orient;
        }

        /// <summary>
        /// Finds a subsegment abutting a triangle.
        /// </summary>
        public void Pivot(ref Osub os)
        {
            os = tri.subsegs[orient];
        }

        /// <summary>
        /// Test for equality of oriented triangles.
        /// </summary>
        public bool Equals(Otri ot)
        {
            return ((tri == ot.tri) && (orient == ot.orient));
        }

        #endregion

        #region Otri primitives (internal)

        /// <summary>
        /// Set Origin
        /// </summary>
        internal void SetOrg(Vertex v)
        {
            tri.vertices[plus1Mod3[orient]] = v;
        }

        /// <summary>
        /// Set Destination
        /// </summary>
        internal void SetDest(Vertex v)
        {
            tri.vertices[minus1Mod3[orient]] = v;
        }

        /// <summary>
        /// Set Apex
        /// </summary>
        internal void SetApex(Vertex v)
        {
            tri.vertices[orient] = v;
        }

        /// <summary>
        /// Bond two triangles together at the resepective handles. [bond(abc, bad)]
        /// </summary>
        internal void Bond(ref Otri ot)
        {
            var n = tri.pool.neighbors;
            n[3 * tri.index + orient] = (ot.tri.index << 2) | ot.orient;

            n[3 * ot.tri.index + ot.orient] = (tri.index << 2) | orient;
        }

        /// <summary>
        /// Dissolve a bond (from one side).  
        /// </summary>
        /// <remarks>Note that the other triangle will still think it's connected to 
        /// this triangle. Usually, however, the other triangle is being deleted 
        /// entirely, or bonded to another triangle, so it doesn't matter.
        /// </remarks>
        internal void Dissolve(Triangle dummy)
        {
            tri.pool.neighbors[3 * tri.index + orient] = dummy.index << 2;
        }

        /// <summary>
        /// Infect a triangle with the virus.
        /// </summary>
        internal void Infect()
        {
            tri.infected = true;
        }

        /// <summary>
        /// Cure a triangle from the virus.
        /// </summary>
        internal void Uninfect()
        {
            tri.infected = false;
        }

        /// <summary>
        /// Test a triangle for viral infection.
        /// </summary>
        internal bool IsInfected()
        {
            return tri.infected;
        }

        /// <summary>
        /// Bond a triangle to a subsegment.
        /// </summary>
        internal void SegBond(ref Osub os)
        {
            tri.subsegs[orient] = os;
            os.seg.triangles[os.orient] = this;
        }

        /// <summary>
        /// Dissolve a bond (from the triangle side).
        /// </summary>
        internal void SegDissolve(SubSegment dummy)
        {
            tri.subsegs[orient].seg = dummy;
        }

        /// <summary>
        /// Check a triangle's deallocation.
        /// </summary>
        internal static bool IsDead(Triangle tria)
        {
            return tria.pool.neighbors[3 * tria.index] == -1;
        }

        /// <summary>
        /// Set a triangle's deallocation.
        /// </summary>
        internal static void Kill(Triangle tri)
        {
            tri.pool.neighbors[3 * tri.index] = -1;
            tri.pool.neighbors[3 * tri.index + 2] = -1;
        }

        #endregion
    }
}
