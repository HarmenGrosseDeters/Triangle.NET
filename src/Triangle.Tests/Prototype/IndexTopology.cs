using System;
using TriangleNet.Geometry;

namespace TriangleNet.Tests.Prototype
{
    /// <summary>
    /// Prototype of an index-based triangle storage: the vertices and neighbors of all triangles
    /// in two arrays instead of an object (and its arrays) per triangle. Triangle 0 is the
    /// "outer space" triangle (dummytri).
    /// </summary>
    internal sealed class TriangleStore
    {
        // Three vertices per triangle.
        public Vertex[] Vertices;

        // Three neighbors per triangle, encoded as (triangle << 2) | orientation.
        public int[] Neighbors;

        public int Count;

        public TriangleStore(int capacity)
        {
            capacity = Math.Max(capacity, 16);
            Vertices = new Vertex[3 * capacity];
            Neighbors = new int[3 * capacity];
            Count = 1;
        }

        public int Make()
        {
            if (3 * (Count + 1) > Neighbors.Length)
            {
                Array.Resize(ref Vertices, 2 * Vertices.Length);
                Array.Resize(ref Neighbors, 2 * Neighbors.Length);
            }

            int t = Count++;
            int i = 3 * t;
            Neighbors[i] = Neighbors[i + 1] = Neighbors[i + 2] = 0;
            Vertices[i] = Vertices[i + 1] = Vertices[i + 2] = null;
            return t;
        }

        public void Kill(int t) => Neighbors[3 * t] = -1;

        public bool IsDead(int t) => Neighbors[3 * t] == -1;
    }

    /// <summary>
    /// Oriented triangle on a <see cref="TriangleStore"/> (same operations as <see cref="Topology.Otri"/>).
    /// </summary>
    internal struct XOtri
    {
        // The store of the running triangulation (prototype: one per thread).
        [ThreadStatic] public static TriangleStore S;

        public int t;
        public int o;

        private static int Plus1(int o) => o == 2 ? 0 : o + 1;
        private static int Minus1(int o) => o == 0 ? 2 : o - 1;

        public static void Make(ref XOtri ot) { ot.t = S.Make(); ot.o = 0; }

        public void Sym(ref XOtri ot) { int c = S.Neighbors[3 * t + o]; ot.t = c >> 2; ot.o = c & 3; }
        public void Sym() { int c = S.Neighbors[3 * t + o]; t = c >> 2; o = c & 3; }
        public void Lnext(ref XOtri ot) { ot.t = t; ot.o = Plus1(o); }
        public void Lnext() { o = Plus1(o); }
        public void Lprev(ref XOtri ot) { ot.t = t; ot.o = Minus1(o); }
        public void Lprev() { o = Minus1(o); }
        public void Onext(ref XOtri ot) { int c = S.Neighbors[3 * t + Minus1(o)]; ot.t = c >> 2; ot.o = c & 3; }
        public void Onext() { int c = S.Neighbors[3 * t + Minus1(o)]; t = c >> 2; o = c & 3; }
        public void Oprev(ref XOtri ot) { int c = S.Neighbors[3 * t + o]; ot.t = c >> 2; ot.o = Plus1(c & 3); }
        public void Oprev() { int c = S.Neighbors[3 * t + o]; t = c >> 2; o = Plus1(c & 3); }
        public void Dnext(ref XOtri ot) { int c = S.Neighbors[3 * t + o]; ot.t = c >> 2; ot.o = Minus1(c & 3); }
        public void Dnext() { int c = S.Neighbors[3 * t + o]; t = c >> 2; o = Minus1(c & 3); }
        public void Dprev(ref XOtri ot) { int c = S.Neighbors[3 * t + Plus1(o)]; ot.t = c >> 2; ot.o = c & 3; }
        public void Dprev() { int c = S.Neighbors[3 * t + Plus1(o)]; t = c >> 2; o = c & 3; }
        public void Rnext(ref XOtri ot) { var x = this; x.Sym(); x.Lnext(); x.Sym(); ot = x; }
        public void Rnext() { Sym(); Lnext(); Sym(); }
        public void Rprev(ref XOtri ot) { var x = this; x.Sym(); x.Lprev(); x.Sym(); ot = x; }
        public void Rprev() { Sym(); Lprev(); Sym(); }

        public Vertex Org() => S.Vertices[3 * t + Plus1(o)];
        public Vertex Dest() => S.Vertices[3 * t + Minus1(o)];
        public Vertex Apex() => S.Vertices[3 * t + o];
        public void SetOrg(Vertex v) => S.Vertices[3 * t + Plus1(o)] = v;
        public void SetDest(Vertex v) => S.Vertices[3 * t + Minus1(o)] = v;
        public void SetApex(Vertex v) => S.Vertices[3 * t + o] = v;

        public void Copy(ref XOtri ot) { ot.t = t; ot.o = o; }
        public bool Equals(XOtri ot) => t == ot.t && o == ot.o;

        public void Bond(ref XOtri ot)
        {
            S.Neighbors[3 * t + o] = (ot.t << 2) | ot.o;
            S.Neighbors[3 * ot.t + ot.o] = (t << 2) | o;
        }

        public void Dissolve() => S.Neighbors[3 * t + o] = 0;
    }
}
