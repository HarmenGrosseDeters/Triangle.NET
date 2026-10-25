// -----------------------------------------------------------------------
// <copyright file="TrianglePool.cs" company="">
// Triangle.NET Copyright (c) 2012-2022 Christian Woltering
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet
{
    using System;
    using System.Collections.Generic;
    using TriangleNet.Geometry;
    using TriangleNet.Topology;

    /// <summary>
    /// Pool datastructure storing triangles of a <see cref="Mesh" />.
    /// </summary>
    /// <remarks>
    /// The data of all triangles is stored in arrays (three vertices, neighbors and subsegments
    /// per triangle, and the attributes), a <see cref="Triangle"/> is an index into them. Position
    /// 0 is the "outer space" triangle of the mesh. A pool can be reused for several meshes, one
    /// after another (see <see cref="Restart"/>), which also reuses the arrays.
    /// </remarks>
    public class TrianglePool : ICollection<Triangle>
    {
        // The total number of currently allocated triangles (positions 1 to size).
        int size;

        // The number of triangles currently used.
        int count;

        // Per triangle (index), three entries per triangle (3 * index + orientation).
        internal Vertex[] vertices;
        internal int[] neighbors; // (index << 2) | orientation of the neighbor, -1: dead
        internal Osub[] subsegs;
        internal int[] ids;
        internal int[] labels;
        internal double[] areas;
        internal bool[] infected;
        internal int[] marks;

        // Whether the triangle at the position is used.
        bool[] alive;

        // A stack of free triangles.
        Stack<int> stack;

        /// <summary>
        /// Gets the total number of currently allocated triangles.
        /// </summary>
        public int Capacity => size;

        /// <summary>
        /// Initializes a new instance of the <see cref="TrianglePool" /> class.
        /// </summary>
        public TrianglePool() : this(1024)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="TrianglePool" /> class with the given
        /// initial capacity (the arrays grow as needed).
        /// </summary>
        internal TrianglePool(int capacity)
        {
            size = 0;

            Allocate(Math.Max(1, capacity));

            stack = new Stack<int>(1024);

            InitializeDummy();
        }

        /// <summary>
        /// Make sure the pool can hold the given number of triangles without growing.
        /// </summary>
        internal void EnsureCapacity(int capacity)
        {
            if (capacity + 1 > ids.Length)
            {
                Allocate(capacity);
            }
        }

        private void Allocate(int capacity)
        {
            capacity += 1; // The outer space triangle.

            Array.Resize(ref vertices, 3 * capacity);
            Array.Resize(ref neighbors, 3 * capacity);
            Array.Resize(ref subsegs, 3 * capacity);
            Array.Resize(ref ids, capacity);
            Array.Resize(ref labels, capacity);
            Array.Resize(ref areas, capacity);
            Array.Resize(ref infected, capacity);
            Array.Resize(ref marks, capacity);
            Array.Resize(ref alive, capacity);
        }

        private void InitializeDummy()
        {
            // The outer space triangle: all neighbors are the outer space triangle itself.
            ids[0] = Mesh.DUMMY;
            neighbors[0] = neighbors[1] = neighbors[2] = 0;
            vertices[0] = vertices[1] = vertices[2] = null;
            labels[0] = 0;
            areas[0] = 0.0;
            infected[0] = false;
            marks[0] = 0;
        }

        /// <summary>
        /// Gets the outer space triangle ('dummytri') of the mesh using this pool.
        /// </summary>
        internal Triangle Dummy => new Triangle(this, 0);

        // After the triangles have been renumbered (ids 0 to n-1), new triangles get the
        // following ids (their hash could collide with the new ids). -1: ids are hashes.
        private int nextId = -1;

        /// <summary>
        /// Continue the numbering of new triangles after renumbering the triangles.
        /// </summary>
        internal void SetNextId(int id)
        {
            nextId = id;
        }

        private int NextId(int index)
        {
            return nextId < 0 ? index - 1 : nextId++;
        }

        /// <summary>
        /// Gets a triangle from the pool.
        /// </summary>
        /// <returns></returns>
        public Triangle Get()
        {
            int index;

            if (stack.Count > 0)
            {
                index = stack.Pop();
                alive[index] = true;

                ids[index] = NextId(index);

                Cleanup(index);
            }
            else if (count < size)
            {
                index = count + 1;
                ids[index] = NextId(index);

                Cleanup(index);

                count++;
            }
            else
            {
                if (size + 2 > ids.Length)
                {
                    Allocate(2 * (ids.Length - 1));
                }

                index = size + 1;
                alive[index] = true;
                ids[index] = NextId(index);

                Cleanup(index);

                count = ++size;
            }

            return new Triangle(this, index);
        }

        /// <summary>
        /// Release triangle (making it a free triangle).
        /// </summary>
        public void Release(Triangle triangle)
        {
            stack.Push(triangle.index);

            // Mark the triangle as free (used by enumerator).
            alive[triangle.index] = false;
        }

        /// <summary>
        /// Restart the triangle pool.
        /// </summary>
        public TrianglePool Restart()
        {
            foreach (var index in stack)
            {
                alive[index] = true;
            }

            stack.Clear();

            count = 0;

            InitializeDummy();

            return this;
        }

        /// <summary>
        /// Samples a number of triangles from the pool.
        /// </summary>
        /// <param name="k">The number of triangles to sample.</param>
        /// <param name="random"></param>
        /// <returns></returns>
        internal IEnumerable<Triangle> Sample(int k, Random random)
        {
            int i, alive = this.Count;

            if (k > alive)
            {
                // TODO: handle Sample special case.
                k = alive;
            }

            // TODO: improve sampling code (to ensure no duplicates).

            // Live triangles are spread over all used slots [0, count), not only the first
            // 'alive' ones. Bound the number of attempts (if almost all slots are free).
            int attempts = 20 * k + 100;

            while (k > 0 && attempts-- > 0)
            {
                i = random.Next(0, count);

                if (this.alive[i + 1])
                {
                    k--;
                    yield return new Triangle(this, i + 1);
                }
            }
        }

        private void Cleanup(int index)
        {
            labels[index] = 0;
            areas[index] = 0.0;
            infected[index] = false;
            marks[index] = 0;

            int i = 3 * index;

            vertices[i] = vertices[i + 1] = vertices[i + 2] = null;
            subsegs[i] = subsegs[i + 1] = subsegs[i + 2] = default(Osub);
            neighbors[i] = neighbors[i + 1] = neighbors[i + 2] = 0;
        }

        /// <summary>
        /// Not supported for this <see cref="ICollection{Triangle}" />.
        /// </summary>
        public void Add(Triangle item)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// Clear the pool.
        /// </summary>
        public void Clear()
        {
            stack.Clear();

            Array.Clear(alive, 0, alive.Length);
            Array.Clear(vertices, 0, vertices.Length);
            Array.Clear(subsegs, 0, subsegs.Length);

            size = count = 0;
            nextId = -1;

            InitializeDummy();
        }

        /// <inheritdoc />
        public bool Contains(Triangle item)
        {
            if (item.pool != this)
            {
                return false;
            }

            int i = item.hash;

            if (i < 0 || i >= size)
            {
                return false;
            }

            return alive[item.index];
        }

        /// <inheritdoc />
        public void CopyTo(Triangle[] array, int index)
        {
            var enumerator = GetEnumerator();

            while (enumerator.MoveNext())
            {
                array[index] = enumerator.Current;
                index++;
            }
        }

        /// <inheritdoc />
        public int Count => count - stack.Count;

        /// <inheritdoc />
        public bool IsReadOnly => true;

        /// <inheritdoc />
        public bool Remove(Triangle item)
        {
            throw new NotImplementedException();
        }

        /// <inheritdoc />
        public IEnumerator<Triangle> GetEnumerator()
        {
            return new Enumerator(this);
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        class Enumerator : IEnumerator<Triangle>
        {
            // TODO: enumerator should be able to tell if collection changed.

            readonly int count;

            readonly TrianglePool pool;

            Triangle current;

            int index, offset;

            public Enumerator(TrianglePool pool)
            {
                this.count = pool.Count;
                this.pool = pool;

                index = 0;
                offset = 0;
            }

            public Triangle Current
            {
                get { return current; }
            }

            public void Dispose()
            {
            }

            object System.Collections.IEnumerator.Current
            {
                get { return current; }
            }

            public bool MoveNext()
            {
                var alive = pool.alive;

                while (index < count)
                {
                    offset++;

                    if (alive[offset])
                    {
                        current = new Triangle(pool, offset);
                        index++;
                        return true;
                    }
                }

                return false;
            }

            public void Reset()
            {
                index = offset = 0;
            }
        }
    }
}
