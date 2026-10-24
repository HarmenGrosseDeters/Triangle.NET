using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Smoothing;
using TriangleNet.Tests.Robustness;
using TriangleNet.Tools;
using TriangleNet.Topology.DCEL;
using TriangleNet.Voronoi;
using TriangleNet.Voronoi.Legacy;

using Vertex = TriangleNet.Geometry.Vertex;

namespace TriangleNet.Tests.Review
{
    /// <summary>
    /// Review of Voronoi, Smoothing and Tools (review E).
    /// </summary>
    public class ReviewETest
    {
        #region Helpers

        private static Polygon Square(double size, int label = 1)
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[]
            {
                new Vertex(0, 0, label), new Vertex(size, 0, label),
                new Vertex(size, size, label), new Vertex(0, size, label)
            }, label));
            return poly;
        }

        private static Polygon SquareWithHole()
        {
            var poly = Square(10);
            poly.Add(new Contour(new[]
            {
                new Vertex(4, 4, 2), new Vertex(6, 4, 2), new Vertex(6, 6, 2), new Vertex(4, 6, 2)
            }, 2), true);
            return poly;
        }

        private static Mesh QualityMesh(Polygon poly, double maxArea = 1.0)
        {
            return (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 25, MaximumArea = maxArea });
        }

        private static double MeshArea(IMesh mesh) => mesh.Triangles.Sum(t => MeshAssert.Area(t));

        private static List<Point> CellPolygon(Face face, int maxEdges = 1000)
        {
            var list = new List<Point>();
            var start = face.Edge;
            var e = start;
            int k = 0;
            do
            {
                list.Add(e.Origin);
                e = e.Next;
                if (++k > maxEdges) throw new InvalidOperationException("Cell not closed.");
            }
            while (e != start);
            return list;
        }

        private static double PolygonArea(List<Point> p)
        {
            double a = 0;
            for (int i = 0, j = p.Count - 1; i < p.Count; j = i++)
            {
                a += (p[j].X * p[i].Y - p[i].X * p[j].Y);
            }
            return a / 2;
        }

        private static (double x, double y) Centroid(IEnumerable<Point> pts)
        {
            var l = pts.ToList();
            return (l.Average(p => p.X), l.Average(p => p.Y));
        }

        #endregion

        #region BoundedVoronoi

        [Test]
        [Description("BoundedVoronoi assigns duplicate half-edge ids: HandleCase1/HandleCase2 set ids from edges.Count AFTER adding (h1.id = Count, h2.id = Count+1, he.id = Count), and ResolveBoundaryEdges starts again at edges.Count.")]
        public void BoundedVoronoi_HalfEdgeIdsAreUnique()
        {
            var mesh = QualityMesh(Square(10));

            var voronoi = new BoundedVoronoi(mesh);

            var ids = voronoi.HalfEdges.Select(e => e.ID).ToList();

            Assert.That(ids.Distinct().Count(), Is.EqualTo(ids.Count), "Duplicate half-edge ids.");
        }

        [Test]
        public void BoundedVoronoi_FaceEnumerateEdgesMatchesNextChain([Range(0, 9)] int seed)
        {
            var mesh = (Mesh)RandomDomain(seed, true).Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 20 });

            var voronoi = new BoundedVoronoi(mesh);

            foreach (var face in voronoi.Faces)
            {
                Assert.That(face.EnumerateEdges().Count(), Is.EqualTo(CellPolygon(face).Count), $"Face {face.ID}");
            }
        }

        [Test]
        [Description("Smoothing.VoronoiFactory (object pool used by SimpleSmoother) never sets Face.label from the generator and does not reset Face.bounded / DCEL Vertex.label when reusing pooled objects, so diagrams built with it differ from DefaultVoronoiFactory (low impact: SimpleSmoother only reads generator.label).")]
        public void VoronoiFactory_PooledObjectsAreReset()
        {
            var meshA = QualityMesh(Helper.SplitRectangle(-1, 1, 1, -1, 3), 0.05);
            var meshB = QualityMesh(Square(10), 1.0);

            var factory = new VoronoiFactory();

            new BoundedVoronoi(meshA, factory, RobustPredicates.Default);
            factory.Reset();
            var pooled = new BoundedVoronoi(meshB, factory, RobustPredicates.Default);

            var fresh = new BoundedVoronoi(meshB);

            Assert.That(pooled.Faces.Select(f => (f.Label, f.Bounded)), Is.EqualTo(fresh.Faces.Select(f => (f.Label, f.Bounded))));
            Assert.That(pooled.Vertices.Select(v => v.Label), Is.EqualTo(fresh.Vertices.Select(v => v.Label)));
        }

        [Test]
        public void BoundedVoronoi_EdgesEnumeratesEachEdgeOnce()
        {
            var mesh = QualityMesh(Square(10));

            var voronoi = new BoundedVoronoi(mesh);

            var expected = new HashSet<(int, int)>();
            foreach (var e in voronoi.HalfEdges)
            {
                int a = e.Origin.ID, b = e.Twin.Origin.ID;
                expected.Add((Math.Min(a, b), Math.Max(a, b)));
            }

            var edges = voronoi.Edges.Select(e => (Math.Min(e.P0, e.P1), Math.Max(e.P0, e.P1))).ToList();

            Assert.That(edges.Count, Is.EqualTo(expected.Count));
            Assert.That(new HashSet<(int, int)>(edges).SetEquals(expected));
        }

        [Test]
        public void BoundedVoronoi_CellsCoverConvexDomain()
        {
            var mesh = QualityMesh(Square(10));

            var voronoi = new BoundedVoronoi(mesh);

            Assert.That(voronoi.IsConsistent(), Is.True);

            double sum = 0;
            foreach (var face in voronoi.Faces)
            {
                var cell = CellPolygon(face);
                double a = PolygonArea(cell);
                Assert.That(a, Is.GreaterThan(0), $"Cell {face.ID} has non-positive area.");
                sum += a;

                foreach (var p in cell)
                {
                    Assert.That(p.X, Is.InRange(-1e-9, 10 + 1e-9));
                    Assert.That(p.Y, Is.InRange(-1e-9, 10 + 1e-9));
                }
            }

            Assert.That(sum, Is.EqualTo(100.0).Within(1e-8));
        }

        [Test]
        public void BoundedVoronoi_CellsCoverDomainWithHole()
        {
            var mesh = QualityMesh(SquareWithHole());

            var voronoi = new BoundedVoronoi(mesh);

            Assert.That(voronoi.IsConsistent(), Is.True);

            double sum = voronoi.Faces.Sum(f => PolygonArea(CellPolygon(f)));

            Assert.That(sum, Is.EqualTo(MeshArea(mesh)).Within(1e-8));
            Assert.That(sum, Is.EqualTo(96.0).Within(1e-8));
        }

        [Test]
        public void BoundedVoronoi_ConstrainedDelaunayWithObtuseBoundaryTriangle()
        {
            // Triangle (0,0),(10,0),(5,1) is obtuse at (5,1): its circumcenter lies
            // outside the domain (HandleCase2).
            var poly = Square(10);
            poly.Add(new Vertex(5, 1, 0));
            poly.Add(new Vertex(5, 9, 0));

            var mesh = (Mesh)poly.Triangulate();

            var voronoi = new BoundedVoronoi(mesh);

            Assert.That(voronoi.IsConsistent(), Is.True);

            double sum = voronoi.Faces.Sum(f => PolygonArea(CellPolygon(f)));

            Assert.That(sum, Is.EqualTo(100.0).Within(1e-8));
        }

        private static Polygon RandomDomain(int seed, bool lshape)
        {
            var random = new Random(seed);
            var poly = new Polygon();

            if (lshape)
            {
                poly.Add(new Contour(new[]
                {
                    new Vertex(0, 0, 1), new Vertex(10, 0, 1), new Vertex(10, 4, 1),
                    new Vertex(4, 4, 1), new Vertex(4, 10, 1), new Vertex(0, 10, 1)
                }, 1));
            }
            else
            {
                poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(8, 0, 1), new Vertex(8, 8, 1), new Vertex(0, 8, 1) }, 1));
            }

            int k = 0;
            while (k < 25)
            {
                double x = random.NextDouble() * 10, y = random.NextDouble() * 10;
                if (lshape && x > 4 && y > 4) continue;
                if (!lshape && (x > 7.99 || y > 7.99)) continue;
                if (x < 0.01 || y < 0.01) continue;
                poly.Add(new Vertex(x, y, 0));
                k++;
            }

            return poly;
        }

        private static void AssertCellsPartitionDomain(BoundedVoronoi voronoi, double area, Rectangle bounds)
        {
            Assert.That(voronoi.IsConsistent(), Is.True);

            double sum = 0;

            foreach (var face in voronoi.Faces)
            {
                var cell = CellPolygon(face);
                double a = PolygonArea(cell);

                Assert.That(a, Is.GreaterThan(0), $"Cell {face.ID} has non-positive area {a}.");

                foreach (var p in cell)
                {
                    Assert.That(bounds.Contains(p), Is.True, $"Cell {face.ID} has vertex ({p.X}, {p.Y}) outside the domain.");
                }

                sum += a;
            }

            Assert.That(sum, Is.EqualTo(area).Within(1e-8));
        }

        [Test]
        public void BoundedVoronoi_RandomConformingDelaunay([Range(0, 9)] int seed, [Values(false, true)] bool lshape)
        {
            var poly = RandomDomain(seed, lshape);

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true });

            var voronoi = MeshAssert.Terminates(() => new BoundedVoronoi(mesh));

            var bounds = new Rectangle(-1e-9, -1e-9, (lshape ? 10 : 8) + 2e-9, (lshape ? 10 : 8) + 2e-9);

            AssertCellsPartitionDomain(voronoi, lshape ? 64.0 : 64.0, bounds);
        }

        [Test]
        [Explicit("Documented limitation: BoundedVoronoi requires a conforming Delaunay mesh (see class remarks).")]
        [Description("BoundedVoronoi on a constrained (non-conforming) Delaunay triangulation: HandleCase2 only clips a circumcenter that lies just beyond its own boundary edge. Circumcenters further outside are kept, so cells get vertices far outside the domain (e.g. (28.9, 4.0) for an 8x8 square) and negative (self-overlapping) area.")]
        public void BoundedVoronoi_ConstrainedDelaunayCellsInsideDomain()
        {
            var poly = RandomDomain(0, false);

            var mesh = (Mesh)poly.Triangulate();

            var voronoi = new BoundedVoronoi(mesh);

            AssertCellsPartitionDomain(voronoi, 64.0, new Rectangle(-1e-9, -1e-9, 8 + 2e-9, 8 + 2e-9));
        }

        #endregion

        #region StandardVoronoi

        [Test]
        public void StandardVoronoi_FewPoints()
        {
            var poly = new Polygon();
            poly.Add(new Vertex(0, 0));
            poly.Add(new Vertex(1, 0));
            poly.Add(new Vertex(0, 1));

            var mesh = (Mesh)poly.Triangulate();

            var voronoi = new StandardVoronoi(mesh);

            Assert.That(voronoi.Faces.Count, Is.EqualTo(3));
            Assert.That(voronoi.Vertices.Count, Is.EqualTo(4));
            Assert.That(voronoi.IsConsistent(false), Is.True);
            Assert.That(voronoi.Faces.All(f => !f.Bounded), Is.True);
        }

        [Test]
        public void StandardVoronoi_RaysEndOnBoundingBox()
        {
            var random = new Random(3);
            var poly = new Polygon();
            for (int i = 0; i < 30; i++)
            {
                poly.Add(new Vertex(random.NextDouble() * 10, random.NextDouble() * 10));
            }

            var mesh = (Mesh)poly.Triangulate();

            var box = new Rectangle(-100, -100, 200, 200);

            var voronoi = new StandardVoronoi(mesh, box);

            // All circumcenters of a 10x10 point set with reasonable distribution lie
            // in the large box, so every infinite vertex must be on the box boundary.
            int n = mesh.Triangles.Count;
            foreach (var v in voronoi.Vertices.Where(v => v.ID >= n))
            {
                bool onBox = Math.Abs(v.X + 100) < 1e-9 || Math.Abs(v.X - 100) < 1e-9 ||
                             Math.Abs(v.Y + 100) < 1e-9 || Math.Abs(v.Y - 100) < 1e-9;
                Assert.That(onBox, Is.True, $"Ray end ({v.X}, {v.Y}) not on box.");
            }
        }

        [Test]
        [Description("StandardVoronoi(mesh, box) calls box.Expand(mesh.bounds) on the caller's Rectangle instance, silently modifying the argument.")]
        public void StandardVoronoi_DoesNotModifyBoxArgument()
        {
            var mesh = QualityMesh(Square(10));

            var box = new Rectangle(2, 2, 3, 3);

            new StandardVoronoi(mesh, box);

            Assert.That((box.Left, box.Bottom, box.Right, box.Top), Is.EqualTo((2.0, 2.0, 5.0, 5.0)));
        }

        #endregion

        #region Legacy Voronoi

        private static Mesh MeshWithDuplicate()
        {
            var poly = Square(10);
            poly.Add(new Vertex(3, 3, 0));
            poly.Add(new Vertex(7, 6, 0));
            poly.Add(new Vertex(3, 3, 0)); // duplicate

            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true });

            Assert.That(mesh.Vertices.Count(v => v.Type == VertexType.UndeadVertex), Is.EqualTo(1));

            return mesh;
        }

        [Test]
        public void StandardVoronoi_WithDuplicateVertex()
        {
            var mesh = MeshWithDuplicate();

            var voronoi = new StandardVoronoi(mesh);

            Assert.That(voronoi.Faces.Count, Is.EqualTo(mesh.Vertices.Count - 1));
        }

        [Test]
        [Description("SimpleVoronoi (legacy) creates a region for undead (duplicate) vertices and walks their stale vertex.tri, which throws or produces garbage; VoronoiBase skips undead vertices.")]
        public void SimpleVoronoi_WithDuplicateVertex()
        {
            var mesh = MeshWithDuplicate();

            SimpleVoronoi voronoi = null;

            Assert.DoesNotThrow(() => voronoi = new SimpleVoronoi(mesh));

            Assert.That(voronoi.Regions.Count, Is.EqualTo(mesh.Vertices.Count - 1));
        }

        [Test]
        [Description("BoundedVoronoiLegacy constructs a cell for undead (duplicate) vertices, whose vertex.tri is stale, and throws.")]
        public void BoundedVoronoiLegacy_WithDuplicateVertex()
        {
            var mesh = MeshWithDuplicate();

#pragma warning disable CS0618
            Assert.DoesNotThrow(() => new BoundedVoronoiLegacy(mesh));
#pragma warning restore CS0618
        }

        [Test]
        [Description("SimpleVoronoi uses the bounding box of the circumcenters (enlarged by 10%) to clip rays. With a single triangle the box degenerates to a point and all rays get length zero.")]
        public void SimpleVoronoi_SingleTriangleRaysHaveLength()
        {
            var poly = new Polygon();
            poly.Add(new Vertex(0, 0));
            poly.Add(new Vertex(1, 0));
            poly.Add(new Vertex(0, 1));

            var mesh = (Mesh)poly.Triangulate();

            var voronoi = new SimpleVoronoi(mesh);

            var c = voronoi.Points[0];

            for (int i = 1; i < voronoi.Points.Length; i++)
            {
                var p = voronoi.Points[i];
                Assert.That(p, Is.Not.Null);
                Assert.That(Math.Abs(p.X - c.X) + Math.Abs(p.Y - c.Y), Is.GreaterThan(0.0),
                    "Ray endpoint coincides with the circumcenter.");
            }
        }

        #endregion

        #region SimpleSmoother

        [Test]
        [Description("SimpleSmoother copies its internal TrianglePool into the smoothed mesh (CopyTo). Reusing the smoother instance restarts that pool and destroys the triangles of the previously smoothed mesh.")]
        public void SimpleSmoother_ReuseDoesNotCorruptPreviousMesh()
        {
            var mesh1 = QualityMesh(Square(10));
            var mesh2 = QualityMesh(Square(5), 0.5);

            var smoother = new SimpleSmoother();

            smoother.Smooth(mesh1, 5);

            int count = mesh1.Triangles.Count;
            double area = MeshArea(mesh1);

            MeshAssert.IsValid(mesh1);

            smoother.Smooth(mesh2, 5);

            Assert.That(mesh1.Triangles.Count, Is.EqualTo(count), "Triangles of first mesh changed.");
            Assert.That(MeshArea(mesh1), Is.EqualTo(area).Within(1e-9));
            MeshAssert.IsValid(mesh1);
        }

        private class CancelOnResetFactory : IVoronoiFactory
        {
            private readonly IVoronoiFactory inner = new DefaultVoronoiFactory();
            private readonly CancellationTokenSource cts;

            public CancelOnResetFactory(CancellationTokenSource cts) => this.cts = cts;

            public void Initialize(int vertexCount, int edgeCount, int faceCount) => inner.Initialize(vertexCount, edgeCount, faceCount);
            public void Reset() => cts.Cancel();
            public TriangleNet.Topology.DCEL.Vertex CreateVertex(double x, double y) => inner.CreateVertex(x, y);
            public HalfEdge CreateHalfEdge(TriangleNet.Topology.DCEL.Vertex origin, Face face) => inner.CreateHalfEdge(origin, face);
            public Face CreateFace(Vertex vertex) => inner.CreateFace(vertex);
        }

        [Test]
        [Description("SimpleSmoother moves the Vertex objects of the input mesh in place (Step on the input mesh, and Rebuild reuses the vertex objects), but only replaces the topology at the end. A cancellation (or exception) leaves the input mesh with moved vertices and stale topology.")]
        public void SimpleSmoother_CancellationLeavesInputMeshUnchanged()
        {
            var mesh = QualityMesh(Square(10));

            var before = mesh.Vertices.ToDictionary(v => v, v => (v.X, v.Y), ReferenceEqualityComparer.Instance);

            var cts = new CancellationTokenSource();
            var smoother = new SimpleSmoother(new CancelOnResetFactory(cts));

            Assert.Throws<OperationCanceledException>(() => smoother.Smooth(mesh, 10, 1e-6, cts.Token));

            int moved = mesh.Vertices.Count(v => before[v] != (v.X, v.Y));

            Assert.That(moved, Is.EqualTo(0), "Input mesh vertices were moved although smoothing was cancelled.");
        }

        private static Polygon ScaledSquareWithPoints(double s)
        {
            var poly = Square(10 * s);
            var random = new Random(1);
            for (int i = 0; i < 15; i++)
            {
                poly.Add(new Vertex((1 + 8 * random.NextDouble()) * s, (1 + 8 * random.NextDouble()) * s, 0));
            }
            return poly;
        }

        private static double FirstStepMaxMove(double s)
        {
            var mesh = (Mesh)ScaledSquareWithPoints(s).Triangulate(new ConstraintOptions { ConformingDelaunay = true });
            var before = mesh.Vertices.Where(v => v.Label == 0).Select(v => (v, v.X, v.Y)).ToList();
            new SimpleSmoother().Smooth(mesh, 1);
            return before.Max(t => Math.Sqrt((t.v.X - t.X) * (t.v.X - t.X) + (t.v.Y - t.Y) * (t.v.Y - t.Y)));
        }

        [Test]
        [Description("SimpleSmoother.Smooth initializes currMax = 1, so after the first iteration the convergence test compares the first movement against the arbitrary constant 1 (absolute units). If the first maximum movement is ~1 the smoother stops after one iteration; the result depends on the coordinate scale.")]
        public void SimpleSmoother_IterationCountIsScaleInvariant()
        {
            // Determine the scale at which the first step moves a vertex by exactly 1.
            double m = FirstStepMaxMove(1.0);
            double s = 1.0 / m;

            Assert.That(FirstStepMaxMove(s), Is.EqualTo(1.0).Within(1e-3));

            var mesh1 = (Mesh)ScaledSquareWithPoints(1.0).Triangulate(new ConstraintOptions { ConformingDelaunay = true });
            var mesh2 = (Mesh)ScaledSquareWithPoints(s).Triangulate(new ConstraintOptions { ConformingDelaunay = true });

            int n1 = new SimpleSmoother().Smooth(mesh1, 20, 0.01);
            int n2 = new SimpleSmoother().Smooth(mesh2, 20, 0.01);

            Assert.That(n1, Is.GreaterThan(1));
            Assert.That(n2, Is.EqualTo(n1), "Iteration count depends on coordinate scale.");
        }

        [Test]
        public void SimpleSmoother_PreservesHoleSegmentsAndRegions()
        {
            var poly = SquareWithHole();
            poly.Add(new Segment(new Vertex(1, 1, 3), new Vertex(3, 1, 3), 3), true);
            poly.Regions.Add(new RegionPointer(8, 8, 7));

            var mesh = QualityMesh(poly);

            var smoother = new SimpleSmoother();
            Assert.That(smoother.Smooth(mesh, 10), Is.GreaterThan(0));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 96.0, 1e-9);
            MeshAssert.ContainsSegments(mesh, poly.Segments);
            Assert.That(mesh.Triangles.All(t => t.Label == 7), Is.True, "Region label lost.");
        }

        #endregion

        #region CuthillMcKee / AdjacencyMatrix

        [Test]
        [Description("CuthillMcKee.Rcm does not order newly numbered neighbors by increasing degree: it reads deg[node - 1] (1-based leftover of the Burkardt port, nodes are 0-based) and the insertion sort loop 'while (fnbr < l)' never compares with the first neighbor (SPARSPAK: L >= FNBR). FindRoot has the same off-by-one (pcol[node - 1]).")]
        public void CuthillMcKee_NeighborsOrderedByDegree()
        {
            var poly = Square(10);
            var mesh = QualityMesh(poly, 0.5);

            var matrix = new AdjacencyMatrix(mesh);
            int n = matrix.ColumnCount;

            var pcol = matrix.ColumnPointers;
            var irow = matrix.RowIndices;

            List<int> Neighbors(int i) => Enumerable.Range(pcol[i], pcol[i + 1] - pcol[i]).Select(j => irow[j]).Where(j => j != i).ToList();

            var perm_inv = new CuthillMcKee().Renumber(matrix);

            // Cuthill-McKee order (old node ids, new number ascending) is the reverse of RCM.
            var rcm = new int[n];
            for (int i = 0; i < n; i++) rcm[perm_inv[i]] = i;
            var cm = rcm.Reverse().ToArray();

            var placed = new bool[n];
            placed[cm[0]] = true;
            int next = 1;

            int violations = 0;

            for (int i = 0; i < n; i++)
            {
                var node = cm[i];
                var block = Neighbors(node).Where(j => !placed[j]).ToList();

                for (int k = 0; k < block.Count; k++)
                {
                    int v = cm[next + k];
                    Assert.That(block, Does.Contain(v), "Not a breadth-first ordering.");
                    if (k > 0 && Neighbors(cm[next + k - 1]).Count > Neighbors(v).Count) violations++;
                }

                foreach (var v in block) placed[v] = true;
                next += block.Count;
            }

            Assert.That(violations, Is.EqualTo(0), "Neighbors not sorted by increasing degree.");
        }

        [Test]
        [Description("CuthillMcKee.Rcm reads deg[nbr - 1]; with an adjacency matrix whose row indices are not sorted, node 0 can appear at a non-first neighbor position and deg[-1] throws IndexOutOfRangeException.")]
        public void CuthillMcKee_UnsortedAdjacencyMatrix()
        {
            // Star graph: center 1 connected to 0, 2, 3, 4 (plus diagonal entries).
            // Row of node 1 is deliberately unsorted: 0 is not the first unnumbered neighbor.
            var rows = new[]
            {
                new[] { 0, 1 },
                new[] { 1, 3, 4, 0, 2 },
                new[] { 2, 1 },
                new[] { 3, 1 },
                new[] { 4, 1 },
            };

            var pcol = new int[rows.Length + 1];
            for (int i = 0; i < rows.Length; i++) pcol[i + 1] = pcol[i] + rows[i].Length;
            var irow = rows.SelectMany(r => r).ToArray();

            var matrix = new AdjacencyMatrix(pcol, irow);

            int[] perm = null;
            Assert.DoesNotThrow(() => perm = new CuthillMcKee().Renumber(matrix));
            Assert.That(perm.OrderBy(i => i), Is.EqualTo(Enumerable.Range(0, 5)));
        }

        [Test]
        public void CuthillMcKee_DisconnectedMesh()
        {
            // Two squares separated by a hole region: the mesh has two components.
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0, 1), new Vertex(3, 0, 1), new Vertex(3, 1, 1), new Vertex(0, 1, 1) }, 1));
            poly.Add(new Contour(new[] { new Vertex(5, 0, 1), new Vertex(8, 0, 1), new Vertex(8, 1, 1), new Vertex(5, 1, 1) }, 1));

            var mesh = (Mesh)poly.Triangulate(new QualityOptions { MaximumArea = 0.1 });

            var matrix = new AdjacencyMatrix(mesh);
            var perm = new CuthillMcKee().Renumber(matrix);

            Assert.That(perm.OrderBy(i => i), Is.EqualTo(Enumerable.Range(0, matrix.ColumnCount)));

            mesh.Renumber(NodeNumbering.CuthillMcKee);
            MeshAssert.IsValid(mesh);
        }

        #endregion

        #region TriangleQuadTree

        [Test]
        [Description("TriangleQuadTree.IsPointInTriangle uses inexact barycentric coordinates (divisions). A point on (or within rounding distance of) an interior edge can be rejected by both adjacent triangles, so Query returns null for a point inside the mesh.")]
        public void TriangleQuadTree_QueryOnInteriorEdges()
        {
            var random = new Random(7);
            var poly = new Polygon();
            for (int i = 0; i < 200; i++)
            {
                poly.Add(new Vertex(random.NextDouble(), random.NextDouble()));
            }

            var mesh = (Mesh)poly.Triangulate();

            mesh.Renumber();
            var verts = mesh.Vertices.ToDictionary(v => v.ID);

            var tree = new TriangleQuadTree(mesh);

            int misses = 0, total = 0;

            foreach (var e in mesh.Edges)
            {
                var p = verts[e.P0];
                var q = verts[e.P1];

                if (e.Label != 0) continue; // interior edges only (hull edges have label 1)

                for (int k = 1; k < 10; k++)
                {
                    double t = k / 10.0;
                    double x = p.X + t * (q.X - p.X);
                    double y = p.Y + t * (q.Y - p.Y);

                    total++;

                    if (tree.Query(x, y) == null && ContainedExactly(mesh, x, y)) misses++;
                }
            }

            Assert.That(total, Is.GreaterThan(500));
            Assert.That(misses, Is.EqualTo(0), $"{misses} of {total} points on interior edges not found.");
        }

        private static bool ContainedExactly(Mesh mesh, double x, double y)
        {
            var p = new Point(x, y);
            var pred = RobustPredicates.Default;
            return mesh.Triangles.Any(t =>
                pred.CounterClockwise(t.GetVertex(0), t.GetVertex(1), p) >= 0 &&
                pred.CounterClockwise(t.GetVertex(1), t.GetVertex(2), p) >= 0 &&
                pred.CounterClockwise(t.GetVertex(2), t.GetVertex(0), p) >= 0);
        }

        [Test]
        public void TriangleQuadTree_QueryVerticesAndOutside()
        {
            var mesh = QualityMesh(Square(10), 0.5);

            var tree = new TriangleQuadTree(mesh);

            foreach (var v in mesh.Vertices)
            {
                var tri = tree.Query(v.X, v.Y);
                Assert.That(tri, Is.Not.Null, $"Vertex ({v.X}, {v.Y}) not found.");
                Assert.That(Enumerable.Range(0, 3).Any(i => tri.GetVertex(i) == v), Is.True);
            }

            Assert.That(tree.Query(-1, 5), Is.Null);
            Assert.That(tree.Query(5, 10.5), Is.Null);
            Assert.That(tree.Query(4.3, 6.1), Is.Not.Null);
        }

        #endregion

        #region PolygonValidator

        [Test]
        [Description("PolygonValidator.IsConsistent counts a null point as an error but then dereferences points[0].id / points[1].id and throws NullReferenceException instead of returning false.")]
        public void PolygonValidator_NullPoint()
        {
            var poly = new Polygon();
            poly.Points.Add(null);
            poly.Points.Add(new Vertex(1, 0));
            poly.Points.Add(new Vertex(0, 1));

            bool result = true;
            Assert.DoesNotThrow(() => result = PolygonValidator.IsConsistent(poly));
            Assert.That(result, Is.False);
        }

        [Test]
        [Description("PolygonValidator.IsConsistent only checks segment endpoint ids when all point ids are equal (then the check is trivially satisfied); with distinct ids, a segment endpoint id outside [0, count) is not detected.")]
        public void PolygonValidator_SegmentEndpointNotInPointList()
        {
            var poly = new Polygon();
            var p = new[] { new Vertex(0, 0) { ID = 0 }, new Vertex(1, 0) { ID = 1 }, new Vertex(0, 1) { ID = 2 } };
            poly.Points.AddRange(p);

            var foreign = new Vertex(5, 5) { ID = 17 };
            poly.Add(new Segment(p[0], foreign), false);

            Assert.That(PolygonValidator.IsConsistent(poly), Is.False);
        }

        [Test]
        public void PolygonValidator_DetectsDuplicates()
        {
            var poly = new Polygon();
            for (int i = 0; i < 50; i++) poly.Add(new Vertex(i % 7, i / 7));
            Assert.That(PolygonValidator.HasDuplicateVertices(poly), Is.False);

            poly.Add(new Vertex(3, 2));
            Assert.That(PolygonValidator.HasDuplicateVertices(poly), Is.True);
        }

        #endregion

        #region IntersectionHelper

        [Test]
        [Description("IntersectionHelper.IsPointOnSegment uses an absolute threshold on the cross product (not scaled by segment length): for small segments, points far away (relative to the segment length) are reported as on the segment.")]
        public void IsPointOnSegment_SmallScale()
        {
            var a = new Point(0, 0);
            var b = new Point(1e-6, 0);

            // Distance to the segment equals the segment length.
            var test = new Point(5e-7, 1e-6);

            Assert.That(IntersectionHelper.IsPointOnSegment(a, b, test), Is.False);
        }

        [Test]
        public void IntersectSegments_HandComputed()
        {
            var c = new Point();
            IntersectionHelper.IntersectSegments(new Point(0, 0), new Point(4, 4), new Point(0, 4), new Point(4, 0), ref c);
            Assert.That(c.X, Is.EqualTo(2).Within(1e-12));
            Assert.That(c.Y, Is.EqualTo(2).Within(1e-12));
        }

        [Test]
        public void LiangBarsky_ClipsAndRejects()
        {
            var rect = new Rectangle(0, 0, 10, 10);
            Point c0 = new Point(), c1 = new Point();

            Assert.That(IntersectionHelper.LiangBarsky(rect, new Point(-5, 5), new Point(15, 5), ref c0, ref c1), Is.True);
            Assert.That((c0.X, c0.Y, c1.X, c1.Y), Is.EqualTo((0.0, 5.0, 10.0, 5.0)));

            // Vertical line outside.
            Assert.That(IntersectionHelper.LiangBarsky(rect, new Point(11, 0), new Point(11, 10), ref c0, ref c1), Is.False);
        }

        [Test]
        [Description("IntersectionHelper.BoxRayIntersection with a zero direction vector returns true and sets the intersection to (0, 0), which is unrelated to the box or the start point.")]
        public void BoxRayIntersection_ZeroDirection()
        {
            var rect = new Rectangle(10, 10, 10, 10);
            var c = new Point();

            bool ok = IntersectionHelper.BoxRayIntersection(rect, new Point(15, 15), 0, 0, ref c);

            Assert.That(!ok || rect.Contains(c), Is.True, $"Returned ({c.X}, {c.Y}) outside the box.");
        }

        #endregion

        #region QualityMeasure / Statistic / Interpolation / VertexSorter

        private static Mesh RightTriangle345()
        {
            var poly = new Polygon();
            poly.Add(new Vertex(0, 0));
            poly.Add(new Vertex(4, 0));
            poly.Add(new Vertex(0, 3));
            return (Mesh)poly.Triangulate();
        }

        [Test]
        public void QualityMeasure_HandComputed345()
        {
            var mesh = RightTriangle345();
            var q = new QualityMeasure();
            q.Update(mesh);

            double minAngle = Math.Asin(3.0 / 5.0);

            Assert.That(q.AreaTotal, Is.EqualTo(6.0).Within(1e-12));
            Assert.That(q.Alpha.Minimum, Is.EqualTo(minAngle * 3 / Math.PI).Within(1e-12));
            Assert.That(q.Eta.Minimum, Is.EqualTo(4 * Math.Sqrt(3) * 6 / (9 + 16 + 25)).Within(1e-12));
            // (b+c-a)(c+a-b)(a+b-c)/(abc) with a=3, b=4, c=5
            Assert.That(q.Q.Minimum, Is.EqualTo(6.0 * 4.0 * 2.0 / 60.0).Within(1e-12));
        }

        [Test]
        public void Statistic_HandComputed345()
        {
            var mesh = RightTriangle345();
            var s = new Statistic();
            s.Update(mesh, 10);

            Assert.That(s.SmallestAngle, Is.EqualTo(Math.Asin(3.0 / 5.0) * 180 / Math.PI).Within(1e-9));
            Assert.That(s.LargestAngle, Is.EqualTo(90.0).Within(1e-9));
            Assert.That(s.ShortestEdge, Is.EqualTo(3.0).Within(1e-12));
            Assert.That(s.LongestEdge, Is.EqualTo(5.0).Within(1e-12));
            Assert.That(s.SmallestArea, Is.EqualTo(6.0).Within(1e-12));
            Assert.That(s.ShortestAltitude, Is.EqualTo(2.4).Within(1e-12));
            Assert.That(s.LargestAspectRatio, Is.EqualTo(5.0 / 2.4).Within(1e-12));

            Assert.That(s.AngleHistogram.Sum(), Is.EqualTo(3));
            Assert.That(s.MinAngleHistogram.Sum(), Is.EqualTo(1));
            Assert.That(s.MaxAngleHistogram.Sum(), Is.EqualTo(1));
            // 36.87 deg -> bin 12 (36-39), 53.13 -> bin 17 (51-54), 90 -> bin 30 (90-93) or 29.
            Assert.That(s.MinAngleHistogram[12], Is.EqualTo(1));
            Assert.That(s.AngleHistogram[12], Is.EqualTo(1));
            Assert.That(s.AngleHistogram[17], Is.EqualTo(1));
        }

        [Test]
        public void Interpolation_ReproducesLinearFunction()
        {
            var poly = new Polygon();
            var random = new Random(5);
            for (int i = 0; i < 30; i++) poly.Add(new Vertex(random.NextDouble() * 10, random.NextDouble() * 10));
            var mesh = (Mesh)poly.Triangulate();
            mesh.Renumber();

            double F(double x, double y) => 2 * x - 3 * y + 1;

            var data = new double[mesh.Vertices.Count];
            foreach (var v in mesh.Vertices) data[v.ID] = F(v.X, v.Y);

            foreach (var t in mesh.Triangles)
            {
                var a = t.GetVertex(0); var b = t.GetVertex(1); var c = t.GetVertex(2);
                var p = new Point(0.2 * a.X + 0.3 * b.X + 0.5 * c.X, 0.2 * a.Y + 0.3 * b.Y + 0.5 * c.Y);
                Assert.That(Interpolation.InterpolatePoint(t, p, data), Is.EqualTo(F(p.X, p.Y)).Within(1e-9));
            }
        }

        [Test]
        public void VertexSorter_SortWithManyDuplicates()
        {
            var random = new Random(11);
            var array = Enumerable.Range(0, 2000).Select(i => new Vertex(random.Next(5), random.Next(3))).ToArray();

            VertexSorter.Sort(array);

            for (int i = 1; i < array.Length; i++)
            {
                var p = array[i - 1];
                var q = array[i];
                Assert.That(p.X < q.X || (p.X == q.X && p.Y <= q.Y), Is.True, $"Not sorted at {i}.");
            }
        }

        #endregion
    }
}
