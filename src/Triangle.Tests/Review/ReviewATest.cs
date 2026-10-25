using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Tests.Robustness;
using TriangleNet.Topology;

namespace TriangleNet.Tests.Review
{
    /// <summary>
    /// Review of Mesh, TriangleLocator, TrianglePool, TriangleSampler, Behavior,
    /// Configuration, MeshValidator and IMesh.
    /// </summary>
    public class ReviewATest
    {
        #region TrianglePool

        [Test]
        [Description("TrianglePool.Contains checks 'i > size' instead of 'i >= size'. For a triangle whose hash equals the pool size (e.g. a triangle of another mesh) it reads an empty pool slot and throws a NullReferenceException instead of returning false.")]
        public void PoolContainsTriangleWithHashEqualToSize()
        {
            var pool = new TrianglePool();
            for (int i = 0; i < 10; i++) pool.Get();

            var other = new TrianglePool();
            Triangle foreign = default;
            for (int i = 0; i < 11; i++) foreign = other.Get(); // hash == 10

            Assert.That(foreign.hash, Is.EqualTo(pool.Capacity));
            Assert.That(() => pool.Contains(foreign), Throws.Nothing);
            Assert.That(pool.Contains(foreign), Is.False);
        }

        [Test]
        [Description("TrianglePool.Sample draws indices only from [0, Count) where Count is the number of LIVE triangles, but live triangles are spread over [0, count+free). If all slots below Count are free, the loop never finds a live triangle and hangs (also: live triangles at higher slots are never sampled). Fix: draw from the number of used slots (count), and bound the number of attempts.")]
        public void PoolSampleWithFreeTrianglesAtLowSlots()
        {
            var pool = new TrianglePool();
            var tris = new List<Triangle>();
            for (int i = 0; i < 10; i++) tris.Add(pool.Get());

            // Free slots 1..5: Count == 5, live triangles are in slots 0, 6, 7, 8, 9.
            // (If slot 0 were freed too, Sample would loop forever.)
            for (int i = 1; i <= 5; i++) pool.Release(tris[i]);

            Assert.That(pool.Count, Is.EqualTo(5));

            var sample = MeshAssert.Terminates(() => pool.Sample(200, new Random(0)).Concat(pool.Sample(200, new Random(1))).ToList(),
                TimeSpan.FromSeconds(3));

            Assert.That(sample.All(t => t.hash >= 0), Is.True);
            Assert.That(sample.Any(t => t.hash >= 6), Is.True,
                "Live triangles in slots >= Count are never sampled (all-free low slots make Sample hang).");
        }

        [Test]
        [Description("TrianglePool.Clear computes the number of triangles per block as (size - i*BLOCKSIZE) % BLOCKSIZE, which is size % BLOCKSIZE for every full block. Full blocks are not cleared, so the pool keeps references to all old triangles (memory is never released). Fix: Math.Min(BLOCKSIZE, size - i*BLOCKSIZE).")]
        public void PoolClearReleasesAllTriangles()
        {
            var pool = new TrianglePool();
            for (int i = 0; i < 1024 + 5; i++) pool.Get();

            pool.Clear();

            // No references to vertices or subsegments of the old triangles remain.
            int remaining = pool.vertices.Count(v => v != null) + pool.subsegs.Count(s => s.seg != null);

            Assert.That(pool.Count, Is.EqualTo(0));
            Assert.That(remaining, Is.EqualTo(0), "Pool still references triangles after Clear().");
        }

        [Test]
        public void PoolEnumerationSkipsReleasedTriangles()
        {
            var pool = new TrianglePool();
            var tris = new List<Triangle>();
            for (int i = 0; i < 2100; i++) tris.Add(pool.Get());
            for (int i = 0; i < 2100; i += 3) pool.Release(tris[i]);

            var live = pool.ToList();

            Assert.That(live.Count, Is.EqualTo(pool.Count));
            Assert.That(live.All(t => t.hash >= 0), Is.True);
            Assert.That(live.Distinct().Count(), Is.EqualTo(live.Count));

            // Reuse of released triangles keeps the enumeration consistent.
            for (int i = 0; i < 100; i++) pool.Get();
            Assert.That(pool.Count(), Is.EqualTo(pool.Count));
        }

        #endregion

        #region Renumber

        [Test]
        [Description("Mesh.Renumber(CuthillMcKee) applies 'node.id = iperm[node.id]' to all vertices, but AdjacencyMatrix gives undead (duplicate) vertices the negative id -i. Any undead vertex after the first live one causes an IndexOutOfRangeException. Fix: skip undead vertices in Mesh.Renumber (and give them an id outside the permutation range).")]
        public void RenumberCuthillMcKeeWithDuplicateVertex()
        {
            var points = new List<Vertex>
            {
                new Vertex(0, 0), new Vertex(1, 0), new Vertex(1, 1), new Vertex(0, 1),
                new Vertex(0.5, 0.5), new Vertex(0.5, 0.5) // duplicate -> undead
            };

            var mesh = (Mesh)new GenericMesher().Triangulate(points);

            Assert.That(mesh.Vertices.Count(v => v.Type == VertexType.UndeadVertex), Is.EqualTo(1));
            Assert.That(() => mesh.Renumber(NodeNumbering.CuthillMcKee), Throws.Nothing);

            var ids = mesh.Vertices.Where(v => v.Type != VertexType.UndeadVertex).Select(v => v.ID).ToList();
            Assert.That(ids.Distinct().Count(), Is.EqualTo(ids.Count));
            Assert.That(ids.All(id => id >= 0 && id < ids.Count), Is.True);
        }

        private static Mesh RenumberedThenRefined()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(4, 4, 2), true);

            var mesh = (Mesh)poly.Triangulate();

            // E.g. to write the mesh or to build a Voronoi diagram.
            mesh.Renumber();

            mesh.Refine(new QualityOptions { MinimumAngle = 25, MaximumArea = 0.5 });

            return mesh;
        }

        [Test]
        [Description("Renumber assigns ids 0..n-1 to live triangles only; released triangles keep their old id (== hash), and TrianglePool.Get does not reset the id of a reused triangle. After Renumber + Refine triangle ids are duplicated, so mesh.Edges (EdgeIterator compares ids) skips edges. Fix: set triangle.id = triangle.hash when a triangle is taken from the free stack.")]
        public void EdgesAfterRenumberAndRefine()
        {
            var mesh = RenumberedThenRefined();

            MeshAssert.IsValid(mesh);

            var edges = mesh.Edges.ToList();
            var distinct = new HashSet<(int, int)>();
            int pairs = 0;

            foreach (var t in mesh.Triangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    int a = t.GetVertexID((i + 1) % 3), b = t.GetVertexID((i + 2) % 3);
                    distinct.Add((Math.Min(a, b), Math.Max(a, b)));
                }
            }
            pairs = distinct.Count;

            Assert.That(edges.Count, Is.EqualTo(pairs), "mesh.Edges is missing edges.");
            Assert.That(mesh.NumberOfEdges, Is.EqualTo(pairs));
        }

        [Test]
        [Description("Same root cause as EdgesAfterRenumberAndRefine: MeshValidator.IsDelaunay tests an edge only if tri.id < neighbor.id, so with duplicated ids after Renumber + Refine some edges are never checked (false negatives).")]
        public void TriangleIdsUniqueAfterRenumberAndRefine()
        {
            var mesh = RenumberedThenRefined();

            var ids = mesh.Triangles.Select(t => t.ID).ToList();

            Assert.That(ids.Distinct().Count(), Is.EqualTo(ids.Count), "Duplicate triangle ids.");
        }

        [Test]
        public void RenumberLinearIsContiguous()
        {
            var mesh = RenumberedThenRefined();

            mesh.Renumber();

            Assert.That(mesh.Vertices.Select(v => v.ID).OrderBy(i => i), Is.EqualTo(Enumerable.Range(0, mesh.Vertices.Count)));
            Assert.That(mesh.Triangles.Select(t => t.ID).OrderBy(i => i), Is.EqualTo(Enumerable.Range(0, mesh.Triangles.Count)));
            Assert.That(mesh.Edges.Count(), Is.EqualTo(mesh.NumberOfEdges));
        }

        #endregion

        #region Mesh state / API

        [Test]
        public void NumberOfEdgesMatchesEdgesWithHolesAndRefinement()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(2, 2, 2), true);
            poly.Add(new Contour(new[] { new Vertex(6, 6), new Vertex(9, 6), new Vertex(6, 9) }), true);

            var mesh = (Mesh)poly.Triangulate(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.3 });

            Assert.That(mesh.Edges.Count(), Is.EqualTo(mesh.NumberOfEdges));

            mesh.Refine(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.1 });

            Assert.That(mesh.Edges.Count(), Is.EqualTo(mesh.NumberOfEdges));
            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100 - 4 - 4.5);
        }

        [Test]
        [Description("For collinear input there are no triangles, but hullsize stays non-zero (RemoveGhosts counts ghost edges), so NumberOfEdges = (3*0 + hullsize)/2 is positive while Edges is empty. Fix: reset hullsize to 0 when the triangulation is empty (or compute NumberOfEdges only from triangles).")]
        public void NumberOfEdgesForCollinearInput([Values(0, 1, 2)] int algorithm)
        {
            var points = new List<Vertex> { new Vertex(0, 0), new Vertex(1, 1), new Vertex(2, 2), new Vertex(3, 3) };

            ITriangulator t = algorithm == 0 ? new Dwyer() : algorithm == 1 ? new SweepLine() : new Incremental();

            var mesh = (Mesh)t.Triangulate(points, new Configuration());

            Assert.That(mesh.Triangles.Count, Is.EqualTo(0));
            Assert.That(mesh.Edges.Count(), Is.EqualTo(0));
            Assert.That(mesh.NumberOfEdges, Is.EqualTo(0));
        }

        [Test]
        [Description("A point-set mesh (GenericMesher.Triangulate(points), no ConstraintMesher) has no hull subsegments. Mesh.Refine then turns on checksegments, and circumcenters outside the convex hull are inserted as if on a boundary edge, so the domain grows beyond the convex hull. Triangle (C) encloses the hull with segments (markhull) before refining. Fix: in Refine, mark the hull when the mesh has no segments.")]
        public void RefinePointSetMeshKeepsConvexHull()
        {
            var random = new Random(3);
            var points = new List<Vertex>();
            for (int i = 0; i < 30; i++) points.Add(new Vertex(random.NextDouble() * 10, random.NextDouble() * 10));

            var mesh = (Mesh)new GenericMesher().Triangulate(points);
            double area = mesh.Triangles.Sum(t => MeshAssert.Area(t));

            MeshAssert.Terminates(() => { mesh.Refine(new QualityOptions { MinimumAngle = 30, SteinerPoints = 5000 }); return 0; });

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, area, 1e-9);
        }

        [Test]
        [Description("Mesh.Refine creates its QualityMesher with 'new Configuration()' if none was set (e.g. meshes from GenericMesher.Triangulate(points), ITriangulator or Converter), so the user's Configuration.Predicates are silently replaced by RobustPredicates.Default. Fix: keep the Configuration passed to the Mesh constructor and use it in Refine.")]
        public void RefineUsesConfiguredPredicates()
        {
            var counting = new CountingPredicates();
            var config = new Configuration(() => counting);

            var random = new Random(1);
            var points = new List<Vertex>();
            for (int i = 0; i < 20; i++) points.Add(new Vertex(random.NextDouble(), random.NextDouble()));

            var mesh = (Mesh)new GenericMesher(config).Triangulate(points);

            int before = counting.Circumcenters;
            // Legacy refinement computes circumcenters with the QualityMesher's predicates.
            mesh.Refine(new QualityOptions { MaximumArea = 0.001, SteinerPoints = 2000, UseLegacyRefinement = true });

            Assert.That(mesh.Triangles.Count, Is.GreaterThan(100));
            Assert.That(counting.Circumcenters, Is.GreaterThan(before), "Refine did not use the configured predicates.");
        }

        [Test]
        public void TriangulatePolygonUsesConfiguredPredicates()
        {
            // Control for RefineUsesConfiguredPredicates: on this path the configured predicates are used.
            var counting = new CountingPredicates();
            var config = new Configuration(() => counting);

            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));

            var mesh = new GenericMesher(config).Triangulate(poly,
                new QualityOptions { MaximumArea = 0.001, SteinerPoints = 2000, UseLegacyRefinement = true });

            Assert.That(mesh.Triangles.Count, Is.GreaterThan(100));
            Assert.That(counting.Circumcenters, Is.GreaterThan(0));
        }

        private class CountingPredicates : IPredicates
        {
            private readonly RobustPredicates inner = new RobustPredicates();
            public int Circumcenters;

            public double CounterClockwise(Point a, Point b, Point c) => inner.CounterClockwise(a, b, c);
            public double InCircle(Point a, Point b, Point c, Point p) => inner.InCircle(a, b, c, p);

            public Point FindCircumcenter(Point org, Point dest, Point apex, ref double xi, ref double eta)
            {
                Interlocked.Increment(ref Circumcenters);
                return inner.FindCircumcenter(org, dest, apex, ref xi, ref eta);
            }

            public Point FindCircumcenter(Point org, Point dest, Point apex, ref double xi, ref double eta, double offconstant)
            {
                Interlocked.Increment(ref Circumcenters);
                return inner.FindCircumcenter(org, dest, apex, ref xi, ref eta, offconstant);
            }
        }

        [Test]
        [Description("Mesh.TransferNodes/MakeVertexMap never reset Vertex.tri. A vertex that was alive in an earlier mesh and is an undead duplicate in a new mesh keeps a handle into the OLD mesh; ConstraintMesher.InsertSegment trusts it (Org() == endpoint) and inserts the segment into the old mesh's triangles. Fix: reset p.tri in TransferNodes (or clear it for all vertices in MakeVertexMap).")]
        public void ReusedVertexObjectsAcrossMeshes()
        {
            var a = new Vertex(0, 0); var b = new Vertex(10, 0); var c = new Vertex(10, 10); var d = new Vertex(0, 10);
            var m = new Vertex(5, 5);

            // First mesh: m is a regular vertex.
            var poly1 = new Polygon();
            poly1.Add(new Contour(new[] { a, b, c, d }));
            poly1.Add(m);
            var mesh1 = (Mesh)poly1.Triangulate();
            MeshAssert.IsValid(mesh1);
            int mesh1Segments = mesh1.Segments.Count;

            // Second mesh: a duplicate of m comes first, so m becomes undead,
            // and m is used as segment endpoint.
            var dup = new Vertex(5, 5);
            var poly2 = new Polygon();
            poly2.Add(dup);
            var a2 = new Vertex(0, 0); var c2 = new Vertex(10, 10);
            poly2.Add(new Contour(new[] { a2, new Vertex(10, 0), c2, new Vertex(0, 10) }));
            poly2.Add(m);
            poly2.Add(new Segment(m, c2));
            poly2.Add(new Segment(a2, m));

            var mesh2 = (Mesh)poly2.Triangulate(new ConstraintOptions { ResolveIntersections = false });

            Assert.That(mesh1.Segments.Count, Is.EqualTo(mesh1Segments), "Old mesh was modified.");
            MeshAssert.IsValid(mesh1);
            MeshAssert.IsValid(mesh2);
            Assert.That(mesh2.Segments.Count, Is.EqualTo(6));
            Assert.That(m.Type, Is.EqualTo(VertexType.UndeadVertex));

            // Every subsegment of mesh2 must be bonded to triangles of mesh2 ...
            var tris2 = new HashSet<Triangle>(mesh2.Triangles);
            foreach (var seg in mesh2.subsegs.Values)
            {
                var t0 = seg.triangles[0].tri;
                var t1 = seg.triangles[1].tri;
                Assert.That((t0.id == Mesh.DUMMY || tris2.Contains(t0)) && (t1.id == Mesh.DUMMY || tris2.Contains(t1)), Is.True,
                    "Subsegment of the new mesh is bonded to a triangle of another mesh.");
            }

            // ... and the old mesh must not reference subsegments it doesn't own.
            var segs1 = new HashSet<SubSegment>(mesh1.subsegs.Values);
            foreach (var t in mesh1.triangles)
            {
                for (int i = 0; i < 3; i++)
                {
                    var s = t.subsegs[i].seg;
                    Assert.That(s.hash == Mesh.DUMMY || segs1.Contains(s), Is.True,
                        "Old mesh references a subsegment of the new mesh.");
                }
            }
        }

        [Test]
        public void BoundsMatchVertices()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(-3, 2, 5));
            poly.Add(new Vertex(1, 3));

            var mesh = poly.Triangulate(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.5 });

            var b = mesh.Bounds;
            Assert.That(b.Left, Is.EqualTo(-3));
            Assert.That(b.Bottom, Is.EqualTo(2));
            Assert.That(b.Right, Is.EqualTo(2));
            Assert.That(b.Top, Is.EqualTo(7));
            Assert.That(mesh.Vertices.All(v => b.Contains(v)), Is.True);
        }

        [Test]
        public void InputWithFewerThanThreePointsThrows()
        {
            var points = new List<Vertex> { new Vertex(0, 0), new Vertex(1, 0) };
            Assert.That(() => new GenericMesher().Triangulate(points), Throws.Exception);
        }

        [Test]
        public void MeshValidatorDetectsInvertedTriangle()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            var mesh = (Mesh)poly.Triangulate();

            Assert.That(MeshValidator.IsConsistent(mesh), Is.True);

            // Swap two vertices of one triangle (inverts it).
            var t = mesh.triangles.First();
            (t.vertices[0], t.vertices[1]) = (t.vertices[1], t.vertices[0]);

            Assert.That(MeshValidator.IsConsistent(mesh), Is.False);
        }

        [Test]
        public void LocateFindsVerticesAndEdges()
        {
            var points = new List<Vertex>();
            for (int i = 0; i < 5; i++)
                for (int j = 0; j < 5; j++)
                    points.Add(new Vertex(i, j));

            var mesh = (Mesh)new GenericMesher().Triangulate(points);

            foreach (var p in points)
            {
                Otri tri = default;
                tri.tri = mesh.dummytri;
                tri.orient = 0;
                tri.Sym();

                Assert.That(mesh.locator.Locate(p, ref tri), Is.EqualTo(LocateResult.OnVertex));
                Assert.That(tri.Org().X == p.X && tri.Org().Y == p.Y, Is.True);
            }

            Otri e = default;
            e.tri = mesh.dummytri; e.orient = 0; e.Sym();
            Assert.That(mesh.locator.Locate(new Point(2.5, 0), ref e), Is.EqualTo(LocateResult.OnEdge));

            e.tri = mesh.dummytri; e.orient = 0; e.Sym();
            Assert.That(mesh.locator.Locate(new Point(2.3, 2.6), ref e), Is.EqualTo(LocateResult.InTriangle));
        }

        #endregion

        #region Behavior

        [Test]
        public void BehaviorDerivedAngles()
        {
            var b = new Behavior();
            b.Quality = true;
            b.MinAngle = 30;
            b.MaxAngle = 140;

            Assert.That(b.goodAngle, Is.EqualTo(Math.Pow(Math.Cos(Math.PI / 6), 2)).Within(1e-15));
            Assert.That(b.maxGoodAngle, Is.EqualTo(Math.Cos(140 * Math.PI / 180)).Within(1e-15));
            Assert.That(b.offconstant, Is.GreaterThan(0));
        }

        [TestCase(0.0, 50.0)]
        [TestCase(70.0, 50.0)]
        [Description("Behavior.Update() sets quality = true and only clears it for an invalid angle. QualityMesher.Apply sets MinAngle, then MaxAngle: an invalid MinimumAngle is reset to 0 and quality is re-enabled by the MaxAngle setter (area constraint applied), but an invalid MaximumAngle disables quality for good, so MaximumArea is silently ignored. Fix: validate in one place; drop only the invalid angle (or throw).")]
        public void InvalidAngleOptionsHandledConsistently(double minAngle, double maxAngle)
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));

            var mesh = poly.Triangulate(new QualityOptions { MinimumAngle = minAngle, MaximumAngle = maxAngle, MaximumArea = 0.01 });

            Assert.That(mesh.Triangles.Max(t => MeshAssert.Area(t)), Is.LessThanOrEqualTo(0.01),
                "Maximum area constraint ignored because of an invalid angle option.");
        }

        #endregion
    }
}
