using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.IO;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Review
{
    /// <summary>
    /// Review of ConstraintMesher, SegmentNoder, GenericMesher, Converter and the triangulators.
    /// </summary>
    public class ReviewCTest
    {
        #region Confirmed bugs

        [Test]
        [Description("Severity: medium. SegmentNoder creates intersection vertices with ID 0. With two or more " +
            "intersections the IDs are no longer pairwise distinct, so Mesh.TransferNodes silently renumbers ALL " +
            "vertices and the user-defined IDs are lost (only because of the default noding).")]
        public void NodingPreservesUserVertexIds()
        {
            var poly = new Polygon();
            int id = 1;
            var square = new[] { new Vertex(0, 0), new Vertex(10, 0), new Vertex(10, 10), new Vertex(0, 10) };
            foreach (var v in square) v.ID = id++;
            poly.Add(new Contour(square));

            // Three segments with three crossings.
            foreach (var (x1, y1, x2, y2) in new[] { (1.0, 1.0, 9.0, 9.0), (1.0, 9.0, 9.0, 1.0), (1.0, 7.0, 9.0, 7.0) })
            {
                poly.Add(new Segment(new Vertex(x1, y1) { ID = id++ }, new Vertex(x2, y2) { ID = id++ }), true);
            }

            // Reference: without noding the IDs are kept.
            var reference = poly.Triangulate(new ConstraintOptions { ResolveIntersections = false });
            Assert.That(reference.Vertices.First(v => v.X == 10 && v.Y == 10).ID, Is.EqualTo(3));

            var mesh = poly.Triangulate();

            Assert.That(mesh.Vertices.First(v => v.X == 10 && v.Y == 10).ID, Is.EqualTo(3),
                "User-defined vertex IDs were renumbered.");
        }

        [Test]
        [Description("Severity: low-medium. When split segments coincide, SegmentNoder.Split keeps the first piece and " +
            "drops the others including their label. The mesher (InsertSubseg) lets a non-zero label replace label 0, " +
            "so the result depends on noding: label 7 without noding, label 0 with noding (matters for CanSplitSegment).")]
        public void NodingKeepsNonZeroLabelOfDuplicateSegment()
        {
            Polygon Input()
            {
                var poly = new Polygon();
                poly.Add(TestData.Square(0, 0, 10));
                poly.Add(new Segment(new Vertex(1, 5), new Vertex(9, 5), 0), true);
                poly.Add(new Segment(new Vertex(1, 5), new Vertex(9, 5), 7), true);
                poly.Add(new Segment(new Vertex(5, 1), new Vertex(5, 9), 3), true);
                return poly;
            }

            int[] Labels(IMesh mesh) => mesh.Segments
                .Where(s => s.GetVertex(0).Y == 5 && s.GetVertex(1).Y == 5).Select(s => s.Label).ToArray();

            var reference = Labels(Input().Triangulate(new ConstraintOptions { ResolveIntersections = false }));
            Assert.That(reference, Is.All.EqualTo(7));

            Assert.That(Labels(Input().Triangulate()), Is.All.EqualTo(7));
        }

        [Test]
        [Description("Severity: medium. If SnapTolerance is larger than a hole, the noder collapses the hole contour " +
            "but keeps the hole point. The hole point then infects the whole surrounding domain and the mesh is " +
            "empty (area 0 instead of ~100), although SnapTolerance promises geometry changes of at most the tolerance.")]
        public void SnapToleranceCollapsingHoleKeepsDomain()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(5, 5, 0.01), true);

            var mesh = poly.Triangulate(new ConstraintOptions { SnapTolerance = 0.1 });

            Assert.That(mesh.Triangles.Sum(MeshAssert.Area), Is.GreaterThan(99.0));
        }

        [Test]
        [Description("Severity: medium. Converter.ToDCEL maps each boundary vertex to ONE outgoing boundary half-edge " +
            "(Dictionary.Add). At a non-manifold boundary vertex (hole touching the outer boundary, two parts touching " +
            "at a vertex) it throws ArgumentException 'same key'.")]
        public void ToDcelWithHoleTouchingBoundary()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Contour(new[] { new Vertex(5, 0), new Vertex(7, 3), new Vertex(3, 3) }), true);

            var mesh = (Mesh)poly.Triangulate();
            MeshAssert.IsValid(mesh);

            var dcel = Converter.Instance.ToDCEL(mesh);

            Assert.That(dcel.IsConsistent(), Is.True);
        }

        [Test]
        [Description("Severity: low-medium (performance). SegmentNoder.FindCrossings sweeps along x only, without an " +
            "active set ordered by y: every segment is compared with all segments overlapping its x-range. For long " +
            "segments sharing an x-range (parallel stripes) noding is O(n^2): 20,000 stripes take ~5 s, the mesher ~0.4 s.")]
        public void NodingParallelStripesIsNotQuadratic()
        {
            const int n = 20000;

            var poly = new Polygon();

            for (int i = 0; i < n; i++)
            {
                poly.Add(new Segment(new Vertex(0, i), new Vertex(1000, i)), true);
            }

            var sw = Stopwatch.StartNew();
            poly.Triangulate(new ConstraintOptions { ResolveIntersections = false });
            long meshing = sw.ElapsedMilliseconds;

            sw.Restart();
            SegmentNoder.Node(poly, RobustPredicates.Default);
            long noding = sw.ElapsedMilliseconds;

            Assert.That(noding, Is.LessThan(Math.Max(1000, 3 * meshing)),
                $"Noding took {noding} ms, meshing (without noding) {meshing} ms.");
        }

        [Test]
        [Description("Severity: medium. Converter.ToMesh looks up mesh.vertices (keyed by vertex hash) and the vertex " +
            "array with the triangle/segment vertex ids. If the input vertices have distinct user IDs that aren't " +
            "0..n-1 (e.g. database IDs), Mesh uses them as hashes and ToMesh throws KeyNotFoundException.")]
        public void ToMeshWithDistinctUserIds()
        {
            var poly = new Polygon();
            var pts = new[] { new Vertex(0, 0), new Vertex(1, 0), new Vertex(1, 1), new Vertex(0, 1) };
            for (int i = 0; i < 4; i++) pts[i].ID = 100 + i;
            poly.Points.AddRange(pts);
            poly.Segments.Add(new Segment(pts[0], pts[1], 3));

            var triangles = new ITriangle[] { new InputTriangle(0, 1, 2), new InputTriangle(0, 2, 3) };

            Mesh mesh = null;
            Assert.DoesNotThrow(() => mesh = Converter.Instance.ToMesh(poly, triangles));
            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count, Is.EqualTo(2));
        }

        [Test]
        [Description("Severity: low. Two region pointers in the same area: RegionIterator.Process spreads the label/area " +
            "of the seed triangle, which the first region already overwrote in the second region's seed triangle. " +
            "So the FIRST region wins, while in Triangle (C) the last one wins (regionplague uses the region list values).")]
        public void OverlappingRegionPointersLastOneWins()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Regions.Add(new RegionPointer(2, 2, 1, 0.5));
            poly.Regions.Add(new RegionPointer(8, 8, 2, 2.0));

            var mesh = poly.Triangulate();

            Assert.That(mesh.Triangles.Select(t => t.Label).Distinct(), Is.EqualTo(new[] { 2 }));
            Assert.That(mesh.Triangles.Select(t => t.Area).Distinct(), Is.EqualTo(new[] { 2.0 }));
        }

        [Test]
        [Description("Severity: low. SegmentNoder.MergeVertices replaces a vertex by a nearby one (snapping) without " +
            "transferring its label, so a vertex marker (here 9) disappears from the mesh.")]
        public void SnapMergedVertexKeepsLabel()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Segment(new Vertex(1, 5), new Vertex(5, 5, 0)), true);
            poly.Add(new Segment(new Vertex(5 + 1e-3, 5, 9), new Vertex(9, 5)), true);

            var mesh = poly.Triangulate(new ConstraintOptions { SnapTolerance = 0.01 });

            var merged = mesh.Vertices.Where(v => Math.Abs(v.X - 5) < 0.01 && v.Y == 5).ToList();
            Assert.That(merged.Count, Is.EqualTo(1));
            Assert.That(merged[0].Label, Is.EqualTo(9));
        }

        #endregion

        #region Regression tests (suspicions refuted)

        static List<string> DelaunayErrors(IMesh mesh)
        {
            var errors = new List<string>();

            foreach (var t in mesh.Triangles)
            {
                var a = t.GetVertex(0); var b = t.GetVertex(1); var c = t.GetVertex(2);

                if (RobustPredicates.Default.CounterClockwise(a, b, c) <= 0) errors.Add("inverted " + t.ID);

                for (int i = 0; i < 3; i++)
                {
                    var n = t.GetNeighbor(i);
                    if (n == null) continue;

                    for (int j = 0; j < 3; j++)
                    {
                        var v = n.GetVertex(j);
                        if (v == a || v == b || v == c) continue;
                        if (RobustPredicates.Default.InCircle(a, b, c, v) > 0) errors.Add($"non-Delaunay {t.ID}/{n.ID}");
                    }
                }
            }

            return errors;
        }

        static IEnumerable<TestCaseData> PointSets()
        {
            var r = new Random(1);
            yield return new TestCaseData((object)Enumerable.Range(0, 200).Select(_ => (r.NextDouble(), r.NextDouble())).ToArray()).SetName("{m}(random)");
            yield return new TestCaseData((object)(from i in Enumerable.Range(0, 12) from j in Enumerable.Range(0, 9) select ((double)i, (double)j)).ToArray()).SetName("{m}(grid)");
            yield return new TestCaseData((object)(from i in Enumerable.Range(0, 3) from j in Enumerable.Range(0, 30) select (i * 10.0, (double)j)).ToArray()).SetName("{m}(vertical-lines)");
            yield return new TestCaseData((object)(from i in Enumerable.Range(0, 3) from j in Enumerable.Range(0, 30) select ((double)j, i * 10.0)).ToArray()).SetName("{m}(horizontal-lines)");
            yield return new TestCaseData((object)Enumerable.Range(0, 50).Select(i => ((double)i, 0.0)).Append((25.0, 1.0)).ToArray()).SetName("{m}(collinear+1)");
            yield return new TestCaseData((object)Enumerable.Range(0, 50).Select(i => ((double)i, 2.0 * i)).Append((25.0, 0.0)).ToArray()).SetName("{m}(diagonal+1)");
            yield return new TestCaseData((object)Enumerable.Range(0, 60).Select(i => ((double)(i % 7), (double)((i * 3) % 5))).ToArray()).SetName("{m}(duplicates)");
            yield return new TestCaseData((object)Enumerable.Range(0, 64).Select(i => (Math.Cos(i * Math.PI / 32), Math.Sin(i * Math.PI / 32))).Append((0.0, 0.0)).ToArray()).SetName("{m}(circle)");
            yield return new TestCaseData((object)Enumerable.Range(0, 100).Select(i => (i * 1000.0, (i % 2) * 1e-3)).ToArray()).SetName("{m}(thin)");
            yield return new TestCaseData((object)Enumerable.Range(0, 200).Select(_ => (380000 + r.NextDouble() * 10, 5710000 + r.NextDouble() * 10)).ToArray()).SetName("{m}(utm)");
            yield return new TestCaseData((object)Enumerable.Range(0, 100).Select(i => ((double)i, -1e-6 * (i - 50) * (i - 50))).ToArray()).SetName("{m}(flat-arc)");
        }

        [TestCaseSource(nameof(PointSets))]
        public void AllTriangulatorsAgree((double, double)[] points)
        {
            var results = new List<(int, int, int)>();

            foreach (var alg in new ITriangulator[] { new Dwyer(), new Dwyer { UseDwyer = false }, new SweepLine(), new Incremental() })
            {
                var mesh = (Mesh)alg.Triangulate(points.Select(p => new Vertex(p.Item1, p.Item2)).ToList(), new Configuration());

                MeshAssert.IsValid(mesh);
                Assert.That(DelaunayErrors(mesh), Is.Empty, alg.GetType().Name);

                results.Add((mesh.Triangles.Count, mesh.hullsize, mesh.undeads));
            }

            Assert.That(results.Distinct().Count(), Is.EqualTo(1), string.Join(" ", results));
        }

        [TestCase(new double[] { 0, 0, 1, 1, 2, 2, 3, 3, 5, 5 }, 0)]
        [TestCase(new double[] { 0, 0, 0, 1, 0, 2, 0, 3 }, 0)]
        [TestCase(new double[] { 0, 0, 1, 0, 0, 0, 1, 0 }, 0)]
        [TestCase(new double[] { 0, 0, 1, 0, 1, 0, 0, 1, 0, 0 }, 1)]
        public void DegenerateInputAllTriangulators(double[] c, int triangles)
        {
            foreach (var alg in new ITriangulator[] { new Dwyer(), new Dwyer { UseDwyer = false }, new SweepLine(), new Incremental() })
            {
                var pts = new List<Vertex>();
                for (int i = 0; i < c.Length; i += 2) pts.Add(new Vertex(c[i], c[i + 1]));

                var mesh = alg.Triangulate(pts, new Configuration());

                Assert.That(mesh.Triangles.Count, Is.EqualTo(triangles), alg.GetType().Name);
            }
        }

        [Test]
        public void StructuredMeshIsCorrect()
        {
            var mesh = (Mesh)GenericMesher.StructuredMesh(new Rectangle(-3, 2, 4, 3), 4, 3);

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count, Is.EqualTo(24));
            Assert.That(mesh.Vertices.Count, Is.EqualTo(20));
            Assert.That(mesh.Segments.Count, Is.EqualTo(14));
            Assert.That(mesh.Segments.Select(s => s.Label), Is.All.EqualTo(1));
            Assert.That(mesh.Vertices.Count(v => v.Label == 1), Is.EqualTo(14));
            MeshAssert.HasArea(mesh, 12.0);
            Assert.That(DelaunayErrors(mesh), Is.Empty);

            Assert.That(Converter.Instance.ToDCEL(mesh).IsConsistent(), Is.True);

            mesh.Refine(new QualityOptions { MaximumArea = 0.1, MinimumAngle = 25 });

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 12.0);
        }

        [Test]
        public void ZeroLengthSegmentsWithCrossings()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            var v = new Vertex(3, 3);
            poly.Add(new Segment(v, v, 4), true);
            poly.Add(new Segment(new Vertex(5, 5), new Vertex(5, 5), 4), true);
            poly.Add(new Segment(new Vertex(1, 1), new Vertex(9, 9)), true);
            poly.Add(new Segment(new Vertex(1, 9), new Vertex(9, 1)), true);
            poly.Add(new Segment(new Vertex(2, 2), new Vertex(2, 2 + 1e-14)), true);

            var mesh = poly.Triangulate();

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100.0);
        }

        [Test]
        public void HoleInHoleAndHolesOutsideDomain()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(TestData.Square(2, 2, 6), new Point(3, 3)); // hole, area 36
            poly.Add(TestData.Square(4, 4, 2), new Point(5, 5)); // hole inside the hole
            poly.Holes.Add(new Point(20, 20));                   // outside the bounds
            poly.Regions.Add(new RegionPointer(4.5, 5.5, 3));    // region inside a hole
            poly.Regions.Add(new RegionPointer(1, 1, 5));

            foreach (var alg in new ITriangulator[] { new Dwyer(), new SweepLine(), new Incremental() })
            {
                var mesh = new GenericMesher(alg).Triangulate(poly, new ConstraintOptions(), new QualityOptions { MinimumAngle = 20 });

                MeshAssert.IsValid(mesh);
                MeshAssert.HasArea(mesh, 64.0);
                Assert.That(mesh.Triangles.Select(t => t.Label), Is.All.EqualTo(5));
            }
        }

        #endregion
    }
}
