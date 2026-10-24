using System;
using System.Linq;
using System.Reflection;
using System.Threading;
using NUnit.Framework;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Data;
using TriangleNet.Tests.Characterization;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Review
{
    public class ReviewBTest
    {
        private static Polygon Square(double size = 10)
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(size, 0), new Vertex(size, size), new Vertex(0, size) }, 1));
            return poly;
        }

        private static Polygon SquareWithRandomPoints(int seed, int n, double size = 10)
        {
            var poly = Square(size);
            var rnd = new Random(seed);
            for (int i = 0; i < n; i++)
            {
                poly.Add(new Vertex(size * (0.05 + 0.9 * rnd.NextDouble()), size * (0.05 + 0.9 * rnd.NextDouble())));
            }
            return poly;
        }

        private static double MaximumAngle(ITriangle t)
        {
            var a = t.GetVertex(0); var b = t.GetVertex(1); var c = t.GetVertex(2);
            double A(Point p, Point q, Point r)
            {
                double ux = q.X - p.X, uy = q.Y - p.Y, vx = r.X - p.X, vy = r.Y - p.Y;
                return Math.Abs(Math.Atan2(ux * vy - uy * vx, ux * vx + uy * vy)) * 180 / Math.PI;
            }
            return Math.Max(A(a, b, c), Math.Max(A(b, c, a), A(c, a, b)));
        }

        [Test]
        [Description("NewLocation shares petalx/wedges work arrays between the 'without max angle' (32 doubles per star point) and the max angle " +
            "wedge intersection (40 per point), but only resizes on 2*numpoints > petalx.Length. Refining a mesh again with MaximumAngle " +
            "after a refinement without it reuses the too small wedges array: IndexOutOfRangeException.")]
        public void RefineAgainWithMaximumAngle_DoesNotThrow()
        {
            var mesh = (Mesh)SquareWithRandomPoints(17, 30).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 33, MaximumArea = 0.05 });

            Assert.DoesNotThrow(() => mesh.Refine(new QualityOptions { MinimumAngle = 33, MaximumAngle = 100, MaximumArea = 0.02 }));
            MeshAssert.IsValid(mesh);
        }

        /// <summary>
        /// Control for <see cref="RefineAgainWithMaximumAngle_DoesNotThrow"/>: the same refinement with
        /// a fresh mesher works.
        /// </summary>
        [Test]
        public void RefineWithMaximumAngle_FreshMesher_Works()
        {
            var mesh = (Mesh)SquareWithRandomPoints(17, 30).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 33, MaximumAngle = 100, MaximumArea = 0.02 });

            MeshAssert.IsValid(mesh);
        }

        [Test]
        [Description("QualityMesher.EnforceQuality only runs the bad triangle loop if MinAngle > 0, an area constraint or UserTest is set. " +
            "A MaximumAngle constraint alone is silently ignored (fix: add behavior.MaxAngle > 0 to the condition).")]
        public void MaximumAngleAlone_IsEnforced([Values(false, true)] bool legacy)
        {
            var mesh = SquareWithRandomPoints(1, 40).Triangulate(new ConstraintOptions(),
                new QualityOptions { MaximumAngle = 100, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            double max = mesh.Triangles.Max(MaximumAngle);
            Assert.That(max, Is.LessThanOrEqualTo(100 + 1e-6));
        }

        [Test]
        public void MaximumAngleWithMinimumAngle_IsEnforced([Values(false, true)] bool legacy, [Values(20, 30)] double minAngle)
        {
            var mesh = MeshAssert.Terminates(() => SquareWithRandomPoints(1, 40).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = minAngle, MaximumAngle = 100, UseLegacyRefinement = legacy }));

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MaximumAngle(t) > 100 + 1e-6), Is.Zero, "max angle");
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < minAngle - 1e-6), Is.Zero, "min angle");
        }

        [Test]
        public void NoBisect_AreaConstraintIsRespected([Values(1, 2)] int splitting, [Values(false, true)] bool legacy, [Values(0, 20, 30)] double minAngle)
        {
            var poly = SquareWithRandomPoints(3, 20);
            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SegmentSplitting = splitting },
                new QualityOptions { MinimumAngle = minAngle, MaximumArea = 0.1, UseLegacyRefinement = legacy }));

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.1 * (1 + 1e-9)), Is.Zero);
        }

        [TestCase(0.0)]
        [TestCase(1e-310)]
        [TestCase(double.Epsilon)]
        [TestCase(double.PositiveInfinity)]
        [Description("BadTriQueue.Enqueue computes 1/key for keys < 1. For key 0 or a subnormal key (squared shortest edge < 2.2e-308) " +
            "1/key is infinite, the exponent loop yields 2048 and the queue index 4096 (IndexOutOfRange); key = +inf yields index -1. " +
            "Fix: clamp the queue number to 0..4095 (or treat non-finite lengths explicitly).")]
        public void BadTriQueue_ExtremeKeys(double key)
        {
            var queue = new BadTriQueue();
            Assert.DoesNotThrow(() => queue.Enqueue(new BadTriangle { key = key }));
            Assert.That(queue.Dequeue(), Is.Not.Null);
        }

        [TestCase(double.MaxValue)]
        [TestCase(double.NaN)]
        [TestCase(-1.0)]
        public void BadTriQueue_OtherUnusualKeys(double key)
        {
            var queue = new BadTriQueue();
            Assert.DoesNotThrow(() => queue.Enqueue(new BadTriangle { key = key }));
            Assert.That(queue.Dequeue(), Is.Not.Null);
        }

        [Test]
        public void BadTriQueue_PriorityOrder()
        {
            var queue = new BadTriQueue();
            double[] keys = { 1e10, 1e-200, 1.0, 0.25, 4.0, 1e300, 1e-300 };
            foreach (var k in keys) queue.Enqueue(new BadTriangle { key = k });

            var order = Enumerable.Range(0, keys.Length).Select(_ => queue.Dequeue().key).ToArray();
            Assert.That(order, Is.EqualTo(keys.OrderBy(k => k).ToArray()));
            Assert.That(queue.Dequeue(), Is.Null);
        }

        [Test]
        public void SteinerPointLimitIsRespected([Values(1, 2, 5, 17, 50)] int limit, [Values(false, true)] bool legacy)
        {
            var poly = SquareWithRandomPoints(5, 20);
            poly.Add(new Segment(poly.Points[0], poly.Points[2]));
            var mesh = poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 30, MaximumArea = 0.01, SteinerPoints = limit, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count, Is.LessThanOrEqualTo(24 + limit));
        }

        [Test]
        public void CancelledToken_Throws()
        {
            var mesh = (Mesh)SquareWithRandomPoints(5, 20).Triangulate();
            var cts = new CancellationTokenSource();
            cts.Cancel();
            Assert.Throws<OperationCanceledException>(() => mesh.Refine(new QualityOptions { MinimumAngle = 30, MaximumArea = 0.01 }, false, cts.Token));
        }

        [Test]
        public void HighDegreeInputVertex_Fan([Values(300, 600)] int n, [Values(false, true)] bool legacy)
        {
            var poly = new Polygon();
            var center = new Vertex(0, 0);
            poly.Add(center);
            var ring = new Vertex[n];
            for (int i = 0; i < n; i++)
            {
                double a = 2 * Math.PI * i / n;
                ring[i] = new Vertex(10 * Math.Cos(a), 10 * Math.Sin(a));
            }
            poly.Add(new Contour(ring, 1));
            for (int i = 0; i < n; i += 1)
            {
                poly.Add(new Segment(center, ring[i]));
            }

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, MaximumArea = 1, UseLegacyRefinement = legacy }), TimeSpan.FromSeconds(30));
            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 1 + 1e-9), Is.Zero);
        }

        [Test]
        public void ConformingDelaunayWithQuality_IsDelaunay([Values(false, true)] bool legacy)
        {
            var poly = SquareWithRandomPoints(7, 15);
            poly.Add(new Segment(poly.Points[0], poly.Points[2]));
            poly.Add(new Segment(new Vertex(1, 9), new Vertex(9.5, 1.5)), true);
            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true },
                new QualityOptions { MinimumAngle = 25, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            Assert.That(MeshValidator.IsDelaunay(mesh), Is.True);
        }

        [Test]
        public void HighMinimumAngle_Terminates([Values(33.8, 34.5, 40)] double minAngle, [Values(false, true)] bool legacy)
        {
            var mesh = MeshAssert.Terminates(() => SquareWithRandomPoints(1, 30).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = minAngle, UseLegacyRefinement = legacy, SteinerPoints = 200000 }));
            MeshAssert.IsValid(mesh);
        }

        [Test]
        [Description("Behavior.Update() sets Quality = false for an out-of-range MaximumAngle (< 60 or > 180). QualityMesher.Apply sets " +
            "MaxAngle after Quality = true, so the whole refinement is skipped and the MaximumArea/MinimumAngle constraints are silently " +
            "ignored (only a log warning). An invalid MinimumAngle does not have this effect because the MaxAngle setter re-enables Quality.")]
        public void InvalidMaximumAngle_DoesNotDisableOtherConstraints()
        {
            var mesh = Square().Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 20, MaximumAngle = 50, MaximumArea = 0.5 });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.5), Is.Zero);
        }

        /// <summary>
        /// Control: an invalid minimum angle doesn't disable the area constraint.
        /// </summary>
        [Test]
        public void InvalidMinimumAngle_AreaConstraintStillApplied()
        {
            var mesh = Square().Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 70, MaximumArea = 0.5 });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.5), Is.Zero);
        }

        [Test]
        [Description("QualityMesher.Apply resets VarArea to false if behavior.Poly is false (input without segments, '// TODO: remove'). " +
            "Per-triangle area constraints set by the user on a triangulated point set are silently ignored by Refine(VariableArea = true).")]
        public void VariableArea_PointSetMesh_IsApplied()
        {
            // Random points (decimal grids produce hull points that are collinear in decimal but
            // ~1e-16 off in binary; hull edges then can't be split at their midpoint, which is a
            // precision limit and not what this test is about).
            var random = new Random(3);
            var points = Enumerable.Range(0, 30).Select(i => new Vertex(10 * random.NextDouble(), 10 * random.NextDouble())).ToList();
            var mesh = (Mesh)new GenericMesher().Triangulate(points);

            foreach (var t in mesh.Triangles) t.Area = 0.5;

            mesh.Refine(new QualityOptions { VariableArea = true });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.5 * (1 + 1e-9)), Is.Zero);
        }

        [Test]
        public void RefineAfterSmoothing([Values(false, true)] bool legacy)
        {
            var mesh = (Mesh)SquareWithRandomPoints(9, 20).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, MaximumArea = 0.5, UseLegacyRefinement = legacy });

            new TriangleNet.Smoothing.SimpleSmoother().Smooth(mesh, 5);
            MeshAssert.IsValid(mesh);

            mesh.Refine(new QualityOptions { MinimumAngle = 28, MaximumArea = 0.1, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.1 * (1 + 1e-9)), Is.Zero);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < 28 - 1e-6), Is.Zero);
        }

        [Test]
        public void RefineTwice_SameOptions_IsStable([Values(false, true)] bool legacy)
        {
            var q = new QualityOptions { MinimumAngle = 30, MaximumArea = 0.2, UseLegacyRefinement = legacy };
            var mesh = (Mesh)SquareWithRandomPoints(11, 20).Triangulate(new ConstraintOptions(), q);
            int n = mesh.Vertices.Count;
            mesh.Refine(q);
            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count, Is.EqualTo(n));
        }

        private static Polygon SquareWithInternalSegments(int seed)
        {
            var poly = SquareWithRandomPoints(seed, 10);
            var rnd = new Random(seed + 1000);
            for (int k = 0; k < 3; k++)
            {
                poly.Add(new Segment(new Vertex(1 + 8 * rnd.NextDouble(), 1 + 8 * rnd.NextDouble()),
                    new Vertex(1 + 8 * rnd.NextDouble(), 1 + 8 * rnd.NextDouble())), true);
            }
            return poly;
        }

        [TestCase(1, false, 20, 0, 5)]
        [TestCase(1, true, 20, 0, 17)]
        [TestCase(2, false, 30, 120, 0)]
        [TestCase(1, false, 20, 120, 0)]
        [Description("Mesh.DeleteVertex only re-tests the retriangulated cavity if NoBisect == 0 (TriangulatePolygon(.., triflaws: NoBisect == 0) " +
            "and 'if (NoBisect == 0) TestTriangle'). With SegmentSplitting 1/2, free vertices deleted by SplitEncSegs (Chew) or by aCute " +
            "relocation leave new triangles that are never queued: too large (and bad) triangles remain. Fix: always test them.")]
        public void SegmentSplitting_DeletedVertexCavityIsTested(int splitting, bool legacy, double minAngle, double maxAngle, int seed)
        {
            var poly = SquareWithInternalSegments(seed);
            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions { SegmentSplitting = splitting },
                new QualityOptions { MinimumAngle = minAngle, MaximumAngle = maxAngle, MaximumArea = 0.05, UseLegacyRefinement = legacy }));

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.05 * (1 + 1e-9)), Is.Zero);
        }

        /// <summary>
        /// Control for <see cref="SegmentSplitting_DeletedVertexCavityIsTested"/>: with segment splitting enabled
        /// all constraints are met.
        /// </summary>
        [Test]
        public void InternalSegments_MaximumAngle_AllConstraintsMet([Values(0, 3, 4)] int seed, [Values(false, true)] bool legacy)
        {
            var poly = SquareWithInternalSegments(seed);
            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, MaximumAngle = 120, MaximumArea = 0.05, UseLegacyRefinement = legacy }));

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.05 * (1 + 1e-9)), Is.Zero, "area");
            Assert.That(mesh.Triangles.Count(t => MaximumAngle(t) > 120 + 1e-6), Is.Zero, "max angle");
        }

        [Test]
        public void UserTest_AreaCriterion([Values(false, true)] bool legacy)
        {
            var mesh = SquareWithRandomPoints(2, 10).Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 20, UserTest = (t, area) => area > 0.1, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.1 * (1 + 1e-9)), Is.Zero);
        }

        [Test]
        public void MaximumAngle_WithHole([Values(false, true)] bool legacy)
        {
            var poly = Square();
            poly.Add(new Contour(new[] { new Vertex(3, 3), new Vertex(7, 4), new Vertex(6, 7), new Vertex(4, 6) }, 2), new Point(5, 5));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, MaximumAngle = 110, MaximumArea = 0.5, UseLegacyRefinement = legacy }));

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 100 - 9);
            Assert.That(mesh.Triangles.Count(t => MaximumAngle(t) > 110 + 1e-6), Is.Zero);
        }

        /// <summary>
        /// Legacy refinement (circumcenters/off-centers) with MaximumAngle below ~95 degrees doesn't
        /// terminate within 10 s on this input (aCute does). Not a port bug: Ruppert/Chew refinement has
        /// no max angle guarantee and SteinerPoints is unlimited by default. Kept as documentation.
        /// </summary>
        [Test, Explicit("Limitation: does not terminate (legacy refinement has no max angle guarantee).")]
        public void Legacy_MaximumAngle90_Terminates()
        {
            var poly = SquareWithInternalSegments(3);
            MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 20, MaximumAngle = 90, UseLegacyRefinement = true }));
        }
    }
}
