using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Invalid or unusual input should either work or be rejected with a meaningful exception.
    /// </summary>
    public class InputValidationTest
    {
        [Test, Category(KnownBug.Category)]
        [Description("Severity: medium. NaN coordinates cause an IndexOutOfRangeException deep inside RobustPredicates.")]
        public void NaNCoordinate()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Vertex(double.NaN, 0.5));

            Assert.Throws<ArgumentException>(() => poly.Triangulate());
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: medium. Infinite coordinates cause an IndexOutOfRangeException deep inside RobustPredicates.")]
        public void InfiniteCoordinate()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Vertex(double.PositiveInfinity, 0.5));

            Assert.Throws<ArgumentException>(() => poly.Triangulate());
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: medium. Mesh.TransferNodes assumes all ids are distinct if the first two are. Otherwise: 'An item with the same key has already been added'.")]
        public void PartiallyDuplicateUserIds()
        {
            var points = new List<Vertex>
            {
                new Vertex(0, 0) { ID = 1 },
                new Vertex(1, 0) { ID = 2 },
                new Vertex(1, 1) { ID = 2 },
                new Vertex(0, 1) { ID = 3 }
            };

            var mesh = new Dwyer().Triangulate(points, new Configuration());

            MeshAssert.IsValid(mesh);
            MeshAssert.HasArea(mesh, 1.0);
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: medium. Re-triangulating the vertices of an existing mesh plus a new vertex fails, because the new vertex has id 0 which is already in use.")]
        public void ReuseMeshVerticesAndAddVertex()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));

            var points = poly.Triangulate().Vertices.ToList();

            points.Add(new Vertex(0.5, 0.5));

            var mesh = new Dwyer().Triangulate(points, new Configuration());

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Vertices.Count, Is.EqualTo(5));
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: high. Dwyer (the default triangulator) recurses infinitely if all points are identical (StackOverflowException kills the process).")]
        public void AllPointsIdenticalDwyer()
        {
            IsolatedProcess.AssertSucceeds(nameof(IsolatedScenarios.DwyerAllPointsIdentical));
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: low. SweepLine throws if all points are identical, while Incremental returns an empty mesh (and all algorithms return an empty mesh for collinear input).")]
        public void AllPointsIdenticalSweepLine()
        {
            var points = Enumerable.Range(0, 5).Select(_ => new Vertex(1, 1)).ToList();

            var mesh = new SweepLine().Triangulate(points, new Configuration());

            Assert.That(mesh.Triangles.Count, Is.Zero);
        }

        [Test]
        public void AllPointsIdenticalIncremental()
        {
            var points = Enumerable.Range(0, 5).Select(_ => new Vertex(1, 1)).ToList();

            var mesh = new Incremental().Triangulate(points, new Configuration());

            Assert.That(mesh.Triangles.Count, Is.Zero);
        }

        [Test]
        public void AllPointsCollinear([ValueSource(nameof(Triangulators))] ITriangulator triangulator)
        {
            var points = Enumerable.Range(0, 10).Select(i => new Vertex(i, 2 * i)).ToList();

            var mesh = triangulator.Triangulate(points, new Configuration());

            Assert.That(mesh.Triangles.Count, Is.Zero);
        }

        [Test]
        public void AllPointsCollinearPolygon()
        {
            var poly = new Polygon();
            poly.Add(new Contour(Enumerable.Range(0, 5).Select(i => new Vertex(i, i))));

            Assert.That(poly.Triangulate().Triangles.Count, Is.Zero);
        }

        [Test]
        public void SegmentEndpointsNotInPointList()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Segment(new Vertex(0.2, 0.2), new Vertex(0.8, 0.8)), false);

            // Documents current behavior: a generic exception with a cryptic message.
            var e = Assert.Throws<Exception>(() => poly.Triangulate());
            Assert.That(e.Message, Does.Contain("Unable to locate PSLG vertex"));
        }

        [Test]
        public void PolygonTriangulatedTwice()
        {
            // The mesher modifies the input vertices (id, type, triangle reference).
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Segment(new Vertex(0.2, 0.2), new Vertex(0.8, 0.8)), true);
            poly.Add(new Vertex(0.2, 0.2));

            var quality = new QualityOptions { MinimumAngle = 25 };

            var mesh1 = poly.Triangulate(quality);
            var mesh2 = poly.Triangulate(quality);

            MeshAssert.IsValid(mesh2);
            MeshAssert.ContainsSegments(mesh2, poly.Segments);
            Assert.That(mesh2.Triangles.Count, Is.EqualTo(mesh1.Triangles.Count));
        }

        private static IEnumerable<ITriangulator> Triangulators()
        {
            yield return new Dwyer();
            yield return new SweepLine();
            yield return new Incremental();
        }
    }
}
