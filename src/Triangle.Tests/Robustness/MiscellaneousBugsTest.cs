using NUnit.Framework;
using System;
using System.Collections.Generic;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;

namespace TriangleNet.Tests.Robustness
{
    public class MiscellaneousBugsTest
    {
        [Test, Category(KnownBug.Category)]
        [Description("Severity: high. SimpleSmoother computes cell centroids with absolute coordinates (catastrophic cancellation far from the origin), producing invalid vertices.")]
        public void SmoothMeshWithLargeCoordinates()
        {
            IsolatedProcess.AssertSucceeds(nameof(IsolatedScenarios.SmoothMeshWithLargeCoordinates), TimeSpan.FromSeconds(15));
        }

        [Test]
        public void SmoothMeshNearOrigin()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            poly.Add(new Segment(new Vertex(1, 1), new Vertex(9, 1.000001)), true);

            var mesh = poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true }, new QualityOptions { MinimumAngle = 25 });

            MeshAssert.Terminates(() => new TriangleNet.Smoothing.SimpleSmoother().Smooth(mesh, 10));
            MeshAssert.IsValid(mesh);
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: low. SweepLine output isn't Delaunay for nearly cocircular points (violations of order 1e-16), because the event queue uses inexact circle tops.")]
        public void SweepLineNearlyCocircularPoints([Range(1, 5)] int seed)
        {
            var random = new Random(seed);
            var points = new List<Vertex>();

            for (int i = 0; i < 10; i++)
            {
                double a = random.NextDouble() * 2 * Math.PI;
                points.Add(new Vertex(Math.Cos(a), Math.Sin(a)));
            }

            var mesh = (Mesh)new SweepLine().Triangulate(points, new Configuration());

            MeshAssert.IsValid(mesh);
            Assert.That(MeshValidator.IsDelaunay(mesh), Is.True);
        }

        [Test]
        public void NearlyCocircularPoints([Values(0.0, 1e6)] double offset)
        {
            foreach (var triangulator in new ITriangulator[] { new Dwyer(), new Incremental() })
            {
                var random = new Random(1);
                var points = new List<Vertex>();

                for (int i = 0; i < 500; i++)
                {
                    double a = random.NextDouble() * 2 * Math.PI;
                    points.Add(new Vertex(offset + Math.Cos(a), offset + Math.Sin(a)));
                }

                var mesh = (Mesh)triangulator.Triangulate(points, new Configuration());

                MeshAssert.IsValid(mesh);
                Assert.That(MeshValidator.IsDelaunay(mesh), Is.True, triangulator.GetType().Name);
            }
        }

        [Test, Category(KnownBug.Category)]
        [Description("Severity: medium. Point.Equals compares coordinates, but Vertex.GetHashCode returns the internal hash, so equal vertices have different hash codes (breaks HashSet/Dictionary).")]
        public void VertexEqualsAndHashCodeContract()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            poly.Add(new Vertex(0, 0));

            poly.Triangulate();

            var a = poly.Points[0];
            var b = poly.Points[4];

            Assert.That(a.Equals(b), Is.True);
            Assert.That(a.GetHashCode(), Is.EqualTo(b.GetHashCode()));
        }
    }
}
