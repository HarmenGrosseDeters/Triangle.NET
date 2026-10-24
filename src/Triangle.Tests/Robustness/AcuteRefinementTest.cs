using NUnit.Framework;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Tests.Characterization;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Bugs of the aCute refinement (default since 2022, <see cref="QualityOptions.UseLegacyRefinement"/>
    /// = false). All of them exist in upstream Triangle.NET.
    /// </summary>
    public class AcuteRefinementTest
    {
        private static Polygon Square(double size = 10)
        {
            var poly = new Polygon();
            poly.Add(new Contour(new[] { new Vertex(0, 0), new Vertex(size, 0), new Vertex(size, size), new Vertex(0, size) }, 1));
            return poly;
        }

        /// <summary>
        /// The aCute location doesn't necessarily destroy the bad triangle (it may lie
        /// outside the circumcircle). The triangle was then dropped from the queue and stayed
        /// in the mesh: upstream leaves 9 triangles larger than the maximum area here.
        /// </summary>
        [Test]
        public void MaximumAreaIsRespected([Values(0, 20, 30)] double minAngle, [Values(false, true)] bool legacy)
        {
            var mesh = Square().Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = minAngle, MaximumArea = 0.5, UseLegacyRefinement = legacy });

            MeshAssert.IsValid(mesh);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.Area(t) > 0.5), Is.Zero);
        }

        /// <summary>
        /// Same cause as <see cref="MaximumAreaIsRespected"/>, for the angle: on real parcel
        /// data (ALKIS Herne) upstream leaves 618 of 2061 triangles below 30°.
        /// </summary>
        [Test]
        public void BadTrianglesAreNotDropped()
        {
            var c = MeshCases.All().First(x => x.Name == "herne_381000_5710400/q30-snap1cm");
            var mesh = c.Run(c.Input());

            MeshAssert.IsValid(mesh);

            // Remaining bad triangles are at small input angles (few).
            int bad = mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < 30 - 1e-6);
            Assert.That(bad, Is.LessThan(20));
        }

        /// <summary>
        /// Area constraints of regions: aCute relocated (deleted and re-inserted) the same
        /// vertex back and forth. Upstream throws "The new vertex is at the circumcenter of
        /// triangle", with the first fixes the refinement cycled until the Steiner point
        /// limit and returned a coarse mesh (5 triangles).
        /// </summary>
        [Test]
        public void RegionAreaConstraint([Values(false, true)] bool diagonal)
        {
            var poly = Square();

            if (diagonal)
            {
                poly.Add(new Segment(poly.Points[0], poly.Points[2], 3));
            }

            poly.Regions.Add(new RegionPointer(7, 2, 1, 0.1));
            poly.Regions.Add(new RegionPointer(2, 7, 2, 2.0));

            var mesh = MeshAssert.Terminates(() => poly.Triangulate(new ConstraintOptions(),
                new QualityOptions { MinimumAngle = 25, VariableArea = true, SteinerPoints = 100000 }));

            MeshAssert.IsValid(mesh);

            double limit = diagonal ? 0.1 : 2.0;

            Assert.That(mesh.Triangles.Where(t => !diagonal || t.Label == 1).Count(t => MeshInvariants.Area(t) > limit * (1 + 1e-9)), Is.Zero);
            Assert.That(mesh.Triangles.Count(t => MeshInvariants.MinimumAngle(t) < 25 - 1e-6), Is.Zero);
            Assert.That(mesh.Vertices.Count, Is.LessThan(5000));
        }
    }
}
