using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Characterization
{
    /// <summary>
    /// Every scenario of <see cref="MeshCases"/> must produce a valid mesh: consistent,
    /// no inverted triangles, (constrained) Delaunay, area preserved, all input segments
    /// present, locked segments unsplit, minimum angle and maximum area met where guaranteed.
    /// </summary>
    [Category("Characterization")]
    public class InvariantTest
    {
        public static IEnumerable<TestCaseData> Cases() =>
            MeshCases.All().Select(c => new TestCaseData(c).SetName("Invariants(" + c.Name + ")"));

        [TestCaseSource(nameof(Cases))]
        public void Invariants(MeshCases.Case c)
        {
            var input = c.Input();
            var mesh = MeshAssert.Terminates(() => c.Run(input), TimeSpan.FromSeconds(120));

            // Reference area: plain triangulation of the same input.
            var reference = c.Input();
            var plain = reference.Segments.Count == 0
                ? (Mesh)new GenericMesher().Triangulate(reference.Points)
                : (Mesh)reference.Triangulate(new ConstraintOptions { SnapTolerance = c.Snap });
            double area = plain.Triangles.Sum(t => MeshInvariants.Area(t));

            var errors = MeshInvariants.Check(mesh, input, c, area);

            Assert.That(errors, Is.Empty, string.Join(", ", errors));
        }
    }
}
