using NUnit.Framework;
using TriangleNet.Meshing;
using TriangleNet.Tools;

namespace TriangleNet.Tests.Tools
{
    public class CuthillMcKeeTest
    {
        [Test]
        public void TestCuthillMcKee()
        {
            var mesh = (Mesh)GenericMesher.StructuredMesh(4.0, 3.0, 2, 1);

            var matrix = new AdjacencyMatrix(mesh);

            Assert.That(new int[] { 0, 4, 7, 11, 17, 21, 24 }, Is.EqualTo(matrix.ColumnPointers).AsCollection);
            Assert.That(new int[] {
                0, 1, 2, 3,
                0, 1, 3,
                0, 2, 3, 4,
                0, 1, 2, 3, 4, 5,
                2, 3, 4, 5,
                3, 4, 5 }, Is.EqualTo(matrix.RowIndices).AsCollection);

            Assert.That(matrix.Bandwidth(), Is.EqualTo(7));

            var p = new CuthillMcKee().Renumber(matrix);

            foreach (var node in mesh.Vertices)
            {
                node.ID = p[node.ID];
            }

            var pmatrix = new AdjacencyMatrix(mesh, false);

            // The structured mesh is already numbered optimally: RCM keeps the numbering (the
            // old result, with bandwidth 9 instead of 7, was caused by 1-based index leftovers
            // in the neighbor sorting).
            Assert.That(p, Is.EqualTo(new int[] { 0, 1, 2, 3, 4, 5 }).AsCollection);
            Assert.That(new int[] { 0, 4, 7, 11, 17, 21, 24 }, Is.EqualTo(pmatrix.ColumnPointers).AsCollection);
            Assert.That(pmatrix.Bandwidth(), Is.EqualTo(7));
        }
    }
}