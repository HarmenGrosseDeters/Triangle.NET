using System;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;
using TriangleNet.Meshing.Algorithm;
using TriangleNet.Smoothing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Scenarios executed in a child process by <see cref="IsolatedProcess"/>, because they
    /// crash the process or don't terminate. Each scenario must throw if the result is wrong.
    /// </summary>
    public static class IsolatedScenarios
    {
        /// <summary>
        /// Dwyer: all input points identical.
        /// </summary>
        public static void DwyerAllPointsIdentical()
        {
            var points = Enumerable.Range(0, 5).Select(_ => new Vertex(1, 1)).ToList();

            var mesh = new Dwyer().Triangulate(points, new Configuration());

            Check(mesh.Triangles.Count == 0, "Expected empty mesh.");
        }

        /// <summary>
        /// Five segments with integer coordinates (found by fuzzing).
        /// </summary>
        public static void FiveCrossingSegments()
        {
            var poly = TestData.SquareWithSegments(
                (9, 3, 2, 0), (7, 2, 1, 8), (7, 3, 3, 6), (10, 6, 9, 1), (10, 10, 8, 2));

            var mesh = poly.Triangulate();

            Check(MeshValidator.IsConsistent((Mesh)mesh), "Mesh is inconsistent.");
        }

        /// <summary>
        /// Smoothing a mesh located far away from the origin (e.g. UTM coordinates).
        /// </summary>
        public static void SmoothMeshWithLargeCoordinates()
        {
            const double offset = 1e6;

            var poly = new Polygon();
            poly.Add(TestData.Square(offset, offset, 10));
            poly.Add(new Segment(new Vertex(offset + 1, offset + 1), new Vertex(offset + 9, offset + 1.000001)), true);

            var mesh = poly.Triangulate(new ConstraintOptions { ConformingDelaunay = true }, new QualityOptions { MinimumAngle = 25 });

            new SimpleSmoother().Smooth(mesh, 10);

            Check(MeshValidator.IsConsistent((Mesh)mesh), "Mesh is inconsistent.");
            Check(mesh.Vertices.All(v => double.IsFinite(v.X) && double.IsFinite(v.Y)), "Mesh contains non-finite vertices.");
        }

        private static void Check(bool condition, string message)
        {
            if (!condition) throw new Exception(message);
        }
    }
}
