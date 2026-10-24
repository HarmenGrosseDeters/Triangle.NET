using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Meshing;

namespace TriangleNet.Tests.Robustness
{
    /// <summary>
    /// Merging of nearly coincident lines: a segment which runs along another segment at a
    /// distance below the wedge tolerance (e.g. a building wall digitized slightly off the
    /// parcel boundary) forms a thin wedge, which refinement can't fix (tiny input angle).
    /// </summary>
    public class WedgeMergeTest
    {
        private static Polygon Wedge(double offset)
        {
            // Parcel boundary (0,0)-(20,0) and a wall starting on it, ending 'offset' above it.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, -10, 20), false);
            poly.Add(new Segment(new Vertex(5, 0), new Vertex(15, offset)), true);
            poly.Add(new Segment(new Vertex(15, offset), new Vertex(15, 5)), true);
            poly.Add(new Segment(new Vertex(0, 0), new Vertex(20, 0)), true);

            return poly;
        }

        private static int BadTriangles(IMesh mesh, double angle) =>
            mesh.Triangles.Count(t => Characterization.MeshInvariants.MinimumAngle(t) < angle - 0.01);

        [Test]
        public void ThinWedgeIsMerged()
        {
            var poly = Wedge(0.025);

            var options = new ConstraintOptions { SnapTolerance = 0.01, WedgeTolerance = 0.03 };
            var mesh = new GenericMesher().Triangulate(poly, options, new QualityOptions { MinimumAngle = 30 });

            MeshAssert.IsValid(mesh);

            // The wall now runs along the boundary: no small input angle remains.
            Assert.That(BadTriangles(mesh, 30), Is.EqualTo(0));
            Assert.That(mesh.Vertices.Any(v => v.X == 15 && v.Y == 0.025), Is.True);
        }

        [Test]
        public void WithoutWedgeToleranceTheWedgeRemains()
        {
            var poly = Wedge(0.025);

            var mesh = new GenericMesher().Triangulate(poly, new ConstraintOptions { SnapTolerance = 0.01 },
                new QualityOptions { MinimumAngle = 30 });

            MeshAssert.IsValid(mesh);

            Assert.That(BadTriangles(mesh, 30), Is.GreaterThan(0), "test case isn't critical");
        }

        [Test]
        public void WideAnglesAreNotMerged()
        {
            // A short segment at a large angle within the wedge tolerance is a small feature,
            // not a nearly coincident line: it is kept.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, -10, 20), false);
            poly.Add(new Segment(new Vertex(0, 0), new Vertex(20, 0)), true);
            poly.Add(new Segment(new Vertex(5, 0), new Vertex(5.02, 0.025)), true);
            poly.Add(new Segment(new Vertex(5.02, 0.025), new Vertex(8, 5)), true);

            var noded = SegmentNoder.Node(poly, new RobustPredicates(), out _, minTolerance: 0.01, wedgeTolerance: 0.03);

            // No segment was bent through (5.02, 0.025).
            Assert.That(noded.Segments.Count(s => s.GetVertex(0).X == 5.02 || s.GetVertex(1).X == 5.02), Is.EqualTo(2));
        }

        [Test]
        public void FarSegmentsAreNotMerged()
        {
            var poly = Wedge(0.05);

            var noded = SegmentNoder.Node(poly, new RobustPredicates(), out _, minTolerance: 0.01, wedgeTolerance: 0.03);

            // The boundary isn't bent through the end of the wall.
            Assert.That(noded.Segments.Count(s => s.GetVertex(0).Y == 0.05 || s.GetVertex(1).Y == 0.05), Is.EqualTo(2));
        }

        [Test]
        public void WedgeChainsAreMerged()
        {
            // A polyline drifting slowly away from the boundary (all vertices within the
            // wedge tolerance) is merged completely.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, -10, 20), false);
            poly.Add(new Segment(new Vertex(0, 0), new Vertex(20, 0)), true);

            Vertex prev = new Vertex(2, 0);

            for (int i = 1; i <= 8; i++)
            {
                var v = new Vertex(2 + 2 * i, 0.003 * i);
                poly.Add(new Segment(prev, v), true);
                prev = v;
            }

            poly.Add(new Segment(prev, new Vertex(18, 6)), true);

            var mesh = new GenericMesher().Triangulate(poly, new ConstraintOptions { SnapTolerance = 0.01, WedgeTolerance = 0.03 },
                new QualityOptions { MinimumAngle = 30 });

            MeshAssert.IsValid(mesh);

            Assert.That(BadTriangles(mesh, 30), Is.EqualTo(0));
        }

        [Test]
        public void MergingMovesSegmentsAtMostTheWedgeTolerance()
        {
            // A polyline drifting away from the boundary beyond the wedge tolerance: merging
            // vertex by vertex must not drag the boundary along further than the tolerance.
            var poly = new Polygon();
            poly.Add(TestData.Square(0, -10, 20), false);
            poly.Add(new Segment(new Vertex(0, 0), new Vertex(20, 0)), true);

            Vertex prev = new Vertex(2, 0);

            for (int i = 1; i <= 8; i++)
            {
                var v = new Vertex(2 + 2 * i, 0.0075 * i);
                poly.Add(new Segment(prev, v), true);
                prev = v;
            }

            var noded = SegmentNoder.Node(poly, new RobustPredicates(), out _, minTolerance: 0.01, wedgeTolerance: 0.03);

            // The boundary from (0,0) to (20,0) still exists as a chain of segments through
            // vertices within the tolerance (merging didn't drag it towards the polyline).
            var reached = new HashSet<(double, double)> { (0, 0) };
            bool grown = true;

            while (grown)
            {
                grown = false;

                foreach (var s in noded.Segments)
                {
                    var p = (s.GetVertex(0).X, s.GetVertex(0).Y);
                    var q = (s.GetVertex(1).X, s.GetVertex(1).Y);

                    if (Math.Abs(p.Item2) > 0.03 + 1e-12 || Math.Abs(q.Item2) > 0.03 + 1e-12) continue;

                    if (reached.Contains(p) && reached.Add(q) || reached.Contains(q) && reached.Add(p))
                    {
                        grown = true;
                    }
                }
            }

            Assert.That(reached.Contains((20, 0)), Is.True);
        }
    }
}
