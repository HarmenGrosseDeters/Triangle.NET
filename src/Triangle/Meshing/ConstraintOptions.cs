
namespace TriangleNet.Meshing
{
    /// <summary>
    /// Mesh constraint options for polygon triangulation.
    /// </summary>
    public class ConstraintOptions
    {
        // TODO: remove ConstraintOptions.UseRegions

        /// <summary>
        /// Gets or sets a value indicating whether to use regions.
        /// </summary>
        [System.Obsolete("Not used anywhere, will be removed in beta 4.")]
        public bool UseRegions { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether to create a Conforming
        /// Delaunay triangulation.
        /// </summary>
        public bool ConformingDelaunay { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether to enclose the convex
        /// hull with segments.
        /// </summary>
        public bool Convex { get; set; }

        /// <summary>
        /// Gets or sets a flag indicating whether to suppress boundary
        /// segment splitting.
        /// </summary>
        /// <remarks>
        /// 0 = split segments (default)
        /// 1 = no new vertices on the boundary
        /// 2 = prevent all segment splitting, including internal boundaries
        /// </remarks>
        public int SegmentSplitting { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether intersecting segments are split at their
        /// intersection points before they are inserted into the mesh (default = true).
        /// </summary>
        /// <remarks>
        /// Inserting intersecting segments one by one is not robust, since the rounded
        /// intersection points don't lie exactly on the segments. See <see cref="SegmentNoder"/>.
        /// </remarks>
        public bool ResolveIntersections { get; set; } = true;

        /// <summary>
        /// Gets or sets the minimum absolute snap tolerance used to resolve intersections
        /// (default = 0, automatic: about 1e-12 times the coordinate magnitude).
        /// </summary>
        /// <remarks>
        /// Vertices closer than the tolerance are merged, vertices closer than the tolerance to
        /// a segment are inserted into the segment. Set this to the precision of the input data
        /// (e.g. 0.01 for coordinates in meters with centimeter accuracy) to remove slivers
        /// between nearly coincident boundaries, which otherwise lead to huge numbers of
        /// triangles when refining with a minimum angle.
        /// </remarks>
        public double SnapTolerance { get; set; }

        /// <summary>
        /// Gets or sets the distance up to which nearly coincident segments are merged
        /// (default = 0, off).
        /// </summary>
        /// <remarks>
        /// A segment which starts at another segment and runs along it at a small angle (less
        /// than <see cref="SegmentNoder.MaxWedgeAngle"/>), e.g. a wall digitized slightly off
        /// a parcel boundary, forms a thin wedge with a tiny input angle which refinement can't
        /// fix. If the other end of the segment is closer than the wedge tolerance to the
        /// segment it runs along, the two are merged. Unlike a larger <see cref="SnapTolerance"/>,
        /// this doesn't affect vertices which are close to a segment in other configurations.
        /// </remarks>
        public double WedgeTolerance { get; set; }

        /// <summary>
        /// Gets or sets the size of the grid all vertices of the mesh are rounded to
        /// (default = 0, no rounding).
        /// </summary>
        /// <remarks>
        /// Set this to the precision of the output (e.g. 0.01 if coordinates in meters are
        /// written with two decimal places). Rounding the coordinates of a mesh afterwards can
        /// flip or flatten triangles near nearly coincident lines. With a grid, the input is
        /// snap rounded (see <see cref="SegmentNoder.SnapRound"/>) and the Steiner points are
        /// moved to the grid where this doesn't flip a triangle (see <see cref="GridSnapper"/>).
        /// </remarks>
        public double GridSize { get; set; }

        /// <summary>
        /// Gets or sets a filter deciding whether a segment may be split during mesh
        /// refinement (in addition to <see cref="SegmentSplitting"/>). If null (default),
        /// all segments may be split.
        /// </summary>
        /// <remarks>
        /// The filter is called with mesh subsegments, so it should decide based on
        /// <see cref="ISegment.Label"/> (labels of input segments are passed on to their
        /// subsegments). Segments that may not be split keep their input vertices only
        /// (except for intersections with other segments), but the quality constraints
        /// can't be guaranteed near them.
        /// </remarks>
        public System.Func<TriangleNet.Geometry.ISegment, bool> CanSplitSegment { get; set; }
    }
}
