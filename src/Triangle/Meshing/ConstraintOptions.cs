
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
    }
}
