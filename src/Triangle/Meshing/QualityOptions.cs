
namespace TriangleNet.Meshing
{
    using System;
    using TriangleNet.Geometry;

    /// <summary>
    /// Mesh constraint options for quality triangulation.
    /// </summary>
    public class QualityOptions
    {
        /// <summary>
        /// Gets or sets a maximum angle constraint.
        /// </summary>
        public double MaximumAngle { get; set; }

        /// <summary>
        /// Gets or sets a minimum angle constraint.
        /// </summary>
        public double MinimumAngle { get; set; }

        /// <summary>
        /// Gets or sets a maximum triangle area constraint.
        /// </summary>
        public double MaximumArea { get; set; }

        /// <summary>
        /// Gets or sets a user-defined triangle constraint.
        /// </summary>
        /// <remarks>
        /// The test function will be called for each triangle in the mesh. The
        /// second argument is the area of the triangle tested. If the function
        /// returns true, the triangle is considered bad and will be refined.
        /// </remarks>
        public Func<ITriangle, double, bool> UserTest { get; set; }

        /// <summary>
        /// Gets or sets a test function for excluding triangles from refinement.
        /// </summary>
        /// <remarks>
        /// The exclude test takes higher priority than all other test, i.e. if a
        /// triangle passes the exclude test, no other test will be executed.
        /// </remarks>
        public Func<ITriangle, bool> Exclude { get; set; }

        /// <summary>
        /// Gets or sets an area constraint per triangle.
        /// </summary>
        /// <remarks>
        /// If this flag is set to true, the <see cref="ITriangle.Area"/> value will
        /// be used to check if a triangle needs refinement.
        /// </remarks>
        public bool VariableArea { get; set; }

        /// <summary>
        /// Gets or sets the maximum number of Steiner points to be inserted into the mesh.
        /// </summary>
        /// <remarks>
        /// If the value is 0 (default), an unknown number of Steiner points may be inserted
        /// to meet the other quality constraints.
        /// </remarks>
        public int SteinerPoints { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether to use the legacy refinement strategy (default = <c>false</c>).
        /// </summary>
        /// <remarks>
        /// If this flag is set to true, the original Triangle refinement algorithm will be
        /// used (Ruppert's algorithm). Otherwise the aCute algorithm is used.
        /// </remarks>
        public bool UseLegacyRefinement { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether redundant Steiner points are removed after
        /// refinement (default = <c>false</c>).
        /// </summary>
        /// <remarks>
        /// Refinement inserts Steiner points greedily, and later points often make earlier ones
        /// unnecessary. If this flag is set, a Steiner point that doesn't lie on a segment is
        /// deleted if all triangles of the retriangulated cavity meet the quality constraints
        /// (and don't encroach upon subsegments). Input vertices, segments and the points on
        /// segments are not changed. Not applied if a <see cref="UserTest"/> is set.
        /// </remarks>
        public bool ReduceSteinerPoints { get; set; }

        /// <summary>
        /// Gets or sets the number of smoothing passes over the Steiner points after refinement
        /// (default = 0).
        /// </summary>
        /// <remarks>
        /// In each pass, every Steiner point that doesn't lie on a segment is moved towards the
        /// position of the optimal Delaunay triangulation (the area-weighted mean of the
        /// circumcenters of its triangles; on an output grid, rounded to the grid). A move is
        /// kept only if the changed triangles get better, so no constraint that the mesh met
        /// before is violated afterwards. Combined with <see cref="ReduceSteinerPoints"/>,
        /// redundant points are removed after each pass. Input vertices, segments and the
        /// points on segments are not changed. Not applied if <see cref="Exclude"/> is set.
        /// </remarks>
        public int SmoothingSteps { get; set; }
    }
}
