// -----------------------------------------------------------------------
// <copyright file="TriangleQuality.cs">
// Triangle.NET code by Christian Woltering, http://triangle.codeplex.com/
// </copyright>
// -----------------------------------------------------------------------

namespace TriangleNet.Meshing
{
    using System;
    using TriangleNet.Geometry;

    /// <summary>
    /// Quality constraints for triangles that may not exist yet (used to check local mesh
    /// changes before or after they are made). Same computations as QualityMesher.TestTriangle
    /// and CheckSeg4Encroach, but without the exemptions for triangles that refinement can't
    /// improve.
    /// </summary>
    internal static class TriangleQuality
    {
        /// <summary>
        /// Check the angle and area constraints of a counterclockwise triangle.
        /// </summary>
        /// <param name="behavior">The quality constraints.</param>
        /// <param name="torg">First vertex.</param>
        /// <param name="tdest">Second vertex.</param>
        /// <param name="tapex">Third vertex.</param>
        /// <param name="areaLimit">The area constraint of the triangle (if variable area constraints are used).</param>
        /// <param name="cos2">The square of the cosine of the smallest angle.</param>
        /// <returns>True, if the triangle meets the constraints.</returns>
        public static bool Check(Behavior behavior, Vertex torg, Vertex tdest, Vertex tapex, double areaLimit, out double cos2)
        {
            return Check(behavior, torg, tdest, tapex, areaLimit, true, out cos2);
        }

        /// <summary>
        /// Check the angle constraints of a counterclockwise triangle (not the area).
        /// </summary>
        public static bool CheckAngles(Behavior behavior, Vertex torg, Vertex tdest, Vertex tapex)
        {
            return Check(behavior, torg, tdest, tapex, 0.0, false, out _);
        }

        private static bool Check(Behavior behavior, Vertex torg, Vertex tdest, Vertex tapex, double areaLimit, bool checkArea, out double cos2)
        {
            double dxod = torg.x - tdest.x;
            double dyod = torg.y - tdest.y;
            double dxda = tdest.x - tapex.x;
            double dyda = tdest.y - tapex.y;
            double dxao = tapex.x - torg.x;
            double dyao = tapex.y - torg.y;

            double apexlen = dxod * dxod + dyod * dyod;
            double orglen = dxda * dxda + dyda * dyda;
            double destlen = dxao * dxao + dyao * dyao;

            // The square of the cosine of the angle opposite to the shortest edge.
            if ((apexlen < orglen) && (apexlen < destlen))
            {
                cos2 = dxda * dxao + dyda * dyao;
                cos2 = cos2 * cos2 / (orglen * destlen);
            }
            else if (orglen < destlen)
            {
                cos2 = dxod * dxao + dyod * dyao;
                cos2 = cos2 * cos2 / (apexlen * destlen);
            }
            else
            {
                cos2 = dxod * dxda + dyod * dyda;
                cos2 = cos2 * cos2 / (apexlen * orglen);
            }

            if (cos2 > behavior.goodAngle)
            {
                return false;
            }

            if (checkArea && (behavior.fixedArea || behavior.VarArea))
            {
                double area = 0.5 * (dxod * dyda - dyod * dxda);

                if (behavior.fixedArea && area > behavior.MaxArea)
                {
                    return false;
                }

                // Nonpositive area constraints are treated as unconstrained.
                if (behavior.VarArea && areaLimit > 0.0 && area > areaLimit)
                {
                    return false;
                }
            }

            if (behavior.MaxAngle != 0.0)
            {
                double maxangle;

                if ((apexlen > orglen) && (apexlen > destlen))
                {
                    maxangle = (orglen + destlen - apexlen) / (2 * Math.Sqrt(orglen * destlen));
                }
                else if (orglen > destlen)
                {
                    maxangle = (apexlen + destlen - orglen) / (2 * Math.Sqrt(apexlen * destlen));
                }
                else
                {
                    maxangle = (apexlen + orglen - destlen) / (2 * Math.Sqrt(apexlen * orglen));
                }

                if (maxangle < behavior.maxGoodAngle)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Check whether the apex lies in the diametral lens (or circle, for conforming
        /// Delaunay meshes) of the subsegment from eorg to edest.
        /// </summary>
        public static bool Encroaches(Behavior behavior, Vertex eorg, Vertex edest, Vertex apex)
        {
            double dotproduct = (eorg.x - apex.x) * (edest.x - apex.x) +
                                (eorg.y - apex.y) * (edest.y - apex.y);

            if (dotproduct < 0.0)
            {
                return behavior.ConformingDelaunay ||
                    (dotproduct * dotproduct >=
                     (2.0 * behavior.goodAngle - 1.0) * (2.0 * behavior.goodAngle - 1.0) *
                     ((eorg.x - apex.x) * (eorg.x - apex.x) +
                      (eorg.y - apex.y) * (eorg.y - apex.y)) *
                     ((edest.x - apex.x) * (edest.x - apex.x) +
                      (edest.y - apex.y) * (edest.y - apex.y)));
            }

            return false;
        }
    }
}
