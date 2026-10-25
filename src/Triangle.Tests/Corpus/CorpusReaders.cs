using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text.Json;
using System.Text.RegularExpressions;
using System.Xml.Linq;

namespace TriangleNet.Tests.Corpus
{
    public static class CorpusReaders
    {
        /// <summary>
        /// Read all test cases of the GEOS XML test suite (directory tests/xmltester/tests of
        /// https://github.com/libgeos/geos). Each case combines the linework of geometries A and B.
        /// </summary>
        public static IEnumerable<CorpusCase> ReadGeosXml(string directory)
        {
            foreach (var file in Directory.GetFiles(directory, "*.xml", SearchOption.AllDirectories).OrderBy(f => f, StringComparer.Ordinal))
            {
                XDocument doc;

                try { doc = XDocument.Load(file); } catch { continue; }

                int n = 0;

                foreach (var c in doc.Descendants("case"))
                {
                    n++;

                    var result = new CorpusCase { Name = Path.GetRelativePath(directory, file) + "#" + n };

                    foreach (var g in c.Elements().Where(e => e.Name == "a" || e.Name == "b"))
                    {
                        var wkt = g.Value.Trim();

                        // Skip WKB (hex) input.
                        if (wkt.Length == 0 || !char.IsLetter(wkt[0])) continue;

                        AddWkt(result, wkt);
                    }

                    if (result.Lines.Sum(l => l.Count) + result.Points.Count >= 3)
                    {
                        yield return result;
                    }
                }
            }
        }

        /// <summary>
        /// Add the linework of a WKT geometry (any type) to the case.
        /// </summary>
        public static void AddWkt(CorpusCase c, string wkt)
        {
            // Innermost coordinate lists "(x y, x y, ...)".
            foreach (Match m in Regex.Matches(wkt, @"\(([^()]*)\)"))
            {
                var points = new List<(double, double)>();

                foreach (var token in m.Groups[1].Value.Split(','))
                {
                    var xy = token.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);

                    if (xy.Length >= 2 &&
                        double.TryParse(xy[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var x) &&
                        double.TryParse(xy[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var y))
                    {
                        points.Add((x, y));
                    }
                }

                if (points.Count == 1) c.Points.Add(points[0]);
                else if (points.Count > 1) c.Lines.Add(points);
            }
        }

        /// <summary>
        /// Read all rings of Polygon/MultiPolygon features of a GeoJSON file (e.g. ALKIS parcels).
        /// </summary>
        public static List<List<(double X, double Y)>> ReadGeoJsonRings(string path)
        {
            var rings = new List<List<(double X, double Y)>>();

            using var stream = File.OpenRead(path);
            using var doc = JsonDocument.Parse(stream);

            foreach (var feature in doc.RootElement.GetProperty("features").EnumerateArray())
            {
                if (!feature.TryGetProperty("geometry", out var geometry) || geometry.ValueKind != JsonValueKind.Object) continue;

                var type = geometry.GetProperty("type").GetString();
                var coordinates = geometry.GetProperty("coordinates");

                if (type == "Polygon")
                {
                    AddPolygon(rings, coordinates);
                }
                else if (type == "MultiPolygon")
                {
                    foreach (var polygon in coordinates.EnumerateArray()) AddPolygon(rings, polygon);
                }
                else if (type == "LineString")
                {
                    // Polylines (e.g. coastlines) are returned like rings.
                    rings.Add(coordinates.EnumerateArray().Select(p => (p[0].GetDouble(), p[1].GetDouble())).ToList());
                }
                else if (type == "MultiLineString")
                {
                    AddPolygon(rings, coordinates);
                }
            }

            return rings;
        }

        private static void AddPolygon(List<List<(double X, double Y)>> rings, JsonElement polygon)
        {
            foreach (var ring in polygon.EnumerateArray())
            {
                rings.Add(ring.EnumerateArray().Select(p => (p[0].GetDouble(), p[1].GetDouble())).ToList());
            }
        }

        /// <summary>
        /// Read the test inputs of the CDT library (https://github.com/artem-ogre/CDT,
        /// CDT/tests/inputs): number of vertices and edges, the vertex coordinates, the edges
        /// as pairs of vertex indices.
        /// </summary>
        public static IEnumerable<CorpusCase> ReadCdtInputs(string directory)
        {
            foreach (var path in Directory.GetFiles(directory, "*.txt").OrderBy(p => p, StringComparer.Ordinal))
            {
                var tokens = File.ReadAllText(path).Split(new[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

                if (tokens.Length < 2 || !int.TryParse(tokens[0], out int nv) || !int.TryParse(tokens[1], out int ne) ||
                    tokens.Length < 2 + 2 * nv + 2 * ne)
                {
                    continue;
                }

                var c = new CorpusCase { Name = "cdt/" + Path.GetFileName(path) };
                var points = new List<(double X, double Y)>(nv);

                for (int i = 0; i < nv; i++)
                {
                    points.Add((double.Parse(tokens[2 + 2 * i], CultureInfo.InvariantCulture),
                        double.Parse(tokens[3 + 2 * i], CultureInfo.InvariantCulture)));
                }

                c.Points.AddRange(points);

                for (int e = 0; e < ne; e++)
                {
                    int a = int.Parse(tokens[2 + 2 * nv + 2 * e], CultureInfo.InvariantCulture);
                    int b = int.Parse(tokens[3 + 2 * nv + 2 * e], CultureInfo.InvariantCulture);

                    if (a >= 0 && a < nv && b >= 0 && b < nv)
                    {
                        c.Lines.Add(new List<(double X, double Y)> { points[a], points[b] });
                    }
                }

                yield return c;
            }
        }

        /// <summary>
        /// Read points of an xyz file (e.g. DGM1 NRW, optionally gzip compressed) inside the given window.
        /// </summary>
        public static List<(double X, double Y)> ReadXyz(string path, double left, double bottom, double right, double top)
        {
            var points = new List<(double X, double Y)>();

            using Stream file = File.OpenRead(path);
            using Stream stream = path.EndsWith(".gz", StringComparison.OrdinalIgnoreCase) ? new GZipStream(file, CompressionMode.Decompress) : file;
            using var reader = new StreamReader(stream);

            string row;

            while ((row = reader.ReadLine()) != null)
            {
                var v = row.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);

                if (v.Length < 2 ||
                    !double.TryParse(v[0], NumberStyles.Float, CultureInfo.InvariantCulture, out var x) ||
                    !double.TryParse(v[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var y)) continue;

                if (x >= left && x <= right && y >= bottom && y <= top) points.Add((x, y));
            }

            return points;
        }

        /// <summary>
        /// Split parcel rings into square tiles. Each tile contains all rings whose bounding box
        /// intersects the tile (rings are not clipped, so neighboring parcels and their shared,
        /// possibly mismatched boundaries are complete). If there are more than maxTiles
        /// non-empty tiles, tiles are sampled evenly.
        /// </summary>
        public static IEnumerable<CorpusCase> Tiles(string name, List<List<(double X, double Y)>> rings, double tileSize, int maxTiles)
        {
            var tiles = new Dictionary<(long, long), List<List<(double X, double Y)>>>();

            foreach (var r in rings)
            {
                long i0 = (long)Math.Floor(r.Min(p => p.X) / tileSize), i1 = (long)Math.Floor(r.Max(p => p.X) / tileSize);
                long j0 = (long)Math.Floor(r.Min(p => p.Y) / tileSize), j1 = (long)Math.Floor(r.Max(p => p.Y) / tileSize);

                for (long i = i0; i <= i1; i++)
                {
                    for (long j = j0; j <= j1; j++)
                    {
                        if (!tiles.TryGetValue((i, j), out var list))
                        {
                            list = new List<List<(double X, double Y)>>();
                            tiles.Add((i, j), list);
                        }

                        list.Add(r);
                    }
                }
            }

            var keys = tiles.Keys.OrderBy(k => k.Item1).ThenBy(k => k.Item2).ToList();
            double step = Math.Max(1.0, (double)keys.Count / maxTiles);

            for (double k = 0; k < keys.Count; k += step)
            {
                var key = keys[(int)k];
                var c = new CorpusCase { Name = $"{name} tile {key.Item1 * tileSize:F0}/{key.Item2 * tileSize:F0}" };

                c.Lines.AddRange(tiles[key]);

                yield return c;
            }
        }
    }
}
