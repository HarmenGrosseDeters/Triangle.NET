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
        /// possibly mismatched boundaries are complete).
        /// </summary>
        public static IEnumerable<CorpusCase> Tiles(string name, List<List<(double X, double Y)>> rings, double tileSize, int maxTiles)
        {
            var boxes = rings.Select(r => (r, r.Min(p => p.X), r.Min(p => p.Y), r.Max(p => p.X), r.Max(p => p.Y))).ToList();

            double left = boxes.Min(b => b.Item2), bottom = boxes.Min(b => b.Item3);
            double right = boxes.Max(b => b.Item4), top = boxes.Max(b => b.Item5);

            int count = 0;

            for (double x = left; x < right && count < maxTiles; x += tileSize)
            {
                for (double y = bottom; y < top && count < maxTiles; y += tileSize)
                {
                    var c = new CorpusCase { Name = $"{name} tile {x:F0}/{y:F0}" };

                    foreach (var b in boxes)
                    {
                        if (b.Item2 <= x + tileSize && b.Item4 >= x && b.Item3 <= y + tileSize && b.Item5 >= y)
                        {
                            c.Lines.Add(b.r);
                        }
                    }

                    if (c.Lines.Count > 0)
                    {
                        count++;
                        yield return c;
                    }
                }
            }
        }
    }
}
