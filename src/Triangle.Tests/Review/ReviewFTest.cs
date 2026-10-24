using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using TriangleNet.Geometry;
using TriangleNet.IO;
using TriangleNet.Meshing;
using TriangleNet.Tests.Robustness;

namespace TriangleNet.Tests.Review
{
    /// <summary>
    /// Review F: IO (Triangle file formats, FileProcessor, MFEM, DebugWriter), RobustPredicates, Log.
    /// </summary>
    public class ReviewFTest
    {
        private string dir;

        [SetUp]
        public void SetUp()
        {
            dir = Path.Combine(Path.GetTempPath(), "ReviewF_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(dir);
        }

        [TearDown]
        public void TearDown()
        {
            try { Directory.Delete(dir, true); } catch { }
        }

        #region Helpers

        private string FilePath(string name) => Path.Combine(dir, name);

        private static void WithCulture(string name, Action action)
        {
            var oldCulture = Thread.CurrentThread.CurrentCulture;
            var oldUi = Thread.CurrentThread.CurrentUICulture;
            try
            {
                Thread.CurrentThread.CurrentCulture = new CultureInfo(name);
                Thread.CurrentThread.CurrentUICulture = new CultureInfo(name);
                action();
            }
            finally
            {
                Thread.CurrentThread.CurrentCulture = oldCulture;
                Thread.CurrentThread.CurrentUICulture = oldUi;
            }
        }

        /// <summary>
        /// Exact value of a finite double as X * 2^-1074 (X integer).
        /// </summary>
        private static BigInteger Exact(double d)
        {
            Assert.That(double.IsFinite(d), Is.True);
            long bits = BitConverter.DoubleToInt64Bits(d);
            bool negative = bits < 0;
            int exp = (int)((bits >> 52) & 0x7FF);
            long mant = bits & ((1L << 52) - 1);
            BigInteger x = exp == 0 ? new BigInteger(mant) : new BigInteger(mant | (1L << 52)) << (exp - 1);
            return negative ? -x : x;
        }

        private static int ExactOrient(Point a, Point b, Point c)
        {
            var ax = Exact(a.X); var ay = Exact(a.Y);
            var bx = Exact(b.X); var by = Exact(b.Y);
            var cx = Exact(c.X); var cy = Exact(c.Y);
            var det = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx);
            return det.Sign;
        }

        private static int ExactInCircle(Point a, Point b, Point c, Point d)
        {
            var dx = Exact(d.X); var dy = Exact(d.Y);
            var adx = Exact(a.X) - dx; var ady = Exact(a.Y) - dy;
            var bdx = Exact(b.X) - dx; var bdy = Exact(b.Y) - dy;
            var cdx = Exact(c.X) - dx; var cdy = Exact(c.Y) - dy;
            var alift = adx * adx + ady * ady;
            var blift = bdx * bdx + bdy * bdy;
            var clift = cdx * cdx + cdy * cdy;
            var det = alift * (bdx * cdy - cdx * bdy)
                    + blift * (cdx * ady - adx * cdy)
                    + clift * (adx * bdy - bdx * ady);
            return det.Sign;
        }

        private static double ToDouble(BigInteger num, BigInteger den)
        {
            if (num.IsZero) return 0.0;
            if (den.Sign < 0) { num = -num; den = -den; }
            int shift = 80 - (int)(BigInteger.Abs(num).GetBitLength() - den.GetBitLength());
            var q = shift >= 0 ? (num << shift) / den : num / (den << -shift);
            return Math.ScaleB((double)q, -shift);
        }

        private static double Ulps(double v, int k)
        {
            for (int i = 0; i < Math.Abs(k); i++)
            {
                v = k > 0 ? Math.BitIncrement(v) : Math.BitDecrement(v);
            }
            return v;
        }

        private static Polygon SamplePolygon()
        {
            // Vertices with explicit IDs (0-based, matching the index the writer emits).
            var pts = new[]
            {
                new Vertex(0.1, 1.0 / 3.0, 1), new Vertex(10.000000000000002, 0.30000000000000004, 2),
                new Vertex(10.5, 9.87654321012345678, 3), new Vertex(-1e-17, 10, 4),
                new Vertex(4.123456789012345, 5.0000000000000009, 0)
            };
            var poly = new Polygon(5, true);
            for (int i = 0; i < pts.Length; i++)
            {
                pts[i].ID = i;
                poly.Add(pts[i]);
            }
            poly.Add(new Segment(pts[0], pts[1], 11));
            poly.Add(new Segment(pts[1], pts[2], 12));
            poly.Add(new Segment(pts[2], pts[3], 13));
            poly.Add(new Segment(pts[3], pts[0], 14));
            poly.Holes.Add(new Point(1.0 / 7.0, 2.0 / 3.0));
            poly.Regions.Add(new RegionPointer(5.5, 5.123456789012345, 7));
            return poly;
        }

        private const string SquarePoly =
            "4 2 0 0\n0 0 0\n1 1 0\n2 1 1\n3 0 1\n4 0\n0 0 1\n1 1 2\n2 2 3\n3 3 0\n0\n";

        #endregion

        #region IO: round trips and culture

        [Test]
        [Description("Regression: .poly round trip keeps coordinates bit-exact (shortest round-trip formatting), markers, holes and region labels, also under de-DE culture.")]
        public void PolyRoundTripIsExactUnderGermanCulture([Values("de-DE", "en-US")] string culture)
        {
            var poly = SamplePolygon();
            string file = FilePath("sample.poly");

            WithCulture(culture, () =>
            {
                new TriangleWriter().WritePoly(poly, file);
                var read = new TriangleReader().ReadPolyFile(file);

                Assert.That(read.Points.Count, Is.EqualTo(poly.Points.Count));
                for (int i = 0; i < poly.Points.Count; i++)
                {
                    Assert.That(read.Points[i].X, Is.EqualTo(poly.Points[i].X), "x of " + i);
                    Assert.That(read.Points[i].Y, Is.EqualTo(poly.Points[i].Y), "y of " + i);
                    Assert.That(read.Points[i].Label, Is.EqualTo(poly.Points[i].Label), "label of " + i);
                }

                Assert.That(read.Segments.Count, Is.EqualTo(4));
                for (int i = 0; i < 4; i++)
                {
                    Assert.That(read.Segments[i].GetVertex(0).X, Is.EqualTo(poly.Segments[i].GetVertex(0).X));
                    Assert.That(read.Segments[i].GetVertex(1).Y, Is.EqualTo(poly.Segments[i].GetVertex(1).Y));
                    Assert.That(read.Segments[i].Label, Is.EqualTo(poly.Segments[i].Label));
                }

                Assert.That(read.Holes.Single().X, Is.EqualTo(1.0 / 7.0));
                Assert.That(read.Holes.Single().Y, Is.EqualTo(2.0 / 3.0));
                Assert.That(read.Regions.Single().Point.Y, Is.EqualTo(5.123456789012345));
                Assert.That(read.Regions.Single().Label, Is.EqualTo(7));
            });
        }

        [Test]
        [Description("TriangleWriter.WritePoly(IPolygon) writes vertex indices 0..n-1 but segment endpoints as Vertex.ID. For a polygon built from Contours (IDs never assigned, all 0) every segment is written as '0 0', so the file is corrupt.")]
        public void PolyRoundTripWithoutExplicitVertexIds()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10));
            string file = FilePath("contour.poly");

            new TriangleWriter().WritePoly(poly, file);
            var read = new TriangleReader().ReadPolyFile(file);

            Assert.That(read.Segments.Count, Is.EqualTo(4));
            for (int i = 0; i < 4; i++)
            {
                var a = read.Segments[i].GetVertex(0);
                var b = read.Segments[i].GetVertex(1);
                Assert.That(a, Is.Not.SameAs(b), $"segment {i} is degenerate (both endpoints index {a.ID})");
                Assert.That(a.X, Is.EqualTo(poly.Segments[i].GetVertex(0).X));
                Assert.That(a.Y, Is.EqualTo(poly.Segments[i].GetVertex(0).Y));
                Assert.That(b.X, Is.EqualTo(poly.Segments[i].GetVertex(1).X));
                Assert.That(b.Y, Is.EqualTo(poly.Segments[i].GetVertex(1).Y));
            }
        }

        [Test]
        [Description("TriangleWriter.WritePoly (IPolygon and Mesh overloads) writes regions as 'i x y label' and drops the area constraint, so a .poly round trip loses RegionPointer.Area.")]
        public void PolyRoundTripKeepsRegionArea()
        {
            var poly = SamplePolygon();
            poly.Regions.Clear();
            poly.Regions.Add(new RegionPointer(5, 5, 3, 0.25));
            string file = FilePath("area.poly");

            new TriangleWriter().WritePoly(poly, file);
            var read = new TriangleReader().ReadPolyFile(file);

            Assert.That(read.Regions.Single().Label, Is.EqualTo(3));
            Assert.That(read.Regions.Single().Area, Is.EqualTo(0.25));
        }

        [Test]
        [Description("TriangleReader.ReadPolyFile parses the region area with NumberStyles.Number, which forbids exponents. '1e-05' (as written by Triangle's C code or .NET for small areas) silently becomes area 0 (no constraint).")]
        public void RegionAreaInScientificNotation()
        {
            string file = FilePath("sci.poly");
            File.WriteAllText(file, SquarePoly + "1\n0 0.5 0.5 1 1e-05\n");

            var read = new TriangleReader().ReadPolyFile(file);

            Assert.That(read.Regions.Single().Area, Is.EqualTo(1e-05));
        }

        [Test]
        [Description("Regression: comments, blank lines, tabs and 1-based indices are handled.")]
        public void PolyWithCommentsBlankLinesAndOneBasedIndices()
        {
            string file = FilePath("onebased.poly");
            File.WriteAllText(file,
                "# header comment\n\n4\t2 0 1\n1 0 0 5\n  # inside\n2 1 0 6\n\n3 1 1 7\n4 0 1 8\n" +
                "4 1\n1 1 2 21\n2 2 3 22\n3 3 4 23\n4 4 1 24\n1\n1 0.25 0.25\n");

            var read = new TriangleReader().ReadPolyFile(file);

            Assert.That(read.Points.Select(p => p.Label), Is.EqualTo(new[] { 5, 6, 7, 8 }));
            Assert.That(read.Segments.Count, Is.EqualTo(4));
            Assert.That(read.Segments[0].GetVertex(0), Is.SameAs(read.Points[0]));
            Assert.That(read.Segments[3].GetVertex(1), Is.SameAs(read.Points[0]));
            Assert.That(read.Segments.Select(s => s.Label), Is.EqualTo(new[] { 21, 22, 23, 24 }));
            Assert.That(read.Holes.Single().X, Is.EqualTo(0.25));
            Assert.That(read.Regions, Is.Empty);
        }

        [TestCase("4 2 0 1\n0 0 0 # corner\n1 1 0 1\n2 1 1 1\n3 0 1 1\n0 0\n0\n", TestName = "InlineCommentAfterVertexWithMarkers")]
        [TestCase("4 2 0 0\n0 0 0\n1 1 0\n2 1 1\n3 0 1\n1 1\n0 0 1 # bottom edge\n0\n", TestName = "InlineCommentAfterSegmentWithMarkers")]
        [Description("TriangleReader only skips whole-line comments. A trailing '# comment' (allowed by Triangle's file format, the C reader stops at '#') is tokenized and int.Parse('#') throws FormatException when the marker column is missing.")]
        public void InlineComments(string content)
        {
            string file = FilePath("inline.poly");
            File.WriteAllText(file, content);

            Polygon read = null;
            Assert.DoesNotThrow(() => read = new TriangleReader().ReadPolyFile(file));
            Assert.That(read.Points.Count, Is.EqualTo(4));
        }

        [Test]
        [Description("TriangleReader.ReadEleFile ignores the nodes-per-triangle header field and always reads the region attribute from column 4. For 6-node (second order) .ele files it reads the 4th node index as region label.")]
        public void SixNodeElementsRegionAttribute()
        {
            string ele = FilePath("quad.ele");
            File.WriteAllText(ele, "2 6 1\n0 0 1 2 4 5 6 5\n1 0 2 3 6 7 8 7\n");

            var tris = new TriangleReader().ReadEleFile(ele);

            Assert.That(tris.Select(t => t.GetVertexID(2)), Is.EqualTo(new[] { 2, 3 }));
            Assert.That(tris.Select(t => t.Label), Is.EqualTo(new[] { 5, 7 }));
        }

        [Test]
        [Description("Regression: mesh round trip through TriangleFormat (.poly + .ele) under de-DE keeps vertices, triangles and region labels.")]
        public void MeshRoundTripUnderGermanCulture()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 10), 1);
            poly.Add(TestData.Square(2.5, 2.5, 1.0 / 3.0), 2);
            var mesh = (Mesh)poly.Triangulate(new ConstraintOptions(), new QualityOptions { MaximumArea = 3 });
            string file = FilePath("mesh.poly");

            WithCulture("de-DE", () =>
            {
                new TriangleFormat().Write(mesh, file);
                var read = (Mesh)new TriangleFormat().Import(file);

                Assert.That(read.Vertices.Count, Is.EqualTo(mesh.Vertices.Count));
                Assert.That(read.Triangles.Count, Is.EqualTo(mesh.Triangles.Count));

                var orig = mesh.Vertices.OrderBy(v => v.ID).ToArray();
                var copy = read.Vertices.OrderBy(v => v.ID).ToArray();
                for (int i = 0; i < orig.Length; i++)
                {
                    Assert.That(copy[i].X, Is.EqualTo(orig[i].X));
                    Assert.That(copy[i].Y, Is.EqualTo(orig[i].Y));
                }

                Assert.That(read.Triangles.Select(t => t.Label).OrderBy(l => l),
                    Is.EqualTo(mesh.Triangles.Select(t => t.Label).OrderBy(l => l)));
                Assert.That(read.Triangles.Sum(t => MeshAssert.Area(t)), Is.EqualTo(100).Within(1e-9));
            });
        }

        [Test]
        [Description("TriangleWriter formats integers (labels/markers) with the current culture. Under cultures whose NegativeSign is U+2212 (e.g. sv-SE, nb-NO with ICU) negative labels are written as '−1', which the reader cannot parse under another culture (FormatException). Fix: format with CultureInfo.InvariantCulture (and parse ints with it).")]
        public void NegativeLabelsWrittenUnderSwedishCulture()
        {
            Assume.That(new CultureInfo("sv-SE").NumberFormat.NegativeSign, Is.EqualTo("−"), "ICU culture data not available");

            var poly = SamplePolygon();
            poly.Points[0].Label = -1;
            ((Segment)poly.Segments[0]).Label = -2;
            string file = FilePath("neg.poly");

            WithCulture("sv-SE", () => new TriangleWriter().WritePoly(poly, file));

            Polygon read = null;
            WithCulture("en-US", () => Assert.DoesNotThrow(() => read = new TriangleReader().ReadPolyFile(file)));
            Assert.That(read.Points[0].Label, Is.EqualTo(-1));
            Assert.That(read.Segments[0].Label, Is.EqualTo(-2));
        }

        [Test]
        [Description("TriangleFormat.IsSupported lower-cases the extension, but Read/Import compare case-sensitively. FileProcessor.Read(\"x.POLY\") passes the check and then throws NotSupportedException.")]
        public void FileProcessorUpperCaseExtension()
        {
            string file = FilePath("SQUARE.POLY");
            File.WriteAllText(file, SquarePoly);

            Assert.That(FileProcessor.IsSupported(file), Is.True);

            IPolygon read = null;
            Assert.DoesNotThrow(() => read = FileProcessor.Read(file));
            Assert.That(read.Points.Count, Is.EqualTo(4));
        }

        private class MeshOnlyFormat : IMeshFormat
        {
            public bool IsSupported(string file) => file.EndsWith(".reviewf-mesh-only");
            public IMesh Import(string filename) => throw new NotSupportedException();
            public void Write(IMesh mesh, string filename) => throw new NotSupportedException();
            public void Write(IMesh mesh, Stream stream) => throw new NotSupportedException();
        }

        [Test]
        [Description("FileProcessor keeps List<IFileFormat> but iterates 'foreach (IPolygonFormat f in formats)' (and IMeshFormat). After registering a format that implements only IMeshFormat, polygon Read/Write of any non-Triangle file throws InvalidCastException instead of 'not supported'. Fix: use formats.OfType<IPolygonFormat>().")]
        public void FileProcessorWithMeshOnlyFormat()
        {
            FileProcessor.Add(new MeshOnlyFormat());

            var ex = Assert.Catch(() => FileProcessor.Read(FilePath("x.unknown")));
            Assert.That(ex, Is.Not.InstanceOf<InvalidCastException>(), ex.ToString());
        }

        [Test]
        [Description("With Behavior.Jettison, TriangleWriter.WriteNodes skips undead (duplicate) vertices and renumbers the written nodes, but .ele/.poly still use the uncompacted Vertex.id, so all elements after the first dropped vertex reference wrong nodes.")]
        public void JettisonWritesConsistentIndices()
        {
            var points = new List<Vertex>
            {
                new Vertex(0, 0), new Vertex(0, 0) /* duplicate -> undead */, new Vertex(1, 0),
                new Vertex(1, 1), new Vertex(0, 1), new Vertex(0.5, 0.4)
            };
            var mesh = (Mesh)new GenericMesher().Triangulate(points);
            Assume.That(mesh.undeads, Is.GreaterThan(0));
            mesh.behavior.Jettison = true;

            string node = FilePath("j.node"), ele = FilePath("j.ele");
            var writer = new TriangleWriter();
            writer.WriteNodes(mesh, node);
            writer.WriteElements(mesh, ele);

            var reader = new TriangleReader();
            var read = reader.ReadNodeFile(node);
            var tris = reader.ReadEleFile(ele);

            Assert.That(read.Points.Count, Is.EqualTo(5));
            var meshTris = mesh.Triangles.ToArray();
            for (int i = 0; i < tris.Count; i++)
            {
                // The writer may start a triangle at another vertex: compare vertex sets.
                var actual = new List<(double, double)>();
                var expected = new List<(double, double)>();

                for (int k = 0; k < 3; k++)
                {
                    int idx = tris[i].GetVertexID(k);
                    Assert.That(idx, Is.InRange(0, read.Points.Count - 1), $"triangle {i} node {k}");
                    actual.Add((read.Points[idx].X, read.Points[idx].Y));
                    expected.Add((meshTris[i].GetVertex(k).X, meshTris[i].GetVertex(k).Y));
                }

                Assert.That(actual, Is.EquivalentTo(expected), $"triangle {i}");
            }
        }

        [Test]
        [Description("Regression: MFEM output is well formed (0-based indices within range, labels shifted by 1, invariant number format under de-DE).")]
        public void MfemOutputIsWellFormed()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1.5), 0);
            var mesh = poly.Triangulate(new ConstraintOptions(), new QualityOptions { MaximumArea = 0.2 });
            string file = FilePath("m.mesh");

            WithCulture("de-DE", () => MfemMesh.Write(mesh, file));

            var lines = File.ReadAllLines(file);
            Assert.That(lines[0], Is.EqualTo("MFEM mesh v1.0"));
            int e = Array.IndexOf(lines, "elements");
            int b = Array.IndexOf(lines, "boundary");
            int v = Array.IndexOf(lines, "vertices");
            int ne = int.Parse(lines[e + 1]), nb = int.Parse(lines[b + 1]), nv = int.Parse(lines[v + 1]);
            Assert.That(ne, Is.EqualTo(mesh.Triangles.Count));
            Assert.That(nb, Is.EqualTo(mesh.Segments.Count));
            Assert.That(nv, Is.EqualTo(mesh.Vertices.Count));

            var coords = lines.Skip(v + 3).Take(nv)
                .Select(l => l.Split(' ').Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray()).ToArray();
            double area = 0;
            for (int i = 0; i < ne; i++)
            {
                var f = lines[e + 2 + i].Split(' ').Select(int.Parse).ToArray();
                Assert.That(f[0], Is.EqualTo(1));
                Assert.That(f[1], Is.EqualTo(2));
                Assert.That(f.Skip(2), Is.All.InRange(0, nv - 1));
                var (p, q, r) = (coords[f[2]], coords[f[3]], coords[f[4]]);
                double a2 = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
                Assert.That(a2, Is.GreaterThan(0), "element orientation must be CCW");
                area += a2 / 2;
            }
            Assert.That(area, Is.EqualTo(2.25).Within(1e-12));
            for (int i = 0; i < nb; i++)
            {
                var f = lines[b + 2 + i].Split(' ').Select(int.Parse).ToArray();
                Assert.That(f[1], Is.EqualTo(1));
                Assert.That(f.Skip(2), Is.All.InRange(0, nv - 1));
            }
        }

        [Test]
        [Description("MfemMesh.Write(IMesh, Stream) wraps the caller's stream in a StreamWriter without leaveOpen, so the caller's stream is disposed after writing (e.g. a MemoryStream can't be read anymore). Fix: new StreamWriter(stream, leaveOpen: true).")]
        public void MfemWriteToStreamLeavesStreamOpen()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            var mesh = poly.Triangulate();

            var stream = new MemoryStream();
            MfemMesh.Write(mesh, stream);

            Assert.That(stream.CanRead, Is.True, "caller's stream was disposed");
        }

        [Test]
        [Description("Regression: internal DebugWriter session produces a readable gzip file.")]
        public void DebugWriterProducesGzipFile()
        {
            var poly = new Polygon();
            poly.Add(TestData.Square(0, 0, 1));
            var mesh = (Mesh)poly.Triangulate();
            string session = FilePath("debug");

            WithCulture("de-DE", () =>
            {
                DebugWriter.Session.Start(session);
                DebugWriter.Session.Write(mesh);
                DebugWriter.Session.Finish();
            });

            using var gz = new GZipStream(File.OpenRead(session + ".mshx"), CompressionMode.Decompress);
            var text = new StreamReader(gz).ReadToEnd();
            Assert.That(text, Does.StartWith("#!N1"));
            Assert.That(text, Does.Contain("#!M0"));
            Assert.That(text, Does.Not.Contain(","));
        }

        #endregion

        #region Log

        [Test]
        [Description("Log.Instance stores events in a plain List<T> without synchronization. Concurrent logging (e.g. parallel meshing with Log.Verbose, PolygonValidator, Converter errors) loses entries or throws (IndexOutOfRange/ArgumentException). Fix: lock in Add/Info/Warning/Error/Clear or use a concurrent collection.")]
        public void LogIsThreadSafe()
        {
            var log = Log.Instance;
            log.Clear();
            const int threads = 8, perThread = 20000;
            Exception error = null;

            try
            {
                Parallel.For(0, threads, new ParallelOptions { MaxDegreeOfParallelism = threads }, t =>
                {
                    for (int i = 0; i < perThread; i++)
                    {
                        log.Warning("w", "ReviewF");
                    }
                });
            }
            catch (Exception ex)
            {
                error = ex;
            }

            int count = log.Data.Count;
            log.Clear();

            Assert.That(error, Is.Null, error?.ToString());
            Assert.That(count, Is.EqualTo(threads * perThread), "lost log entries");
        }

        #endregion

        #region RobustPredicates

        [Test]
        [Description("Regression: static initialization yields epsilon = 2^-53 and splitter = 2^27 + 1.")]
        public void StaticInitialization()
        {
            var t = typeof(RobustPredicates);
            var flags = System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static;
            _ = RobustPredicates.Default;
            Assert.That((double)t.GetField("epsilon", flags).GetValue(null), Is.EqualTo(Math.ScaleB(1, -53)));
            Assert.That((double)t.GetField("splitter", flags).GetValue(null), Is.EqualTo(Math.ScaleB(1, 27) + 1));
        }

        private static IEnumerable<Point[]> NearlyCollinear(int seed, double scale, double offset)
        {
            var rnd = new Random(seed);
            for (int n = 0; n < 2000; n++)
            {
                var a = new Point(offset + scale * rnd.NextDouble(), offset + scale * rnd.NextDouble());
                var b = new Point(offset + scale * rnd.NextDouble(), offset + scale * rnd.NextDouble());
                double t = rnd.NextDouble() * 3 - 1;
                double cx = a.X + t * (b.X - a.X), cy = a.Y + t * (b.Y - a.Y);
                var c = new Point(Ulps(cx, rnd.Next(-2, 3)), Ulps(cy, rnd.Next(-2, 3)));
                yield return new[] { a, b, c };
            }
        }

        [Test]
        [Description("Regression: CounterClockwise sign agrees with exact rational arithmetic on nearly collinear points (several scales, mixed magnitudes) for the Default and a new instance.")]
        public void CounterClockwiseMatchesExact(
            [Values(1.0, 1e15, 1e-15, 1e100, 1e-100)] double scale,
            [Values(0.0, 1.0)] double offsetFactor)
        {
            var preds = new[] { RobustPredicates.Default, new RobustPredicates() };
            // Mixed magnitudes: tiny differences on a large offset.
            double offset = offsetFactor * scale * 1e9;
            int nonzero = 0;
            foreach (var p in NearlyCollinear(42, scale, offset))
            {
                int exact = ExactOrient(p[0], p[1], p[2]);
                if (exact != 0) nonzero++;
                foreach (var pred in preds)
                {
                    Assert.That(Math.Sign(pred.CounterClockwise(p[0], p[1], p[2])), Is.EqualTo(exact),
                        $"({p[0].X:R},{p[0].Y:R}) ({p[1].X:R},{p[1].Y:R}) ({p[2].X:R},{p[2].Y:R})");
                }
            }
            Assert.That(nonzero, Is.GreaterThan(0));
        }

        private static IEnumerable<Point[]> NearlyCocircular(int seed, double scale, double offset)
        {
            var rnd = new Random(seed);
            for (int n = 0; n < 2000; n++)
            {
                double cx = offset + scale * rnd.NextDouble(), cy = offset + scale * rnd.NextDouble();
                double r = scale * (0.1 + rnd.NextDouble());
                var angles = Enumerable.Range(0, 4).Select(_ => rnd.NextDouble() * 2 * Math.PI).OrderBy(x => x).ToArray();
                var pts = angles.Select(a => new Point(Ulps(cx + r * Math.Cos(a), rnd.Next(-1, 2)),
                    Ulps(cy + r * Math.Sin(a), rnd.Next(-1, 2)))).ToArray();
                yield return pts;
            }
        }

        [Test]
        [Description("Regression: InCircle sign agrees with exact rational arithmetic on nearly cocircular points (several scales, mixed magnitudes).")]
        public void InCircleMatchesExact(
            [Values(1.0, 1e15, 1e-15, 1e50, 1e-50)] double scale,
            [Values(0.0, 1.0)] double offsetFactor)
        {
            var pred = new RobustPredicates();
            double offset = offsetFactor * scale * 1e6;
            foreach (var p in NearlyCocircular(7, scale, offset))
            {
                int exact = ExactInCircle(p[0], p[1], p[2], p[3]);
                Assert.That(Math.Sign(pred.InCircle(p[0], p[1], p[2], p[3])), Is.EqualTo(exact),
                    string.Join(" ", p.Select(q => $"({q.X:R},{q.Y:R})")));
                Assert.That(Math.Sign(pred.NonRegular(p[0], p[1], p[2], p[3])), Is.EqualTo(exact));
            }

            // Exactly cocircular input must give exactly 0.
            Assert.That(pred.InCircle(new Point(0, 0), new Point(1, 0), new Point(1, 1), new Point(0, 1)), Is.EqualTo(0.0));
            double s = scale;
            Assert.That(pred.InCircle(new Point(offset, offset), new Point(offset + s, offset),
                new Point(offset + s, offset + s), new Point(offset, offset + s)),
                Is.EqualTo(Math.Sign(ExactInCircle(new Point(offset, offset), new Point(offset + s, offset),
                    new Point(offset + s, offset + s), new Point(offset, offset + s)))).Or.EqualTo(0.0));
        }

        [Test]
        [Explicit("Documented limitation of Shewchuk's predicates: coordinate differences below about 1e-75 underflow (see docs/Robustheitsanalyse.md).")]
        [Description("Inherited from Shewchuk's predicates (documented assumption: no underflow): with coordinates around 1e-300 the products underflow to 0 and CounterClockwise returns 0 (collinear) for a clearly non-degenerate right triangle. Inputs should be rejected or rescaled before meshing.")]
        public void CounterClockwiseUnderflow()
        {
            const double s = 1e-300;
            var a = new Point(0, 0);
            var b = new Point(s, 0);
            var c = new Point(0, s);
            Assert.That(ExactOrient(a, b, c), Is.EqualTo(1));
            Assert.That(Math.Sign(new RobustPredicates().CounterClockwise(a, b, c)), Is.EqualTo(1));
        }

        [Test]
        [Description("Fixed: coordinates beyond Mesh.MaxCoordinate (where the exact predicates overflow) are rejected.")]
        public void CoordinatesBeyondPredicateRangeAreRejected([Values(1e71, 1e160)] double s)
        {
            var poly = new Polygon();
            poly.Add(new Vertex(0, 0));
            poly.Add(new Vertex(s, 0));
            poly.Add(new Vertex(0, s));

            Assert.Throws<ArgumentException>(() => poly.Triangulate());
        }

        [Test]
        [Description("Regression: the full allowed coordinate range can be meshed (incircle products don't overflow).")]
        public void MeshAtMaximumCoordinates()
        {
            var poly = new Polygon();
            var random = new Random(1);

            for (int i = 0; i < 200; i++)
            {
                poly.Add(new Vertex((random.NextDouble() * 2 - 1) * Mesh.MaxCoordinate, (random.NextDouble() * 2 - 1) * Mesh.MaxCoordinate));
            }

            var mesh = poly.Triangulate();

            Robustness.MeshAssert.IsValid(mesh);
        }

        [Test]
        [Explicit("Documented limitation of Shewchuk's predicates: products overflow for coordinates around 1e160; such input is rejected by Mesh (MaxCoordinate).")]
        [Description("Inherited from Shewchuk's predicates (documented assumption: no overflow): with coordinates around 1e160 both partial products overflow to +Infinity, det = Inf - Inf = NaN and the adaptive stage also yields NaN, so the orientation of a clearly CCW triangle is not positive.")]
        public void CounterClockwiseOverflow()
        {
            const double s = 1e160;
            var c = new Point(0, 0);
            var a = new Point(1.5 * s, s);
            var b = new Point(s, 1.5 * s);
            Assert.That(ExactOrient(a, b, c), Is.EqualTo(1));
            double det = new RobustPredicates().CounterClockwise(a, b, c);
            Assert.That(det, Is.GreaterThan(0.0), "det = " + det);
        }

        [Test]
        [Description("Regression: tiny but non-underflowing (1e-160) and large non-overflowing coordinates give correct orientation.")]
        public void CounterClockwiseLargeRangeWithoutOverflow([Values(1e-160, 1e-140, 1e140, 1e150)] double s)
        {
            var a = new Point(0, 0);
            var b = new Point(s, 0);
            var c = new Point(Ulps(s, -1), Ulps(s, 1));
            Assert.That(Math.Sign(new RobustPredicates().CounterClockwise(a, b, c)), Is.EqualTo(ExactOrient(a, b, c)));
        }

        [Test]
        [Description("Regression: FindCircumcenter (without off-center) agrees with the exact circumcenter to ~1e-10 of the circumradius for random and moderately skinny triangles; xi/eta are consistent.")]
        public void FindCircumcenterAccuracy()
        {
            var pred = new RobustPredicates();
            var rnd = new Random(3);
            for (int n = 0; n < 2000; n++)
            {
                double scale = Math.Pow(10, rnd.Next(-8, 9));
                double off = rnd.Next(2) * 1000 * scale;
                var o = new Point(off + scale * rnd.NextDouble(), off + scale * rnd.NextDouble());
                var d = new Point(off + scale * rnd.NextDouble(), off + scale * rnd.NextDouble());
                double h = Math.Pow(10, -rnd.Next(0, 6));
                // apex roughly at distance h*|od| from od.
                var mid = new Point((o.X + d.X) / 2, (o.Y + d.Y) / 2);
                var a = new Point(mid.X - h * (d.Y - o.Y), mid.Y + h * (d.X - o.X));
                if (ExactOrient(d, a, o) <= 0) continue;

                double xi = 0, eta = 0;
                var c = pred.FindCircumcenter(o, d, a, ref xi, ref eta);
                var c2 = pred.FindCircumcenter(o, d, a, ref xi, ref eta, 0.0);

                // Exact circumcenter.
                var ox = Exact(o.X); var oy = Exact(o.Y);
                var xdo = Exact(d.X) - ox; var ydo = Exact(d.Y) - oy;
                var xao = Exact(a.X) - ox; var yao = Exact(a.Y) - oy;
                var dodist = xdo * xdo + ydo * ydo; var aodist = xao * xao + yao * yao;
                var den = 2 * (xdo * yao - xao * ydo); // scaled by 2^-2148, numerators by 2^-3222
                var unit = BigInteger.One << 1074;
                double dx = ToDouble(yao * dodist - ydo * aodist, den * unit);
                double dy = ToDouble(xdo * aodist - xao * dodist, den * unit);
                double r = Math.Sqrt(dx * dx + dy * dy);

                Assert.That(double.IsFinite(c.X) && double.IsFinite(c.Y), Is.True);
                double err = Math.Sqrt(Math.Pow(c.X - (o.X + dx), 2) + Math.Pow(c.Y - (o.Y + dy), 2));
                Assert.That(err, Is.LessThanOrEqualTo(1e-10 * r + 4 * Math.Abs(off) * 1.2e-16), $"n={n} scale={scale} h={h}");
                Assert.That(c2.X, Is.EqualTo(c.X));
                Assert.That(c2.Y, Is.EqualTo(c.Y));
            }
        }

        [Test]
        [Description("Regression: the off-center (offconstant > 0) lies on the bisector of the shortest edge and is never farther from it than the circumcenter.")]
        public void FindCircumcenterOffCenter()
        {
            var pred = new RobustPredicates();
            // Skinny triangle: shortest edge org-dest.
            var o = new Point(0, 0); var d = new Point(1, 0); var a = new Point(0.5, 20);
            double xi = 0, eta = 0;
            var cc = pred.FindCircumcenter(o, d, a, ref xi, ref eta, 0.0);
            var off = pred.FindCircumcenter(o, d, a, ref xi, ref eta, 0.5);

            Assert.That(off.X, Is.EqualTo(0.5).Within(1e-15));
            Assert.That(off.Y, Is.EqualTo(0.5).Within(1e-15)); // 0.5*ydo + 0.5*xdo
            Assert.That(off.Y, Is.LessThan(cc.Y));
            Assert.That(eta, Is.EqualTo((off.Y) / 20).Within(1e-12));
        }

        #endregion
    }
}
