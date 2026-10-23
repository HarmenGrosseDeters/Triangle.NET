# Robustheitsanalyse Triangle.NET

Stand: Oktober 2026, Commit `fa6f2fd` (master). Fokus: Verhalten bei topologisch unsauberen
Eingaben und Gleitkomma-Grenzfällen.

> **Update:** Alle Befunde sind inzwischen behoben, siehe [Abschnitt „Fixes“](#fixes). Die
> Analyse unten beschreibt den Zustand **vor** den Fixes. Die Bewertung des alten,
> angepassten Stands steht in [Alter-Stand-Bewertung.md](Alter-Stand-Bewertung.md).

## Kurzfazit

* **Die reine Delaunay-Triangulation von Punktmengen ist sehr robust.** Die adaptiven exakten
  Prädikate von Shewchuk sind korrekt portiert. Doppelte und fast doppelte Punkte, Kollinearitäten,
  Raster, fast kozirkuläre Punkte und Koordinaten bis 1e9 liefern mit Dwyer (Default) und
  Incremental korrekte Netze.
* **Die Schwachstelle sind sich kreuzende Segmente.** Bei zufälligen PSLGs auf einem ganzzahligen
  Raster (Kreuzungen, Berührungen, kollineare Überlappungen) scheitern **43 von 200 Eingaben
  (≈ 22 %)**: in 28 Fällen **stillschweigend** mit einem Netz aus invertierten Dreiecken, in
  15 Fällen mit einer Exception. Ein 5-Segment-Fall bringt durch einen Stack Overflow den
  **ganzen Prozess** zum Absturz. Drei kreuzende Segmente reichen bereits aus.
* **Das ist kein Portierungsfehler.** Ein Differenztest gegen das C-Original (Triangle 1.6) zeigt
  bei allen 43 Fällen dasselbe Fehlverhalten. Die Ursache liegt im Algorithmus: Schnittpunkte
  werden auf `double` gerundet eingefügt und liegen danach auf keinem der beiden Segmente exakt.
* **Gut umgehbar:** Wenn die Schnittpunkte vorab berechnet und die Segmente geteilt werden
  (Noding), scheitern **0 von 200** Eingaben.
* **Thread-Sicherheit:** `RobustPredicates.Default` ist **nicht thread-sicher**. Bei parallelem
  Vermaschen liefern genau die heiklen, fast degenerierten Prädikat-Aufrufe falsche Ergebnisse.
  Abhilfe: pro Mesher eine eigene `RobustPredicates`-Instanz verwenden.

Mit den Workarounds unten (Noding, eigene Prädikate, Steiner-Limit, Eingabeprüfung) ist die
Bibliothek für unsaubere Daten gut einsetzbar.

## Methodik

1. **Code-Review** der robustheitsrelevanten Teile (`RobustPredicates`, `ConstraintMesher`,
   `Mesh.InsertVertex/TransferNodes`, Triangulatoren, `QualityMesher`, Smoother).
2. **Sondierungstests** mit gezielt „schmutzigen" Eingaben, z. B. Duplikate, Fast-Duplikate,
   Selbstüberschneidungen, Spikes, kollineare Überlappungen, berührende Löcher, große
   Koordinaten, kleine Winkel und Slivers zwischen Nachbarflächen.
3. **Fuzzing** mit zufälligen Segmenten (ganzzahlig und reell), Prüfung auf Konsistenz,
   invertierte Dreiecke, vollständig enthaltene Eingabesegmente, Gesamtfläche und Terminierung.
4. **Minimierung** jedes Fehlschlags auf den kleinsten reproduzierenden Fall (Delta-Debugging,
   jede Prüfung in eigenem Prozess wegen der Abstürze).
5. **Differenztest gegen das C-Original** (Shewchuks Triangle über das PyPI-Paket `triangle`),
   um geerbte Fehler von Portierungsfehlern zu trennen.
6. **Verifikation der Ursache** durch einen probeweisen Fix, wo das mit wenig Aufwand ging
   (Smoother: rot → grün).

## Befunde

| # | Befund | Schwere | Geerbt aus C? | Test |
|---|--------|---------|---------------|------|
| 1 | Kreuzende Segmente erzeugen **stillschweigend** ungültige Netze (invertierte Dreiecke) | kritisch | ja | `SegmentIntersectionTest.ThreeCrossingSegmentsProduceInvalidMesh` |
| 2 | Kreuzende Segmente: Endlosrekursion `ScoutSegment`/`ConstrainedEdge` → StackOverflow, **Prozess stirbt** | kritisch | ja (C hängt) | `SegmentIntersectionTest.FiveCrossingSegmentsCrashProcess` |
| 3 | Kreuzende Segmente: `Exception("Topological inconsistency after splitting a segment")` | hoch | ja | `SegmentIntersectionTest.ThreeCrossingSegmentsThrow`, `RandomIntegerSegments` |
| 4 | `RobustPredicates.Default` nicht thread-sicher (gemeinsame Arbeits-Arrays) | kritisch bei Parallelbetrieb | nein (C ist single-threaded) | `ThreadSafetyTest.DefaultPredicatesInParallel` |
| 5 | Dwyer (Default) mit nur identischen Punkten: Endlosrekursion → StackOverflow | hoch | – | `InputValidationTest.AllPointsIdenticalDwyer` |
| 6 | `SimpleSmoother` bei großen Koordinaten (z. B. UTM): hängt/stürzt ab | hoch | nein | `MiscellaneousBugsTest.SmoothMeshWithLargeCoordinates` |
| 7 | NaN/Infinity-Koordinaten: `IndexOutOfRangeException` tief in den Prädikaten | mittel | – | `InputValidationTest.NaNCoordinate`, `InfiniteCoordinate` |
| 8 | Vertex-IDs: Heuristik „erste zwei IDs verschieden ⇒ alle verschieden" → `ArgumentException: same key` | mittel | nein | `InputValidationTest.PartiallyDuplicateUserIds`, `ReuseMeshVerticesAndAddVertex` |
| 9 | `Vertex.Equals` (Koordinaten) und `GetHashCode` (interner Hash) widersprechen sich | mittel | nein | `MiscellaneousBugsTest.VertexEqualsAndHashCodeContract` |
| 10 | SweepLine: bei fast kozirkulären Punkten nicht exakt Delaunay (Abweichungen ~1e-16) | niedrig | vermutlich | `MiscellaneousBugsTest.SweepLineNearlyCocircularPoints` |
| 11 | SweepLine wirft bei nur identischen Punkten, Incremental liefert ein leeres Netz | niedrig | – | `InputValidationTest.AllPointsIdenticalSweepLine` |

### Details und Ursachen

**1–3 Kreuzende Segmente** (`src/Triangle/Meshing/ConstraintMesher.cs:663`, `SegmentIntersection`)
Beim Einfügen eines Segments, das ein bereits eingefügtes Segment kreuzt, wird der Schnittpunkt in
Gleitkomma berechnet und als neuer Knoten eingefügt. Der gerundete Punkt liegt nicht exakt auf den
Segmenten. Kreuzen sich drei Segmente auf engem Raum, bilden die gerundeten Schnittpunkte ein
winziges Dreieck mit falscher Orientierung. Dann
* wirft `SegmentIntersection` die Exception aus Zeile 768, oder
* entstehen invertierte Dreiecke **ohne Fehlermeldung** (gefährlichster Fall), oder
* pendeln `ScoutSegment` und `ConstrainedEdge` endlos (StackOverflow, im C-Original eine
  Endlosschleife).

Minimalbeispiel (Quadrat 0..10 plus drei Segmente):
`(2,4)-(4,8)`, `(1,5)-(6,6)`, `(1,7)-(5,3)` → inkonsistentes Netz ohne Exception.

Das C-Original verhält sich identisch. Von den 200 Fuzz-Eingaben scheitern in Triangle.NET 43,
in C 44 (die 43 plus eine weitere). Mit vorab geteilten Segmenten (Noding) scheitert keine der
200 Eingaben, auch nicht beim Runden der Schnittpunkte auf 1e-6.

**4 Thread-Sicherheit** (`src/Triangle/RobustPredicates.cs:37`, `:1259`)
`RobustPredicates.Default` ist ein Singleton. Die exakte Arithmetik schreibt Zwischenergebnisse in
Instanz-Arrays (`fin1`, `abdet`, `axbc` …). Im Test gaben 4 Threads auf fast kozirkulären
Punkten **über 100.000 falsche Vorzeichen** zurück. Betroffen sind also genau die Fälle, für die
die exakte Arithmetik da ist. Die Instanz wird von `new Configuration()` verwendet und zusätzlich
direkt von `MeshValidator`, `Contour.FindInteriorPoint`, den Voronoi-Klassen und `SimpleSmoother`.
`MeshValidator` schaltet außerdem das statische `Behavior.NoExact` um, und `Statistic` sowie
`Log.Instance` sind globale, nicht synchronisierte Zustände.

**5 Dwyer mit identischen Punkten** (`src/Triangle/Meshing/Algorithm/Dwyer.cs:487`)
Nach dem Entfernen der Duplikate bleibt ein einziger Punkt übrig. `DivconqRecurse` hat für
`vertices == 1` keinen Abbruchfall.

**6 Smoother** (`src/Triangle/Smoothing/SimpleSmoother.cs:181`)
Der Zellschwerpunkt wird per Shoelace-Formel mit **absoluten** Koordinaten berechnet.
Bei Koordinaten um 1e6 sind die Produkte ~1e12, die Zellfläche ~1e-1, und die Auslöschung frisst
alle signifikanten Stellen. Die Division durch die falsch berechnete Fläche liefert Unsinn/NaN.
Verifiziert: Werden die Koordinaten relativ zum Generator der Zelle gerechnet, ist der Test grün.

**7 NaN/Infinity**: Es gibt keine Eingabeprüfung. Die Ausnahme entsteht in
`FastExpansionSumZeroElim`, wo bereits ein Workaround gegen Indexfehler im Code steht
(`RobustPredicates.cs:639`). Im Smoother-Kontext führen NaN-Werte auch zu Hängern.

**8 Vertex-IDs** (`src/Triangle/Mesh.cs:400`): Sind die IDs der ersten beiden Punkte verschieden,
werden alle IDs als eindeutig angenommen. Ein realistischer Auslöser ist
`mesh.Vertices` eines bestehenden Netzes plus ein neuer Punkt (ID 0).

**9 Hashcode-Vertrag** (`src/Triangle/Geometry/Vertex.cs:123`): Zwei koordinatengleiche Vertices
sind `Equals`, haben aber verschiedene Hashcodes. `HashSet<Vertex>` bzw.
`Dictionary<Vertex,…>` verhalten sich dadurch fehlerhaft.

**10 SweepLine**: Die Reihenfolge der Kreis-Ereignisse wird in Gleitkomma bestimmt
(`SweepLine.cs:655`, `CircleTop`). Das Ergebnis ist konsistent, aber nicht exakt Delaunay.

### Kein Bug, aber eine Praxisfalle: Slivers + Qualitätsvermaschung

Zwei Nachbarflächen, deren gemeinsame Kante doppelt und minimal abweichend digitalisiert wurde,
erzeugen schmale Slivers. Mit `MinimumAngle` **terminiert die Verfeinerung praktisch nicht**,
weil ein Sliver der Breite ε etwa Länge/ε Dreiecke braucht (gemessen: unbegrenzt > 30 s bzw.
400.000 Dreiecke bei 200.000 Steinerpunkten). Ohne Qualitätsoptionen oder mit
`QualityOptions.SteinerPoints` ist das Ergebnis konsistent. Das Verhalten ist algorithmisch
korrekt, es fehlt aber eine Standard-Obergrenze.

### Was robust funktioniert (grüne Regressionstests)

Die folgenden Fälle stehen in `RobustnessRegressionTest`:
geschlossene Ringe mit wiederholtem Startpunkt (GIS-Konvention), selbstüberschneidende Konturen
(Bowtie), kollinear überlappende Segmente, doppelte/umgekehrte Segmente, Spikes, Fast-Duplikate
(bis 1e-15), Punkte fast auf Segmenten, Segmente durch Punkte, kleine Eingabewinkel bis 0,01°,
dünne Löcher, Lochpunkt auf dem Rand, sich berührende Löcher, Loch am Außenrand, große
Koordinaten (1e5–1e9) mit Millimeter-Details, zweifaches Verfeinern, Maximalwinkel,
Raster mit allen drei Algorithmen, zufällige reelle Kreuzungen inkl. Qualitätsvermaschung,
paralleles Vermaschen mit eigenen Prädikaten.

## Empfehlungen für den Einsatz mit unsauberen Daten

1. **Eingabe vorher noden.** Alle Segmentschnitte selbst berechnen und die Segmente dort teilen,
   damit der Mesher nie kreuzende Segmente sieht. Für GIS-Daten bietet sich NetTopologySuite an
   (Noding bzw. Snap-Rounding, z. B. `GeometryNoder`/`SnapRoundingNoder`). Das beseitigt
   Befunde 1–3 und im Test alle 43 Fehlschläge.
2. **Gemeinsame Punkte als dieselben `Vertex`-Objekte übergeben** oder vorab auf ein Raster
   snappen. Das ist nicht zwingend nötig (Duplikate werden erkannt), vermeidet aber Slivers.
3. **Bei Parallelbetrieb eigene Prädikate pro Mesher:**
   `new GenericMesher(new Configuration(() => new RobustPredicates()))`.
   `MeshValidator` und `SimpleSmoother` nicht parallel verwenden.
4. **Immer ein `SteinerPoints`-Limit setzen** und die Laufzeit begrenzen (`CancellationToken`).
5. **Ergebnis validieren:** `MeshValidator.IsConsistent(mesh)` nach der Triangulation, weil
   Fehler sonst still bleiben können. Bei Fehlschlag Fallback nutzen (z. B. Noding/Snapping mit
   gröberem Raster).
6. **Eingabe prüfen:** NaN/Infinity filtern, IDs eindeutig setzen oder alle auf 0 lassen.
7. **Für den Smoother Koordinaten zentrieren** (lokales Koordinatensystem) oder den Smoother-Fix
   abwarten.
8. **Default-Algorithmus Dwyer beibehalten.** Er ist exakt Delaunay; SweepLine ist es nicht.

## Fixes

| # | Fix | Datei |
|---|-----|-------|
| 1–3 | **Noding mit Snapping** vor dem Einfügen der Segmente (`ConstraintOptions.ResolveIntersections`, Standard an): alle Schnittpunkte werden vorab berechnet (exakte Kreuzungserkennung), neue Punkte näher als r ≈ 1e-12 · Ausdehnung an einem bestehenden Punkt werden mit ihm verschmolzen, und Punkte näher als r an einem Segment werden ins Segment eingebunden. Eingabepunkte werden nie verschoben. Das Verfahren terminiert nachweisbar, weil neue Punkte einen Mindestabstand haben. | `Meshing/SegmentNoder.cs` |
| 1–3 | **Sicherheitsnetz:** `ScoutSegment`/`ConstrainedEdge` als Schleife statt Rekursion (kein StackOverflow), Schrittzähler gegen Endlosschleifen, Orientierungsprüfung nach dem Einfügen von Schnittpunkten. Ungültige Netze werden damit nie still zurückgegeben. | `Meshing/ConstraintMesher.cs` |
| 4 | Arbeits-Arrays der exakten Arithmetik thread-lokal; jede `RobustPredicates`-Instanz ist jetzt thread-sicher. | `RobustPredicates.cs` |
| 5, 11 | Identische Punkte: leeres Netz statt StackOverflow (Dwyer) bzw. Exception (SweepLine). | `Dwyer.cs`, `SweepLine.cs` |
| 6 | Smoother: Schwerpunkt relativ zum Generator. | `Smoothing/SimpleSmoother.cs` |
| 7, 8 | NaN/Infinity → `ArgumentException`; vollständige Prüfung auf eindeutige IDs. | `Mesh.cs` |
| 9 | `Vertex` nutzt den koordinatenbasierten Hashcode von `Point`. | `Geometry/Vertex.cs` |
| 10 | SweepLine: abschließender Flip-Durchlauf mit exakten Prädikaten. | `SweepLine.cs` |
| neu | **aCute-Verfeinerung** (seit 2022 Standard) startete die Punktsuche an einer Kante, links von der der neue Punkt nicht liegt → Punkt im falschen Dreieck, invertierte Dreiecke. Jetzt wird die Startkante per Orientierungstest gewählt. | `Meshing/QualityMesher.cs` |
| neu | aCute-Ort fällt auf einen bestehenden Punkt → Rückfall auf den Umkreismittelpunkt statt Exception. | `Meshing/QualityMesher.cs` |

**Ergebnis (Fuzzing, jeder Fall in eigenem Prozess):**

| Szenario | vorher | nachher |
|----------|--------|---------|
| 15 zufällige Integer-Segmente, ohne Qualität | 157/200 | 500/500 |
| 40 Segmente, Mindestwinkel 25° | – (meist Fehler) | 300/300 |
| 40 Segmente, 30° | – | 200/200 |
| 15 Segmente, 20°, Offset 1e6 | – | 200/200 |
| 100 Segmente, 20° / 28° | – | 200/200, 100/100 |
| 200 Segmente auf 50er-Raster, 20° | – | 50/50 |
| Nachbarflächen mit Abweichung 1e-12, 25°, **ohne** Steiner-Limit | terminiert nicht | ~30 ms |

Die Testsuite hat jetzt 203 Tests, alle grün. Die früheren `KnownBug`-Tests sind reguläre
Regressionstests geworden.

## Die Testsuite (`src/Triangle.Tests/Robustness`)

* **Red Tests** tragen die Kategorie `KnownBug` und dokumentieren jeweils einen Fehler mit
  Schweregrad in der `[Description]`. Wird ein Fehler behoben, wird der Test grün; dann die
  Kategorie entfernen, und er wird zum Regressionstest. Derzeit gibt es keine offenen
  `KnownBug`-Tests.
* **Grüne Tests** (`RobustnessRegressionTest` u. a.) sichern das heute korrekte Verhalten ab.
* `MeshAssert` prüft Konsistenz, invertierte Dreiecke, vollständige Eingabesegmente, Fläche und
  Terminierung (mit Timeout).
* Fälle, die den Prozess abstürzen lassen (StackOverflow) oder hängen, laufen über
  `IsolatedProcess` in einem **Kindprozess**: Die Test-Assembly hat dafür einen eigenen
  Einstiegspunkt (`Program.cs`). So reißen sie nicht den ganzen Testlauf mit.
* CI (`.github/workflows/dotnet.yml`) führt die regulären Tests blockierend und die `KnownBug`-Tests
  informativ (`continue-on-error`) aus.

```
dotnet test src/Triangle.Tests --filter "TestCategory!=KnownBug"   # muss grün sein
dotnet test src/Triangle.Tests --filter "TestCategory=KnownBug"    # bekannte Bugs (rot)
```

Für jede neue Bug-Vermutung: Minimalfall als Test mit `[Category(KnownBug.Category)]` anlegen,
mit `MeshAssert` die erwartete Eigenschaft prüfen und bei Absturz- oder Hängegefahr ein Szenario in
`IsolatedScenarios` hinterlegen.
