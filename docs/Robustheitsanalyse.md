# Robustheitsanalyse Triangle.NET

Stand: Oktober 2026, Commit `fa6f2fd` (master). Fokus: Verhalten bei topologisch unsauberen
Eingaben und Gleitkomma-Grenzfällen.

> **Update:** Alle Befunde sind inzwischen behoben, siehe [Abschnitt „Fixes“](#fixes). Die
> Analyse unten beschreibt den Zustand **vor** den Fixes. Die Bewertung des alten,
> angepassten Stands steht in [Alter-Stand-Bewertung.md](Alter-Stand-Bewertung.md).
>
> **Zweite Runde (Qualitätsvermaschung):** Drei Fehler der aCute-Verfeinerung (auch in Upstream)
> und ein Noder-Fehler beim Snapping sind behoben; auf ALKIS-Daten (Flurstücke, Gebäude, Nutzung)
> sank der Anteil schlechter Dreiecke von 8,7 % auf 1,1 %. Siehe
> [Ergebnisse mit echten Daten](#ergebnisse-mit-echten-daten-alkis-und-dgm1-herne-opengeodatanrwde)
> und [Absicherung für Performance-Arbeit](#absicherung-für-performance-arbeit-srctriangletestscharacterization).
>
> **Dritte Runde (Raster, Keile, Gelände):** Die 1,1 % gelten für die ungerundeten Koordinaten.
> Nach dem Runden auf cm sind es 6–10 %, und rund 1 % der Dreiecke kippt. Mit Raster und
> Keil-Toleranz sind es 2,2 % ohne gekippte Dreiecke. Neu sind Snap-Rounding
> auf das Ausgaberaster (`GridSize`), das Zusammenführen dünner Keile (`WedgeTolerance`) und eine
> TIN-Vereinfachung für DGM-Raster (`TerrainSimplifier`), siehe
> [Raster, Keile und Geländevereinfachung](#raster-keile-und-geländevereinfachung).

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
| 1–3 | **Noding mit Snapping** vor dem Einfügen der Segmente (`ConstraintOptions.ResolveIntersections`, Standard an). Toleranz r ≈ 1e-12 × Koordinatengröße (bei UTM ≈ 5 µm). Punkte näher als r zueinander werden verschmolzen (exakte Duplikate bleiben erhalten). Alle Schnittpunkte werden vorab berechnet (exakte Kreuzungserkennung) und mit Punkten in Reichweite r zusammengefasst. Punkte näher als r an einem Segment werden ins Segment eingebunden. Konvergiert das nicht (z. B. viele fast deckungsgleiche Kanten), wird die Toleranz in Stufen bis 2,3e-10 × Koordinatengröße (UTM ≈ 1,4 mm) erhöht, wie beim Snapping-Noder von GEOS. Danach gibt es eine klare `InvalidOperationException`, keinen Hänger. | `Meshing/SegmentNoder.cs` |
| 1–3 | **Sicherheitsnetz:** `ScoutSegment`/`ConstrainedEdge` als Schleife statt Rekursion (kein StackOverflow), Schrittzähler gegen Endlosschleifen, Orientierungsprüfung nach dem Einfügen von Schnittpunkten. Ungültige Netze werden damit nie still zurückgegeben. | `Meshing/ConstraintMesher.cs` |
| 4 | Arbeits-Arrays der exakten Arithmetik thread-lokal; jede `RobustPredicates`-Instanz ist jetzt thread-sicher. | `RobustPredicates.cs` |
| 5, 11 | Identische Punkte: leeres Netz statt StackOverflow (Dwyer) bzw. Exception (SweepLine). | `Dwyer.cs`, `SweepLine.cs` |
| 6 | Smoother: Schwerpunkt relativ zum Generator. | `Smoothing/SimpleSmoother.cs` |
| 7, 8 | NaN/Infinity → `ArgumentException`; vollständige Prüfung auf eindeutige IDs. | `Mesh.cs` |
| 9 | `Vertex` nutzt den koordinatenbasierten Hashcode von `Point`. | `Geometry/Vertex.cs` |
| 10 | SweepLine: abschließender Flip-Durchlauf mit exakten Prädikaten. | `SweepLine.cs` |
| neu | **aCute-Verfeinerung** (seit 2022 Standard) startete die Punktsuche an einer Kante, links von der der neue Punkt nicht liegt → Punkt im falschen Dreieck, invertierte Dreiecke. Jetzt wird die Startkante per Orientierungstest gewählt. | `Meshing/QualityMesher.cs` |
| neu | aCute-Ort fällt auf einen bestehenden Punkt → Rückfall auf den Umkreismittelpunkt statt Exception. | `Meshing/QualityMesher.cs` |
| neu | **Präzisionsgrenze bei der Verfeinerung:** Würde das Teilen eines Segments (mit exakter Prüfung) ein flaches/invertiertes Dreieck erzeugen oder fällt der Teilungspunkt auf einen Endpunkt, wird das Teilsegment nicht geteilt und für den Rest der Verfeinerung gesperrt (statt still invertierter Dreiecke bzw. „Ran out of precision“). Zu kleine Dreiecke bleiben stehen (statt „new vertex at circumcenter“). Abschließende Orientierungsprüfung → laute Exception statt ungültigem Netz. | `Meshing/QualityMesher.cs` |
| neu | **`ConstraintOptions.SnapTolerance`** (Standard 0 = automatisch): Mindesttoleranz des Noders in Koordinateneinheiten, passend zur Datengenauigkeit (z. B. `0.01` = 1 cm). Fast deckungsgleiche Linien (Gebäude an Flurstücksgrenzen, doppelt digitalisierte Grenzen) werden zusammengeführt statt zu Slivers vermascht. Die Geometrie ändert sich dabei um höchstens die Toleranz. | `Meshing/ConstraintOptions.cs`, `Meshing/SegmentNoder.cs` |
| neu | Snap-Zyklus: Bei großer Toleranz konnte ein Punkt in ein Segment gesnappt werden, dessen Teilstück dann den ursprünglichen Endpunkt wieder einfing (Endlosschleife bis zum Iterationslimit → Exception). Bereits geteilte Segmente werden jetzt nicht erneut aus ihren Teilstücken erzeugt. | `Meshing/SegmentNoder.cs` |
| neu | **aCute: schlechte Dreiecke gingen verloren.** Der aCute-Ort liegt nicht immer im Umkreis des schlechten Dreiecks; dann blieb das Dreieck nach dem Einfügen bestehen, war aber schon aus der Warteschlange entfernt. Folge: `MaximumArea` wurde verletzt (Quadrat, 20°: 9 zu große Dreiecke), und auf ALKIS-Daten blieben viele Dreiecke unter dem Mindestwinkel (Kachel 381000/5710400, 30°: **618 statt 9**, zugleich 20× langsamer). Jetzt wird ein aCute-Ort nur verwendet, wenn er echt im Umkreis und nicht (fast) auf einer Ecke liegt, und ein überlebendes Dreieck wird über den Umkreismittelpunkt geteilt. Fehler auch in Upstream. | `Meshing/QualityMesher.cs` |
| neu | **aCute: Endlosverschiebung von Punkten.** aCute verschiebt („relocation“) Punkte, indem es sie löscht und neu einfügt. Mit Flächenvorgaben pro Region wurde derselbe Punkt immer wieder hin und her verschoben. Upstream wirft „The new vertex is at the circumcenter of triangle“; nach den ersten Fixes lief die Verfeinerung bis zum Steiner-Limit und lieferte ein grobes Netz (5 Dreiecke). Jetzt wird jeder Punkt höchstens einmal verschoben. | `NewLocation.cs`, `Meshing/QualityMesher.cs` |
| neu | **Noder: Schnittpunkt am Segmentende blockierte das Snapping.** Ein Schnittpunkt nahe einem Segmentende wurde mit dem Endpunkt verschmolzen und das Segment dort „geteilt“. Das änderte nichts, markierte das Segment aber als bereits geteilt, sodass ein fast kollinearer Punkt nicht mehr hineingesnappt wurde. Zwei Segmente mit 0,0002° Zwischenwinkel blieben stehen, und die Qualitätsvermaschung verbrauchte das gesamte Steiner-Budget in diesem Keil (ALKIS Herne mit Nutzung, 1 cm: 203.175 Dreiecke, davon 154.870 unter 20° → 24.355 Dreiecke, 222 unter 20°). Die unnötigen Teilungen verbrauchten außerdem Noding-Iterationen: Gestapelte Quadrate mit 1e-6 Versatz werden jetzt vernetzt statt abgelehnt. | `Meshing/SegmentNoder.cs` |
| neu | **Segmente, die nicht geteilt werden dürfen** (`SegmentSplitting` 1/2, `CanSplitSegment`): Lag der neue Punkt im Umkreis eines solchen Segments oder jenseits davon, wurde das schlechte Dreieck ersatzlos verworfen (Upstream-Issue #9: Rechteck 10×20 mit Maximalfläche 1,73 ergab 8 Dreiecke). Jetzt wird der Punkt trotzdem eingefügt bzw. ein zu großes Dreieck am Schwerpunkt geteilt. Segmente, die nur wegen der Präzisionsgrenze nicht mehr geteilt werden, sind ausgenommen. | `Meshing/QualityMesher.cs` |
| neu | **Beeinträchtigte Segmente vor dem Einfügen prüfen:** Statt einen Punkt einzufügen, Kanten zu flippen und bei einem beeinträchtigten Segment alles zurückzunehmen, wird der Hohlraum vorab mit denselben Tests bestimmt. Nadelförmige Gebiete (GEOS `TestRelateAA#74`, 80.000 Punkte): 79 s → 2,1 s; der GEOS-Korpus läuft jetzt ohne Timeout. | `Meshing/QualityMesher.cs` |
| neu | `FastExpansionSumZeroElim` las (wie das C-Original) ein Element hinter dem Ende einer Expansion; in C# eine `IndexOutOfRangeException`, ausgelöst durch unendliche Koordinaten. | `RobustPredicates.cs` |

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

Die Testsuite hat jetzt 752 Tests, alle grün. Die früheren `KnownBug`-Tests sind reguläre
Regressionstests geworden.

### Externer Korpus: GEOS-Robustheitstests

Die XML-Testsuite von GEOS (`tests/xmltester/tests`) enthält 2156 Fälle, an denen Overlay/Noding
in JTS, GEOS, PostGIS, QGIS oder Shapely gescheitert ist (OSM-Wasserflächen, Isochronen,
Mikropolygone mit Koordinatenrauschen, fast deckungsgleiche Kanten, Nadeldreiecke). Jeder Fall
wurde als PSLG trianguliert (alle Kanten von A und B plus Rahmen), ohne und mit 20°
Mindestwinkel, jeweils in eigenem Prozess:

| Stand | ohne Qualität | 20° |
|-------|---------------|-----|
| nach den ersten Fixes | 2142 gültig, 7 Abstürze/Exceptions, 3 Hänger | 2105 gültig, 31 Exceptions, **2 still inkonsistent**, 5 Hänger |
| jetzt | **2152 gültig** | **2151 gültig**, 1 langsam (> 60 s) |

Die verbleibenden 4 Fälle je Spalte enthalten absichtlich NaN/Inf und werden korrekt mit
`ArgumentException` abgewiesen. Der langsame Fall (extrem lange Nadeldreiecke, 10⁶ : 1) ist
gültig, braucht aber ~70 s. Der Original-Stand ist dort 30× schneller, liefert aber bis zu 18.000
invertierte Dreiecke. Bei normalen Daten liegt der Mehraufwand der Fixes bei 0–25 %.

Der Korpuslauf ist Teil der Testsuite (Kategorie `Corpus`, explizit), ebenso Läufe mit
**ALKIS-Flurstücken** und **DGM1** aus NRW:

```
# GEOS: git clone --depth 1 --filter=blob:none --sparse https://github.com/libgeos/geos
#       git -C geos sparse-checkout set tests/xmltester/tests
export TRIANGLE_GEOS_TESTS=/pfad/geos/tests/xmltester/tests
# NRW (Open Data, DL-DE Zero 2.0): Flurstücke als GeoJSON (UTM32), DGM1 als xyz(.gz)
export TRIANGLE_NRW_ALKIS=/pfad/flurstuecke.geojson
export TRIANGLE_NRW_DGM=/pfad/dgm1_32_xxx_yyyy_1_nw.xyz.gz
export TRIANGLE_CORPUS_TILES=200   # Anzahl Kacheln (Standard 200)
export TRIANGLE_CORPUS_TILE=200    # Kachelgröße in m (Standard 200)
dotnet test src/Triangle.Tests --filter TestCategory=Corpus
```

Die NRW-Läufe zerlegen die Flurstücke in Kacheln (vollständige Flurstücke inklusive ihrer
Nachbarn, also mit doppelt digitalisierten gemeinsamen Grenzen) bzw. kombinieren ein 1-m-Raster
des DGM1 mit den Flurstücksgrenzen als Bruchkanten. `TRIANGLE_CORPUS_ANGLES` (z. B. `20,25,30,33`)
legt die Mindestwinkel fest; die Ausgabe enthält je Winkel Laufzeit, Dreiecke, Anzahl der Fälle
am Steiner-Limit, Anteil schlechter Dreiecke und die schlechtesten Kacheln.

### Ergebnisse mit echten Daten: ALKIS und DGM1 Herne (opengeodata.nrw.de)

201 Kacheln à 200 m (gleichmäßig über das Stadtgebiet verteilt), jeweils 20°, 25°, 30° und 33°
Mindestwinkel, 100.000 Steiner-Punkte, jeder Lauf in eigenem Prozess (Release):

| Daten | Snap | gültig | schlechte Dreiecke 20° / 30° | vorher (vor aCute- und Noder-Fix) |
|-------|------|--------|-----------------------------|-----------------------------------|
| Flurstücke | 5 µm (auto) | 804/804 | 0,44 % / 0,47 % | nicht gemessen (Einzelkachel 381000/5710400, 30°: 618 statt 9 schlechte Dreiecke) |
| Flurstücke | 1 cm | 804/804 | 0,44 % / 0,47 % | 0,44 % / 1,08 % |
| Flurstücke + Gebäude + Nutzung | 1 cm | 804/804, kein Steiner-Limit | **1,10 % / 0,98 %** | 8,70 % / 10,74 %, 1–2 Kacheln am Steiner-Limit |
| DGM1-Raster (1 m) + Flurstücksgrenzen | 5 µm | 8/8 | – | – |

Bei Flurstücken + Gebäuden + Nutzung ohne Snapping laufen einzelne Kacheln ans Steiner-Limit
(z. B. 400.000 statt 50.000 Dreiecke), deshalb ist 1 cm im Fallback-Adapter jetzt Standard.

Die verbleibenden schlechten Dreiecke liegen fast alle an Segmenten, in Keilen sehr kleiner
Eingabewinkel: Linien verschiedener Ebenen (Nutzung, Gebäude, Flurstück), die über viele Meter
mit 0,01–0,1° auseinanderlaufen und erst nach mehr als 1 cm getrennt sind. Dort verfeinert
Triangle bewusst nicht weiter (sonst terminiert die Verfeinerung nicht). Abhilfe ist nur eine
größere Snap-Toleranz, also eine Datenentscheidung: In der schlechtesten Kachel sinkt der Anteil
mit 2 cm von 1.931 auf 105 schlechte Dreiecke (und halb so viele Dreiecke).

## Absicherung für Performance-Arbeit (`src/Triangle.Tests/Characterization`)

Performance-Optimierungen dürfen das Ergebnis nicht unbemerkt verändern. Dafür gibt es drei
Bausteine, die alle auf demselben Szenario-Katalog `MeshCases` arbeiten (77 Szenarien: Punktmengen
mit allen drei Algorithmen, Polygone mit Loch, Stern mit 5°-Winkeln, kreuzende Segmente,
Flurstücke mit 1-mm-Abweichungen, Bruchkanten in Gauß-Krüger-Koordinaten, drei echte
ALKIS-Kacheln aus Herne; jeweils CDT, Conforming Delaunay, 20°/25°/28° (legacy)/30°/33°,
`SegmentSplitting` 1/2, gesperrte Segmente, Flächenvorgaben fest und pro Region, zweifaches
Verfeinern, Glätten, Snapping 1 cm):

| Baustein | Prüft | Zweck |
|----------|-------|-------|
| `InvariantTest` | Konsistenz, keine flachen/invertierten Dreiecke, **(eingeschränkt) Delaunay** mit exaktem InCircle-Test, Fläche gegenüber der unverfeinerten Triangulierung, alle Eingabesegmente vorhanden, gesperrte Segmente ungeteilt, Mindestwinkel (wo garantiert), Maximalfläche | Eigenschaften, die jede korrekte Implementierung erfüllen muss. Gilt auch nach Optimierungen, die das Ergebnis legitim ändern. |
| `GoldenMeshTest` | Fingerabdruck (SHA-256 über alle Dreiecke und Teilsegmente, bitgenau, unabhängig von der Reihenfolge) gegen `golden.txt` | Verhaltensneutrale Optimierungen müssen alle Fingerabdrücke erhalten. Ändert eine Optimierung das Ergebnis bewusst: Invarianten prüfen, Qualitätsstatistik vergleichen, dann mit `TRIANGLE_UPDATE_GOLDEN=1` neu erzeugen und die Änderung im Commit begründen. |
| `Triangle.Benchmarks` | BenchmarkDotNet (Zeit, Allokationen) für 12 repräsentative Szenarien | Messen vor/nach jeder Optimierung. |

```
dotnet test src/Triangle.Tests --filter TestCategory=Characterization
TRIANGLE_UPDATE_GOLDEN=1 dotnet test src/Triangle.Tests --filter Name~Golden   # bewusst neu erzeugen
dotnet run -c Release --project src/Triangle.Benchmarks -- --filter "*"
```

Die Invariantentests haben bereits beim Aufbau drei echte Fehler gefunden (aCute, siehe Fixes)
und eine Präzisionsgrenze sichtbar gemacht: Ein auf ein Segment eingefügter Punkt wird auf den
nächsten darstellbaren Punkt gerundet und kann einige ulp neben dem Segment liegen. Die Kanten an
diesem Punkt werden (wie im C-Original) nicht geflippt, sodass der exakte InCircle-Test in seltenen
Fällen eine Delaunay-Verletzung meldet. Das ist für die Gültigkeit des Netzes unerheblich und wird
im Test als Rundungsartefakt erkannt (Kollinearität bis auf 8 ulp).

Die Zeilenabdeckung der Kernklassen liegt bei 87–98 % (`Mesh` 92 %, `QualityMesher` 92 %,
`ConstraintMesher` 87 %, `SegmentNoder` 98 %, Prädikate 93 %); `NewLocation` (aCute) 74 %.

### Gemessene Hotspots (Ausgangspunkt für die Performance-Arbeit)

Profil mit `dotnet-trace` (Sampling, Release):

* **Reale Daten** (ALKIS-Kachel, 30°, Snap 1 cm): Die aCute-Ortsbestimmung (`NewLocation`)
  braucht rund 45 % der Zeit, davon `GetNeighborsVertex` 19 %, `SplitConvexPolygon` 9 % und
  `HalfPlaneIntersection` 4 %; `Mesh.InsertVertex` 23 %. Die Allokationen liegen bei rund 100 MB
  für 100.000 Dreiecke.
* **Pathologischer Fall** (GEOS `TestRelateAA#74`, zwei Nadeldreiecke 10⁶ : 1, 8 Eingabepunkte):
  92 % der Zeit in `InsertVertex` und `UndoVertex`. Ein Umkreismittelpunkt wird eingefügt, löst
  eine lange Flip-Kaskade aus, beeinträchtigt dann ein Segment („encroaching“) und wird komplett
  zurückgenommen. Das ist quadratisch (100.000 Punkte in 85 s statt rund 1 s). Ansatz: das
  Beeinträchtigen von Segmenten vor dem Umbau des Netzes prüfen.

Ausgangswerte (`Triangle.Benchmarks`, `--job short`, Release, Container mit 4 Kernen):

| Szenario | Zeit | Allokationen |
|----------|-----:|-------------:|
| points-random/dwyer (3.000 Punkte) | 3,0 ms | 1,8 MB |
| points-grid/dwyer (1.600 Rasterpunkte) | 3,3 ms | 7,3 MB |
| points-utm/sweepline | 4,0 ms | 2,3 MB |
| points-random/q30 | 44 ms | 10,5 MB |
| crossing-segments/q20 | 4,6 ms | 2,0 MB |
| utm-breaklines/q33 | 11,5 ms | 4,0 MB |
| regions/q25-variable-area | 5,7 ms | 1,8 MB |
| square/smooth | 2,5 ms | 1,7 MB |
| parcels-mismatch/q20 (Steiner-Limit) | 1.513 ms | 246 MB |
| herne_371800_5710600/q20-snap1cm | 42 ms | 9,4 MB |
| herne_373600_5709000/q20-snap1cm | 317 ms | 51 MB |
| herne_373600_5709000/q30-snap1cm | 902 ms | 104 MB |

Vorgehen pro Optimierung: Benchmark vorher/nachher (Standard-Job), Charakterisierungstests grün
(bei verhaltensneutralen Änderungen unveränderte Fingerabdrücke).

### Performance-Optimierung

Profil der realen Fälle: In großen Netzen gingen 25–30 % der Laufzeit in Garbage-Collection-
Pausen. Das wachsende Netz ist komplett lebendig, und jede Sammlung kopiert alle neuen Objekte –
die Pausen hängen an der **Anzahl** der Objekte. Jedes Dreieck bestand aus vier Heap-Objekten
(das Dreieck und je ein Array für Ecken, Nachbarn, Teilsegmente), das waren über die Hälfte aller
Allokationen. Änderungen (alle verhaltensneutral, Golden-Fingerprints unverändert):

* Dreiecke und Teilsegmente speichern Ecken, Nachbarn und Teilsegmente unter .NET 8 als
  `[InlineArray]` direkt im Objekt (die Bibliothek baut dafür zusätzlich für `net8.0`;
  `netstandard2.0`, z. B. .NET Framework 4.8, behält die Arrays).
* `InCircleAdapt` legte bei jedem Aufruf rund 40 Hilfsarrays an; sie liegen jetzt im
  thread-lokalen Arbeitsspeicher (fast kozirkuläre Punkte, z. B. im Rastermodus, rufen ihn oft auf).
* Rastermodus: Die aCute-Relocation wurde berechnet und dann verworfen (24 % der Zeit), am Ende
  wurden für alle Punkte Sterne aufgebaut, obwohl fast alle schon auf dem Raster liegen.
* aCute-Keilschnitt: Winkelfunktionen einmal pro Aufruf statt pro Kante.

Gemessen mit BenchmarkDotNet (Standard-Job), alter und neuer Stand je Szenario direkt
hintereinander (zwei getrennte Läufe im Abstand von Stunden sind auf der Cloud-Maschine nicht
vergleichbar, die CPU-Leistung schwankte dort um 20–25 %):

| Szenario | vorher | nachher | |
|----------|-------:|--------:|---:|
| ALKIS-Kachel 30°, Snap 1 cm | 501 ms | 398 ms | −21 % |
| ALKIS-Kachel 30°, Raster 1 cm + Keil 3 cm | 638 ms | 327 ms | −49 % |
| ALKIS-Kachel 20°, Snap 1 cm | 178 ms | 147 ms | −17 % |
| kleine ALKIS-Kachel 20° | 26,1 ms | 23,5 ms | −10 % |
| Zufallspunkte 30° | 33,2 ms | 28,3 ms | −15 % |
| `parcels-mismatch` (Steiner-Limit) | 1.010 ms | 839 ms | −17 % |
| kreuzende Segmente 20° | 3,73 ms | 3,55 ms | −5 % |
| UTM-Bruchkanten 33° | 10,5 ms | 9,7 ms | −7 % |

Der Herne-Korpus (201 Kacheln, Raster + Keil) läuft in 60/92 s statt 109/180 s (20°/30°).

#### Speicherlayout für .NET Framework 4.8 (`TriangleLayout`)

`[InlineArray]` gibt es erst ab .NET 8. Für `netstandard2.0` (also net48) lässt sich beim Bauen
ein Layout wählen, bei dem Ecken, Nachbarn und Teilsegmente als einzelne Felder im Objekt
liegen (ein Heap-Objekt pro Dreieck statt vier). Der Zugriff läuft über kleine Strukturen mit
`ref`-Indexer, der übrige Code bleibt unverändert:

| Eigenschaft | Layout |
|-------------|--------|
| (Standard) | `net8.0`: InlineArray, `netstandard2.0`: Arrays |
| `-p:TriangleLayout=Fields` | einzelne Felder in allen Zielplattformen |
| `-p:TriangleLayout=Arrays` | Arrays in allen Zielplattformen (altes Verhalten) |

Alle Tests laufen in allen drei Varianten grün, die Golden-Fingerprints sind gleich. Auf .NET 10
(BenchmarkDotNet, je zwei Runden abwechselnd) ist `Fields` aber langsamer als `Arrays`, der
`ref`-Indexer mit Verzweigung kostet mehr, als die gesparten Objekte bringen:

| Szenario | Arrays | Fields | InlineArray |
|----------|-------:|-------:|------------:|
| kleine Kachel 30° | 9,1 ms | 10,8 ms | 8,6 ms |
| kleine Kachel 20° | 25,6 ms | 29,3 ms | 23,7 ms |
| Zufallspunkte 30° | 32,2 ms | 35,1 ms | 29,7 ms |
| Kachel 30°, Snap 1 cm | 474 ms | 506 ms | 410 ms |
| Kachel 30°, Raster + Keil | 400 ms | 402 ms | 338 ms |

Unter net48 kann es anders aussehen (anderer JIT und GC); hier läuft kein net48. Deshalb bleibt
`Arrays` der Standard für `netstandard2.0`. `Triangle.Benchmarks` baut für den Vergleich auf
Windows zusätzlich `net48`:

```
dotnet run -c Release -f net48 --project src/Triangle.Benchmarks -p:TriangleLayout=Arrays -- --filter "*"
dotnet run -c Release -f net48 --project src/Triangle.Benchmarks -p:TriangleLayout=Fields -- --filter "*"
```

Mit `TRIANGLE_BENCH_SCENARIOS=herne_373600_5709000/q30-snap1cm` (kommagetrennt) lässt sich die
Auswahl einschränken. Läufe abwechselnd starten, da die CPU-Leistung schwanken kann.

Einbinden im eigenen Projekt, wenn `Fields` gewinnt:

```xml
<ProjectReference Include="..\Triangle.NET\src\Triangle\Triangle.csproj"
                  AdditionalProperties="TriangleLayout=Fields" />
```

oder beim Bauen `msbuild /p:TriangleLayout=Fields` bzw. `dotnet build -p:TriangleLayout=Fields`.

## Standard-Benchmarks für CDT (`BenchmarkDatasetsTest`, Korpus)

Immer im Testlauf (`Robustness/BenchmarkDatasetsTest.cs`, Daten in `Robustness/Data`):

* **Exakte Prädikate:** je 1.000 extreme Grenzfälle für `orient2d` und `incircle` mit bekanntem
  Vorzeichen (Nanevski et al., Werte bis 10³³): alle richtig.
* **Ruppert-Ahornblatt** (Kanadische Flagge) ohne Qualität und mit 20°, 30°, 33°.
* **Gitter** (kozirkulär): 10×10 (auch Abstand 0,1 und UTM-Koordinaten), 100×100: deterministisch,
  mit allen drei Triangulatoren und gemischter Reihenfolge Delaunay; mit diagonalen Zwangskanten
  bei 30° keine schlechten Dreiecke.
* **Konzentrische Kreise** mit Loch (kein Dreieck im Loch, Fläche exakt) und ohne Loch (kein
  Dreieck kreuzt den inneren Kreis).
* **Spitzer Stern** mit Spitzenwinkeln von 10⁻³ bis 10⁻¹⁵ rad bei drei Maßstäben (1, UTM, 10⁻⁶).

Korpus (explizit, externe Daten):

| Daten | Ergebnis |
|-------|----------|
| Testsammlung von artem-ogre/CDT (43 Dateien: hängende und überlappende Constraints, Punkte auf Kanten, Schweden, Gitarre …) | alle gültig, auch `issue-211` (siehe unten) |
| Natural Earth Küsten 110m / 50m / 10m (10.000 / 59.000 / 407.000 Punkte) | CDT 0,1 / 1,0 / 4,8 s |
| Natural Earth 10m mit Qualität (ausreichend Steiner-Punkte) | 20°: 11 s, 1,7 Mio. Dreiecke, 0,27 % schlecht; 30°: 22 s, 2,8 Mio., 0,39 % |
| Natural Earth 50m, 30° | 2,6 s, 0,23 % schlecht |

Gefunden und behoben:

* **`issue-211`** (lange Kanten, die eine Polylinie unter praktisch 0° vielfach kreuzen): Der Noder
  konvergierte bei keiner Toleranz, die Triangulierung brach mit einer Exception ab (die CDT-
  Bibliothek selbst wirft hier ebenfalls). Jetzt folgt als letzte Stufe Snap-Rounding auf die größte
  Toleranz. Dabei werden die Segmente zuerst durch die vorhandenen Punkte geführt und erst dann
  neue Schnittpunkte berechnet; umgekehrt explodierte die Punktzahl (244 → 12.634 in vier Runden).
  Eingaben, die bisher konvergierten, sind unverändert.
* Der Korpus-Harness selbst war für große Daten quadratisch (Segmentabdeckung, Streifen-
  Klassifizierung) und lief bei Natural Earth 10m in den Timeout; die Triangulierung brauchte
  nur 3 s. Außerdem zählt die Steiner-Limit-Erkennung jetzt wie die Bibliothek (auch wieder
  gelöschte Punkte).

Thingi10k (3D-Modelle) wurde nicht verwendet: Dafür müssten erst 2D-Schnitte erzeugt werden,
das testet eher den Schnittgenerator als die Triangulierung.

## Raster, Keile und Geländevereinfachung

Testgetrieben entwickelt (Tests zuerst rot: `GridSnapRoundingTest`, `WedgeMergeTest`,
`TerrainSimplifierTest`).

**Raster (`ConstraintOptions.GridSize`, z. B. 0,01).** Wer die Koordinaten des Netzes beim
Schreiben auf cm rundet, verschiebt jeden Punkt um bis zu 7 mm. An fast deckungsgleichen Linien
sind die Dreiecke schmaler, dort kippen sie. Mit Raster liegen alle Punkte schon im Netz auf dem
Raster, Runden ändert also nichts mehr:

* Eingabe: Snap-Rounding (`SegmentNoder.SnapRound`, iteriertes Snap-Rounding nach Hobby bzw.
  Halperin/Packer). Punkte und Schnittpunkte werden gerundet, Segmente, die durch die Rasterzelle
  eines Punktes laufen, werden durch ihn geführt. Danach kreuzt sich nichts, und kein Punkt liegt
  näher als eine halbe Zelle an einem fremden Segment.
* Verfeinerung: Steiner-Punkte werden gerundet. Teilungspunkte auf Segmenten werden vorher auf das
  Eingabesegment projiziert, sonst summieren sich die Abweichungen über die Teilungsstufen.
  aCute-Relocation ist im Rastermodus aus (sie lief im Kreis). Dreiecke mit Kanten unter zwei
  Rasterzellen werden nicht mehr wegen des Winkels verfeinert, sonst kaskadiert die Verfeinerung
  an kleinen Eingabewinkeln bis auf das Raster (Zufallsdaten: 360.000 statt 1.500 Punkte).
* Zum Schluss verschiebt `GridSnapper.SnapVertices` übrige Punkte nur, wenn kein Dreieck kippt.
  Die Rundung liefert für Dezimalraster dieselben Werte wie `Math.Round(x, n)`.

**Keile (`ConstraintOptions.WedgeTolerance`, z. B. 0,03).** Eine Linie, die von einer anderen
unter weniger als 10° abgeht und deren anderes Ende näher als die Toleranz an ihr liegt, wird mit
ihr zusammengeführt (Gebäudewand knapp neben der Flurstücksgrenze). Andere Punkte in derselben
Entfernung bleiben unberührt, anders als bei einer größeren Snap-Toleranz. Ein Segment bewegt sich
dabei höchstens um die Toleranz, auch bei Ketten von Keilen (geprüft gegen das Eingabesegment).

Ergebnis, ALKIS Herne (Flurstücke + Gebäude + Nutzung, 201 Kacheln, Snap 1 cm), gemessen **nach
dem Runden auf cm**:

| Variante | schlechte Dreiecke 20° / 30° | gekippte Dreiecke 20° / 30° | Dreiecke 30° |
|----------|-----------------------------|-----------------------------|--------------|
| bisher | 6,13 % / 9,55 % | 33.858 / 92.501 (in 133 / 148 Kacheln) | 7,18 Mio. |
| Raster 1 cm | 2,91 % / 2,73 % | 0 / 0 | 6,83 Mio. |
| Keil 3 cm | 3,72 % / 6,44 % | 5.208 / 13.166 | 6,21 Mio. |
| **Raster 1 cm + Keil 3 cm** | **2,22 % / 2,22 %** | **0 / 0** | 6,59 Mio. |

Ohne Rundung gemessen hätte „bisher“ 1,10 % / 0,98 %. Das ist aber nicht das Netz, das beim
Nutzer ankommt. Der Fallback-Adapter im alten Repo nutzt jetzt Raster (Rundungsgenauigkeit) und
Keil-Toleranz (dreifache Snap-Toleranz) als Standard.

Die wichtigste Korrektur im Rastermodus: Teilsegmente, bei denen kein Rasterpunkt eine gültige
Teilung ergibt, sind häufig (neben sehr dünnen Dreiecken). Dreiecke, deren Umkreismittelpunkt ein
solches Teilsegment bedrängte, wurden bisher dauerhaft verworfen. So blieben schlechte Dreiecke
mit bis zu 40 m Kantenlänge stehen. Jetzt wird der Punkt trotzdem eingefügt, wie bei gesperrten
Kanten (die Raster-Untergrenze verhindert eine Kaskade). Damit sank der Anteil von 5,3 % / 6,0 %
auf 2,2 % / 2,2 %; die Laufzeit stieg um etwa ein Drittel (15 % mehr Dreiecke).

Wo die übrigen schlechten Dreiecke liegen (alle 201 Kacheln, Raster 1 cm + Keil 3 cm; Korpus-Kennzahlen
`ba`/`bf`/`bn`/`bs`/`bi`):

| Klasse | 20° | 30° | vermeidbar? |
|--------|-----|-----|-------------|
| an einem kleinen Eingabewinkel (zwei Segmente < Mindestwinkel) | 4.892 | 6.517 | nein |
| kürzeste Kante unter zwei Rasterzellen | 16.099 | 30.436 | nein (Raster) |
| schmaler Streifen: anderes Segment näher als 5 cm | 39.124 | 89.070 | kaum (Raster) |
| an einem Segment, sonst nichts in der Nähe | 5.172 | 14.565 | vielleicht |
| im Inneren | 514 | 5.337 | vielleicht |
| **zusammen** | 65.801 (2,22 %) | 145.925 (2,22 %) | |

Vermeidbar wären höchstens die letzten beiden Klassen: 0,19 % bzw. 0,30 % aller Dreiecke. Der
große Rest liegt in Streifen zwischen fast parallelen Linien, die sich nicht treffen (z. B.
Gebäudewand 2–5 cm neben der Grenze): Bei 3 cm Breite verschiebt das cm-Raster die Punkte um bis
zu ein Sechstel der Kantenlänge, ein Mindestwinkel von 30° ist dort kaum zu halten. Abhilfe wäre
nur, solche Streifen zusammenzulegen, also Linien um mehrere cm zu verschieben (Datenentscheidung).

Die übrigen schlechten Dreiecke liegen damit an Segmenten im Bereich 2–5 cm und in echten
Eingabekeilen. Dort lässt sich mit Punkten auf einem cm-Raster kein guter Winkel erreichen.
Größere Toleranzen helfen, sind aber eine Datenentscheidung. Ausprobiert und verworfen: den
Abschnitt eines langen Keils zusammenführen, auf dem die Linien näher als die Toleranz liegen. Die
Keilspitze wandert dabei nur, der kleine Winkel bleibt; auf den Herne-Kacheln war das neutral bis
leicht schlechter.

**TIN-Vereinfachung (`Tools.TerrainSimplifier.Select`).** Wählt aus einem dichten DGM die Punkte
aus, deren Triangulierung alle Punkte mit einem vertikalen Fehler unter der Schranke
wiedergibt: Greedy Insertion nach Garland/Heckbert, wahlweise mit Bruchkanten. Pro Runde wird in
jedem Dreieck der schlechteste Punkt eingefügt, sofern sein Fehler mindestens halb so groß ist
wie der größte Fehler. Das spart gegenüber „alle auf einmal“ etwa 14 % Punkte. Eingefügt wird
inkrementell, und nur Punkte in geänderten Dreiecken werden neu lokalisiert (Max-Heap).

Die Delaunay-Triangulation von Rasterpunkten ist nicht eindeutig (kozirkuläre Punkte), eine neu
aufgebaute Triangulierung kann also andere Diagonalen wählen – in flachen Bereichen mit wenigen
Punkten mit Fehlern über 2 m. Deshalb wird der Fehler für jede kozirkuläre Gruppe über alle
Dreiecke aus ihren Punkten bewertet (jedes davon kommt in irgendeiner Delaunay-Triangulierung
vor). Die Schranke gilt damit für jede Delaunay-Triangulierung der Auswahl; eine abschließende
Prüfung gegen eine neu aufgebaute Triangulierung bestätigt das.

| DGM1 Herne, 1 Mio. Punkte | ausgewählt | Laufzeit | danach Qualitätsnetz 30° |
|---------------------------|------------|----------|--------------------------|
| Fehler ≤ 20 cm | 35.700 (3,6 %) | 11 s | ca. 75.000 Punkte, 1 s |
| Fehler ≤ 10 cm | 106.100 (10,6 %) | 16 s | ca. 183.000 Punkte, 2 s |
| Fehler ≤ 5 cm | 268.400 (26,8 %) | 28 s | ca. 393.000 Punkte, 4 s |

Datenabhängige Triangulierung (Kantenflips nach Geländeform) wurde bewusst nicht umgesetzt: Sie
erzeugt schmale Dreiecke und widerspricht damit dem Ziel der Qualitätsvermaschung.

**Haben die Bugfixes die schwierigen Fälle verbessert?** Die Fixes des klassenweisen Reviews
ändern an den Korpuszahlen nichts (GEOS und Herne bitgleich). Die Verbesserungen stammen aus den
vorherigen Runden:

| Fall | vorher | jetzt |
|------|--------|-------|
| GEOS, 2156 Fälle | 1 Fehler | 0 Fehler |
| GEOS `TestRelateAA.xml#74` (Nadel) | 88 s | 2,7 s |
| GEOS Isochronen am Steiner-Limit (20° / 30°) | 13 / 20 | 13 / 20 (unverändert, echte Feinstrukturen) |
| Herne Nutzung, Snap 1 cm (ungerundet) | 8,70 % / 10,74 %, 1–2 Kacheln am Limit | 1,10 % / 0,98 %, kein Limit |
| Herne Nutzung ohne Snap | lief nicht durch | läuft durch, aber 110 von 201 Kacheln am Limit, 78 % schlecht: Snap ist nötig |

## Steiner-Punkte ausdünnen und glätten (`SteinerReductionTest`, `SteinerSmoothingTest`)

Recherche (Oktober 2026) nach neueren, integrierbaren Verfahren. Die Grundverfahren sind älter und
im Fork schon enthalten (Off-Center nach Üngör, aCute nach Erten/Üngör, Snap-Rounding,
Greedy Insertion). Übernommen wurden zwei Ideen, die auf einem fertigen Qualitätsnetz arbeiten:

* **Lokale Suche über Steiner-Punkte** (Abrahamsen, Brunck, Conradi, Kolbe, Nusser: „Computing
  Non-Obtuse Triangulations with Few Steiner Points“, SoCG 2025, Gewinner der CG:SHOP Challenge
  2025): eine constrained Delaunay-Triangulierung, in der Steiner-Punkte entfernt, verschoben
  oder hinzugefügt werden, je nachdem, was das Netz am meisten verbessert. Ähnlich „Delaunay
  Sifting“: Steiner-Punkte entfernen, ohne die Qualitätsmaße zu verschlechtern.
* **Optimale Delaunay-Triangulierung (ODT)** (Chen/Xu 2004, Alliez u. a. 2005) und die
  Verzahnung von Verfeinerung und Optimierung (Tournois, Alliez, Devillers, IMR 2007: weniger
  Steiner-Punkte und bessere Qualität als reine Delaunay-Verfeinerung).

Nicht übernommen: exakte implizite Punkte (Attene 2020, Diazzi u. a. 2023; die Ausgabe braucht
ohnehin Raster-/Gleitkommakoordinaten), Stable Snap Rounding, randomisiertes Einfügen von
Segmenten (GIS-Segmente sind kurz), parallele und lernende Verfahren (nicht deterministisch).

Umsetzung (beide optional, Standard aus; Eingabepunkte, Segmente und Punkte auf Segmenten
werden nie verändert, gesperrte Kanten also auch nicht):

* `QualityOptions.ReduceSteinerPoints`: Ein freier Steiner-Punkt wird gelöscht, wenn alle
  Dreiecke der Delaunay-Triangulierung seines Hohlraums die Qualitätsvorgaben erfüllen (ohne die
  Ausnahmen der Verfeinerung, z. B. das Rasterminimum) und kein Teilsegment verletzen. Der
  Hohlraum wird vorab genauso trianguliert wie bei `Mesh.DeleteVertex`. Geht das nicht, wird ein
  benachbarter freier Punkt auf die Mitte der gemeinsamen Kante geschoben (Kantenkollaps), wenn
  der Punkt danach entfernbar ist und kein anderes Dreieck schlechter wird als vorher. Die Flips
  dieser Verschiebung werden sonst exakt rückgängig gemacht.
* `QualityOptions.SmoothingSteps`: Durchläufe, in denen jeder freie Steiner-Punkt zur
  ODT-Position (flächengewichtetes Mittel der Umkreismittelpunkte seiner Dreiecke) oder halb
  dorthin rückt, im Rastermodus auf das Raster gerundet, mit Delaunay-Flips (nie über
  Teilsegmente). Behalten wird die Verschiebung nur, wenn die geänderten Dreiecke besser werden
  (weniger Verletzungen, sonst größerer kleinster Winkel). Kombiniert wird nach jedem Durchlauf
  ausgedünnt.

Ergebnis (Dreiecke, in Klammern Dreiecke unter dem Mindestwinkel; ALKIS-Kacheln Herne):

| Szenario | ohne | ausdünnen | 3× glätten | 3× glätten + ausdünnen |
|----------|-----:|----------:|-----------:|-----------------------:|
| Kachel 30°, Snap 1 cm | 107.190 (1.147) | 104.312 (1.147) | 107.190 (1.147) | 103.984 (1.147), −3,0 % |
| Kachel 30°, Raster 1 cm + Keil 3 cm | 112.353 (3.319) | 97.015 (3.207) | 112.353 (3.153) | 92.167 (3.120), −18,0 % |
| kleine Kachel 20°, Snap 1 cm | 8.952 (184) | 8.596 (184) | 8.952 (184) | 8.514 (184), −4,9 % |
| Zufallspunkte 30° | 13.679 (0) | 13.587 (0) | 13.679 (0) | 13.567 (0), −0,8 % |

Im Rastermodus lohnt es sich am meisten: Die gerundeten Steiner-Punkte der Verfeinerung
liegen oft dichter als nötig. Glätten hebt den mittleren kleinsten Winkel um 0,1–0,5°.
Durch das Ausdünnen sinkt er leicht, weil weniger Punkte da sind; der Mindestwinkel bleibt
eingehalten. Die schlechten Dreiecke an kleinen Eingabewinkeln bleiben (sie hängen an
Eingabepunkten und Segmenten).

Mit `ConformingDelaunay` (so ruft der Fallback im alten Repo auf) dieselbe Kachel mit Raster:
136.265 Dreiecke (5.516 schlecht), ausgedünnt 119.271 (−12,5 %), geglättet und ausgedünnt
112.713 (−17,3 %, 5.210 schlecht). Laufzeit (Minimum aus drei Läufen): 0,5 s, 1,3 s, 2,7 s,
nur geglättet 0,7 s.

Tests: alle Qualitätsszenarien des Katalogs mit allen Invarianten (Konsistenz, constrained bzw.
conforming Delaunay, Fläche, Segmente, gesperrte Segmente, Winkelgarantie), dazu: nur freie
Steiner-Punkte ändern sich, Segmente identisch, nicht mehr schlechte Dreiecke, kleinster Winkel
sinkt beim Glätten nie, Maximalfläche und -winkel, Raster, `UserTest`, Determinismus.

### Breite Bewertung am Korpus

Alle 201 Herne-Kacheln (Raster 1 cm, Keil 3 cm), dazu Herne nur mit Snap, Natural Earth und die
CDT-Testfälle (`TRIANGLE_CORPUS_REDUCE`, `TRIANGLE_CORPUS_SMOOTH`,
`TRIANGLE_CORPUS_CONFORMING`). Alle Läufe gültig (keine Fehler, keine Kippungen beim Runden im
Rastermodus). Zeiten sind die Summe der Einzelläufe:

| Herne, Raster + Keil | Dreiecke 20° | Zeit 20° | Dreiecke 30° | schlecht 30° | Zeit 30° |
|----------------------|-------------:|---------:|-------------:|-------------:|---------:|
| ohne | 2.965.369 | 66 s | 6.586.381 | 145.925 | 102 s |
| ausdünnen | −5,9 % | 112 s | −8,7 % | 143.794 | 196 s |
| 3× glätten | ±0 | 96 s | ±0 | 142.260 | 159 s |
| 3× glätten + ausdünnen | −6,7 % | 153 s | −11,6 % | 140.621 | 275 s |
| conforming, ohne | 4.288.070 | 67 s | 8.100.427 | 211.063 | 96 s |
| conforming, ausdünnen | −6,4 % | 124 s | −9,3 % | 207.402 | 208 s |
| conforming, 3× glätten | ±0 | 113 s | ±0 | 204.608 | 171 s |
| conforming, 3× glätten + ausdünnen | −7,2 % | 186 s | −12,4 % | 202.069 | 323 s |

Nur Snap (ohne Raster): −2,5 % bis −4 % Dreiecke. Natural Earth und CDT-Fälle: unter 1 %
(dort hängen die schlechten Dreiecke an winzigen Eingabewinkeln und am Steiner-Limit).

Bewertung: Lohnt sich, wenn die Netzgröße zählt (Speicher, Weiterverarbeitung), vor allem im
Rastermodus bei 30°: rund 10–12 % weniger Dreiecke und 3–4 % weniger schlechte, bei 2–3,4-facher
Laufzeit. Glätten allein verbessert die Qualität nur wenig (−2,5 bis −3 % schlechte Dreiecke).
Als Standard nicht sinnvoll, als Option für große Exporte ja.

### Pooling

Kurzlebige Objekte (Umkreismittelpunkte, Iteratoren, Hilfsarrays) sind für den .NET-GC billig. Teuer
sind die langlebigen Netzobjekte, die bei jeder Sammlung kopiert werden: Allokationsprofil der
Herne-Kachel mit `Arrays` (wie net48): `Osub[]` 13 %, `Otri[]` 13 %, `Triangle` 11 %, `Vertex[]` 9 %,
dazu Punkte, Vertices und Dictionary-Einträge. `ArrayPool<T>` hilft dafür nicht (die Arrays leben so
lange wie das Netz), die Hilfsarrays der Prädikate liegen schon im Thread-Arbeitsspeicher.

Was hilft, gibt es schon: `TrianglePool` hält die Dreiecke eines Netzes und lässt sich über
`Configuration` für mehrere Vermaschungen nacheinander wiederverwenden
(`new Configuration(() => RobustPredicates.Default, () => pool.Restart())`). Die Dreiecke samt ihren
Arrays liegen dann schon in Gen 2 und werden nicht neu angelegt (Ergebnis identisch, gleicher
Fingerprint). Gemessen (Mittel aus 10–40 abwechselnden Läufen, .NET 10):

| Layout | Szenario | neuer Pool | wiederverwendeter Pool |
|--------|----------|-----------:|-----------------------:|
| Arrays (wie net48) | Kachel 30°, Raster + Keil | 550 ms | 384 ms (−30 %) |
| Arrays | kleine Kachel 20° | 45 ms | 38 ms (−16 %) |
| Arrays, kleines Gen-0-Budget (4 MB) | Kachel 30°, Raster + Keil | 769 ms | 583 ms (−24 %) |
| InlineArray (net8) | Kachel 30°, Raster + Keil | 434 ms | 354 ms (−18 %) |
| InlineArray | kleine Kachel 20° | 34 ms | 37 ms (±) |

Voraussetzung: Das vorige Netz wird nicht mehr benutzt, wenn der Pool neu startet (der Fallback im
alten Repo wandelt das Netz sofort in `NetzDaten` um; Referenz- und Hauptnetz brauchen dort aber
getrennte Pools, weil beide gleichzeitig leben). Je Thread ein Pool, `TrianglePool` ist nicht
threadsicher.

## Vergleich mit anderen Bibliotheken und öffentlichen Testdaten

Gleiche Maschine, .NET 10, Median aus wiederholten Läufen. „Original“ ist Triangle.NET von
wo80 (Commit `fa6f2fd`, Basis dieses Forks).

**Delaunay ohne Constraints** (Szenario des BurstTriangulator-Benchmarks: n×n-Gitter im
Einheitsquadrat, dazu gleich viele Zufallspunkte):

| Punkte | Original Gitter | Fork Gitter | delaunator-sharp Gitter | Original zufällig | Fork zufällig | delaunator-sharp zufällig |
|-------:|----------------:|------------:|------------------------:|------------------:|--------------:|--------------------------:|
| 10.000 | 25 ms | 10 ms | 3,8 ms | 11 ms | 8,7 ms | 4,7 ms |
| 99.856 | 1.004 ms | 132 ms | 44 ms | 188 ms | 110 ms | 52 ms |
| 1.000.000 | 9.188 ms | 4.373 ms | 768 ms | 4.545 ms | 2.356 ms | 1.189 ms |

BurstTriangulator ist in seiner eigenen Grafik 7–8× schneller als delaunator-sharp (nativer
Code über Burst/LLVM, Unity). Für reine Delaunay-Triangulierung großer Punktmengen bleibt der Fork
also deutlich langsamer; das liegt an den Objekten pro Dreieck (siehe Pooling) und ist nur mit
einer index-basierten Struktur aufzuholen.

**Verfeinerung:** Lake Superior (aus BurstTriangulator, 518 Punkte, 6 Löcher), Flächengrenze 100
(wirkungslos), Mindestwinkel wie im Burst-Benchmark:

| Mindestwinkel | Dreiecke | Zeit |
|--------------:|---------:|-----:|
| 20° | 1.003 | 2,8 ms |
| 30° | 1.481 | 3,9 ms |
| 33,73° (Grenze von Burst) | 1.812 | 4,5 ms |
| 36° | 2.144 | 5,5 ms |
| 38° | 2.751 | 7,3 ms |

Laut BurstTriangulator-Dokumentation (v3.6) braucht Burst dort bis zu ~7.800 Dreiecke und 69 ms und
endet bei 33,73°. Der Fork vernetzt mit rund einem Viertel der Dreiecke, ist etwa 15× schneller und
kommt bis 38° (40° endet am Steiner-Limit). Das Original rechnet bis 38° gleich, stürzt bei 40° aber
in den Prädikaten ab (`IndexOutOfRangeException`). Mit `ConformingDelaunay` ändern sich die Zahlen
kaum.

Gefunden und behoben dabei: reine Flächenverfeinerung (Einheitsquadrat, Maximalfläche 0,001) war
mit aCute 10× langsamer als nötig, weil für jedes zu große Dreieck versucht wurde, eine Ecke zu
verschieben (Relocation verbessert Winkel, nicht die Fläche): 148 → 14 ms bei gleicher
Dreieckszahl; Original 125 ms. Die Vorabprüfung auf verletzte Teilsegmente bricht jetzt ab, wenn
keines betroffen ist (klassische Verfeinerung wieder so schnell wie im Original).

**Öffentliche Testdaten** (`CorpusTest.PublicFixtures`, `TRIANGLE_PUBLIC_FIXTURES`): Testfälle
von mapbox/earcut (61 Polygone aus echten Daten und Fehlerberichten: Gewässer, sich berührende
Löcher, selbstberührende Ringe), mapbox/delaunator (Robustheitsfälle, Ukraine) und jhasse/poly2tri
(22 Umrisse). Jeweils ohne Qualität, 20° und 30°: 273 Läufe, alle gültig (Konsistenz, keine
gekippten Dreiecke, alle Linien abgedeckt, Delaunay).

Direkter Vergleich mit Original (Ringe als Linien und als Löcher, ohne Qualität und 30°):

| | gültig | leer (erwartet) | Exception | Endlosschleife | inkonsistent |
|---|---:|---:|---:|---:|---:|
| Original | 305 | 8 | 31 | 2 | 2 |
| Fork | 336 | 12 | 0 | 0 | 0 |

Die leeren Ergebnisse entsprechen den Erwartungen von earcut (0 Dreiecke für entartete
Eingaben). Lochringe ohne Fläche (earcut `water-huge`, `issue83`) warfen vorher eine Exception;
jetzt werden ihre Kanten als Linien übernommen (earcut ignoriert solche Löcher).

## Klassenweises Review (`src/Triangle.Tests/Review`)

Alle 80 Quelldateien wurden in sechs Bereichen auf Fehler untersucht; jede Vermutung wurde mit
einem Test geprüft (Review A–F, 269 Testfälle). 60 Fehler bestätigten sich, alle sind
behoben, die Tests sind jetzt Regressionstests; 3 weitere Befunde sind dokumentierte Grenzen. Widerlegte Vermutungen sind als
Regressionstests erhalten.

| Bereich | Wichtigste Befunde (alle behoben) |
|---------|-----------------------------------|
| A Mesh, Pool, Behavior | Doppelte Dreiecks-IDs nach `Renumber` + `Refine` (`mesh.Edges` ließ Kanten aus); `Refine` eines Punktnetzes vergrößerte das Gebiet (Hülle nicht markiert) und ignorierte die Konfiguration; Vertex-Objekte aus einem anderen Netz verbanden zwei Netze; ungültiger Winkel schaltete alle Qualitätsvorgaben ab; `TrianglePool.Contains/Sample/Clear` |
| B Qualitätsverfeinerung | `MaximumAngle` allein wirkungslos; `IndexOutOfRange` beim Verfeinern mit Maximalwinkel (geteilte Arbeitsarrays); nach dem Löschen von Punkten wurden Dreiecke bei eingeschränkter Segmentteilung nicht geprüft (zu große Dreiecke); `VariableArea` bei Punktnetzen ignoriert; Warteschlange bei extremen Kantenlängen |
| C Noding, Constraints, Converter | Schnittpunkte bekamen ID 0 (Benutzer-IDs wurden neu vergeben); Snap-Toleranz größer als ein Loch leerte das ganze Netz; Labels bei deckungsgleichen Teilstücken und verschmolzenen Punkten; mehrere Regionspunkte in einem Gebiet; `ToDCEL` bei Löchern, die den Rand berühren; `ToMesh` mit beliebigen IDs; Noding langer paralleler Segmente quadratisch (20.000 Linien: 4,7 s) |
| D Geometrie, Topologie | **Lochpunkt landete bei UTM-Koordinaten auf dem Lochrand** (13 von 154 zufälligen dreieckigen Löchern: Netz um das Loch herum gelöscht, ohne Fehlermeldung); dünne Konturen ohne Innenpunkt (Exception); `Rectangle`-Kopierkonstruktor, `Intersects`; `VertexCirculator` verschachtelt bzw. für doppelte Punkte; DCEL an Berührpunkten |
| E Voronoi, Glätten, Tools | Wiederverwendeter `SimpleSmoother` zerstörte das zuvor geglättete Netz; Abbruch hinterließ halb verschobene Punkte; Abbruchkriterium skalenabhängig; Reverse Cuthill-McKee sortierte Nachbarn falsch (Bandbreite 9 statt 7); `TriangleQuadTree` fand Punkte auf inneren Kanten nicht; `PolygonValidator`; Legacy-Voronoi mit doppelten Punkten |
| F IO, Prädikate, Log | **`.poly` schreiben: alle Segmente als „0 0“** bei Polygonen aus Konturen; Regionsflächen gingen beim Schreiben verloren bzw. wurden in Exponentialschreibweise nicht gelesen; Kommentare am Zeilenende; 6-Knoten-Elemente; kulturabhängige Ganzzahlen; `Jettison`-Nummerierung; Log nicht thread-sicher; Koordinaten > 1e70 (Überlauf der exakten Arithmetik) werden jetzt abgewiesen |

Dokumentierte Grenzen (keine Fehler): Die exakten Prädikate setzen voraus, dass Koordinaten-
differenzen nicht unter etwa 1e-75 liegen (Unterlauf, wie im C-Original). `BoundedVoronoi`
setzt ein Conforming-Delaunay-Netz voraus.

## Upstream-Issues (github.com/wo80/Triangle.NET/issues)

Alle 48 Issues wurden gesichtet (Stand Oktober 2026). Fehlerberichte mit genug Information sind
in `UpstreamIssuesTest` als Test nachgestellt; Fragen zur Benutzung, Feature-Wünsche und
Diskussionen sind nicht aufgeführt.

| Issue | Status upstream | Inhalt | Stand im Fork |
|-------|-----------------|--------|---------------|
| #59 | offen | `SweepLine.RightOfHyperbola`: NullReferenceException | Keine Eingabedaten im Bericht. 50 Zufalls-/Degenerationsfälle (Duplikate, Kollinearität, UTM) laufen fehlerfrei; zusätzlich der Flip-Fix von SweepLine (Befund 10). Nicht abschließend bestätigt. |
| #58 | geschlossen (Workaround) | `GetStarPoints` vergrößert Arbeitsarray endlos, Speicherüberlauf | Puffer begrenzt (Upstream-Fix enthalten). |
| #54, #19 | offen / geschlossen | `RobustPredicates.Default` nicht thread-sicher | **Behoben** (thread-lokaler Arbeitsspeicher, Befund 4); Test mit 200 parallelen Triangulierungen. |
| #48 | offen | `VertexCirculator` wirft NullReferenceException für Eingabepunkte | Nicht reproduzierbar: Umlauf und anschließendes Verfeinern funktionieren. |
| #39 | geschlossen | `EnforceQuality` hängt | Steiner-Limit, aCute-Fixes, Endlosschleifen-Schutz; Korpus ohne Hänger. |
| #35 | geschlossen | Example 5 (Regionsflächen mit aCute): „new vertex is at the circumcenter“ | **Behoben** (aCute-Verschiebung, siehe Fixes); Test. |
| #31 | geschlossen | Ungleichmäßige Elementgrößen | Upstream durch aCute behoben; der Fork behebt zusätzlich die verlorenen Dreiecke (Maximalfläche). |
| #18 | geschlossen | `SimpleSmoother` stürzt bei doppeltem Punkt ab | Funktioniert; Test. |
| #14 | geschlossen | `Polygon(int capacity)` ignoriert die Kapazität | Behoben; Test. |
| #13 | geschlossen | „Ran out of precision“ mit maximaler Kantenlänge (`UserTest`) | Funktioniert (Präzisionsgrenze bei der Verfeinerung); Test. |
| #9 | offen | Maximalfläche ohne Segmentteilung: 10×30-Rechteck geht, 10×20 nicht | **Behoben.** Ursache: Liegt der neue Punkt im Umkreis eines Segments, das nicht geteilt werden darf, oder jenseits davon, wurde das Dreieck ersatzlos verworfen (10×20: 8 statt ≥ 116 Dreiecke). Jetzt wird der Punkt trotzdem eingefügt bzw. ein zu großes Dreieck am Schwerpunkt geteilt. Betrifft auch gesperrte Bruchkanten (`CanSplitSegment`). |
| #8 | geschlossen | Bounded Voronoi nach Glättung hängt | Funktioniert (10 Zufallsfälle); Test. |
| #7 | offen | IDs über `GetHashCode` | IDs sind fortlaufende Zähler; `Vertex` nutzt den koordinatenbasierten Hashcode (Befund 9). |
| #5, #1 | offen / geschlossen | „Topological inconsistency after splitting a segment“ | **Behoben** durch Noding (Befunde 1–3); die Daten aus #1 sind als Test nachgestellt. |
| #3 | geschlossen | Glätten mit freistehendem Segment | Funktioniert; Test. |
| #49 | geschlossen | Loch wird nicht vollständig entfernt, wenn Segmente es kreuzen | Bekannte Einschränkung: Löcher werden von einem Startpunkt aus bis zu Segmenten „infiziert“; ein Segment im Loch trennt den Rest ab. Abhilfe: Regionen statt Löcher (siehe Alter-Stand-Bewertung). |

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
