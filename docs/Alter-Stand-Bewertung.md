# Bewertung des alten, angepassten Stands

Gegenstand: `HarmenGrosseDeters/Triangle.Net-Old-Custom` (TriangleLib, früher Triangle.NET-Stand
um 2012/13 mit `InputGeometry`, `Primitives.cs`, `Carver.cs`) plus eigene Erweiterungen.
Verglichen mit diesem Fork nach den Robustheits-Fixes.

## Kurzfazit

* **Fallback auf den alten Stand bringt nichts.** In einem Differenztest (600 Eingaben × 3 Varianten)
  (Details unten) gab es **keinen einzigen Fall, in dem der alte Stand ein gültiges Netz liefert
  und der neue nicht**. Umgekehrt scheitert der alte Stand bei kreuzenden Bruchkanten in
  ~30 % der Fälle, davon etwa ein Drittel **still** mit invertierten Dreiecken oder NaN-Flächen.
  Als Fallback eingesetzt, würde er also vor allem Fehler einschleppen, die ohne
  Validierung nicht auffallen.
* **Dein Wrapper schaltet die exakte Arithmetik ab** (`Behavior.NoExact = true` in
  `TriangulierenV2.cs`). Damit verlieren Shewchuks Prädikate jede Robustheitsgarantie. Das
  sollte auf keinen Fall übernommen werden.
* Von den fachlichen Erweiterungen lohnen sich **gesperrte Kanten** und eine
  **Iterations-/Fehlerberichterstattung**. Höheninterpolation und das Retten verworfener
  Dreiecke lassen sich besser außerhalb der Bibliothek lösen (siehe unten).

## Differenztest alt ↔ neu

Alter Stand mit Stubs für die RWE-Typen gebaut (`Punkt`, `Dreieck`, `PointInPolygon`,
`QuadTree`) und so konfiguriert wie `TriangulierenV2` (`ConformingDelaunay`, `Quality`,
`MinAngle`), einmal mit `NoExact = true` (wie im Wrapper) und einmal mit exakter Arithmetik.
Jede Triangulierung in eigenem Prozess; geprüft: keine flachen/invertierten Dreiecke (exakt),
Gesamtfläche korrekt. Je 100 zufällige Eingaben.

| Szenario | alt, NoExact (Wrapper) | alt, exakt | neu |
|----------|------------------------|------------|-----|
| 15 Integer-Segmente, ohne Qualität | 67 OK · 22 Exception · **10 still ungültig** · 1 Endlosschleife | 71 OK · 18 Exception · **10 still ungültig** · 1 Endlosschleife | **100 OK** |
| dto., 20° | 66 OK · 24 Exception · **9 still ungültig (NaN/invertiert)** · 1 Timeout | 69 OK · 30 Exception · **1 still ungültig** | **100 OK** |
| dto. mit UTM-Offset (3,5e5), ohne Qualität | 73 OK · 15 Exception · **12 still ungültig** | 71 OK · 15 Exception · **14 still ungültig** | **100 OK** |
| dto. mit UTM-Offset, 20° | 71 OK · 18 Exception · **11 still ungültig** | 68 OK · 31 Exception · 1 Timeout | **100 OK** |
| Nicht kreuzende Bruchkanten + 300 Punkte, Gauß-Krüger (5,6e6), 20° | 98 OK · **2 still ungültig** | 99 OK · **1 still ungültig** | **100 OK** |
| Parallele Bruchkanten 0,1 mm Abstand, ohne Qualität | 100 OK | 100 OK | **100 OK** |
| dto., 20° | terminiert nicht (ohne `maxIterationen`) | – | **OK** (mit Steiner-Limit) |

Auffällig: Selbst **ohne jede Kreuzung** (realistische Bruchkanten in Gauß-Krüger-Koordinaten)
liefert der alte Stand in 1–2 % der Fälle still ein Netz mit invertierten Dreiecken.

## Die Anpassungen im Einzelnen

| Anpassung | Wo | Bewertung | Empfehlung |
|-----------|----|-----------|------------|
| `Behavior.NoExact = true` | `TriangulierenV2.cs:186` | Schaltet die exakten Prädikate ab, also genau den Mechanismus, der Triangle robust macht. Wahrscheinlich eine wesentliche Ursache der früheren Gleitkomma-Probleme. | **Nicht übernehmen.** |
| Abbruchzähler in `ScoutSegment`/`ConstrainedEdge` (`ScoutSegmentInfinityLoopException` bei 1000 Rekursionen) | `Mesh.cs:2440`, `:2770` | Richtige Idee (StackOverflow → Exception). | **Erledigt**, besser gelöst: Schleife statt Rekursion plus Schrittzähler, und durch das Noding tritt der Fall praktisch nicht mehr auf. |
| Eigene Exception-Typen mit Koordinaten (`TopologicalInconsistencyException`, `RanOutOfPrecisionException`, …) | `Exceptions/` | Diagnostisch wertvoll (wo im Gelände ist das Problem?), aber an `Rwe.Core.Model.Entities.Punkt` gebunden und 8 fast identische Klassen. | **Konzept übernehmen:** *eine* `TriangulationException` mit `Point`-Property und Fehlerart als Enum. Klein. |
| Gesperrte Kanten (`GesperrteKanten`, `NoBisect = 3`): Bruchkanten dürfen nicht geteilt werden, andere Segmente schon | `Mesh.cs:267`, `Quality.cs:328/351`, `Mesh.cs:1125/2071` | Fachlich sinnvoll (Bruchkanten im DGM unverändert lassen). Der neue Stand kennt nur `SegmentSplitting` 0/1/2 global. | **Portieren**, aber sauberer: über das Segment-Label statt Index-Lookup, z. B. `ConstraintOptions.CanSplitSegment = label => …`. Mittel. Hinweis: Nicht teilbare Segmente heben die Winkelgarantie auf; daher immer mit Steiner-Limit kombinieren. |
| `maxIterationen` (globale Grenze) | `Quality.cs:1103` | Schutz gegen Endlosverfeinerung. | **Vorhanden:** `QualityOptions.SteinerPoints` + `CancellationToken`. |
| `anzahlOk` (Dreieck höchstens n-mal neu einreihen), Rückgabe von `schlechteDreiecke`/`problematischeDreiecke`, `maxOK` | `Quality.cs:594`, `:1118`, `:1148–1190` | Nützlich, um bei gesperrten Kanten nicht endlos am selben Dreieck zu arbeiten und dem Aufrufer zu melden, wo die Qualität nicht erreicht wurde. | **Teilweise portieren:** Pro-Dreieck-Limit zusammen mit gesperrten Kanten; die Liste der schlechten Dreiecke kann man auch nachträglich über die Winkel ermitteln (`QualityMeasure`), ohne die Bibliothek zu ändern. |
| Höhe (Z) neuer Punkte aus der **ursprünglichen** Triangulierung per QuadTree statt `xi`/`eta` | `Mesh.cs:556`, `Quality.cs:956` | Inhaltlich richtig, denn die Original-Interpolation extrapoliert, wenn der neue Punkt außerhalb des Dreiecks liegt. Aber: **feste Fallback-Höhen 211.0 bzw. 199.0** bei Fehlschlag → stillschweigend falsche Höhen. | **Nicht in die Bibliothek.** Besser: erst ohne Qualität triangulieren (Referenz-TIN), dann mit Qualität; Z der neuen Punkte danach per `TriangleQuadTree` im Referenz-TIN interpolieren. Kein Treffer = Fehler, keine Magic Numbers. Der neue Stand braucht dafür kein `USE_Z`. |
| `pruefeDreieckExtern` / `FehlerhaftVerworfeneDreiecke`: beim Löcherschneiden „versehentlich“ entfernte Dreiecke wieder hinzufügen | `Carver.cs:200`, `Mesh.cs:3083` | Workaround für ein Symptom: Wenn Bruchkanten einen Innenring „öffnen“ oder kreuzen, ist das Loch nicht geschlossen und die Infektion beim Carving läuft aus. Die geretteten Dreiecke gehören dann nicht zur Topologie des Netzes. | **Nicht übernehmen.** Ursache beheben: Das Noding hält Ringe jetzt auch bei kreuzenden Bruchkanten geschlossen. Robuster: Innenringe über **Regionen** (Label) statt Lochpunkte markieren und die Dreiecke danach per Label filtern; dann gibt es kein Auslaufen mehr. |
| Lochpunkt-Bestimmung im Wrapper (Schwerpunkt der ersten konvexen Ecke, die innen liegt) | `TriangulierenV2.cs:671` | Kann bei schmalen/konkaven Ringen fehlschlagen; dann fehlt das Loch. | Neu: `Contour.FindInteriorPoint()` bzw. Regionen-Ansatz. |
| aCute-Puffer 500 → 1500 Punkte | `NewLocation.cs:2089` | Sinnvoll. Der neue Stand hat 500 und bricht bei Überlauf aCute ab (Issue #58); dann greift jetzt der neue Rückfall auf den Umkreismittelpunkt. | **Optional übernehmen** (dynamische Größe). Klein. |
| Rundung der Ergebnis-Koordinaten (cm) und Zuordnung neuer Punkte zu Bruchkanten/Ringen mit 1 cm Toleranz | `TriangulierenV2.cs:307`, `:376–487` | Nachbearbeitung im Aufrufer. Achtung: Durch das Runden **nach** der Triangulierung können Dreiecke kippen. | Im Aufrufer belassen; nach dem Runden Orientierung prüfen. Besser: Eingabe vorher auf das Zielraster runden, dann ist die Triangulierung exakt auf dem Raster. |

## Rolle des Clipper-Wrappers

`Core.Wrapper.ClipperLibrary` rechnet in Ganzzahlkoordinaten (Faktor 10⁵ bzw. 10¹) mit Clippers
exakter Arithmetik:

* In `TriangleLib` wird nur `PointInPolygon<T>` genutzt (Ring- und Innenringtests in
  `pruefeDreieckExtern`, Lochpunkt im Wrapper). Ergebnis `Innen`/`Auf`/`Außen`.
* `GlaettePolygone` (`Clipper.SimplifyPolygon`, entfernt Eigenschnitte) und `BereinigePolygone`
  (`Clipper.CleanPolygon`, entfernt Fast-Kollinearitäten) sind faktisch Snap-Rounding
  **pro Ring**. Kreuzungen **zwischen** Bruchkanten untereinander und zwischen Bruchkanten und
  Ringen behandeln sie nicht. Genau diese Fälle haben im alten Stand die Fehler erzeugt, und
  genau sie löst jetzt der `SegmentNoder` im Fork.

Die Clipper-Vorverarbeitung der Ringe kann bleiben; sie schadet nicht.

## Empfohlene Architektur statt Fallback

```
Eingabe ──► (Clipper: Ringe bereinigen, wie bisher)
        ──► Fork: Triangulate(poly, constraints, quality{SteinerPoints, ggf. CanSplitSegment})
              │    (Noding/Snapping intern, exakte Prädikate, thread-sicher)
              ├─ MeshValidator.IsConsistent(mesh) ── ok ──► Z aus Referenz-TIN interpolieren
              └─ Exception / inkonsistent
                    ├─► Retry 1: QualityOptions.UseLegacyRefinement = true (ohne aCute)
                    ├─► Retry 2: Eingabe auf gröberes Raster runden (z. B. 1 mm), erneut
                    └─► Fehler mit Koordinate melden (TriangulationException)
```

Der alte Stand kommt in dieser Kette nicht mehr vor. Falls du ihn für eine Übergangszeit
trotzdem parallel betreiben willst, dann nur **mit** Validierung des Ergebnisses
(Orientierung aller Dreiecke, Fläche) und mit `NoExact = false`.

## Nebenbefund zum Repository

Im alten Repository liegen `bin/` und `obj/` mit eingecheckt, darunter interne
Binärbibliotheken (`Rwe.Core.*`) und Drittanbieter-DLLs (z. B. Oracle, PdfSharp). Falls das
Repository nicht privat ist oder geteilt wird, sollten diese Verzeichnisse entfernt und per
`.gitignore` ausgeschlossen werden.
