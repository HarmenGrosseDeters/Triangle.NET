# Test data

* `orient2d.txt`, `incircle.txt`: 1000 near-degenerate cases each with the expected sign
  (index, coordinates, sign). From *Automatic Generation of Staged Geometric Predicates*
  (Nanevski, Blelloch, Harper, ICFP 2001),
  https://www.cs.cmu.edu/afs/cs/project/pscico/pscico/src/arithmetic/compiler1/test/,
  via `test/fixtures` of https://github.com/mourner/robust-predicates (public domain,
  Unlicense) and https://github.com/artem-ogre/CDT (`incircle` is `insphere.2d` there).
* `maple-leaf.txt`: maple leaf of the Canadian flag (Ruppert's classic refinement example),
  from `Examples/canadian_flag_raw_data.txt` of
  https://github.com/Samson-Mano/2DTriangle_Mesh_Generator (MIT License, Copyright (c) 2023
  Mano). Converted to the corpus format (`L` = polyline).
* `Public/*.txt`: the public test fixtures on which the original Triangle.NET throws, does not
  terminate or produces an inconsistent mesh (constrained, with and without quality), used by
  `ExternalDataRegressionTest`. Converted to the corpus format (`L` = polyline, `P` = point):
  * `earcut-*`: `test/fixtures` of https://github.com/mapbox/earcut (ISC License, Copyright (c)
    Mapbox): bad-hole, earcut, infinite-loop-jhl, issue17, issue83, issue142, self-tangent-3,
    self-tangent-4, touching-holes6, water, water-huge, water-huge2, water-huge3.
  * `delaunator-*`: `test/fixtures` of https://github.com/mapbox/delaunator (ISC License,
    Copyright (c) Mapbox): issue43, robustness1 to robustness4.
  * `poly2tri-sketchup`: `testbed/data/sketchup.dat` of https://github.com/jhasse/poly2tri
    (BSD 3-Clause License, Copyright (c) Poly2Tri Contributors).
