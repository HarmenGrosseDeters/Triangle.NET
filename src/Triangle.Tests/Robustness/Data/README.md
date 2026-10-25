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
