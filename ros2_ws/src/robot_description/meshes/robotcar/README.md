# robotcar CAD meshes

Imported without scaling or geometry changes from `/home/qwertym/Desktop/robotcar./robotcar/meshes` on 2026-11-20. The source joint transforms and inertial properties are preserved in `../../urdf/dase7503_sim.urdf`, with wheel joints fixed for ideal motion and additional ground/sensor interface frames. Mesh units are metres.

The `base_footprint` to `base_link` ground offset is 0.016328981906188 m, derived from the minimum transformed tyre vertex. The source `RB_joint` has a distinct origin and rotation; it is retained as exported. Camera and LiDAR sensing origins are assumed to coincide with the exported `Camera` and `Lidar` link origins.
