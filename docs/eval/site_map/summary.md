# Planning on the SLAM map vs the authored layout

Same scenarios, same conditions: drifting odometry + AMCL, 0.5 m/s² drive limits, IMU tilt guard; each run from a clean restart. The only difference is where the site's known obstacles come from (`--site`):

- **layout** — the simulation's authored obstacle list (`worker_mover.OBSTACLES`);
- **map** — the SLAM map (`maps/construction_site`), plus keep-out boxes for objects the 2D LiDAR can't represent, plus 4 erased speckle cells (`config/site.yaml`).

| Scenario | Site | Success | Sim time (s) | Closest real clearance (m) | Max pose error (m) | Slow-down time (s) |
|---|---|---|---|---|---|---|
| bricks | map | 2/2 | 150.0 | 0.192 | 0.103 | 0.0 |
| bricks | layout | 2/2 | 152.4 | 0.304 | 0.077 | 0.0 |
| blocks | map | 2/2 | 144.2 | 0.221 | 0.076 | 2.0 |
| blocks | layout | 2/2 | 167.4 | 0.220 | 0.063 | 2.6 |
| custom | map | 1/1 | 116.6 | 0.298 | 0.078 | 0.0 |
| custom | layout | 1/1 | 117.0 | 0.390 | 0.073 | 0.0 |
| goals | map | 1/1 | 158.3 | 0.133 | 0.065 | 0.0 |
| goals | layout | 1/1 | 159.5 | 0.148 | 0.084 | 0.0 |
| debris | map | 1/1 | 159.1 | 0.168 | 0.072 | 1.8 |
| debris | layout | 1/1 | 162.4 | 0.290 | 0.070 | 3.2 |

**Clearance:** "real" is measured against the world's actual collision shapes at robot height. The obstacle-list boxes are drawn larger than the objects, for example around a sign's board, which is above robot height. Planning on the map uses the real sizes, so it passes closer to those boxes. Measured against the boxes it reads 0.0 m in four map runs, but the closest real gap was 0.133 m (to a partition wall).

**Keep-outs**, found by measuring rather than guessing:

- A first map-mode run passed 2 cm from a rubble pile. Rubble is 0.37 m tall, so the LiDAR at 0.27 m mapped only its top.
- Every object lower than 0.5 m is now a keep-out, enforced by `test_site_model`.
- So are the pallet bases, which stick out beyond their stacks, and the objects that could otherwise be routed through on the map.

Data: [runs.csv](runs.csv).
