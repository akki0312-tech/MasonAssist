# Navigation evaluation - 2026-10-23 13:43

11 runs, headless, each from a clean restart with the robot at the dock. Mean ± sd (min–max) over successful runs.

| Scenario | Success | Time (s) | Travelled (m) | Path ratio | Min body clearance (m) | Max arrival error (m) | Replans / reroutes | Learned cells (max) | Supervisor stops | Odom vs ground truth (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| bricks | 3/3 | 151.7 ± 0.4 (151.5–152.2) | 26.23 ± 0.01 (26.23–26.24) | 0.94 ± 0.00 (0.94–0.94) | 0.093 | 0.287 | 0 / 0 | 0 | 0 | 0.0 |
| blocks | 3/3 | 165.9 ± 1.5 (164.7–167.6) | 29.42 ± 0.02 (29.40–29.43) | 0.96 ± 0.00 (0.95–0.96) | 0.218 | 0.289 | 0 / 0 | 1 | 0 | 0.0 |
| custom | 2/2 | 114.2 ± 0.7 (113.7–114.7) | 19.94 ± 0.03 (19.92–19.96) | 0.94 ± 0.00 (0.94–0.94) | 0.257 | 0.286 | 0 / 0 | 0 | 0 | 0.0 |
| goals | 1/1 | 157.1 | 28.12 | 0.95 | 0.101 | 0.293 | 0 / 0 | 0 | 0 | 0.0 |
| debris | 2/2 | 162.0 ± 0.6 (161.6–162.4) | 27.91 ± 0.03 (27.89–27.93) | 1.00 ± 0.00 (1.00–1.00) | 0.089 | 0.293 | 0 / 2 | 27 | 0 | 0.0 |

**Scenarios**

- `bricks`: fixed job bricks_to_building
- `blocks`: fixed job blocks_to_scaffold
- `custom`: custom job, map point (4.5, -1.5) -> map point (-5.5, -1.0)
- `goals`: 5 chained map-click goals across the site
- `debris`: bricks_to_building with an unmapped 1.0 x 0.8 m box spawned on the route at (-3.17, -2.9)

**Failures**

- none

Path ratio = distance driven ÷ shortest planned route on the known layout (1.00 = the planned route; slightly below 1 is normal because the robot cuts waypoint corners within 0.35 m and stops within 0.35 m of each station; above 1 = detours). Body clearance = smallest gap between the robot footprint and any known obstacle box (the boxes cover each object's collision shape, so this is conservative). Supervisor stops = times the safety supervisor overrode navigation. Odom vs ground truth = largest gap between odometry and Gazebo's model pose, checked while the robot was stopped.
