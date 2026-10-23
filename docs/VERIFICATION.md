# MasonAssist - verification log

Run-by-run records of how each feature was checked live in simulation, in the order the features were built. The headline, reproducible results are in the [README](../README.md#results). These are small samples: they show each behaviour working, not statistical reliability.

Several of these runs predate later changes: the localisation stack (they used perfect odometry), the 0.5 m/s² drive acceleration limit and the richer site. Where a number was re-measured after a change, both values are given.

## Missions and queue

**Repeatability** — 6 runs, each from a clean restart, robot starting at the dock, final position from Gazebo ground truth:

| Mission | Result | Time (s) | Final distance to dock |
|---|---|---|---|
| bricks_to_building | 3/3 complete | 166.0 / 166.6 / 168.2 | 0.28 m |
| blocks_to_scaffold | 3/3 complete | 184.0 / 186.4 / 185.8 | 0.29 m |

**Job queue** — 3 queued jobs ran back to back with no operator input (151.8 s, 162.6 s, 148.2 s, all complete). Cancelling a running job recorded it as aborted, paused the queue and stopped the robot; resuming started the next job, which completed from mid-site.

## Safety

**Safety** — a person-sized obstacle placed 0.75 m in front of the moving robot: it stopped within one 0.5 s sample, held still while the obstacle stayed, and resumed automatically after it was removed. In normal missions, passing workers caused slow-downs only, with no false stops from known structures.

**Safety supervisor** — each condition triggered on its own while the robot was moving at ~0.2 m/s, speed taken from `/odom`:

| Test | Result |
|---|---|
| Before the supervisor existed: navigation node frozen (`kill -STOP`) mid-drive | robot kept driving on its last command, **over 3 m** (x −0.49 → −3.72) |
| Navigation node frozen | stopped in 0.34–0.38 s, moved 0.06–0.07 m (`stale_cmd` at 0.3 s). With the 0.5 m/s² drive acceleration limit added later: 0.69 s, 0.09 m |
| **Safety supervisor itself** frozen | before the backstop: robot kept going (0.59 m in 3 s). With the track controllers' 0.5 s command timeout: stopped in 0.62 s, moved 0.12 m |
| LiDAR relay (`scan_frame_fix`) frozen | stopped in 0.36 s, moved 0.07 m (`stale_scan` at 0.51 s) |
| Mission server frozen | stopped in 1.67 s, moved 0.39 m (`link_lost` once the last heartbeat was 2 s old) |
| 1.0 m/s injected on `/cmd_vel_nav` | measured top speed 0.250 m/s |
| E-stop → Clear E-Stop → Resume, nav commands still coming | 0.000 → 0.000 (`hold_after_estop`) → 0.150 m/s |
| Cancel, then a new goal | 0.000 (`cancelled`) → 0.150 m/s |
| Driven straight at a box 0.44 m ahead, navigation frozen | stopped with 0.12 m between body and box (ground truth); turning was still allowed (72° in 2 s) |
| Full missions with the supervisor in the loop | bricks_to_building complete 175.6 s (started 1.1 m from the dock), blocks_to_scaffold complete 186.1 s. During blocks, `stale_cmd` fired twice for 75 ms and 140 ms: navigation went quiet for over 0.3 s and the robot got zero until it came back |
| Same bricks mission with the Gazebo window open (0.8× real time) | wall-clock timeouts: 9 `stale_cmd` + 2 `stale_scan` false stops; after switching to sim-time ages: 0 false stops (3,618 status samples all `ok`). The freeze tests above were repeated after the change with the same results (0.32 / 0.38 / 1.76 s) |
| `/clock` bridge frozen | navigation kept driving on live LiDAR, correctly: nothing it uses was stale |
| `/clock` bridge **and** navigation node frozen | stopped in 1.04 s, moved 0.21 m (`stale_cmd` from the 1.0 s wall-clock backstop) |

**Tilt guard** — the IMU tilt guard was checked on the terrain lanes with the supervisor in the loop: [terrain/supervisor_guard.md](terrain/supervisor_guard.md). Adding the IMU (10 Hz) cost about 3 % simulation speed (real-time factor 0.89 vs 0.92 without it); a bricks job with it active completed with no supervisor stops (151.9 s of sim time, as before).

## Site and obstacles

**Richer site (current world)** — after adding the site facilities and equipment (15 new obstacles, among them a formwork stack placed on the straight-line brick route), with the safety supervisor in the loop and each run from a clean restart:

| Run | Result | Time (s) | Final distance to dock | Learned cells | Replans / reroutes |
|---|---|---|---|---|---|
| bricks_to_building ×2 | 2/2 complete | 170.7 / 169.3 | 0.29 / 0.28 m | 0 / 0 | 0 / 0 |
| blocks_to_scaffold ×3 | 3/3 complete | 185.5 / 199.4\* / 182.7 | 0.27 / 0.28 / 0.28 m | 3 / 3 / 0 | 0 / 0 |

The planner now takes the brick legs around the formwork stack (+0.46 m and +0.62 m). Five map-click goals into the new areas (west yard past the formwork, gate, beside the site office, east yard by the generator, back to dock) were all reached, each within 0.27–0.29 m by ground truth, with no false stops. The 3 learned cells in the first two blocks runs showed an **existing** registry error: `pipe_pile` is rolled 90°, so its pipes reach 0.15 m beyond its obstacle box. `rebar_bundle` and `timber_stack` had the same kind of error. All three boxes were corrected, and the third blocks run learned 0 cells. A test now checks each model's real 3D shape extents against the registry, and it caught two more boxes that were too small (the site office's entry steps and the skip's outward-leaning end walls). \*The 199.4 s run had an extra logging process running alongside it; the sim slows under that load and mission time is wall-clock.

**Live obstacle map** — an unmapped 1.2 × 1.0 m pallet placed on the planned route to the brick pickup: the robot learned it (63 cells, all on the pallet, none elsewhere), rerouted around it 5 times across two legs with no stuck re-plans, and completed the mission in 170.2 s. After the pallet was removed, the learned cells cleared by themselves within ~6 s of the robot seeing through the space, and a full mission then completed in 148.4 s with 0 reroutes and 0 learned cells — passing workers were not learned.

## Custom jobs and failure handling

**Custom jobs** — three custom jobs queued back to back all completed, each going load → carrying → unload → back to the dock: station → station (`block_pickup` → `building_door`, 135.4 s), map point → map point ((4.5, −1.5) → (−5.5, −1.0), 119.4 s) and map point → station ((−3.7, 4.5) → `scaffold`, 124.2 s). Each ended 0.27–0.30 m from the dock, with 0 replans and 0 learned cells. Rejected with reasons: a point inside the site office, a point outside the site, an unknown station, pick-up = drop-off, and a drop-off inside the building that the planner can't reach through the door.

**Failure handling** — a goal sealed off by two temporary walls: re-planned at 46 s and 91 s, then the mission failed at 136 s with *"no progress toward goal after 2 replans"* and the robot stopped.

## Fixes found by measuring

**Fixes found by measuring** — the TrackedVehicle plugin's own odometry drifted ~1.7 m / 35° after a minute of skid-steer turning (now replaced by `OdometryPublisher`); simulation speed went from 0.57× to 0.81× real time with the GUI and 0.99× headless after removing per-tick CLI process launches.
