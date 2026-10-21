# MasonAssist — ROS 2 / Gazebo Development Handoff

> **See [README.md](README.md) for the current system and how to run the demo.** This
> document records the original robot geometry, TF and propulsion validation; its
> "Not Yet Implemented" list below is historical.

> **Status:** This document reflects the current, verified simulation baseline. Earlier
> revisions of this file described an in-progress diagnostic state (invisible RViz
> geometry, unverified mesh transforms, no Gazebo integration). Those issues are
> resolved; this revision documents what is actually working today, verified by
> direct inspection rather than assumption.

## 1. Project Overview

**Project:** MasonAssist — Autonomous Construction Material Handling Robot

The robot is designed for construction sites to transport materials such as bricks,
cement bags, hollow blocks, granite, and other construction materials.

### Target Specifications

* **Maximum payload:** 50 kg
* **Preliminary total operating mass:** ~80 kg including payload
* **Robot type:** Tracked skid-steer
* **Drivetrain:** Two independently driven tracks
* **Terrain:** Designed for uneven construction-site terrain
* **Scope:** Initially a functional prototype, not a commercial-ready machine

### Functional Roadmap

1. Manual/basic movement
2. Straight-line driving
3. Skid steering
4. Pivot/zero-radius turning
5. Slope climbing
6. Obstacle detection/avoidance
7. Autonomous navigation
8. Payload transport
9. ROS 2 + Gazebo simulation
10. Physical prototype validation

---

## 2. Environment (Confirmed)

* **OS:** Ubuntu 24.04 ARM64 (VM)
* **ROS 2:** Jazzy
* **Gazebo:** Harmonic, `gz sim` **8.11.0**
* **SDF:** `sdformat14` / SDF spec version **1.11**
* **Build tool:** `colcon`

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Workspace:** `/home/akkshay/Desktop/masonassist_ws`
**Package:** `src/masonassist_description` (`ament_cmake`)

---

## 3. Robot Geometry (Currently Defined)

| Parameter | Value |
| :--- | ---: |
| Chassis footprint (raw STL bounds) | 725 × 165 × 525 mm |
| Track separation | **0.6555 m** |
| Track width | 0.085 m |
| Drive sprocket effective radius | 0.070 m |
| Track pitch | 0.020 m |
| Approximate track loop length | 1.280 m |
| Pitch count | 64 |
| `base_footprint` → `base_link` | **0.07683 m** (xyz `0 0 0.07683`, rpy `0 0 0`) |

`track_width` and `drive_radius` are declared as Xacro properties but are not
currently referenced anywhere in the geometry itself (pre-existing, not addressed
in this phase).

---

## 4. Validated Mesh Transforms (LOCKED — do not alter)

**Chassis** (`meshes/chassis.stl`):
```xml
scale="0.001 0.001 0.001"
rpy="1.5708 0 0"
xyz="0 0 0.1525"
```

**Left/right tracks** (`meshes/left_track.stl`, `meshes/right_track.stl` — byte-identical
files, same transform for both):
```xml
scale="0.001 0.001 0.001"
rpy="-1.1687 0 0"
xyz="-0.24605 -0.06752 0.05185"
```

Both transforms were independently re-derived from the actual STL vertex/triangle
data (not just trusted) and confirmed correct, including the nominal belt contact
plane and `base_footprint` ground-reference height.

---

## 5. Gazebo Architecture (LOCKED)

* **Dedicated hand-authored SDF model:** `models/masonassist/model.sdf` — not
  generated from the Xacro.
* Three separate physics links: `base_link`, `left_track`, `right_track`.
* `left_track`/`right_track` are rigidly attached to `base_link` via `fixed` joints
  (not `continuous` — that joint type remains in the Xacro for RViz/TF purposes only,
  see §7).
* Full STL collision geometry on all three links (not simplified).
* **`gz::sim::systems::TrackedVehicle`** plugin: `tracks_separation=0.6555`,
  `steering_efficiency=0.5`.
* **Two `gz::sim::systems::TrackController`** instances, one per track:
  `track_orientation="0 0 0"`, `min_velocity=-1.0`, `max_velocity=1.0`.
* Spawned via `ros_gz_sim create -file <model.sdf>` (not `-topic robot_description`).

### Why a dedicated SDF, not a Xacro-generated one

Tested and rejected: annotating the Xacro's fixed track joints with
`<gazebo reference="..."><preserveFixedJoint>true</preserveFixedJoint></gazebo>`
and converting via `gz sdf -p` (the same URDF→SDF path `ros_gz_sim create -topic`
uses internally). Directly inspecting the generated SDF confirmed the annotation
is silently ignored by this installed `sdformat14`/Gazebo 8.11.0 build — the fixed
joints are still lumped into `base_link`, leaving no separate `left_track`/
`right_track` physics links for `TrackedVehicle`/`TrackController` to find. A
dedicated hand-authored SDF, spawned via `-file`, is therefore the locked
architecture — proven end-to-end with real STL collisions and working propulsion.

---

## 6. ROS Interfaces (Verified)

| Interface | Direction | Verified |
|---|---|---|
| `/cmd_vel` | ROS → Gazebo | `geometry_msgs/Twist` ↔ `gz.msgs.Twist`, working |
| `/odom` | Gazebo → ROS | `nav_msgs/Odometry` ↔ `gz.msgs.Odometry`, working |
| `/tf` | mixed | see §7 |
| `/tf_static` | `robot_state_publisher` | working |

Gazebo's own `/model/masonassist/tf` (`gz.msgs.Pose_V`) is **deliberately not
bridged** — verified directly that its Pose entries carry no entity name, so the
bridged `TFMessage` would always have an empty `child_frame_id`, which floods any
tf2 consumer with `TF_NO_CHILD_FRAME_ID` errors. A small node
(`scripts/odom_to_tf_broadcaster.py`) republishes `/odom` as TF instead.

---

## 7. Final TF Tree (Verified)

```
odom
  └── base_footprint      (odom_to_tf_broadcaster — dynamic)
        └── base_link     (robot_state_publisher — static)
              ├── left_track   (robot_state_publisher — dynamic)
              └── right_track  (robot_state_publisher — dynamic)
```

* `robot_state_publisher` publishes from the Xacro, which still declares
  `left_track_joint`/`right_track_joint` as `continuous` (unchanged from the
  original model — only the *Gazebo-side* SDF uses `fixed` joints for these).
  `joint_state_publisher` is required in `gazebo.launch.py` for those two edges
  to exist at all.
* Exactly one publisher owns each edge — verified via `ros2 topic info -v`.
* Full chain resolution (`odom` → `left_track`) verified via `tf2_echo` during
  real commanded motion, with matching sim-time timestamps throughout and zero
  `TF_NO_CHILD_FRAME_ID` / `TF_REPEATED_DATA` / `TF_OLD_DATA` / extrapolation
  errors.
* RViz Fixed Frame is `odom` (in `gazebo.launch.py`'s context, where the
  broadcaster is running). `display.launch.py` does not run Gazebo or the
  broadcaster, so `odom` does not exist in that context — see §10.

---

## 8. Propulsion Baseline (Measured, Real Repository)

Commanded via `/cmd_vel`, verified against actual Gazebo model pose
(`gz model -m masonassist -p`), not just plugin-reported odometry:

| Test | Commanded | Observed |
|---|---|---|
| Straight | v=0.2 m/s, ω=0 | ~0.21 m/s actual, straight line, no lateral drift |
| Turn | v=0.2 m/s, ω=0.3 rad/s | Yaw increases steadily (~0.43 rad/s), correct direction |
| Pivot | v=0, ω=0.5 rad/s | Yaw rotates (~0.9 rad/s), correct direction; small (~4 cm) lateral drift, better-centered than earlier feasibility testing |
| Zero command | v=0, ω=0 | Continues briefly (~1 s, momentum/friction-driven), then comes to a complete, stable stop |

No jitter, tipping, or instability observed in any test.

---

## 9. Known Limitation

`TrackedVehicle`'s odometry is a **dead-reckoning estimate** computed from
commanded track velocities, not measured from actual physics — it differs in
magnitude from Gazebo ground truth because of real tracked-vehicle slip (observed
consistently: actual throughput has ranged from roughly under to roughly meeting
commanded values depending on ground-plane friction settings). This is expected,
documented, and **not tuned** as of this baseline.

---

## 10. Not Yet Implemented

```text
- LiDAR
- IMU
- SLAM
- Nav2
- autonomous navigation
- physical hardware integration
- collision mesh simplification (full STL currently used throughout)
- friction / steering_efficiency tuning
```
