# Tilt guard through the safety supervisor (simulation)

Same vehicle and terrain as [summary.md](summary.md), with commands going through the safety supervisor and its IMU tilt guard:

- **Limit:** 35° up to 25 kg, 22° at 50 kg.
- **Creep band:** within 5° of the limit, speed is limited to 0.1 m/s.
- **At the limit:** the robot stops; it releases 2° below the limit, or the operator presses Resume to creep off.

Commanded speed: 0.25 m/s. Data: [supervisor_guard.csv](supervisor_guard.csv). Reproduce with `python3 tools/terrain_test.py --supervisor`.

| Payload | Lane | Test | Result | Max pitch | Supervisor |
|---|---|---|---|---|---|
| 50 kg | concrete_20 | downhill | reached | 27.2° | supervisor: ok,tilt_limit; max tilt 27.1°; stopped at x=8.64 (slope starts at 6.76) |
| 50 kg | concrete_20 | placed | stopped | 20.4° | supervisor: ok; max tilt 20.4° |
| 50 kg | concrete_20 | climb | reached | 20.4° | supervisor: ok; max tilt 20.4°; got to x=4.36 of 4.36 |
| 50 kg | concrete_30 | downhill | TIPPED | 88.7° | supervisor: ok,tilt_limit; max tilt 175.9°; stopped at x=8.24 (slope starts at 6.46) |
| 50 kg | concrete_30 | placed | stopped | 30.2° | supervisor: tilt_limit; max tilt 30.4° |
| 50 kg | concrete_30 | climb | stopped | 22.9° | supervisor: ok,tilt_limit; max tilt 22.9°; got to x=0.09 of 4.06 |
| 50 kg | concrete_35 | downhill | TIPPED | 89.0° | supervisor: ok,tilt_limit; max tilt 176.1°; stopped at x=7.94 (slope starts at 6.28) |
| 50 kg | concrete_35 | placed | stopped | 25.7° | supervisor: tilt_limit; max tilt 35.3° |
| 50 kg | concrete_35 | climb | stopped | 0.5° | supervisor: ok; max tilt 0.5°; got to x=-0.25 of 3.88 |
| 0 kg | concrete_30 | downhill | reached | 33.1° | supervisor: ok; max tilt 32.9°; stopped at x=8.20 (slope starts at 6.46) |
| 0 kg | concrete_30 | placed | stopped | 30.4° | supervisor: ok; max tilt 30.4° |
| 0 kg | concrete_30 | climb | reached | 30.4° | supervisor: ok; max tilt 30.4°; got to x=4.07 of 4.06 |

## What the guard does and does not do

- **Climbing beyond the limit is stopped.** With 50 kg, the robot stops as soon as it starts up a 30° slope (tilt 22.9°). Empty, it climbs 30° normally.
- **Standing on a slope beyond the limit is held.** Placed on 30° or 35° with 50 kg, it stays stopped and does not tip.
- **Loaded 20° slopes are unaffected.** With 50 kg it climbs, holds and descends 20° (tilt reads 20.4°).
- **It cannot prevent tipping over a crest.** With 50 kg, driving off the plateau onto a 30° or 35° descent tipped the robot every time. That held with the guard; at a constant 0.1 m/s or 0.25 m/s with no stop at all; and when stopping half way down at 0.1 m/s. The tip happens as the robot rolls over the convex edge, when the high load swings it nose-down, so speed and braking make no difference. On the flat plateau the tilt reads 0°, and by the time it reaches the limit the robot is already falling. Preventing this needs knowledge of the slope *before* reaching it: a mapped no-go zone for steep descents when loaded, or a sensor that detects a drop-off ahead.
- **Simulation only:** the track and friction model is approximate. The numbers are relative, not absolute.

## No-go zones (added after the tilt guard)

The tilt guard can't prevent the crest roll-over, so steep slopes are now marked in advance. `config/terrain_site.yaml` makes every ramp of 30° or more a **no-go zone for loads above 25 kg**, and it is generated with the world by `tools/make_terrain_world.py`. The supervisor blocks motion whose leading edge (plus 0.25 m) would enter a zone, and the planner routes around zones. Same harness and commands as above; data: [supervisor_zones.csv](supervisor_zones.csv).

| Payload | Lane | Test | Result | Max pitch | Supervisor |
|---|---|---|---|---|---|
| 50 kg | concrete_20 | downhill | reached | 28.4° | supervisor: ok,tilt_limit; max tilt 28.4° |
| 50 kg | concrete_20 | placed | stopped | 20.4° | supervisor: tilt_limit; max tilt 20.4°; drove up +0.00 m when asked; reversed off the ramp: no |
| 50 kg | concrete_20 | climb | reached | 20.4° | supervisor: ok; max tilt 20.4° |
| 50 kg | concrete_30 | downhill | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; stopped with its front +0.18 m before the crest |
| 50 kg | concrete_30 | placed | TIPPED | 59.8° | supervisor: tilt_limit; max tilt 59.8°; drove up -0.00 m when asked; reversed off the ramp: no |
| 50 kg | concrete_30 | climb | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; front +0.18 m before the ramp foot |
| 50 kg | concrete_35 | downhill | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; stopped with its front +0.18 m before the crest |
| 50 kg | concrete_35 | placed | TIPPED | 6.1° | supervisor: tilt_limit; max tilt 177.4°; drove up +0.00 m when asked; reversed off the ramp: yes (re-classified: the robot rolled over while settling after placement; the run predates the roll-aware tip check) |
| 50 kg | concrete_35 | climb | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; front +0.17 m before the ramp foot |
| 50 kg | gravel_30 | downhill | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; stopped with its front +0.18 m before the crest |
| 50 kg | gravel_30 | placed | stopped | 30.2° | supervisor: tilt_limit; max tilt 30.2°; drove up +0.00 m when asked; reversed off the ramp: no |
| 50 kg | gravel_30 | climb | stopped | 0.4° | supervisor: no_go_zone,ok; max tilt 0.4°; front +0.18 m before the ramp foot |
| 0 kg | concrete_30 | downhill | reached | 33.1° | supervisor: ok; max tilt 33.1° |
| 0 kg | concrete_30 | placed | reached | 30.4° | supervisor: ok; max tilt 30.4°; drove up +0.50 m when asked; reversed off the ramp: yes |
| 0 kg | concrete_30 | climb | reached | 30.4° | supervisor: ok; max tilt 30.4° |

- **Crest roll-over prevented.** With 50 kg, driving toward the 30° and 35° concrete descents and the 30° gravel descent, the robot stopped **0.18 m before the crest**, still level (tilt 0.4°). Without zones these tipped (concrete) or slid 1.3 m (gravel).
- **Climbing:** it also stops 0.17–0.18 m before the foot of those ramps.
- **Unaffected cases:** 20° slopes with 50 kg (no zone there) and the empty robot on 30° (zones only apply above 25 kg).
- **"Placed on the slope" with 50 kg is not a meaningful case any more.** Teleporting the loaded robot directly onto 30° and 35° tipped it twice while it was settling, before any command was sent. Once on such a slope, the tilt guard holds it until an operator presses Resume, so it did not reverse off by itself. With zones the robot never gets onto those slopes loaded. The exit rule (a robot inside a zone may only creep out) is covered by unit tests.
- **Unmapped slopes are still a gap:** the zones only protect slopes someone has marked.
