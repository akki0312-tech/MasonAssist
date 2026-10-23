# Payload & terrain test (simulation)

Vehicle only: no navigation or safety supervisor; commands sent straight to the tracks. Robot 100 kg + payload; drive acceleration limited to 0.5 m/s². Gazebo track model with Coulomb friction (track mu 0.9, surface mu as listed) - read the numbers as relative, not absolute.

**Climb from level ground at 0.2 m/s (time to plateau, slip on the slope, max pitch)**

| Payload | 10° concrete (μ 1.0) | 20° concrete (μ 1.0) | 30° concrete (μ 1.0) | 35° concrete (μ 1.0) | 10° gravel (μ 0.6) | 20° gravel (μ 0.6) | 30° gravel (μ 0.6) | 10° clay (μ 0.4) | 20° clay (μ 0.4) |
|---|---|---|---|---|---|---|---|---|---|
| 0 kg | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✓ 59.3 s, slip 0%, pitch 30.4° | ✗ stalled at the level-to-slope corner 78.5 s | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✗ stalled at the level-to-slope corner 80.8 s | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° |
| 15 kg | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✓ 76.2 s, slip 0%, pitch 30.4° | ✗ stalled at the level-to-slope corner 78.5 s | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.7 s, slip 0%, pitch 20.4° | ✗ stalled at the level-to-slope corner 80.8 s | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° |
| 25 kg | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✓ 71.1 s, slip 0%, pitch 30.4° | ✗ stalled at the level-to-slope corner 78.5 s | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.7 s, slip 0%, pitch 20.4° | ✗ stalled at the level-to-slope corner 80.8 s | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° |
| 50 kg | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✗ stalled at the level-to-slope corner 80.8 s | ✗ stalled at the level-to-slope corner 78.5 s | ✓ 30.8 s, slip 0%, pitch 10.4° | ✓ 30.8 s, slip 0%, pitch 20.4° | ✗ stalled at the level-to-slope corner 80.8 s | ✓ 30.7 s, slip 0%, pitch 10.4° | ✓ 30.9 s, slip 0%, pitch 20.4° |

**Placed on the slope: slide in 5 s with zero command / hill start at 0.25 m/s (slip)**

| Payload | 10° concrete (μ 1.0) | 20° concrete (μ 1.0) | 30° concrete (μ 1.0) | 35° concrete (μ 1.0) | 10° gravel (μ 0.6) | 20° gravel (μ 0.6) | 30° gravel (μ 0.6) | 10° clay (μ 0.4) | 20° clay (μ 0.4) |
|---|---|---|---|---|---|---|---|---|---|
| 0 kg | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% |
| 15 kg | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% |
| 25 kg | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% |
| 50 kg | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.146 m / ✗ unstable | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% | 0.0 m / ✓ slip 0% |

**Downhill at 0.25 m/s, zero command half way down: stopping distance / slide in 5 s**

| Payload | 10° concrete (μ 1.0) | 20° concrete (μ 1.0) | 30° concrete (μ 1.0) | 35° concrete (μ 1.0) | 10° gravel (μ 0.6) | 20° gravel (μ 0.6) | 30° gravel (μ 0.6) | 10° clay (μ 0.4) | 20° clay (μ 0.4) |
|---|---|---|---|---|---|---|---|---|---|
| 0 kg | 0.073 m / 0.0 m | 0.072 m / 0.0 m | 0.071 m / 0.0 m | 0.068 m / 0.0 m | 0.077 m / 0.0 m | 0.076 m / 0.0 m | 0.176 m / 0.0 m | 0.076 m / 0.0 m | 0.096 m / 0.0 m |
| 15 kg | 0.072 m / 0.0 m | 0.072 m / 0.0 m | 0.072 m / 0.0 m | 0.067 m / 0.0 m | 0.072 m / 0.0 m | 0.078 m / 0.0 m | 0.171 m / 0.0 m | 0.068 m / 0.0 m | 0.109 m / 0.0 m |
| 25 kg | 0.071 m / 0.0 m | 0.066 m / 0.0 m | 0.072 m / 0.0 m | 0.075 m / 0.0 m | 0.074 m / 0.0 m | 0.071 m / 0.0 m | 0.168 m / 0.0 m | 0.068 m / 0.0 m | 0.102 m / 0.0 m |
| 50 kg | 0.069 m / 0.0 m | 0.067 m / 0.0 m | 3.231 m / 0.041 m ✗ unstable | 3.6 m / 0.001 m ✗ unstable | 0.072 m / 0.0 m | 0.07 m / 0.0 m | 1.342 m / 0.0 m | 0.071 m / 0.0 m | 0.106 m / 0.0 m |

**Level ground and side slope**

| Payload | Flat stop from 0.25 m/s | 10° side slope: max roll / sideways drift per m |
|---|---|---|
| 0 kg | 0.067 m | 10.0° / -0.0 m |
| 15 kg | 0.076 m | 10.0° / 0.0 m |
| 25 kg | 0.065 m | 10.0° / 0.0 m |
| 50 kg | 0.075 m | 10.0° / 0.0 m |
