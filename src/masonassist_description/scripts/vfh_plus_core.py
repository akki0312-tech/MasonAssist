#!/usr/bin/env python3
"""Standalone VFH+ (Vector Field Histogram Plus) core for MasonAssist.

No rclpy, no ROS messages, no Gazebo, no TF. Pure Python/numpy so the
algorithm can be exercised directly with synthetic data. A future ROS node
wraps this module; it does not reimplement any of the math.

COORDINATE CONVENTION
----------------------
Every angle and distance this module consumes must already be expressed in
the ROBOT frame (base_link): distance is measured from the robot's own
origin (not the LiDAR), and angle 0 rad = straight ahead (+X, forward),
with positive angles turning left (+Y), matching this project's existing
convention (positive angular.z = left turn, verified throughout the
propulsion baseline). Angles are always in (-pi, pi].

The LiDAR is mounted 0.30 m forward of base_link (see HANDOFF.md). This
module never hard-codes that offset - it is the caller's job to convert
raw sensor-frame (range, angle) pairs into robot-frame (range, angle)
pairs before calling `VFHPlusCore.process()`. `sensor_point_to_robot_frame`
below is a small, optional helper for doing exactly that conversion; it is
not part of the core pipeline and the core has no dependency on it.

PIPELINE
--------
1. Preprocessing: drop NaN/inf/out-of-range readings. Invalid data is never
   treated as "clear" - it is simply absent from the histogram.
2. Footprint inflation: each valid point at distance d is spread across an
   angular half-width gamma = asin(r_effective / d) around its own bearing,
   where r_effective = r_robot + safety_margin. This is what makes the
   histogram aware of the robot's actual body size rather than treating it
   as a point - directly targeting the corner-contact failure observed with
   the earlier stateless controller (see test F).
3. Polar histogram: 72 sectors of 5 degrees each. Every valid point
   contributes a distance-weighted magnitude (see `point_magnitude`) to
   every sector its inflated angular span covers.
4. Smoothing: a small circular moving average damps single-sector noise.
5. Binary histogram (stateful): dual-threshold hysteresis classifies each
   sector occupied/free, keeping its previous classification when the
   smoothed magnitude falls between the two thresholds.
6. Valley extraction: contiguous runs of free sectors. Each valley's
   proposed candidate direction is then validated by a direct geometric
   clearance check (`is_direction_safe`) against every valid point in the
   CURRENT scan - not just "is the sector nominally free," and not by
   compressing the valley's boundary into one arbitrary bordering
   obstacle's distance (an earlier version did this; it could reject
   genuinely safe geometry - see `is_direction_safe`'s docstring).
7. Candidate selection (stateful): among valleys wide enough for the robot,
   pick the direction minimizing a weighted goal/heading/previous-direction
   cost. The previous-direction term is what gives VFH+ hysteresis, in
   contrast to the earlier stateless controller.

v1 ASSUMPTION: goal_direction is fixed at 0 rad (straight ahead in the
robot's own current frame) - there is no Nav2/SLAM/waypoint source yet.
`current_heading_error` is likewise 0.0 by default for the same reason: in
this ROS-free core there is no odometry input, so "current heading" and
"goal direction" both reduce to the same reference (0). This makes the two
cost terms mathematically redundant in v1 specifically - not a bug, an
explicit, documented consequence of not having a real goal or heading
source yet. Both terms are still implemented distinctly so the interface
does not need to change when a real goal/heading source exists.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

@dataclass
class VFHParams:
    """All values below marked "initial/tunable" have not been validated
    against real histogram data or real runs - they exist so the algorithm
    and its unit tests can execute, per the explicit instruction not to
    pretend otherwise."""

    # Robot footprint. r_robot is derived from the real chassis footprint
    # (0.725 m x 0.525 m): sqrt(0.3625^2 + 0.2625^2) = 0.4475 m
    # (circumscribed radius from base_link origin to the farthest corner).
    r_robot: float = 0.4475
    # Initial/tunable: absorbs track slip, localization slop, sensor noise.
    # Proposed range was 0.15-0.20 m; 0.15 chosen as the initial default.
    safety_margin: float = 0.15

    # Planning horizon, m. Returns beyond this never enter the histogram -
    # distant clutter should not dominate local steering. Initial/tunable.
    d_max: float = 4.0

    # Sensor limits (must match the actual LiDAR config: 0.1-10.0 m).
    range_min: float = 0.1
    range_max: float = 10.0

    # Histogram: 5 deg sectors -> 72 sectors, dividing 360 deg cleanly.
    sector_width_deg: float = 5.0

    # Smoothing window, sectors. Must be odd for a symmetric window.
    smoothing_window: int = 3

    # Binary histogram hysteresis. Initial/tunable - not validated against
    # real magnitude distributions yet. Units match `point_magnitude`
    # (dimensionless, summed per sector; see point_magnitude docstring).
    # A single point's magnitude is at most 1.0 (touching); these are set
    # low enough that one or a few nearby beams can cross threshold_high,
    # matching what a real, sparsely-returning obstacle should do.
    threshold_high: float = 0.4
    threshold_low: float = 0.2

    # Candidate-direction cost weights. goal dominates by design; see
    # `select_direction`. Initial/tunable.
    weight_goal: float = 5.0
    weight_heading: float = 1.0
    weight_previous: float = 2.0

    # v1: fixed, no goal source exists yet.
    goal_direction: float = 0.0

    # Optional planning horizon, m (None = d_max, the original behavior).
    # A caller with a nearby goal sets it to goal_distance + r_effective:
    # returns beyond the point where the robot will stop cannot be reached,
    # and without this a wall 2.5 m BEHIND a goal 1 m away vetoed every
    # direction toward that goal (observed live: open space, 0/72 sectors
    # occupied, goal direction never accepted, robot orbited the goal).
    horizon: Optional[float] = None

    @property
    def planning_horizon(self) -> float:
        return self.d_max if self.horizon is None else min(self.d_max, self.horizon)

    @property
    def sector_width_rad(self) -> float:
        return math.radians(self.sector_width_deg)

    @property
    def num_sectors(self) -> int:
        return int(round(360.0 / self.sector_width_deg))

    @property
    def r_effective(self) -> float:
        return self.r_robot + self.safety_margin


class VFHState(Enum):
    OK = "ok"                # a valid direction was selected
    BLOCKED = "blocked"      # valid scan data, but no valley fits the robot
    NO_DATA = "no_data"      # no valid scan points at all


@dataclass
class Valley:
    start_sector: int
    end_sector: int          # inclusive; may be < start_sector (wraps past 71->0)
    width_rad: float
    candidate_direction: float
    accepted: bool


@dataclass
class VFHResult:
    state: VFHState
    selected_direction: Optional[float]
    selected_valley: Optional[Valley]
    histogram: np.ndarray            # smoothed magnitude per sector
    binary_histogram: np.ndarray     # bool per sector, True = occupied
    candidate_valleys: List[Valley]  # accepted (wide-enough) valleys
    rejected_valleys: List[Valley]   # too-narrow-for-the-footprint valleys
    min_obstacle_distance: Optional[float]  # closest valid point this scan, m


# ---------------------------------------------------------------------------
# Small pure helpers
# ---------------------------------------------------------------------------

def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angdiff(a: float, b: float) -> float:
    """Signed shortest angular difference a - b, in (-pi, pi]."""
    return wrap_to_pi(a - b)


def sensor_point_to_robot_frame(
    range_: float, angle: float, sensor_offset_x: float, sensor_offset_y: float = 0.0
) -> Tuple[float, float]:
    """Optional helper (NOT part of the core pipeline): convert one raw
    sensor-frame (range, angle) reading into a robot-frame (distance, angle)
    pair, given the sensor's fixed offset from base_link. For MasonAssist,
    sensor_offset_x=0.30, sensor_offset_y=0.0 (see module docstring)."""
    x = range_ * math.cos(angle) + sensor_offset_x
    y = range_ * math.sin(angle) + sensor_offset_y
    d = math.hypot(x, y)
    a = math.atan2(y, x)
    return d, a


def enlargement_half_angle(d: float, r_effective: float) -> float:
    """Angular half-width an obstacle at distance d occupies around its own
    bearing, so that steering outside it keeps the robot's r_effective away.

    For d > r_effective: gamma = asin(r_effective / d) - the standard VFH+
    enlargement angle.
    For d <= r_effective: the point is at or inside the safety envelope.
    asin is undefined there (its argument would exceed 1), so this is
    capped at pi/2 - the limit of asin(r_effective/d) as d -> r_effective
    from above. That means a full hemisphere centered on the obstacle's
    bearing is marked occupied: conservative, continuous with the formula
    just outside the envelope, and deliberately not extended to a full
    circle (see module docstring test C / test discussion for why)."""
    if d <= r_effective:
        return math.pi / 2.0
    ratio = min(1.0, r_effective / d)
    return math.asin(ratio)


def point_magnitude(d: float, d_max: float) -> float:
    """Distance-weighted magnitude of one valid return, dimensionless,
    range [0, 1]. 1.0 at d=0 (touching), decaying linearly to 0.0 at
    d=d_max, clamped to 0 beyond d_max. A sector's total magnitude is the
    sum of every contributing point's magnitude (denser/closer clusters of
    returns push a sector toward "occupied" faster than a single distant
    one) - the "stronger near, weaker far" behavior VFH requires, without
    the extra unexplained constants of the classic c^2*(a-b*d) form."""
    if d >= d_max:
        return 0.0
    if d <= 0.0:
        return 1.0
    return (d_max - d) / d_max


def angle_to_sector(angle: float, num_sectors: int) -> int:
    """Nearest sector index for `angle`, matching the convention that
    sector 0 is centered exactly at angle 0 (see `sector_center`)."""
    sector_width = 2.0 * math.pi / num_sectors
    theta = angle % (2.0 * math.pi)
    return int(round(theta / sector_width)) % num_sectors


def sector_center(k: int, num_sectors: int) -> float:
    """Center angle of sector k, wrapped to (-pi, pi]. Sector 0 is centered
    at exactly 0 rad by construction."""
    sector_width = 2.0 * math.pi / num_sectors
    return wrap_to_pi(k * sector_width)


def sectors_in_span(center_angle: float, half_angle: float, num_sectors: int) -> List[int]:
    """All sector indices whose own angular INTERVAL overlaps the point's
    continuous danger interval [center_angle - half_angle,
    center_angle + half_angle] - a direct interval-overlap test, not a
    snap-the-bearing-to-a-sector-then-round-the-span approximation.

    The earlier version snapped `center_angle` to its nearest sector center
    (via `angle_to_sector`) and then ceil-rounded `half_angle` to a whole
    number of sector widths. Both steps are individually conservative
    (never undershoot) but their errors add, and - verified directly
    against real Gazebo data (VFH+ K-scenario validation) - could manufacture
    up to ~1.5 sector-widths of ANGULAR EXCESS beyond the true continuous
    danger zone for a single point, purely from where that point's bearing
    happened to fall within its own sector. That excess is what caused a
    corridor wall point at bearing 22.09 deg, d=2.134 m (true danger
    interval [5.83, 38.35] deg) to mark sector 0 (centered on 0 deg)
    occupied, despite 0 deg never being inside [5.83, 38.35].

    This version uses the point's TRUE bearing directly - never snapped -
    and tests each sector's own true [center - width/2, center + width/2]
    interval for overlap with the danger interval, using closed
    (boundary-inclusive) comparisons so exact boundary contact still counts
    as occupied (matching the original "never undershoot" safety intent,
    without the incidental excess the snap+ceil combination introduced).

    Safety note: `is_direction_safe` (not this histogram) is the final,
    unconditional arbiter of every candidate direction generated elsewhere
    in this module (see `find_valleys`/`_valley_candidates`). This function
    only decides which candidates get PROPOSED for consideration; it has no
    authority over whether one gets ACCEPTED. Tightening it therefore
    cannot make an unsafe direction selectable - at worst it lets a
    genuinely-safe candidate get proposed and independently re-verified,
    where before it would never have been proposed at all."""
    sector_width = 2.0 * math.pi / num_sectors
    if half_angle >= math.pi:
        return list(range(num_sectors))

    center_angle = wrap_to_pi(center_angle)
    lo = center_angle - half_angle
    hi = center_angle + half_angle

    result = []
    for k in range(num_sectors):
        c_k = sector_center(k, num_sectors)
        sector_lo = c_k - sector_width / 2.0
        sector_hi = c_k + sector_width / 2.0
        # Check the sector's interval and its +-2*pi wrapped copies: the
        # danger interval [lo, hi] is expressed on an unwrapped real line
        # (it can extend below -pi or above +pi when center_angle is near
        # the wrap boundary), while the sector's own interval is anchored
        # inside roughly (-pi, pi]. wrap_to_pi(center_angle) above plus
        # half_angle < pi bounds [lo, hi] within (-2*pi, 2*pi), so these
        # three shifted copies are always sufficient to catch every true
        # overlap, on either side of the +-pi branch cut.
        for shift in (-2.0 * math.pi, 0.0, 2.0 * math.pi):
            if lo <= sector_hi + shift and sector_lo + shift <= hi:
                result.append(k)
                break
    return result


def is_direction_safe(
    candidate_direction: float,
    ranges: np.ndarray,
    angles: np.ndarray,
    r_effective: float,
    d_max: float,
) -> bool:
    """Direct geometric clearance gate for one candidate direction against
    the CURRENT scan's raw valid points - replaces an earlier version that
    compressed a valley's boundary into one arbitrary bordering obstacle's
    distance and reapplied the enlargement-angle formula a second time.
    That approach could reject genuinely safe geometry (verified directly:
    an isolated pair of points at +-45 deg, d=1.2 m left a real 0.849 m of
    perpendicular clearance at 0 rad, well over r_effective, yet the old
    check rejected it) and gave sampling-dependent results for continuous
    walls, since which single point happened to border a histogram sector
    was an artifact of discretization, not geometry.

    For each point (d, angle) with d <= d_max, decomposed relative to the
    candidate heading:
        delta   = wrap_to_pi(angle - candidate_direction)
        along   = d * cos(delta)         - progress along the heading
        lateral = d * abs(sin(delta))    - perpendicular offset from it

    The candidate is unsafe if any such point has:
        d <= r_effective
    or
        along >= -r_effective and lateral < r_effective

    `d <= r_effective` is the same unconditional "inside the safety
    envelope" rule used elsewhere in this module (see
    `enlargement_half_angle`) - direction-independent by construction.

    `along >= -r_effective` (rather than `along >= 0`) is a deliberate,
    verified conservatism, not a tighter version of the same test: for a
    point with along in [-r_effective, 0) and d > r_effective, moving
    forward along the candidate heading only ever increases the distance
    to that point (checked numerically during design - the minimum
    distance over the whole forward path occurs at the start, at exactly
    d, which is already > r_effective). Such a point poses no real hazard
    to forward travel. The check still flags it anyway: r_effective is a
    circumscribed radius meant to cover the robot's body in any
    orientation, and VFH+ does not model the exact swept path of a
    skid-steer turn - this band absorbs some of that unmodeled lateral
    sweep the same way the circumscribed-radius approximation itself is
    already more conservative than a tight rectangular footprint check
    would be. It can flag a small number of technically-harmless points as
    unsafe; it never does the reverse.
    """
    for d, a in zip(ranges, angles):
        if d > d_max:
            continue
        if d <= r_effective:
            return False
        delta = wrap_to_pi(a - candidate_direction)
        along = d * math.cos(delta)
        lateral = d * abs(math.sin(delta))
        if along >= -r_effective and lateral < r_effective:
            return False
    return True


# ---------------------------------------------------------------------------
# Preprocessing
# ---------------------------------------------------------------------------

def filter_valid(
    ranges: np.ndarray, angles: np.ndarray, range_min: float, range_max: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Drop NaN, inf, and out-of-[range_min, range_max] readings.
    Deterministic; invalid input never becomes an implicit "clear" signal -
    it is simply excluded from both the returned arrays and, downstream,
    from the histogram entirely."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return ranges[valid], angles[valid]


# ---------------------------------------------------------------------------
# Histogram construction
# ---------------------------------------------------------------------------

def build_raw_histogram(
    ranges: np.ndarray, angles: np.ndarray, params: VFHParams
) -> np.ndarray:
    """Take, per sector, the MAXIMUM distance-weighted magnitude among every
    valid point whose footprint-inflated angular span covers that sector -
    not a sum. A sector's occupancy evidence reflects how close the nearest
    real danger in that direction is; it must not grow simply because a
    continuous surface (e.g. a wall) happens to return more LiDAR points
    than an isolated obstacle at the same distance would. Verified directly
    against real Gazebo data: a dense 286-point corridor scan drove the old
    summed magnitude to 11.55 (r_effective clearance was in fact 0.78 m,
    comfortably safe) purely from point count, not real proximity - see the
    VFH+ Gazebo validation report. Points beyond d_max are skipped entirely
    (never enter the histogram), same as before."""
    n = params.num_sectors
    hist = np.zeros(n, dtype=float)
    for d, a in zip(ranges, angles):
        if d > params.planning_horizon:
            continue
        mag = point_magnitude(d, params.d_max)
        half_angle = enlargement_half_angle(d, params.r_effective)
        for k in sectors_in_span(a, half_angle, n):
            hist[k] = max(hist[k], mag)
    return hist


def smooth_histogram(hist: np.ndarray, window: int) -> np.ndarray:
    """Deterministic circular moving average, window sectors wide (odd)."""
    if window <= 1:
        return hist.copy()
    n = len(hist)
    half = window // 2
    smoothed = np.zeros(n, dtype=float)
    for k in range(n):
        total = 0.0
        for offset in range(-half, half + 1):
            total += hist[(k + offset) % n]
        smoothed[k] = total / window
    return smoothed


# ---------------------------------------------------------------------------
# Valley extraction
# ---------------------------------------------------------------------------

def _valley_candidates(s: int, e: int, n: int, sector_width: float, goal_angle: float) -> List[float]:
    """All candidate directions to try for one circular free-sector run
    [s, e]: the center angle of every sector in the run, using the EXISTING
    5-deg sector grid (no new resolution introduced), plus the existing
    near-goal/edge candidate from `_clip_to_valley`. A run is only ever
    discarded if NONE of these pass `is_direction_safe` - not merely
    because this one, first-tried candidate does (see module docstring and
    the VFH+ design review: a wide valley with a small near-goal obstacle
    must not be thrown away over one failed probe when most of the same
    valley is geometrically safe)."""
    span = (e - s) % n + 1
    candidates = [sector_center((s + i) % n, n) for i in range(span)]
    candidates.append(_clip_to_valley(goal_angle, s, e, n, sector_width))
    return candidates


def find_valleys(
    binary_hist: np.ndarray,
    valid_ranges: np.ndarray,
    valid_angles: np.ndarray,
    params: VFHParams,
) -> Tuple[List[Valley], List[Valley]]:
    """Contiguous free-sector runs, circular (correctly handles a valley
    that crosses the 71->0 boundary). Every sector-center direction within
    a run is validated by `is_direction_safe` against the current scan's
    raw valid points (see `_valley_candidates`); a run becomes multiple
    accepted Valley entries - one per safe candidate it contains - and is
    rejected only if none of its candidates pass. `VFHPlusCore._select`
    then picks the minimum-cost entry across ALL accepted candidates from
    ALL runs, unchanged - so a run with several safe candidates competes on
    exactly the same existing goal/heading/previous-direction cost terms
    as before, just with more of its own geometry actually considered."""
    n = params.num_sectors
    sector_width = params.sector_width_rad

    def safe(candidate: float) -> bool:
        return is_direction_safe(candidate, valid_ranges, valid_angles, params.r_effective,
                                 params.planning_horizon)

    def accept_run(s: int, e: int, width: float, accepted: List[Valley], rejected: List[Valley]) -> None:
        found_safe = False
        for candidate in _valley_candidates(s, e, n, sector_width, params.goal_direction):
            if safe(candidate):
                accepted.append(Valley(
                    start_sector=s, end_sector=e, width_rad=width,
                    candidate_direction=candidate, accepted=True,
                ))
                found_safe = True
        if not found_safe:
            # No candidate in this run is safe: keep one representative
            # rejected entry (the near-goal/edge candidate) for diagnostics,
            # same as before.
            primary = _clip_to_valley(params.goal_direction, s, e, n, sector_width)
            rejected.append(Valley(
                start_sector=s, end_sector=e, width_rad=width,
                candidate_direction=primary, accepted=False,
            ))

    if not binary_hist.any():
        # Nothing occupied at all - one valley spanning the whole circle.
        # Still gated by the same geometric check: the histogram being
        # all-free means no sector crossed threshold_high, not that there
        # are zero points at all (some may simply be too weak to occupy
        # any sector yet still close enough to matter geometrically).
        accepted: List[Valley] = []
        rejected: List[Valley] = []
        accept_run(0, n - 1, 2.0 * math.pi, accepted, rejected)
        return accepted, rejected

    if binary_hist.all():
        return [], []

    # Walk the circular array once, starting just after an occupied sector,
    # so every free run is captured whole (no run is split across index 0).
    start_idx = int(np.argmax(binary_hist))  # first True; free run begins after it
    order = [(start_idx + i) % n for i in range(n)]

    accepted: List[Valley] = []
    rejected: List[Valley] = []
    run: List[int] = []

    def close_run(run_sectors: List[int]) -> None:
        if not run_sectors:
            return
        s, e = run_sectors[0], run_sectors[-1]
        width = len(run_sectors) * sector_width
        accept_run(s, e, width, accepted, rejected)

    for idx in order:
        if binary_hist[idx]:
            close_run(run)
            run = []
        else:
            run.append(idx)
    close_run(run)

    return accepted, rejected


def _clip_to_valley(goal_angle: float, s: int, e: int, n: int, sector_width: float) -> float:
    """Direction within valley [s, e] (circular) closest to goal_angle: the
    goal angle itself if it falls inside the valley, otherwise the nearer
    of the two valley edges."""
    span = (e - s) % n + 1
    goal_k_rel = (angle_to_sector(goal_angle, n) - s) % n
    if goal_k_rel < span:
        return wrap_to_pi(goal_angle)
    # Outside the valley: pick whichever edge is angularly closer.
    start_angle = sector_center(s, n)
    end_angle = sector_center(e, n)
    if abs(angdiff(goal_angle, start_angle)) <= abs(angdiff(goal_angle, end_angle)):
        return start_angle
    return end_angle


# ---------------------------------------------------------------------------
# Stateful VFH+ core
# ---------------------------------------------------------------------------

class VFHPlusCore:
    """Holds exactly the state VFH+ needs across calls: the previous binary
    histogram (for hysteresis), the previously selected direction (for
    steering-preference hysteresis), and the previous angular command (for
    a future slew-rate limiter). No map, no path history, no SLAM."""

    def __init__(self, params: Optional[VFHParams] = None):
        self.params = params or VFHParams()
        self.binary_state = np.zeros(self.params.num_sectors, dtype=bool)
        self.previous_direction: Optional[float] = None
        self.previous_angular_command: float = 0.0

    def process(
        self, ranges, angles, current_heading_error: float = 0.0
    ) -> VFHResult:
        """ranges/angles: already robot-frame polar coordinates (see module
        docstring). current_heading_error: 0.0 by default in v1 - see
        module docstring for why this is not yet a real odometry input."""
        p = self.params
        valid_ranges, valid_angles = filter_valid(ranges, angles, p.range_min, p.range_max)

        if valid_ranges.size == 0:
            return VFHResult(
                state=VFHState.NO_DATA,
                selected_direction=None,
                selected_valley=None,
                histogram=np.zeros(p.num_sectors),
                binary_histogram=self.binary_state.copy(),
                candidate_valleys=[],
                rejected_valleys=[],
                min_obstacle_distance=None,
            )

        min_obstacle_distance = float(np.min(valid_ranges))

        raw_hist = build_raw_histogram(valid_ranges, valid_angles, p)
        smoothed = smooth_histogram(raw_hist, p.smoothing_window)

        new_binary = np.empty(p.num_sectors, dtype=bool)
        for k in range(p.num_sectors):
            if smoothed[k] > p.threshold_high:
                new_binary[k] = True
            elif smoothed[k] < p.threshold_low:
                new_binary[k] = False
            else:
                new_binary[k] = self.binary_state[k]
        self.binary_state = new_binary

        accepted, rejected = find_valleys(new_binary, valid_ranges, valid_angles, p)

        if not accepted:
            return VFHResult(
                state=VFHState.BLOCKED,
                selected_direction=None,
                selected_valley=None,
                histogram=smoothed,
                binary_histogram=new_binary.copy(),
                candidate_valleys=[],
                rejected_valleys=rejected,
                min_obstacle_distance=min_obstacle_distance,
            )

        selected = self._select(accepted, current_heading_error)
        self.previous_direction = selected.candidate_direction
        self.previous_angular_command = selected.candidate_direction

        return VFHResult(
            state=VFHState.OK,
            selected_direction=selected.candidate_direction,
            selected_valley=selected,
            histogram=smoothed,
            binary_histogram=new_binary.copy(),
            candidate_valleys=accepted,
            rejected_valleys=rejected,
            min_obstacle_distance=min_obstacle_distance,
        )

    def _select(self, valleys: List[Valley], current_heading_error: float) -> Valley:
        p = self.params

        def cost(v: Valley) -> float:
            c = p.weight_goal * abs(angdiff(v.candidate_direction, p.goal_direction))
            c += p.weight_heading * abs(angdiff(v.candidate_direction, current_heading_error))
            if self.previous_direction is not None:
                c += p.weight_previous * abs(angdiff(v.candidate_direction, self.previous_direction))
            return c

        best = min(
            valleys,
            key=lambda v: (round(cost(v), 9), abs(v.candidate_direction), -v.candidate_direction),
        )
        return best
