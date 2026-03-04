# Plan: New Merge Algorithm — "Split-in-Middle, Right-Decel-First"

## Goal

Replace the current binary-split merge algorithm in `constant_jerk_planner.h` with a smarter approach that:
- Produces larger superblocks (better motion quality)
- Uses informed split points (rightmost violated junction) instead of blind binary halving
- Computes the right side as a pure decel ramp, which is simpler and gives the max feasible entry speed

## Algorithm Overview

```
1. split = max compatible left extent (left_end, capped at buffer_size / 2 as before)
2. Compute max_right_entry:
   - Model right side as pure 3-phase decel [-j, 0, +j] from V to max_safe_entry[block_count]
     (max_safe_entry[block_count] = 0 when no more blocks visible beyond buffer)
   - For each right junction, check that decel speed at that distance ≤ junction ceiling
   - Reduce V if any junction is violated
   - Cap by vmax_junction[left_end] (the junction AT the split point)
3. Plan left as full 7-phase S-curve: plan_full(left_entry_speed, target_exit, ...)
   - target_exit = min(maxReachableSpeed(..., v_cap, ...), max_right_entry)
     where v_cap = min(left_nominal, vmax_junction[left_end], max_right_entry)
   - Note: if v_cap < left_entry_speed, maxReachableSpeed returns v_cap (ceiling).
     plan_full may then be infeasible (can't decel enough in the available distance).
   - plan_full returns bool (feasible or not). No jerk inflation — removed entirely.
   - If infeasible: leave TODO comment (fix involves remembering previous cycle's right block count)
4. Check left interior junctions using getVelocityAtDistance() on the traj object
   - Scan right to left, stop on first violation (= rightmost violated junction)
   - If violated: set left_end = that junction index, goto 2
5. Emit left superblock with exit = target_exit
   The right side is discarded — it only existed to determine a safe exit speed
```

## Key Insight: O(1) Velocity-at-Distance in the Decel Ramp

The 3-phase decel ramp from V to v_exit is a trapezoidal acceleration profile
(accel ramps from 0 to -a_max, holds, ramps back to 0). Within each phase,
position can be expressed as a closed-form function of velocity — eliminating
the time variable entirely.

**Phase A** (jerk = -j, accel goes 0 → -a_peak, velocity starts at V):
Starting conditions: v=V, a=0. So `v(t) = V - 0.5*j*t²`, giving `t = sqrt(2*(V-v)/j)`.
Substituting into position `s(t) = V*t - (j/6)*t³`:

```
s_A(v) = (2V + v) / 3 * sqrt(2*(V - v) / j)
```

This gives the distance from the start of the ramp to the point where velocity = v.

**Phase B** (jerk = 0, constant accel = -a_max):
Starting conditions: v=v_endA, a=-a_max (or -a_peak if triangular, but then phase B has
zero duration). Standard quadratic: `delta_s = v_B*t + 0.5*(-a_max)*t²`, with
`v(t) = v_B - a_max*t` → `t = (v_B - v)/a_max` → plug into delta_s for closed-form s(v).

```
s_B(v) = s_endA + (v_B² - v²) / (2 * a_max)
```

**Phase C** (jerk = +j, accel goes -a_peak → 0, velocity ends at v_exit):
By time-reversal symmetry from v_exit's perspective, measuring distance from the END:

```
s_from_end(v) = (2*v_exit + v) / 3 * sqrt(2*(v - v_exit) / j)
```

To get s_C(v) from the START: `s_C(v) = total_ramp_dist - s_from_end(v)`.

**Usage in maxDecelEntry**: for each junction with ceiling_k, compute s(ceiling_k) in
the appropriate phase. If s(ceiling_k) > d_k (junction distance from start), the speed
at the junction exceeds ceiling_k → must cap V ≤ ceiling_k. All O(1).

**Determining which phase**: compute phase durations via `cj_planRamp(v_exit, V, j, a_max, true, ta, tb, tc)`
once, simulate the two phase boundaries (end of A, end of B) to get cumulative distances
s_endA and s_endB. Then compare junction distance against these.

**Important**: `cj_planRamp` assumes a=0 at BOTH endpoints, which is exactly the case for
the full decel ramp (a=0 at V, a=0 at v_exit). Internal junctions do NOT have a=0 —
that's the whole point: we're checking velocities inside the ramp where a≠0.

## Changes

### File 1: `trajectory_constant_jerk.h`

(Symlink: `src/c/marlin/trajectory_constant_jerk.h` → `Marlin/.../ft_motion/trajectory_constant_jerk.h`)

#### 1a. Add `getVelocityAtDistance(float d)` to `ConstantJerkTrajectoryGenerator`

After `getJerkAtTime()` (line ~230), add a new public method. This is used by the
merge loop (step 4) to check left interior junctions.

After `plan_full()`, the trajectory object already has cached phase data via
`buildPhaseCache()`: `phase_start_pos[]`, `phase_start_v[]`, `phase_start_a[]`, `phase_dt[]`.

```cpp
float getVelocityAtDistance(const float d) const;
```

Implementation:
1. `findPhaseByDist(d)` — linear scan comparing d against `phase_start_pos[]`. O(1) (7 entries).
   Use `(ph < 6) ? phase_start_pos[ph+1] : distance` for phase end position.
2. Compute `delta_s = d - phase_start_pos[ph]`
3. For j=0 phases (1, 3, 5):
   - Phase 3 (cruise, a=0): velocity = phase_start_v[ph] (constant). Trivial.
   - Phases 1, 5 (constant accel): quadratic `delta_s = v0*t + 0.5*a0*t²`
     → `t = (-v0 + sqrt(v0² + 2*a0*delta_s)) / a0`
     → velocity = `v0 + a0*t`
4. For j≠0 phases (0, 2, 4, 6): Newton's method (~3-4 iterations) on
   `f(t) = v0*t + 0.5*a0*t² + (j_k/6)*t³ - delta_s = 0`
   - `f'(t) = v0 + a0*t + 0.5*j_k*t²` (= velocity, always positive → monotone)
   - Initial guess: `t = phase_dt[ph] * (delta_s / phase_dist)` where
     `phase_dist = phase_start_pos[ph+1] - phase_start_pos[ph]`
   - Clamp t to `[0, phase_dt[ph]]` each iteration
   - Return `v0 + a0*t + 0.5*j_k*t²`

Cost: ~70-80 cycles on STM32H7 worst case (jerk phase with Newton). Zero for cruise phase.

#### 1b. Change `plan_full` from `void` to `bool`

Remove the recursive `jerk * 1.1` inflation. When `minimum_distance > distance`
(the ramp between v0 and v1 doesn't fit), return `false` instead. The caller
(merge loop) handles infeasibility by adjusting the split or falling back.

The `SERIAL_ECHOLNPGM("CJ ERROR: infeasible target:...")` diagnostic can stay
(useful for debugging) but the recursive call is removed.

#### 1c. Add decel-ramp `s(v)` helper (free function or inline)

For the right-side check in `maxDecelEntry` (step 2). Given a 3-phase decel ramp
from V to v_exit with known phase boundaries (s_endA, s_endB, v_endA, v_endB),
compute the distance from ramp start at which velocity = v_target.

```cpp
// Returns distance from start of decel ramp at which velocity = v_target.
// Requires: v_exit ≤ v_target ≤ V
// Phase boundaries (s_endA, v_endA, s_endB, v_endB) precomputed.
static float cj_decelRampDistAtVelocity(
    float V, float v_exit, float v_target,
    float j, float a_max,
    float s_endA, float v_endA, float s_endB, float v_endB,
    float total_ramp_dist);
```

Implementation uses the O(1) formulas from each phase:
- If `v_target ≥ v_endA`: in phase A → `s = (2V + v_target)/3 * sqrt(2*(V - v_target)/j)`
- If `v_target ≥ v_endB`: in phase B → `s = s_endA + (v_endA² - v_target²) / (2*a_max)`
- Else: in phase C → `s = total_ramp_dist - (2*v_exit + v_target)/3 * sqrt(2*(v_target - v_exit)/j)`

### File 2: `constant_jerk_planner.h`

(Symlink: `src/c/marlin/constant_jerk_planner.h` → `Marlin/.../ft_motion/constant_jerk_planner.h`)

#### 2a. Add `maxDecelEntry()` private method

Computes the max entry speed for a pure decel ramp over blocks [from..to),
decelerating to v_exit, respecting all interior junction ceilings.

```
float maxDecelEntry(mm_arr, vjunction, from, to, v_exit, a_max, j_max):
    total_dist = sumDist(mm, from, to)
    V = maxReachableSpeed(v_exit, total_dist, ∞, a_max, j)  // max possible entry
    V = min(V, vmax_junction[from])                           // cap by entry junction

    // Compute 3-phase decel ramp from V to v_exit
    cj_planRamp(v_exit, V, j, a_max, true, ta, tb, tc)
    // Simulate phase boundaries to get s_endA, v_endA, s_endB, v_endB

    for each junction k in [from+1..to):
        if ceiling_k >= V: skip (junction can't be violated)
        if ceiling_k <= v_exit: must cap V (even at ramp end, speed > ceiling)
            Actually this shouldn't happen — backward pass ensures max_safe_entry
            respects junctions. But guard anyway.
        d_k = cumulative distance from 'from' to k
        s_at_ceiling = cj_decelRampDistAtVelocity(V, v_exit, ceiling_k, ...)  // O(1)
        if s_at_ceiling > d_k:
            // haven't slowed to ceiling_k by junction → cap V
            V = min(V, ceiling_k)
            // Recompute ramp phases (V changed)
            cj_planRamp(v_exit, V, j, a_max, true, ta, tb, tc)
            // Re-simulate boundaries

    return V
```

When V is capped, the ramp must be recomputed. But V only decreases monotonically,
and most junctions won't trigger a cap, so recomputation is rare in practice.

#### 2b. Replace the merge `while(true)` loop (lines 325-419)

```
New flow:
  while(true) {
    // Step 2: Compute max_right_entry via decel analysis
    // Use min(accel) across ALL right blocks (conservative, no compatibility needed)
    float right_a = minVal(accel, left_end, block_count);
    float max_right_entry_v = maxDecelEntry(mm, vmax_junction,
                                             left_end, block_count,
                                             max_safe_entry[block_count],  // 0 if end of buffer
                                             right_a, jerk_max);

    // Step 3: Plan left (compute on demand, no cumulative arrays)
    float left_mm = sumDist(mm, 0, left_end);
    float left_a = minVal(accel, 0, left_end);
    float left_nominal = nominal[0];
    // v_cap may be < left_entry_speed (e.g. max_right_entry is low).
    // In that case maxReachableSpeed returns v_cap (it's a ceiling).
    float v_cap = _MIN(left_nominal, vmax_junction[left_end], max_right_entry_v);
    float target_exit = maxReachableSpeed(left_entry_speed, left_mm, v_cap, left_a, jerk_max);

    // Step 4: Plan and check feasibility + left interior junctions
    // plan_full returns false if infeasible (no jerk inflation — removed)
    bool feasible = traj.plan_full(left_entry_speed, target_exit, left_a, jerk_max, left_mm, left_nominal);
    if (!feasible) {
      // TODO: Left can't decelerate to target_exit (v_cap < left_entry_speed
      // and distance too short). Fix involves remembering previous cycle's
      // right block count. For now, fall back to min_left_safe_exit.
      target_exit = min_left_safe_exit;
      traj.plan_full(left_entry_speed, target_exit, left_a, jerk_max, left_mm, left_nominal);
      break;
    }

    if (left_end > 1) {
      uint8_t rightmost_violation = 0;
      for (uint8_t k = left_end - 1; k >= 1; k--) {  // right to left, stop on first
        float d_k = sumDist(mm, 0, k);  // distance to junction k from left start
        float v_at_k = traj.getVelocityAtDistance(d_k);
        if (v_at_k > vmax_junction[k] + 0.1f) {  // tolerance
          rightmost_violation = k;
          break;
        }
      }
      if (rightmost_violation > 0) {
        left_end = rightmost_violation;  // move split to rightmost violated junction
        continue;
      }
    }

    // Success — traj already has the feasible plan with all phases
    left_exit_speed = target_exit;
    min_left_size = block_count - left_end;  // right side block count
    min_left_safe_exit = max_safe_entry[left_end];  // from backward pass
    break;
  }
```

**Note on `left_end` minimum**: The junction violation scan iterates k from `left_end-1`
down to 1, so `left_end` can shrink to minimum 1 (single block). It cannot reach 0.
If `left_end == 1` and `plan_full` is infeasible, that's the TODO fallback case.

**Note on loop termination**: Each iteration either breaks or strictly decreases `left_end`.
Since `left_end ≥ 1` and violations set `left_end = k ≥ 1`, and k < previous left_end,
the loop terminates in at most `initial_left_end` iterations.

#### 2c. Cross-cycle guarantees and backward pass

The backward pass (lines 263-269) is no longer needed for the main merge path — the
right side now computes its own max entry via pure decel.

**Keep the backward pass**, it's used for:
- `max_safe_entry[block_count]` = v_exit target for the right decel ramp (0 at buffer end,
  non-zero when blocks exist beyond the visible window — this is the "future optimization"
  that's actually already built in: the decel ramp targets max_safe_entry, not always 0)
- The infeasible-left fallback (TODO case): `min_left_safe_exit` from backward pass
- **Future**: when the left can't slow down, use decel over previous right block count
  targeting that non-zero exit speed instead of standstill

`min_left_size` and `min_left_safe_exit` work the same way:
- `min_left_size` = number of right blocks (prevents splitting left below previous right size)
- `min_left_safe_exit` = `max_safe_entry[left_end]` (conservative fallback from backward pass)

#### 2d. Remove right-compatible group scan and cumulative arrays

Remove lines 306-318 (current right-compatible group finding). The right side is now
ALL remaining blocks `[left_end..block_count)`. No same-nominal or accel-ratio
compatibility needed — the right side is modeled as pure decel with `min(accel)`.

Remove cumulative arrays `cum_mm[]`, `cum_min_a[]`, `cum_vmax_junction[]` (lines 280-303).
These were an optimization for the old algorithm that iterated many times with binary splits.
With the new algorithm, `left_end` only decreases (few iterations), so computing
`sumDist(mm, 0, left_end)` and `minVal(accel, 0, left_end)` on demand is cheap and simpler.

Left compatibility scan is kept but simplified: just find `left_end` (max compatible extent)
by scanning blocks with same `nominal[0]` and accel ratio ≤ 1.1. No cumulative arrays needed.

### File 3: `test_trajectory.cpp`

(`src/c/test_trajectory.cpp`)

#### 3a. Add tests for `getVelocityAtDistance()`

- Test velocity at known positions (d=0 → v0, d=distance → v1)
- Test midpoint of cruise phase (velocity = v_peak)
- Cross-check: sample many time values, for each get distance via `getDistanceAtTime(t)`,
  then verify `getVelocityAtDistance(d) ≈ getVelocityAtTime(t)` within tolerance
- Test all 7 phase types: ensure correct results in jerk phases (0,2,4,6),
  constant-accel phases (1,5), and cruise phase (3)
- Edge cases: d=0, d=distance, d just past phase boundaries

#### 3b. Existing streaming tests should still pass

All 78 existing tests should continue to pass. The new merge algorithm should produce
equal or better results (larger superblocks, same or fewer groups).

Some test assertions may need updating if group counts change (e.g. `check("merged_count<6")`
might need adjustment if the new algorithm produces fewer groups).

#### 3c. Add targeted tests for new merge behavior

- Test that a low junction in the middle of a merge-compatible group causes a split
  at that exact junction (not binary halving)
- Test that the right side pure-decel model correctly limits entry speed when a right
  junction has a low ceiling
- Test infeasible left (can't decel to meet right) falls back gracefully
- Test with `max_safe_entry[block_count] > 0` (simulating blocks beyond visible window)

## Implementation Order

1. **`plan_full` → return bool** (1b): change signature, remove jerk inflation
2. **Add `getVelocityAtDistance()`** (1a): to trajectory generator + unit tests
3. **Add decel ramp `s(v)` helper** (1c): O(1) closed-form function
4. **Add `maxDecelEntry()`** (2a): helper using the s(v) helper
5. **Replace merge loop** (2b): new algorithm, remove right-group scan (2d)
6. **Run all tests** — fix any regressions
7. **Build WASM** — verify web simulator works

## Non-Obvious Implementation Details

### `uint8_t k` underflow in right-to-left scan
The junction violation loop `for (uint8_t k = left_end - 1; k >= 1; k--)` is safe
because we break on first violation and k≥1 is the loop condition. But if left_end is
1, the loop body never executes (left_end - 1 = 0, which is < 1). This is correct:
single-block left has no interior junctions.

### `maxDecelEntry` with ceiling_k < v_exit
If a junction ceiling is below v_exit, the decel ramp can never satisfy it (even at the
end of the ramp, speed = v_exit > ceiling_k). This shouldn't normally happen because the
backward pass ensures max_safe_entry respects junctions. But if it does, we must cap V
to ceiling_k. This means the ramp from V to v_exit is shorter, and we cruise at V=ceiling_k
for the remaining distance. Since ceiling_k < v_exit, the cruise speed is below the exit
target — the model breaks down. Handle by capping V and accepting the conservative result.

### `cj_planRamp` is called with decel=true
When computing the right decel ramp, we call `cj_planRamp(v_exit, V, j, a_max, true, ...)`.
The `decel=true` flag makes it simulate from V downward, which gives the correct distances.
The returned phase durations ta/tb/tc are for [-j, 0, +j] phases respectively.

### Phase A `s(v)` formula derivation
Starting from v=V, a=0, jerk=-j:
- `a(t) = -j*t`
- `v(t) = V - 0.5*j*t²` → `t = sqrt(2*(V-v)/j)`
- `s(t) = V*t - (j/6)*t³`

Let `u = sqrt(2*(V-v)/j)`, then `t = u` and `t³ = u³ = (2*(V-v)/j)^(3/2)`:
```
s = V*u - (j/6) * (2*(V-v)/j)^(3/2)
  = V*u - (j/6) * (2/j)^(3/2) * (V-v)^(3/2)
  = V*u - (1/6) * sqrt(8/j) * (V-v)^(3/2)
```

But more simply, using `u = sqrt(2*(V-v)/j)`:
```
s = V*u - (j/6)*u³
  = u * (V - j*u²/6)
  = u * (V - (V-v)/3)
  = u * (2V+v)/3
  = (2V+v)/3 * sqrt(2*(V-v)/j)
```

### The s(v) trick works because jerk is constant within a phase
Within any single constant-jerk phase, `v(t)` is quadratic in t, so t can be expressed
as a function of v (via square root). Then s(t) (cubic in t) becomes s(v) — a closed
form involving `sqrt(dv)` and polynomial terms. This works for ANY constant-jerk phase,
not just the decel ramp. The `getVelocityAtDistance()` for the 7-phase left trajectory
could potentially use the same trick (computing s(v) then comparing), but it's simpler
to use Newton on s(t) there since we need v given s (the inverse direction).

### The right side uses min(accel) across ALL remaining blocks
Unlike the current algorithm which finds a "right-compatible group" with same nominal
and accel ratio ≤ 1.1, the new algorithm uses ALL blocks [left_end..block_count).
This is valid because:
- The right side is a pure decel model, not an actual planned trajectory
- Using min(accel) is conservative: the actual per-block capability is ≥ this
- No nominal speed compatibility needed: we're just decelerating, not cruising at nominal

### vmax_junction[left_end] is the ceiling at the split point
`vmax_junction[i]` is the max entry speed for block i. The junction between left and
right superblocks is at block index `left_end` (first block of the right side). So the
ceiling there is `vmax_junction[left_end]`. This caps both `max_right_entry` and the
left's exit speed.

### When left_end == block_count (no right side)
All blocks are in the left group. target_exit should be max_safe_entry[block_count] (= 0
at buffer end). The right decel computation is skipped (no right blocks). This is the
"plan to standstill" case.

## Edge Cases to Handle

- `left_end == 1` (single block left): no interior junctions to check, just plan directly
- `left_end == block_count` (no right side): exit speed = max_safe_entry[block_count], plan directly
- `block_count == 1`: single block, plan entry → max_safe_entry[1]
- Very short blocks where decel ramp is infeasible
- `maxDecelEntry` returns v_exit (can't go faster than exit) — left targets v_exit
- Left infeasible (can't decel to target_exit) — TODO: use previous cycle's right block count

## What This Plan Changes

- `plan_full()` — returns bool, jerk inflation removed (see 1b)
- Merge loop in `planNext()` — completely rewritten (see 2b)
- Right-compatible group scan — removed (see 2d)
- Cumulative arrays (`cum_mm`, `cum_min_a`, `cum_vmax_junction`) — removed, compute on demand (see 2d)

## What This Plan Does NOT Change

- The backward pass — kept for max_safe_entry (see 2c)
- `maxReachableSpeed()` / `minReachableSpeed()` — unchanged
- `peakExceedsCeiling()` — no longer used in merge loop, but kept (may be useful elsewhere)
- Left compatibility scan — kept (same nominal + accel ratio requirement)
- The streaming execution model (`checkBlockBoundary`, `advanceBlock`, etc.)
- `cj_rampDist`, `cj_rampDistDeriv`, `cj_planRamp`, `cj_simulatePhase` — unchanged
