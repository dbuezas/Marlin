# Plan: New Merge Algorithm — "Split-in-Middle, Right-Decel-First"

## Goal

Replace the current binary-split merge algorithm in `constant_jerk_planner.h` with a smarter approach that:
- Produces larger superblocks (better motion quality)
- Uses informed split points (rightmost violated junction) instead of blind binary halving
- Computes the right side as a pure decel ramp to standstill, which is simpler and gives the max feasible entry speed

## Algorithm Overview

```
1. split = buffer_size / 2  (or max compatible left extent)
2. Compute max_right_entry:
   - Model right side as pure 3-phase decel [-j, 0, +j] from V to max_safe_entry[right_boundary]
     (right_boundary = block_count; max_safe_entry[block_count] = 0 when no more blocks visible)
   - For each right junction, check that decel speed at that distance ≤ junction ceiling
   - Reduce V if any junction is violated
   - Cap by vmax_junction[split]
3. Plan left as full 7-phase S-curve: plan_full(v_left_entry, target_exit, ...)
   - target_exit = min(maxReachableSpeed(...), max_right_entry)
   - Note: if max_right_entry < left_entry_speed, maxReachableSpeed returns max_right_entry
     (v_cap acts as ceiling). plan_full may then be infeasible (can't decel enough).
   - plan_full returns bool (feasible or not). No jerk inflation — removed entirely.
   - If infeasible: leave TODO comment (fix involves remembering previous cycle's right block count)
4. Check left interior junctions using getVelocityAtDistance() on the trajectory object
   - Scan right to left, stop on first violation (= rightmost violated junction)
   - If violated: move split to that junction, goto 2
5. Emit left superblock with exit = actual_exit
   The right side is discarded — it only existed to determine a safe exit speed
```

## Key Insight: O(1) Velocity-at-Distance in the Decel Ramp

The 3-phase decel ramp from V to v_exit is a trapezoidal acceleration profile (accel ramps from 0 to -a_max, holds, ramps back to 0). Within each phase, position can be expressed as a closed-form function of velocity (no time variable needed).

**Phase A** (jerk = -j, accel goes 0 → -a_peak, velocity starts at V):
`v(t) = V - 0.5*j*t²`, so `t = sqrt(2*(V-v)/j)`. Substituting into the position equation:

```
s(v) = (2V + v) / 3 * sqrt(2*(V - v) / j)
```

**Phase B** (jerk = 0, constant accel = -a_max): quadratic in time, trivially invertible.

**Phase C** (jerk = +j, accel goes -a_peak → 0, velocity ends at v_exit):
Similar closed-form. By time-reversal symmetry from v_exit's perspective:
`s_from_end(v) = (2*v_exit + v) / 3 * sqrt(2*(v - v_exit) / j)`
(distance from the end of the ramp to where velocity = v, measured backwards)

This means checking whether the velocity at a junction's distance exceeds its ceiling is **O(1) per junction** — no Newton, no cubic root-finding. We compute `s(ceiling_k)` in the appropriate phase and compare against the junction distance.

To determine which phase a junction falls in: compute phase durations via `cj_planRamp` once, simulate the two phase boundaries (end of A, end of B) to get cumulative distances, then compare junction distance against these two boundaries.

## Changes

### File 1: `src/c/marlin/trajectory_constant_jerk.h`

#### 1a. Add `getVelocityAtDistance(float d)` method to `ConstantJerkTrajectoryGenerator`

After `getJerkAtTime()` (line ~230), add a new public method.

Within the full 7-phase trajectory, the same `s(v)` closed-form trick applies per-phase (each phase has constant jerk). For j=0 phases, it's a simple quadratic. For j≠0 phases, use the substitution `t = f(v)` into `s(t)` to get `s(v)` directly.

However, to find velocity at a given distance, we need the inverse: given s, find v. The `s(v)` expressions involve `sqrt(V-v)` terms, so inverting them is non-trivial. **Use Newton's method on `s(v) - target = 0`** with ~3-4 iterations per junction. The derivative `ds/dv` is also closed-form.

Alternative: for the right-side decel ramp specifically, we don't need the inverse — we compute `s(ceiling_k)` and compare against `d_k`. Only the left-side junction check (step 4) needs the inverse.

**For the left side (step 4)**: After `plan_full()`, the trajectory object already has `phase_start_pos[]`, `phase_start_v[]`, `phase_start_a[]`, `phase_dt[]` via `buildPhaseCache()`. Add:

```cpp
float getVelocityAtDistance(const float d) const;
```

Implementation:
1. `findPhaseByDist(d)` — linear scan of `phase_start_pos[]`, O(1)
2. For j=0 phases: quadratic formula on `delta_s = v0*t + 0.5*a0*t²`
3. For j≠0 phases: Newton's method (~3-4 iterations) on the cubic `delta_s = v0*t + 0.5*a0*t² + (j/6)*t³`, with distance-fraction initial guess `t = phase_dt[ph] * (delta_s / phase_dist)`

Need helper `findPhaseByDist(float d)`. Use `(ph < 6) ? phase_start_pos[ph+1] : distance` for phase end position.

#### 1b. Add `distAtVelocityInDecelRamp()` free function (or inline in planner)

For the right-side decel ramp check (step 2), we need `s(v)` in each phase — the forward direction. This is O(1) per junction:

```
Phase A: s(v) = (2V + v) / 3 * sqrt(2*(V - v) / j)
Phase B: quadratic
Phase C: similar closed-form
```

Compute once: `cj_planRamp(0, V, j, a_max, true, ta, tb, tc)` + simulate two phase boundaries. Then for each junction, compute `s(ceiling_k)` in the appropriate phase and compare.

### File 2: `src/c/marlin/constant_jerk_planner.h`

#### 2a. Add `maxDecelEntry()` helper

New private method that computes the max entry speed for a pure decel-to-standstill ramp over a set of blocks, respecting junction ceilings.

```
float maxDecelEntry(mm_arr, vjunction, from, to, v_exit, a_max, j_max):
    // v_exit = max_safe_entry[to] (0 when no blocks past 'to')
    total_dist = sumDist(mm, from, to)
    V = maxReachableSpeed(v_exit, total_dist, ∞, a_max, j)  // max possible entry
    V = min(V, vmax_junction[from])                           // cap by entry junction

    // Compute 3-phase decel ramp from V to v_exit
    cj_planRamp(v_exit, V, j, a_max, true, ta, tb, tc)
    // Simulate phase boundaries to get cumulative positions (s_endA, s_endB)

    for each junction k in [from+1..to):
        d_k = cumulative distance from 'from' to k
        // Compute s(ceiling_k) in appropriate phase — O(1)
        // If s(ceiling_k) > d_k: junction not yet at ceiling → speed > ceiling
        //   → cap V = min(V, ceiling_k), recompute ramp phases
        // If s(ceiling_k) ≤ d_k: junction satisfied → skip

    return V
```

When V is capped, we need to recompute the ramp phases. But V only decreases, so we can iterate left-to-right and recompute only when V changes. In practice, most junctions won't cap V (they're deep in the decel ramp), so recomputation is rare.

#### 2b. Replace the merge `while(true)` loop (lines 325-419)

Replace the current merge loop with the new algorithm:

```
Current flow (lines 325-419):
  while(true) {
    compute right superblock params
    compute max_left_exit, max_right_entry
    v_junction_candidate = min(max_left_exit, max_right_entry)
    validate right interior junctions (peakExceedsCeiling) → split right
    validate left interior junctions (peakExceedsCeiling) → split left or fallback
    break
  }

New flow:
  while(true) {
    // Step 2: Compute max_right_entry via decel analysis
    float right_a = minVal(accel, left_end, block_count);  // or use per-block min
    float max_right_entry_v = maxDecelEntry(mm, vmax_junction,
                                             left_end, block_count,
                                             max_safe_entry[block_count],  // 0 if no more blocks
                                             right_a, jerk_max);

    // Step 3: Plan left
    float left_mm = cum_mm[left_end - 1];
    float left_a = cum_min_a[left_end - 1];
    float left_nominal = nominal[0];
    // v_cap may be < left_entry_speed (e.g. max_right_entry is low).
    // In that case maxReachableSpeed returns v_cap (it's a ceiling).
    float v_cap = min(left_nominal, vmax_junction[left_end], max_right_entry_v);
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
        float d_k = cum_mm[k - 1];  // distance to junction k from left start
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

    // Success
    left_exit_speed = actual_exit;
    min_left_size = block_count - left_end;  // right side block count
    min_left_safe_exit = max_safe_entry[left_end];  // from backward pass
    break;
  }
```

#### 2c. Cross-cycle guarantees and backward pass

The backward pass (lines 263-269) is no longer needed for the main merge path — the
right side now computes its own max entry via pure decel to standstill.

**Keep the backward pass for now**, but it's only used for:
- The infeasible-left fallback (TODO case): `min_left_safe_exit` from backward pass
- **Future optimization**: right side decel targets `max_safe_entry[right_boundary]`
  instead of 0. This allows higher `max_right_entry` when incompatible blocks after
  the right side can still handle the speed. For example, when the left can't slow
  down enough, we use a decel ramp over the previous right block count but target
  that non-zero exit speed instead of standstill — much more achievable.

`min_left_size` and `min_left_safe_exit` work the same way:
- `min_left_size` = number of right blocks (prevents splitting left below previous right size)
- `min_left_safe_exit` = `max_safe_entry[left_end]` (conservative fallback from backward pass)

#### 2d. Remove right-compatible group scan

Remove lines 306-318. The right side is now ALL remaining blocks `[left_end..block_count)`. No need for same-nominal or accel-ratio compatibility — the right side is modeled as pure decel with `min(accel)` across all right blocks.

Left compatibility scan (lines 286-303) is kept unchanged.

#### 2e. plan_full: remove jerk inflation, return bool

Change `plan_full` from `void` to `bool`. Remove the recursive `jerk * 1.1` inflation entirely. When the ramp between v0 and v1 doesn't fit in the distance, return `false` instead. The merge loop uses this to detect infeasible plans and adjust. Once the loop exits, `traj` already contains a feasible plan with all phases computed — no need to call `plan_full` again.

### File 3: `src/c/test_trajectory.cpp`

#### 3a. Add tests for `getVelocityAtDistance()`

- Test velocity at known positions (start, end, midpoint of cruise phase)
- Test against `getVelocityAtTime(t)` by sampling time, getting distance, then checking velocity
- Test edge cases: d=0, d=distance, d in each of the 7 phases

#### 3b. Existing streaming tests should still pass

All 78 existing tests should continue to pass. The new merge algorithm should produce equal or better results (larger superblocks, same or fewer groups).

#### 3c. Add targeted tests for the new merge behavior

- Test that a low junction in the middle of a merge-compatible group causes a split at that junction (not binary halving)
- Test that the right side pure-decel model correctly limits entry speed
- Test infeasible left (can't decel to meet right) falls back gracefully

## Implementation Order

1. **Add `getVelocityAtDistance()`** to trajectory generator + unit tests
2. **Add decel ramp `s(v)` helpers** for right-side O(1) junction checks
3. **Add `maxDecelEntry()`** helper to planner
4. **Replace merge loop** with new algorithm
5. **Run all tests** — fix any regressions
6. **Build WASM** — verify web simulator works

## Edge Cases to Handle

- `left_end == 1` (single block left): no interior junctions to check, just plan directly
- `left_end == block_count` (no right side): exit speed = 0, plan to standstill
- `block_count == 1`: single block, plan entry→0
- Very short blocks where decel ramp is infeasible
- `maxDecelEntry` returns v_exit (can't go faster than exit) — left targets v_exit
- Left infeasible (can't decel to actual_exit) — TODO: use previous cycle's right block count

## What This Plan Does NOT Change

- The backward pass — kept but only for fallback/future optimization (see 2c)
- `plan_full()` — changed: returns bool, jerk inflation removed (see 2e)
- `maxReachableSpeed()` / `minReachableSpeed()` — unchanged
- `peakExceedsCeiling()` — no longer used in merge loop, but kept (may be useful elsewhere)
- Left compatibility scan — kept (same nominal + accel ratio requirement)
- The streaming execution model (`checkBlockBoundary`, `advanceBlock`, etc.)
