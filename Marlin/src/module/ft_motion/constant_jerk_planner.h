/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../planner.h"
#include "trajectory_constant_jerk.h"

#define CJP_MERGE_AMAX_RATIO 1.1f

/**
 * Constant-jerk block planner
 *
 * The standard trapezoidal planner assumes instant acceleration changes.
 * Under jerk constraints, S-curves need more distance for the same speed
 * change, so a different planner is needed.
 *
 * This planner:
 * - Ignores Marlin's trapezoidal entry/exit speeds entirely
 * - Uses block->vmax_junction as the junction speed ceiling (geometric limit,
 *   never overwritten by the trapezoidal recalculate pass)
 * - Runs its own jerk-aware backward pass on all visible blocks enforcing
 *   zero acceleration at junctions
 * - Merges compatible consecutive blocks into a single "left" S-curve,
 *   using the remaining blocks as a pure decel model for the right side
 *
 * ─── Notation ───
 *
 *   Blocks are indexed 0..N-1. Each block i has:
 *     mm[i]             distance (mm)
 *     accel[i]          max acceleration (mm/s²)
 *     nominal[i]        max cruise speed (mm/s)
 *     vmax_junction[i]  geometric junction speed ceiling at entry of block i
 *
 *   A "superblock" [a..b) treats blocks a..b-1 as one merged segment:
 *     distance  = sum(mm[a..b))
 *     accel     = min(accel[a..b))        (conservative)
 *     nominal   = nominal[a]              (all blocks in group share this)
 *
 *   Merge compatibility: max(accel)/min(accel) ≤ CJP_MERGE_AMAX_RATIO (1.1).
 *   Using min(accel) is conservative: the superblock is slower than a
 *   per-block pass would allow, so superblock-feasible ⇒ per-block
 *   feasible (ignoring zero accel at junction constraint).
 *
 * ─── Key functions ───
 *
 *   maxReachableSpeed(v_from, dist_total, v_max, a_max, j_max) → float
 *     Max speed reachable from v_from over dist_total under jerk/accel limits,
 *     capped by v_max. Newton's method on closed-form cj_rampDist (~4 iters).
 *     When v_from > v_max, returns v_max (v_max is a hard ceiling).
 *
 *   minReachableSpeed(v_from, dist_total, a_max, j_max) → float
 *     Min speed after decelerating from v_from over dist_total.
 *     Returns 0 if full stop fits; otherwise binary search on cj_rampDist.
 *     Returns hi (slight overestimate): actual min speed ∈ [hi - 0.001, hi].
 *
 *   maxDecelEntry(mm, v_junction, from, to, v_exit, a_max, j_max) → float
 *     Max entry speed for a pure decel ramp over blocks [from..to),
 *     decelerating to v_exit while respecting all interior junction
 *     ceilings. Uses cj_decelRampDistAtVelocity() (O(1)) to check where
 *     the decel ramp reaches each ceiling and lowers v_entry when violated.
 *
 *   cj_rampDist(v_start, v_peak, j_max, a_max) → float
 *     Closed-form distance for a 3-phase jerk-limited ramp.
 *     Monotone in v_peak (for fixed v_start): higher v_peak → more distance.
 *     Symmetric: accel and decel ramps consume equal distance.
 *     This symmetry means maxReachableSpeed(v_a, dist) ≥ v_b implies
 *     decel from v_b to v_a also fits in dist.
 *
 *   cj_rampDistDeriv(v_start, v_peak, j_max, a_max) → float
 *     Derivative ds/d(v_peak) of cj_rampDist, used by Newton's method.
 *
 *   cj_decelRampDistAtVelocity(v_entry, v_exit, v_target, j_max, a_max, ...) → float
 *     O(1) distance from start of a 3-phase decel ramp (v_entry → v_exit) to where
 *     velocity first drops to v_target. Used by maxDecelEntry to check
 *     junction ceilings without simulating the full ramp.
 *
 *   getVelocityAtDistance(d) (on ConstantJerkTrajectoryGenerator)
 *     Returns velocity at distance d within the planned 7-phase trajectory.
 *     Hybrid Newton/bisection on the cubic position polynomial (~4-6 iters
 *     typical, 16 max). Used to check interior junction ceilings in left.
 *
 * ─── plan_full feasibility ───
 *
 *   plan_full(v_entry, v_exit, a_max, j_max, dist_total, v_nominal) → bool
 *     Returns false if the ramp between boundary speeds doesn't fit:
 *       cj_planRamp(min(v_entry,v_exit), max(v_entry,v_exit), j_max, a_max) > dist_total
 *     The planner handles infeasibility by either splitting the left group
 *     or, as a last resort, inflating jerk by 10% (up to 5 retries) to
 *     absorb borderline numerical precision issues.
 *
 * ─── Backward pass ───
 *
 *   max_safe_entry[i] = max speed at which block i can be entered such that
 *   the remaining blocks [i..N) can decelerate to a stop by block N.
 *
 *   Computed right-to-left:
 *     max_safe_entry[N] = 0
 *     max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i+1], mm[i],
 *                            min(nominal[i], vmax_junction[i]), accel[i], j_max)
 *
 *   The v_max parameter = min(nominal, vmax_junction) folds the junction
 *   ceiling into the search so the result never exceeds either limit.
 *
 *   Property (backward-monotone):
 *     max_safe_entry over [i..N) ≤ max_safe_entry over [i..N+k)
 *     Adding blocks to the tail only raises safe entry speeds (more room).
 *
 * ─── Merge algorithm (split-in-middle, right-decel-first) ───
 *
 *   The visible buffer is partitioned into:
 *     left = [0..left_end),  right = [left_end..right_end),  tail = [right_end..N)
 *
 *   Left is a merge-compatible superblock (same nominal, accel ratio ≤ 1.1).
 *   Right is modeled as a pure deceleration ramp (not planned as an S-curve —
 *   only used to determine left's exit speed).
 *
 *   When the buffer is not full, no more blocks are coming: left can take all
 *   blocks and right_end = block_count (no tail). When the buffer is full,
 *   left is capped to half and right is capped to half-1 blocks, leaving at
 *   least one tail block. Right decels to max_safe_entry[right_end] (non-zero)
 *   instead of 0 — the tail blocks guarantee that speed is braking-safe.
 *
 *   Algorithm:
 *   1. Find left-compatible extent: scan blocks with same nominal and
 *      accel ratio ≤ CJP_MERGE_AMAX_RATIO, up to half when buffer is full.
 *
 *   2. Compute v_right_entry_max via maxDecelEntry(): the maximum speed
 *      at which the right side [left_end..right_end) can be entered
 *      while decelerating to max_safe_entry[right_end], respecting
 *      all interior junction ceilings along the way.
 *
 *   3. Compute left exit speed:
 *      v_cap = min(v_left_nominal, right_nominal, vmax_junction[left_end],
 *                  v_right_entry_max)
 *      v_target_exit = maxReachableSpeed(v_left_entry, dist_left, v_cap, ...)
 *      If left can't decelerate to v_target_exit (minReachableSpeed > target),
 *      trigger the can't-brake fallback (see Cross-cycle guarantees).
 *
 *   4. Plan the left trajectory via plan_full(). If infeasible (returns
 *      false), inflate jerk by 1.1× up to 5 times as a numerical fallback.
 *
 *   5. Check interior junction ceilings: use getVelocityAtDistance() to
 *      sample velocity at each interior block boundary within left.
 *      Tolerance: max(1.0 mm/s, ceiling × 3%).
 *      If violated, split at the rightmost violation point: set
 *      left_end = rightmost_violation and restart from step 2.
 *
 *   Only the left group is emitted as a trajectory. The right side is
 *   never planned — it just constrains the left's exit speed.
 *
 * ─── Cross-cycle guarantees ───
 *
 *   Each cycle emits left=[0..left_end) and advances the buffer past those
 *   blocks. The remaining blocks [left_end..block_count) stay in the buffer
 *   and become (at minimum) the start of the next cycle's visible window.
 *   This is the key insight: this cycle's RIGHT side becomes next cycle's
 *   LEFT side (plus any newly arrived blocks appended to the tail).
 *
 *   Stored between cycles:
 *     v_last_right_exit     = max_safe_entry[block_count] — the speed at the
 *                           far end of the visible buffer (typically 0,
 *                           since the backward pass assumes full stop).
 *     dist_last_right_decel = distance the right side needs to decel from
 *                           the emitted target_exit down to v_last_right_exit.
 *
 *   These are stored because the right side's blocks will form the next
 *   cycle's left. If the next cycle can't brake to its new target_exit,
 *   it can instead decel to v_last_right_exit — a speed we know the tail
 *   can handle because backward-monotone guarantees that adding blocks
 *   to the tail only raises (or preserves) safe entry speeds.
 *
 *   Can't-brake fallback (when minReachableSpeed > v_target_exit):
 *   1. If dist_last_right_decel > 0 and v_last_right_exit < v_left_entry:
 *      expand left to cover the decel ramp, decel to v_last_right_exit,
 *      then cruise. If expansion runs out of blocks, fall back to
 *      minReachableSpeed over the expanded left.
 *   2. Otherwise: use minReachableSpeed (best achievable exit speed).
 *   Both paths break immediately (skip junction check).
 *   Safe because backward-monotone: the new tail (with more blocks
 *   appended since last cycle) can handle v_last_right_exit.
 */

/**
 * Does the S-curve peak velocity over dist strictly exceed ceiling?
 * - ceiling ≥ v_nominal → false (peak capped by cruise speed)
 * - ceiling < max(v_entry, v_exit) → true (boundary already exceeds)
 * - otherwise: true iff ramp to ceiling fits in dist (peak goes higher)
 */
static bool peakExceedsCeiling(float v_entry, float v_exit, float a_max,
                                float j_max, float dist_total, float v_nominal,
                                float v_ceiling) {
  if (v_ceiling >= v_nominal) return false;
  const float v_small = _MIN(v_entry, v_exit);
  const float v_large = _MAX(v_entry, v_exit);
  if (v_ceiling < v_large) return true;
  return cj_rampDist(v_small, v_ceiling, j_max, a_max)
       + cj_rampDist(v_large, v_ceiling, j_max, a_max) < dist_total;
}

static float sumDist(const float* mm_arr,  uint8_t from, uint8_t to) {
  float total = 0;
  for (uint8_t i = from; i < to; i++) total += mm_arr[i];
  return total;
}

// Minimum value in arr[from..to-1]
static float minVal(const float* arr, uint8_t from, uint8_t to) {
  float v = arr[from];
  for (uint8_t i = from + 1; i < to; i++) v = _MIN(v, arr[i]);
  return v;
}



class ConstantJerkBlockPlanner {
 public:
  void reset() {
    orig_block_index = 0;
    v_last_right_exit = 0;
    dist_last_right_decel = 0;
    cant_brake_count = 0;
    traj.reset();
  }

  /**
   * Plan trajectory for the current block (already consumed from planner buffer).
   *
   * Looks ahead at future blocks via get_future_block(), runs a jerk-aware
   * reverse/forward pass across all visible blocks, then plans the first
   * block (or merged group) as an S-curve trajectory.
   *
   * Returns true if a trajectory is ready for execution.
   */
  bool planNext(const float j_max) {
    float mm[BLOCK_BUFFER_SIZE];
    float nominal[BLOCK_BUFFER_SIZE];
    float accel[BLOCK_BUFFER_SIZE];
    float vmax_junction[BLOCK_BUFFER_SIZE];  // max entry speed ceiling per block

    uint8_t block_count = 0;

    // Map from move-block index to buffer offset (for tracking consumed range)
    uint8_t buf_offset[BLOCK_BUFFER_SIZE]; // buf_offset[i] = buffer position of move block i

    // Look ahead at future blocks.
    // get_future_block(offset) returns block_buffer[tail + offset].
    // The current block is at tail (offset 0), so offset 1 = next block.
    // Track whether the buffer is full (all BLOCK_BUFFER_SIZE slots occupied).
    // Sync blocks occupy buffer slots but are skipped for planning,
    // so block_count alone can't determine buffer fullness.
    bool buffer_full = true;
    for (uint8_t i = 0; i < BLOCK_BUFFER_SIZE; i++) {
      block_t* blk = planner.get_future_block(i, false);
      if (!blk) { buffer_full = false; break; }
      if (blk->is_sync()) continue; // skip sync blocks in lookahead

      buf_offset[block_count] = i;
      mm[block_count] = blk->millimeters;
      nominal[block_count] = blk->nominal_speed;
      accel[block_count] = blk->acceleration;
      vmax_junction[block_count] = blk->vmax_junction;
      block_count++;
    }
    if (block_count == 0) {
      // should never happen
      SERIAL_ECHOLNPGM("CJ ERROR: block_count == 0");
      traj.reset();
      return false;
    }

    // Backward pass: max_safe_entry[i] = max feasible entry speed for [i..N)
    // to decelerate to standstill by block N. See "Backward pass" in header.
    float max_safe_entry[BLOCK_BUFFER_SIZE + 1];
    max_safe_entry[block_count] = 0.0f;
    for (int8_t i = block_count - 1; i > 0; i--) {
      max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i + 1], mm[i], _MIN(nominal[i], vmax_junction[i]), accel[i], j_max);
    }
    float v_left_entry = traj.getExitSpeed();
    if (v_left_entry == 0) {
      // Starting from rest: no previous cycle to honor.
      // TODO: handle this more cleanly — this happens when the trajectory
      // generator was reset externally (e.g. by ft_motion on idle/error).
      // Ideally the planner would detect the reset via a flag rather than
      // inferring it from exit speed being zero.
      v_last_right_exit = 0;
      dist_last_right_decel = 0;
    }

    // Left-compatible group: find max extent with same nominal and accel ratio ≤ 1.1.
    // When the buffer is not full, no more blocks are coming, so left can take
    // all blocks. When full, cap left to half so the right side retains enough
    // blocks for decel analysis.
    uint8_t left_end = 1;
    const uint8_t half = (block_count + 1) / 2;
    {
      float a_min = accel[0], a_max_l = accel[0];
      const uint8_t max_left_end = buffer_full ? half : block_count;
      for (uint8_t i = 1; i < max_left_end; i++) {
        if (nominal[i] != nominal[0]) break;
        float new_a_min = _MIN(a_min, accel[i]);
        float new_a_max = _MAX(a_max_l, accel[i]);
        if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
        a_min = new_a_min;
        a_max_l = new_a_max;
        left_end++;
      }
    }

    // New merge algorithm: split-in-middle, right-decel-first.
    // Right side = blocks [left_end..right_end), modeled as pure decel.
    // When buffer is full, right is capped to half-1 blocks, leaving at least
    // one tail block beyond right. This way right doesn't have to decel to 0 —
    // it decels to max_safe_entry[right_end], and the tail blocks guarantee
    // that speed is braking-safe.
    float v_left_exit;

    while (true) {
      const uint8_t right_end = buffer_full
        ? _MIN((uint8_t)(left_end + half - 1), (uint8_t)(block_count - 1))
        : block_count;
      const float right_exit_v = max_safe_entry[right_end];

      // Step 2: Compute v_right_entry_max via decel analysis
      float v_right_entry_max = 0;
      if (left_end < right_end) {
        float a_right = minVal(accel, left_end, right_end);
        v_right_entry_max = maxDecelEntry(mm, vmax_junction,
                                           left_end, right_end,
                                           right_exit_v,
                                           a_right, j_max);
      }

      // Step 3: Plan left
      float dist_left = sumDist(mm, 0, left_end);
      float a_left = minVal(accel, 0, left_end);
      float v_left_nominal = nominal[0];
      float v_cap = left_end < right_end
        ? _MIN(v_left_nominal, _MIN(nominal[left_end], _MIN(vmax_junction[left_end], v_right_entry_max)))
        : right_exit_v;  // no right side: exit = safe entry at right_end
      float v_target_exit = maxReachableSpeed(v_left_entry, dist_left, v_cap, a_left, j_max);

      // ─── "Can't brake" fallback ───
      //
      // Left can't decelerate to v_target_exit within dist_left.
      // Strategy: decel to v_last_right_exit instead — a speed we know the
      // tail can handle because the previous cycle's right side (now our
      // left) was proven safe down to that speed, and backward-monotone
      // guarantees the new (longer) tail is at least as permissive.
      //
      // We expand left to cover the decel ramp distance, then cruise at
      // v_last_right_exit for any remaining distance. Both can't-brake paths
      // break immediately (skip junction check) to avoid cascade.
      // If expansion runs out of blocks, fall back to minReachableSpeed.
      if (v_target_exit < v_left_entry) {
        float v_min_exit = minReachableSpeed(v_left_entry, dist_left, a_left, j_max);
        if (v_min_exit > v_target_exit) {
          cant_brake_count++;
          // Try decel-then-cruise to v_last_right_exit if it's a valid
          // decel target (below entry speed).
          if (dist_last_right_decel > 0 && v_last_right_exit < v_left_entry) {
            // Expand left to cover the decel ramp distance, recomputing
            // with actual block accels as we go.
            float dist_cum = 0;
            float a_cur = accel[0];
            left_end = 0;
            for (uint8_t k = 0; k < block_count; k++) {
              dist_cum += mm[k];
              a_cur = _MIN(a_cur, accel[k]);
              left_end = k + 1;
              float dist_needed = cj_rampDist(v_last_right_exit, v_left_entry, j_max, a_cur);
              if (dist_cum >= dist_needed) break;
            }
            dist_left = sumDist(mm, 0, left_end);
            a_left = minVal(accel, 0, left_end);
            // Verify the decel ramp fits in the expanded distance.
            // If the expansion exhausted all blocks without enough distance,
            // fall back to minReachableSpeed over the expanded left.
            float dist_decel_needed = cj_rampDist(v_last_right_exit, v_left_entry, j_max, a_left);
            if (dist_left < dist_decel_needed) {
              v_target_exit = minReachableSpeed(v_left_entry, dist_left, a_left, j_max);
            } else {
              v_target_exit = v_last_right_exit;
            }
            // Nominal must be >= v_left_entry so plan_full's peak
            // can reach entry speed (v_peak >= max(v_entry,v_exit)).
            float j_plan = j_max;
            float v_plan_nominal = _MAX(v_left_entry, v_target_exit);
            bool ok = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_plan_nominal);
            if (!ok) {
              for (int retry = 0; retry < 5 && !ok; retry++) {
                j_plan *= 1.1f;
                ok = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_plan_nominal);
              }
            }
            v_left_exit = v_target_exit;
            storeKnownRight(accel, left_end, block_count, v_target_exit, max_safe_entry, j_max);
            break;
          }
          // No valid decel-then-cruise target. Use best achievable exit speed
          // and break immediately to prevent junction check cascade.
          v_target_exit = v_min_exit;
          float j_plan = j_max;
          bool ok = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_left_nominal);
          if (!ok) {
            for (int retry = 0; retry < 5 && !ok; retry++) {
              j_plan *= 1.1f;
              ok = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_left_nominal);
            }
          }
          v_left_exit = v_target_exit;
          storeKnownRight(accel, left_end, block_count, v_target_exit, max_safe_entry, j_max);
          break;
        }
      }

      // Step 4: Plan and check feasibility + left interior junctions
      // If infeasible due to numerical precision at ramp boundaries, inflate jerk slightly
      float j_plan = j_max;
      bool feasible = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_left_nominal);
      if (!feasible) {
        // Borderline infeasible — inflate jerk to make ramp fit (numerical tolerance)
        for (int retry = 0; retry < 5 && !feasible; retry++) {
          j_plan *= 1.1f;
          feasible = traj.plan_full(v_left_entry, v_target_exit, a_left, j_plan, dist_left, v_left_nominal);
        }
        v_left_exit = v_target_exit;
        storeKnownRight(accel, left_end, block_count, v_target_exit, max_safe_entry, j_max);
        break;
      }

      if (left_end > 1) {
        uint8_t rightmost_violation = 0;
        float dist_cum = 0;
        for (uint8_t k = 0; k + 1 < left_end; k++) dist_cum += mm[k];
        // Scan right to left for junction violations
        for (uint8_t k = left_end - 1; k >= 1; k--) {
          float v_at_junction = traj.getVelocityAtDistance(dist_cum);
          // Tolerance: peakExceedsCeiling in old algorithm was less precise,
          // allowing small overshoots. Use relative tolerance (3%) or 1mm/s min.
          const float tol = _MAX(1.0f, vmax_junction[k] * 0.03f);
          if (v_at_junction > vmax_junction[k] + tol) {
            rightmost_violation = k;
            break;
          }
          dist_cum -= mm[k - 1];
        }
        if (rightmost_violation > 0) {
          left_end = rightmost_violation;
          continue;
        }
      }

      // Success
      v_left_exit = v_target_exit;
      storeKnownRight(accel, left_end, block_count, v_target_exit, max_safe_entry, j_max);
      break;
    }

    // Set up execution tracking
    orig_block_index = 0;
    orig_block_start_dist = 0;
    group_block_count = left_end;
    group_buffer_consumed = buf_offset[left_end - 1] + 1;
    orig_block_end_dist = mm[0];
    return true;
  }

  /**
   * Check if the current distance within the merged trajectory has crossed
   * into the next original block.
   */
  bool checkBlockBoundary(float dist_in_merged) {
    if (orig_block_index >= group_block_count - 1) return false;
    return dist_in_merged >= orig_block_end_dist;
  }

  // Call after confirming a boundary crossing and obtaining the next block
  void advanceBlock(float next_block_mm) {
    orig_block_start_dist = orig_block_end_dist;
    orig_block_index++;
    orig_block_end_dist += next_block_mm;
  }

  /**
   * Get the distance within the current sub-block, given a merged distance.
   */
  float localDistance(float dist_in_merged) const {
    return dist_in_merged - orig_block_start_dist;
  }

  ConstantJerkTrajectoryGenerator& trajectory() { return traj; }
  uint8_t blockCount() const { return group_block_count; }
  uint8_t bufferConsumed() const { return group_buffer_consumed; }
  uint8_t currentBlockIndex() const { return orig_block_index; }
  uint16_t cantBrakeCount() const { return cant_brake_count; }

 private:
  /**
   * Max speed reachable from v_from over dist_total via jerk-limited ramp,
   * capped by v_max. Newton's method on closed-form cj_rampDist.
   * When v_from > v_max, returns v_max (hard ceiling).
   * Monotone in dist_total: more distance → higher or equal result.
   */
  float maxReachableSpeed(float v_from, float dist_total,
                          float v_max, float a_max, float j_max) {
    float v_trap = SQRT(v_from * v_from + 2.0f * a_max * dist_total);
    float hi = _MIN(v_max, v_trap);

    if (hi <= v_from) return hi;
    if (cj_rampDist(v_from, hi, j_max, a_max) <= dist_total) return hi;

    // Initial guess: trapezoidal quadratic inverse
    //   dv² + B*dv + C = 0  where B = 2*v + am²/j, C = 2*v*am²/j - 2*s*am
    const float am2j = a_max * a_max / j_max;
    const float B = 2.0f * v_from + am2j;
    const float C = 2.0f * v_from * am2j - 2.0f * dist_total * a_max;
    const float disc = B * B - 4.0f * C;
    float v_peak;
    if (disc >= 0.0f) {
      const float dv = (-B + SQRT(disc)) * 0.5f;
      v_peak = (dv > 0.0f) ? _MIN(v_from + dv, hi) : hi;
    } else {
      v_peak = hi;
    }

    // Newton: f(v_peak) = cj_rampDist(v_from, v_peak) - dist_total = 0
    for (int i = 0; i < 10; i++) {
      if (v_peak <= v_from) { v_peak = v_from + 0.001f; }
      const float f = cj_rampDist(v_from, v_peak, j_max, a_max) - dist_total;
      const float fp = cj_rampDistDeriv(v_from, v_peak, j_max, a_max);
      if (fp < 1e-10f) break;
      const float step = f / fp;
      v_peak -= step;
      if (v_peak < v_from) v_peak = v_from;
      if (v_peak > hi) v_peak = hi;
      if (f <= 0.0f && f > -0.01f) break; // close enough, conservative
      if (step < 0.001f && step > -0.001f) break;
    }
    // Guarantee: returned speed is conservative (ramp fits in distance).
    // Newton may converge slightly above the root; bisect down if needed.
    v_peak = _MIN(v_peak, hi);
    if (v_peak > v_from && cj_rampDist(v_from, v_peak, j_max, a_max) > dist_total) {
      float v_lo = v_from, v_hi = v_peak;
      for (int i = 0; i < 10; i++) {
        float mid = 0.5f * (v_lo + v_hi);
        if (cj_rampDist(v_from, mid, j_max, a_max) <= dist_total)
          v_lo = mid;
        else
          v_hi = mid;
      }
      v_peak = v_lo;
    }
    return v_peak;
  }

  /** Store right-side state for the next cycle's can't-brake fallback.
   *
   *  This cycle's right blocks [left_end..block_count) will become the
   *  start of the next cycle's visible window. We store:
   *
   *  v_last_right_exit       = max_safe_entry[block_count] — the speed at the
   *    end of the current buffer (typically 0: backward pass assumes stop).
   *  dist_last_right_decel   = distance to decel from this cycle's emitted
   *    exit speed (v_target_exit) down to v_last_right_exit.
   *
   *  If the next cycle can't brake to its computed target, it can instead
   *  decel to v_last_right_exit (a known-safe speed), then cruise. This works
   *  because backward-monotone guarantees: the new tail (old right + new
   *  blocks) can handle any speed the old tail could handle.
   */
  void storeKnownRight(const float* accel_arr,
                       uint8_t left_end, uint8_t block_count,
                       float v_target_exit, const float* max_safe_entry,
                       float j_max) {
    if (left_end < block_count) {
      v_last_right_exit = max_safe_entry[block_count];
      float a_right = minVal(accel_arr, left_end, block_count);
      dist_last_right_decel = cj_rampDist(v_last_right_exit, v_target_exit, j_max, a_right);
    } else {
      v_last_right_exit = 0;
      dist_last_right_decel = 0;
    }
  }

  /**
   * Max entry speed for a pure decel ramp over blocks [from..to),
   * decelerating to v_exit, respecting all interior junction ceilings.
   */
  float maxDecelEntry(const float* mm_arr, const float* v_junction,
                      uint8_t from, uint8_t to, float v_exit,
                      float a_max, float j_max) {
    float dist_total = sumDist(mm_arr, from, to);
    float v_entry = maxReachableSpeed(v_exit, dist_total, 1e10f, a_max, j_max);
    v_entry = _MIN(v_entry, v_junction[from]);

    if (v_entry <= v_exit) return v_entry;

    // Compute 3-phase decel ramp phase boundaries
    float dt_jerk1, dt_hold, dt_jerk2;
    float dist_endA, v_endA, dist_endB, v_endB, dist_ramp_total;
    auto recomputeRamp = [&]() {
      cj_planRamp(v_exit, v_entry, j_max, a_max, true, dt_jerk1, dt_hold, dt_jerk2);
      float v_cur = v_entry, a_cur = 0.0f, dist_cur = 0.0f;
      cj_simulatePhase(-j_max, dt_jerk1, v_cur, a_cur, dist_cur);
      dist_endA = dist_cur; v_endA = v_cur;
      cj_simulatePhase(0, dt_hold, v_cur, a_cur, dist_cur);
      dist_endB = dist_cur; v_endB = v_cur;
      cj_simulatePhase(j_max, dt_jerk2, v_cur, a_cur, dist_cur);
      dist_ramp_total = dist_cur;
    };
    recomputeRamp();

    float dist_cum = 0;
    for (uint8_t k = from; k + 1 < to; k++) {
      dist_cum += mm_arr[k];
      const float v_ceiling = v_junction[k + 1];
      if (v_ceiling >= v_entry) continue;
      if (v_ceiling <= v_exit) { v_entry = v_ceiling; break; }
      const float dist_at_ceiling = cj_decelRampDistAtVelocity(
        v_entry, v_exit, v_ceiling, j_max, a_max,
        dist_endA, v_endA, dist_endB, v_endB, dist_ramp_total);
      if (dist_at_ceiling > dist_cum) {
        v_entry = v_ceiling;
        if (v_entry <= v_exit) break;
        recomputeRamp();
      }
    }
    return v_entry;
  }

  ConstantJerkTrajectoryGenerator traj;
  uint8_t orig_block_index = 0;
  uint8_t group_block_count = 0;
  uint8_t group_buffer_consumed = 0;
  float v_last_right_exit = 0;          // speed at end of last cycle's buffer (right becomes next left)
  float dist_last_right_decel = 0;     // distance last right needed to decel from emitted exit to v_last_right_exit
  uint16_t cant_brake_count = 0;    // diagnostic: how many times the can't-brake fallback triggered
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;

  /**
   * Min speed reachable by decelerating from v_from over dist_total.
   * If the decel ramp from v_from to 0 fits in dist_total, returns 0.
   * Otherwise binary search on cj_rampDist (O(1) per iteration).
   * Returns hi (slight overestimate within 0.001 mm/s).
   * Monotone decreasing in dist_total: more distance → lower result.
   */
  float minReachableSpeed(float v_from, float dist_total,
                          float a_max, float j_max) {
    if (v_from <= 0.0f || dist_total <= 0.0f) return v_from;

    if (cj_rampDist(0, v_from, j_max, a_max) <= dist_total) return 0;

    float lo = 0, hi = v_from;
    for (int i = 0; i < 32; i++) {
      float mid = 0.5f * (lo + hi);
      if (cj_rampDist(mid, v_from, j_max, a_max) <= dist_total)
        hi = mid;
      else
        lo = mid;
      if (hi - lo < 0.001f) break;
    }
    return hi;
  }
};
