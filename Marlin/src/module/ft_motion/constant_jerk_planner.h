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
 * - Runs its own jerk-aware backward pass on all visible blocks
 * - Merges compatible consecutive blocks into a single S-curve, relaxing
 *   the zero-acceleration-at-junctions constraint
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
 *   per-block pass would allow, so superblock-feasible ⇒ per-block feasible.
 *
 * ─── Key functions ───
 *
 *   maxReachableSpeed(v_from, total_mm, v_max, a_max, j_max) → float
 *     Max speed reachable from v_from over total_mm under jerk/accel limits,
 *     capped by v_max. Binary search over cj_planRamp (32 iterations).
 *     When v_from > v_max, returns v_max (v_max is a hard ceiling).
 *     Returns lo (underestimate): actual reachable speed ∈ [lo, lo + 0.001].
 *
 *   minReachableSpeed(v_from, total_mm, a_max, j_max) → float
 *     Min speed after decelerating from v_from over total_mm.
 *     Returns 0 if full stop fits; otherwise binary search.
 *     Returns hi (overestimate): actual min speed ∈ [hi - 0.001, hi].
 *
 *   peakExceedsCeiling(v_entry, v_exit, a_max, j_max, dist, nominal, ceiling) → bool
 *     Whether the S-curve peak over dist strictly exceeds ceiling. O(1).
 *     If ceiling ≥ nominal, returns false (peak is capped by nominal).
 *     If ceiling < max(v_entry, v_exit), returns true (already above).
 *     Otherwise checks if ramp distance to ceiling fits in dist.
 *
 *   cj_planRamp(v_start, v_peak, j, a_max, decel, ...) → float
 *     Distance consumed by a 3-phase jerk-limited ramp.
 *     Monotone in |v_peak - v_start|: larger speed change → more distance.
 *     Symmetric: accel and decel ramps consume equal distance.
 *     This symmetry means maxReachableSpeed(v_a, dist) ≥ v_b implies
 *     decel from v_b to v_a also fits in dist.
 *
 * ─── plan_full feasibility ───
 *
 *   plan_full(v0, v1, a_max, j, dist, nominal) requires:
 *     cj_planRamp(min(v0,v1), max(v0,v1), j, a_max) ≤ dist
 *   i.e. the ramp between the two boundary speeds must fit.
 *
 *   The planner ensures this in both directions:
 *     Forward: maxReachableSpeed(v0, dist) ≥ v1  (by construction)
 *     Decel:   minReachableSpeed(v0, dist) ≤ v1  (checked explicitly)
 *   If decel is infeasible, groups are split until it holds.
 *   As a last resort, plan_full inflates jerk by 10% recursively.
 *
 * ─── Backward pass ───
 *
 *   max_safe_entry[i] = max speed at which block i can be entered such that
 *   the remaining blocks [i..N) can decelerate to a stop by block N.
 *
 *   Computed right-to-left:
 *     max_safe_entry[N] = 0
 *     max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i+1], mm[i],
 *                            min(nominal[i], vmax_junction[i]), accel[i], j)
 *
 *   The v_max parameter = min(nominal, vmax_junction) folds the junction
 *   ceiling into the search, so the result never exceeds either limit.
 *
 *   Property (backward-monotone):
 *     max_safe_entry over [i..N) ≤ max_safe_entry over [i..N+k)
 *     Adding blocks to the tail only raises safe entry speeds (more room).
 *
 * ─── Merge algorithm ───
 *
 *   The visible buffer is partitioned into:
 *     left = [0..L),  right = [L..R),  tail = [R..N)
 *
 *   Left and right are superblocks (same nominal, close accel).
 *   The junction speed between them:
 *     v_junction = min(max_left_exit, max_right_entry)
 *   where max_left_exit  = maxReachableSpeed(entry, left_mm, v_max, ...)
 *         max_right_entry = maxReachableSpeed(safe_entry[R], right_mm, v_max, ...)
 *   and v_max = min(nominal, vmax_junction[L]) caps at the junction ceiling.
 *
 *   Decel check: minReachableSpeed(entry, left_mm) must be ≤ v_junction,
 *   otherwise the left can't slow down enough to reach the junction speed.
 *
 *   Interior junction check: the peak velocity within each superblock
 *   must not exceed any interior junction ceiling:
 *     !peakExceedsCeiling(entry, exit, a, j, mm, nominal, min_interior_jv)
 *
 *   On failure, binary-split the offending group and retry.
 *   Right is split first (may lower v_junction, fixing the left).
 *   If left must split, its second half becomes the new right.
 *
 * ─── Cross-cycle guarantees ───
 *
 *   Each cycle emits left=[0..L) and remembers right=[L..R).
 *   Next cycle, those right blocks become the new (minimum) left group.
 *
 *   Stored between cycles:
 *     min_left_size  = len(right)           — guarantee (1)
 *     min_safe_exit  = max_safe_entry[R]    — guarantee (2)
 *
 *   Let V_j = junction speed chosen in previous cycle,
 *       V_e = max_safe_entry[R] from previous cycle.
 *
 *   (1) Decel feasibility: the previous right group could decelerate from
 *       V_j to V_e over its total distance with min(accel). Now those same
 *       blocks are the left group with the same min(accel) and distance.
 *       By backward-monotone, new max_safe_entry[R] ≥ V_e (tail grew).
 *       min_left_size prevents splitting below the previous right size,
 *       preserving the distance over which V_j was validated.
 *
 *   (2) Interior junction feasibility: previous cycle validated that the
 *       peak over the right group (with entry=V_j, exit=V_e) did not
 *       exceed any interior junction. The new left has the same blocks,
 *       same min(accel), same entry (V_j). But the new right group may
 *       be longer, raising max_right_entry above V_e. A higher exit
 *       raises the peak, potentially exceeding interior junctions.
 *       When splitting can't resolve this (left_end == min_left_size,
 *       right_len ≤ 1), we fall back to min_safe_exit (= V_e), which
 *       reproduces the previous cycle's peak (same entry, exit, accel,
 *       distance). This is safe because V_e was the speed at which the
 *       previous cycle proved we could decelerate to a stop by the end
 *       of the tail. Since the tail only grew (backward-monotone), we
 *       can still stop if needed. In practice the left group advances
 *       each cycle and connects to the new blocks before that happens.
 */

/**
 * Does the S-curve peak velocity over dist strictly exceed ceiling?
 * O(1): a single cj_totalRampDist call replaces a binary search.
 * - ceiling ≥ v_nominal → false (peak capped by cruise speed)
 * - ceiling < max(v_entry, v_exit) → true (boundary already exceeds)
 * - otherwise: true iff ramp to ceiling fits in dist (peak goes higher)
 */
static bool peakExceedsCeiling(float v_entry, float v_exit, float a_max_val,
                                float j_max_val, float dist, float v_nominal,
                                float ceiling) {
  if (ceiling >= v_nominal) return false;
  const float v_small = _MIN(v_entry, v_exit);
  const float v_large = _MAX(v_entry, v_exit);
  if (ceiling < v_large) return true;
  return cj_totalRampDist(ceiling, v_small, v_large, j_max_val, a_max_val) < dist;
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
    min_left_size = 1;
    min_safe_exit = 0;
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
  bool planNext(const float jerk_max) {
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
    for (uint8_t i = 0; i < BLOCK_BUFFER_SIZE; i++) {
      block_t* blk = planner.get_future_block(i, false);
      if (!blk) break;
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
      max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i + 1], mm[i], _MIN(nominal[i], vmax_junction[i]), accel[i], jerk_max);
    }
    float left_entry_speed = traj.getExitSpeed();
    if (left_entry_speed == 0) {
      min_left_size = 1; // TODO: do this more cleanly (if trajectory was reset, min_left_size should be reset too)
      min_safe_exit = 0;
    }

    // Left-compatible group: cumulative superblock parameters for [0..i+1).
    //   cum_mm[i]             = sum(mm[0..i+1))            — superblock distance
    //   cum_min_a[i]          = min(accel[0..i+1))         — within CJP_MERGE_AMAX_RATIO of max
    //   cum_vmax_junction[i]  = min(vmax_junction[1..i+1)) — tightest interior junction
    float cum_mm[BLOCK_BUFFER_SIZE];
    float cum_min_a[BLOCK_BUFFER_SIZE];
    float cum_vmax_junction[BLOCK_BUFFER_SIZE];


    uint8_t left_end = 1;
    {
      cum_mm[0] = mm[0];
      cum_min_a[0] = accel[0];
      cum_vmax_junction[0] = left_entry_speed;  // unused
      float cum_max_a = accel[0];
      const uint8_t max_left_end = _MIN(block_count, BLOCK_BUFFER_SIZE / 2);
      for (uint8_t i = 1; i < max_left_end; i++) {
        if (nominal[i] != nominal[0]) break;
        float new_a_min = _MIN(cum_min_a[i - 1], accel[i]);
        float new_a_max = _MAX(cum_max_a, accel[i]);
        if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
        cum_max_a = new_a_max;
        cum_mm[i] = cum_mm[i - 1] + mm[i];
        cum_min_a[i] = new_a_min;
        cum_vmax_junction[i] = (i == 1) ? vmax_junction[1] : _MIN(cum_vmax_junction[i - 1], vmax_junction[i]);
        left_end++;
      }
    }

    // Find right-compatible group starting at left_end
    uint8_t right_end = left_end;
    {
      float cum_a_min = accel[left_end], cum_a_max = accel[left_end];
      for (uint8_t i = left_end; i < block_count; i++) {
        if (nominal[i] != nominal[left_end]) break;
        float new_a_min = _MIN(cum_a_min, accel[i]);
        float new_a_max = _MAX(cum_a_max, accel[i]);
        if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
        cum_a_min = new_a_min;
        cum_a_max = new_a_max;
        right_end++;
      }
    }

    // Iteratively refine: split groups until junction constraints are met.
    // See "Merge algorithm" and "Cross-cycle guarantees" in header.
    float left_exit_speed;
    uint32_t right_len;

    while (true) {
      right_len = right_end - left_end;

      // Left superblock [0..left_end): forward pass as single block
      float left_mm = cum_mm[left_end - 1];
      float left_a = cum_min_a[left_end - 1];
      float left_nominal = nominal[0];
      float max_left_exit = 0;
      // Right superblock [left_end..right_end) parameters.
      // Only valid when right_len > 0; guarded to avoid out-of-bounds reads.
      float right_mm = 0;
      float right_a = 0;
      float right_nominal = 0;
      float max_right_entry = 0;
      if (right_len > 0) {
        right_mm = sumDist(mm, left_end, right_end);
        right_a = minVal(accel, left_end, right_end);
        right_nominal = nominal[left_end];
        // max_right_entry: max speed right superblock can accept and still
        // decelerate to max_safe_entry[right_end] over right_mm.
        // Conservative: uses min(accel) so any per-block pass would allow ≥ this.
        max_right_entry = maxReachableSpeed(max_safe_entry[right_end], right_mm, _MIN(right_nominal, vmax_junction[left_end]), right_a, jerk_max);

        // max_left_exit: max speed left superblock can reach from left_entry_speed.
        // Capped by junction ceiling at left_end (entry of right/next group).
        max_left_exit = maxReachableSpeed(left_entry_speed, left_mm, _MIN(left_nominal, vmax_junction[left_end]), left_a, jerk_max);
      }

      // Junction = min of what left can provide, what right can accept.
      float v_junction_candidate = _MIN(max_left_exit, max_right_entry);

      // min_left_exit: minimum speed the left can decelerate to from left_entry_speed.
      // If this exceeds v_junction_candidate, the left can't slow down enough.
      // Only relevant when there IS a right group — without one, exit is 0
      // (stop at buffer end) which plan_full handles via its own ramp planning.
      bool valid_junction = true;
      if (right_len > 0) {
        const float min_left_exit = minReachableSpeed(left_entry_speed, left_mm, left_a, jerk_max);
        valid_junction = min_left_exit <= v_junction_candidate;
      }


      // Validate right interior junctions: peak within right superblock
      // must not exceed any interior junction ceiling.
      if (right_len > 1) {
        if (valid_junction) {
          float right_min_jv = minVal(vmax_junction, left_end + 1, right_end);
          valid_junction = !peakExceedsCeiling(max_safe_entry[right_end], v_junction_candidate, right_a, jerk_max, right_mm, right_nominal, right_min_jv);
        }

        if (!valid_junction) {
          right_end = left_end + right_len / 2;  // split right, retry
          continue;
        }
      }

      // Validate left interior junctions: peak within left superblock
      // must not exceed any interior junction ceiling.
      if (valid_junction && left_end > 1) {
        float left_min_jv = minVal(vmax_junction, 1, left_end);
        valid_junction = !peakExceedsCeiling(left_entry_speed, v_junction_candidate, left_a, jerk_max, left_mm, left_nominal, left_min_jv);
      }

      if (!valid_junction) {
        if (right_len > 1) {
          right_end = left_end + right_len / 2;  // try shrinking right first
          continue;
        }
        // Split left, but not below min_left_size — guarantee (1)
        if (left_end > min_left_size) {
          right_end = left_end;
          left_end = _MAX(left_end / 2, min_left_size);
          continue;
        }
        // Can't split either side. Fall back to min_safe_exit — guarantee (2):
        // previous cycle validated peak(V_j, min_safe_exit) ≤ interior junctions
        // for these same blocks, so min_safe_exit is known-safe.

        // left_len == min_left_size && right_len < 2
        left_exit_speed = min_safe_exit;
        min_left_size = 1;
        min_safe_exit = right_len == 0 ? 0: minReachableSpeed(left_exit_speed, right_mm, right_a, jerk_max);
        break;

      }
      // Both superblocks pass interior junction checks
      left_exit_speed = v_junction_candidate;

      // Store right group state for next cycle's cross-cycle guarantees:
      //   min_left_size = right_len        → (1) preserves decel feasibility distance
      //   min_safe_exit = safe_entry[R]    → (2) known-valid exit for fallback
      min_left_size = right_len;
      min_safe_exit = max_safe_entry[right_end];
      break;
    }


    // --- 5. Plan trajectory ---
    traj.plan_full(left_entry_speed, left_exit_speed, cum_min_a[left_end - 1], jerk_max, cum_mm[left_end - 1], nominal[0]);

    // Set up execution tracking
    orig_block_index = 0;
    orig_block_start_dist = 0;
    group_block_count = left_end;
    // Raw buffer entries consumed (includes skipped sync blocks)
    group_buffer_consumed = buf_offset[left_end - 1] + 1;
    orig_block_end_dist = cum_mm[0];
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

 private:
  /**
   * Max speed reachable from v_from over total_mm via jerk-limited ramp,
   * capped by v_max. Returns lo (underestimate within 0.001 mm/s).
   * When v_from > v_max, returns v_max (hard ceiling).
   * Monotone in total_mm: more distance → higher or equal result.
   */
  float maxReachableSpeed(float v_from, float total_mm,
                          float v_max, float a_max_val, float j_max_val) {
    float v_trap = SQRT(v_from * v_from + 2.0f * a_max_val * total_mm);
    float hi = _MIN(v_max, v_trap);
    float lo = v_from;

    if (hi <= lo) return hi;

    float pa, pb, pc;
    float s = cj_planRamp(v_from, hi, j_max_val, a_max_val, false, pa, pb, pc);
    if (s <= total_mm) return hi;

    for (int i = 0; i < 32; i++) {
      float mid = 0.5f * (lo + hi);
      s = cj_planRamp(v_from, mid, j_max_val, a_max_val, false, pa, pb, pc);
      if (s <= total_mm)
        lo = mid;
      else
        hi = mid;
      if (hi - lo < 0.001f) break;
    }
    return lo;
  }

  ConstantJerkTrajectoryGenerator traj;
  uint8_t orig_block_index = 0;
  uint8_t group_block_count = 0;
  uint8_t group_buffer_consumed = 0;
  uint8_t min_left_size = 1;  // (1): previous right group size — floor for left splitting
  float min_safe_exit = 0; // (2): previous right group exit — known-valid fallback
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;

  /**
   * Min speed reachable by decelerating from v_from over total_mm.
   * If the decel ramp from v_from to 0 fits in total_mm, returns 0.
   * Otherwise returns the speed at which the decel ramp exactly fills total_mm.
   * Monotone decreasing in total_mm: more distance → lower result.
   */
  float minReachableSpeed(float v_from, float total_mm,
                          float a_max_val, float j_max_val) {
    if (v_from <= 0.0f || total_mm <= 0.0f) return v_from;

    float pa, pb, pc;
    float s = cj_planRamp(0, v_from, j_max_val, a_max_val, true, pa, pb, pc);
    if (s <= total_mm) return 0;

    float lo = 0, hi = v_from;
    for (int i = 0; i < 32; i++) {
      float mid = 0.5f * (lo + hi);
      s = cj_planRamp(mid, v_from, j_max_val, a_max_val, true, pa, pb, pc);
      if (s <= total_mm)
        hi = mid;
      else
        lo = mid;
      if (hi - lo < 0.001f) break;
    }
    return hi;
  }
};
