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

#define CJP_MERGE_AMAX_RATIO 1.1f

/**
 * Constant-jerk block planner (1-block emission with non-zero a_entry)
 *
 * The standard trapezoidal planner assumes instant acceleration changes.
 * Under jerk constraints, S-curves need more distance for the same speed
 * change, so a different planner is needed.
 *
 * This planner:
 * - Ignores Marlin's trapezoidal entry/exit speeds entirely
 * - Uses block->vmax_junction as the junction speed ceiling (geometric limit)
 * - Runs a jerk-aware backward pass on all visible blocks
 * - Merges compatible consecutive blocks into a superblock trajectory,
 *   but emits only the first block, carrying exit (v, a) to the next cycle
 *
 *
 * ─── 1-block emission model ───
 *
 *   Each planNext() call:
 *   1. Plans a multi-block superblock trajectory [0..left_end) for lookahead
 *   2. Truncates the trajectory to block 0's distance (truncateToDistance)
 *   3. Stores exit (v, a) at block 0 boundary for the next call
 *   4. Consumes only 1 block from the buffer
 *
 *   The non-zero exit acceleration (a_exit) from block 0's cut point is
 *   passed as a_entry to the next cycle's plan_full(). This gives smoother
 *   transitions than forcing a=0 at every block boundary.
 *
 *   When a_entry is infeasible for plan_full (e.g., too high for the
 *   available distance), plan_full falls back to a_entry=0 automatically.
 *
 * ─── Backward pass ───
 *
 *   max_safe_entry[i] = max speed at which block i can be entered (with a=0)
 *   such that blocks [i..N) can decelerate to a stop by block N.
 *
 *   Computed right-to-left:
 *     max_safe_entry[N] = 0
 *     max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i+1], mm[i],
 *                            min(nominal[i], vmax_junction[i]), accel[i], j_max)
 *
 *   Also used at block 0's exit to cap v_exit_stored, ensuring the
 *   next cycle can decelerate safely. This is conservative when a≠0
 *   at block boundaries (max_safe_entry assumes a=0).
 *
 * ─── Merge algorithm (right-side superblock braking) ───
 *
 *   Bottom-up left_end selection with warm start from previous call:
 *
 *   1. Find left-compatible extent: same nominal, accel ratio ≤ 1.1,
 *      up to half the buffer when full.
 *   2. For each candidate left_end (bottom-up from warm start):
 *      a. Find right-compatible group starting at left_end
 *      b. Compute v_junction = maxSafeJunctionSpeed(...) using right-side
 *         superblock braking. The right group is a single continuous decel
 *         ramp (a≠0 at interior boundaries), which is more efficient than
 *         the per-block backward pass that wastes distance zeroing accel.
 *      c. v_junction ≤ vmax_junction[left_end] (geometric cap)
 *      d. v_junction ≥ max_safe_entry[left_end] (at least as good as per-block)
 *      e. Plan left S-curve toward v_junction, check interior junctions
 *   3. If left_end=0 (nothing feasible), emit the right side decel ramp.
 *   4. Truncate trajectory to block 0, store (v, a) exit state.
 */

class ConstantJerkTrajectoryGenerator;  // Forward declaration

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
  // Reset all planner state (called by the generator's planRunout/reset).
  void resetPlannerState() {
    orig_block_index = 0;
    cant_brake_count = 0;
    v_exit_stored = 0;
    a_exit_stored = 0;
    prev_left_end = 0;
    prev_plan_blocks = 0;
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
  bool planNext(ConstantJerkTrajectoryGenerator& traj, float j_max);

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
                          float v_max, float a_max, float j_max);

  /**
   * Min speed reachable by decelerating from v_from over dist_total.
   * If the decel ramp from v_from to 0 fits in dist_total, returns 0.
   * Otherwise binary search on cj_rampDist (O(1) per iteration).
   * Returns hi (slight overestimate within 0.001 mm/s).
   * Monotone decreasing in dist_total: more distance → lower result.
   */
  float minReachableSpeed(float v_from, float dist_total,
                          float a_max, float j_max);

  /**
   * Max junction speed between left and right groups using right-side
   * superblock braking. The right group decelerates continuously from
   * v_junction to v_exit_right (a≠0 at interior block boundaries).
   *
   * Constraints:
   * - v_junction ≤ vmax_junction[left_end] (geometric cap at boundary)
   * - The decel ramp fits in the right group's total distance
   * - At each interior block boundary, velocity ≤ vmax_junction[i]
   *
   * // TODO: Instead of binary search, find the max v_junction analytically
   * // by constraining the decel ramp to be tangent to the tightest interior
   * // junction limit. This would be O(N) instead of O(N * log(1/eps)).
   */
  float maxSafeJunctionSpeed(
      const float* mm, const float* nominal, const float* vmax_junction,
      const float* accel, uint8_t left_end, uint8_t right_end,
      float v_exit_right, float j_max);

  // Execution tracking
  uint8_t orig_block_index = 0;
  uint8_t group_block_count = 0;
  uint8_t group_buffer_consumed = 0;
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;

  // Diagnostics
  uint16_t cant_brake_count = 0;    // how many times the can't-brake fallback triggered

  // Proposal B: stored exit state from last emitted block
  float v_exit_stored = 0;
  float a_exit_stored = 0;

  // Warm start for bottom-up left_end selection
  uint8_t prev_left_end = 0;

  // How many blocks the previous plan's trajectory spanned. Used by
  // full_stop_fallback to compute a_min over the original superblock's blocks,
  // excluding new tail blocks that weren't part of the validated plan.
  uint8_t prev_plan_blocks = 0;
};
