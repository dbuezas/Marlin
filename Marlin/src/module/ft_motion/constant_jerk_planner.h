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
 *   max_safe_entry_to_unmerged_tail[i] = max speed at which block i can be entered (with a=0)
 *   such that blocks [i..N) can decelerate to a stop by block N.
 *
 *   Computed right-to-left:
 *     max_safe_entry_to_unmerged_tail[N] = 0
 *     max_safe_entry_to_unmerged_tail[i] = maxReachableSpeed(max_safe_entry_to_unmerged_tail[i+1], mm[i],
 *                            min(nominal[i], vmax_junction[i]), accel[i], j_max)
 *
 * ─── Merge algorithm (largest feasible left_end) ───
 *
 *   1. Find left-compatible extent: same nominal, accel ratio ≤ 1.1.
 *   2. Try from largest left_end downward, take first feasible:
 *      a. v_junction = min(max_safe_entry_to_unmerged_tail[left_end], nominal, vmax_junction)
 *      b. Plan left S-curve toward v_junction
 *      c. Check velocity at block 0 exit against vmax_junction[1]
 *      d. Check interior junctions against vmax_junction[k]
 *   3. For candidate=1 if decel overshoots, replan targeting v=0
 *      (full decel uses LESS distance than partial — S-curve asymmetry).
 *   4. Truncate trajectory to block 0, store (v, a) exit state.
 *
 *   Replanning each cycle from actual (v, a) compensates for the
 *   conservative backward pass (which assumes a=0 at boundaries).
 */

class ConstantJerkTrajectoryGenerator;  // Forward declaration

// Pre-truncation trajectory state for reuse optimization.
// Shared between ConstantJerkBlockPlanner and ConstantJerkTrajectoryGenerator.
struct CJTrajectorySnapshot {
  float phase_dt[7] = {};
  float v_entry = 0, a_entry = 0;
  float v_exit = 0, a_exit = 0;
  float a_max = 0, j_max = 0;
  float dist_total = 0;
  float total_duration = 0;
  bool valid = false;
};

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

// Maximum value in arr[from..to-1]
static float maxVal(const float* arr, uint8_t from, uint8_t to) {
  float v = arr[from];
  for (uint8_t i = from + 1; i < to; i++) v = _MAX(v, arr[i]);
  return v;
}


class ConstantJerkBlockPlanner {
 public:
  // Reset all planner state (called by the generator's planRunout/reset).
  void resetPlannerState() {
    orig_block_index = 0;
    v_exit_stored = 0;
    a_exit_stored = 0;
    prev_left_end = 0;
    prev_v_junction = 0;
    prev_traj_state.valid = false;
    prev_mm_block0 = 0;
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

 private:
  /**
   * Max speed reachable from v_from over dist_total via jerk-limited ramp,
   * capped by v_max. Newton's method on closed-form cj_rampDist.
   * When v_from > v_max, returns v_max (hard ceiling).
   * Monotone in dist_total: more distance → higher or equal result.
   *
   * a_entry: initial acceleration (default 0). When non-zero, the ramp must
   * first absorb a_entry (bring a to 0), which uses distance and reduces
   * the max reachable speed. Uses cj_planRamp with a_entry for distance.
   */
  float maxReachableSpeed(float v_from, float dist_total,
                          float v_max, float a_max, float j_max,
                          float a_entry = 0.0f);

  // Execution tracking
  uint8_t orig_block_index = 0;
  uint8_t group_block_count = 0;
  uint8_t group_buffer_consumed = 0;
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;

  // Stored exit state from last emitted block
  float v_exit_stored = 0;
  float a_exit_stored = 0;

  // Previous call's left_end and v_junction. Used as fallback:
  // prev_left_end - 1 with prev_v_junction is guaranteed feasible
  // because the previous trajectory already proved it.
  uint8_t prev_left_end = 0;
  float prev_v_junction = 0;

  // Trajectory reuse: pre-truncation state from previous planNext call.
  // When the backward pass at the reuse junction hasn't improved, we can
  // advance within the previous trajectory instead of replanning.
  CJTrajectorySnapshot prev_traj_state;
  float prev_mm_block0 = 0;  // distance of block 0 that was consumed
};
