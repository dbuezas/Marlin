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
 * The standard trapeozidal planner assumes instant acceleration changes.
 * Under jerk constraints, S-curves need more distance for the same speed
 * change, so a diferent planner is needed.
 *
 * This planner:
 * - Ignores block->entry_speed and block->exit_speed entirely
 * - Uses block->max_entry_speed_sqr as junction speed ceiling (geometric, valid)
 * - Runs its own jerk-aware reverse/forward pass on all visible blocks enforcing
 * zero acceleration at junctions
 * - Optionally merges compatible consecutive blocks into a single S-curve to remove
 * the relax the zero acceleration at junctions limitation
 *
 *
 * Merge algorithm:
 * - Find left-compatible group and right-compatible group (same nominal speed, close max axel)
 * - Treat right group as a superblock for better left group exit speed
 * - Check v_peak of left and right groups against min interior junction limit
 * - Binary split on failure until valid or single block on each side
 */

/**
 * Compute the peak velocity of an S-curve trajectory (without building phases).
 */
static float peakSpeed(float v_entry, float v_exit, float a_max_val,
                        float j_max_val, float dist, float v_nominal) {
  const float v_small = _MIN(v_entry, v_exit);
  const float v_large = _MAX(v_entry, v_exit);
  float v_peak = _MAX(v_large, v_nominal);
  float s_ramps = cj_totalRampDist(v_peak, v_small, v_large, j_max_val, a_max_val);

  if (s_ramps > dist) {
    float v_hi = v_peak, v_lo = v_large;
    if (cj_totalRampDist(v_lo, v_small, v_large, j_max_val, a_max_val) > dist)
      return v_lo;
    for (int i = 0; i < 16; i++) {
      float mid = 0.5f * (v_lo + v_hi);
      float s = cj_totalRampDist(mid, v_small, v_large, j_max_val, a_max_val);
      if (s > dist)
        v_hi = mid;
      else
        v_lo = mid;
      if (v_hi - v_lo < 0.01f) break;
    }
    v_peak = v_lo;
  }
  return v_peak;
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
    last_exit_speed = 0;
    traj.reset();
  }

  /**
   * Plan trajectory for the current block (already consumed from planner buffer).
   *
   * @param current_block The block already consumed by plan_next_block()
   *
   * Looks ahead at future blocks via get_future_block(), runs a jerk-aware
   * reverse/forward pass across all visible blocks, then plans the first
   * block (or merged group) as an S-curve trajectory.
   *
   * Returns true if a trajectory is ready for execution.
   */
  bool planNext(const block_t* current_block, const float jerk_max) {
    float mm[BLOCK_BUFFER_SIZE];
    float nominal[BLOCK_BUFFER_SIZE];
    float accel[BLOCK_BUFFER_SIZE];
    float max_entry_speed[BLOCK_BUFFER_SIZE];  // max entry speed ceiling per block

    mm[0] = current_block->millimeters; // TODO: many of these are probably not needed
    nominal[0] = current_block->nominal_speed;
    accel[0] = current_block->acceleration;
    max_entry_speed[0] = SQRT(current_block->max_entry_speed_sqr);

    uint8_t block_count = 1;

    // Look ahead at future blocks.
    // get_future_block(offset) returns block_buffer[tail + offset].
    // The current block is at tail (offset 0), so offset 1 = next block.
    for (uint8_t i = 1; i < BLOCK_BUFFER_SIZE; i++) {
      block_t* blk = planner.get_future_block(i);
      if (!blk || blk->is_sync()) break; // TODO: skip synch blocks

      mm[i] = blk->millimeters;
      nominal[i] = blk->nominal_speed;
      accel[i] = blk->acceleration;
      max_entry_speed[i] = SQRT(blk->max_entry_speed_sqr);
      block_count++;
    }

    float entry_v[BLOCK_BUFFER_SIZE + 1];
    // Backward pass
    entry_v[block_count] = 0.0f;
    for (int8_t i = block_count - 1; i > 0; i--) {
      float v_reachable = maxReachableSpeed(entry_v[i + 1], mm[i], nominal[i], accel[i], jerk_max);
      entry_v[i] = _MIN(v_reachable, max_entry_speed[i]);
    }

    // Forward pass
    entry_v[0] = last_exit_speed;
    for (uint8_t i = 0; i < block_count - 1; i++) {
      float v_reachable = maxReachableSpeed(entry_v[i], mm[i], nominal[i], accel[i], jerk_max);
      entry_v[i + 1] = _MIN(v_reachable, entry_v[i + 1]);
    }

    // Find left-compatible group, tracking cumulative mm, min accel, min junction
    float group_a_min = accel[0], group_a_max = accel[0];
    float cum_mm[BLOCK_BUFFER_SIZE];      // cum_mm[i] = sumDist(mm, i+1)
    float cum_min_a[BLOCK_BUFFER_SIZE];   // cum_min_a[i] = minVal(accel, 0, i+1)
    float cum_max_entry_speed[BLOCK_BUFFER_SIZE];  // cum_max_entry_speed[i] = minVal(max_entry_speed, 1, i+1) for i>=1
    cum_mm[0] = mm[0];
    cum_min_a[0] = accel[0];
    cum_max_entry_speed[0] = last_exit_speed;  // unused but initialized

    uint8_t left_end = 1;
    const uint8_t max_left_end = _MIN(block_count, BLOCK_BUFFER_SIZE / 2);
    for (uint8_t i = 1; i < max_left_end; i++) {
      if (nominal[i] != nominal[0]) break;
      float new_a_min = _MIN(group_a_min, accel[i]);
      float new_a_max = _MAX(group_a_max, accel[i]);
      if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
      group_a_min = new_a_min;
      group_a_max = new_a_max;
      cum_mm[i] = cum_mm[i - 1] + mm[i];
      cum_min_a[i] = _MIN(cum_min_a[i - 1], accel[i]);
      cum_max_entry_speed[i] = (i == 1) ? max_entry_speed[1] : _MIN(cum_max_entry_speed[i - 1], max_entry_speed[i]);
      left_end++;
    }

    // Find right-compatible group starting at left_end
    uint8_t right_end = left_end;
    if (left_end < block_count) {
      float r_a_min = accel[left_end], r_a_max = accel[left_end];
      for (uint8_t i = left_end; i < block_count; i++) {
        if (nominal[i] != nominal[left_end]) break;
        float new_a_min = _MIN(r_a_min, accel[i]);
        float new_a_max = _MAX(r_a_max, accel[i]);
        if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
        r_a_min = new_a_min;
        r_a_max = new_a_max;
        right_end++;
      }
    }

    // Iteratively refine: split groups until junction constraints are met
    // Floor for left group size: when continuing from a boundary, the first
    float left_exit_speed;
    float max_left_exit = -1;
    while (true) {
      // Right side, only backwards pass to calculate max_right_entry
      float max_right_entry = 0;
      uint32_t right_len = right_end - left_end;
      const float right_mm = sumDist(mm, left_end, right_end);
      const float right_a = minVal(accel, left_end, right_end);
      const float right_nominal = nominal[left_end];

      if (left_end == block_count) {
        // no right side, left must break to standstill
        max_right_entry = 0;
      } else if (right_len < 2){
        // right is not a super block, use precalculated entry
        max_right_entry = entry_v[right_end - 1];
      } else {
        // right is a super block, calculate its max entry speed
        const float right_exit_speed = entry_v[right_end];
        const float v_reach = maxReachableSpeed(right_exit_speed, right_mm, right_nominal, right_a, jerk_max);
        max_right_entry = _MIN(v_reach, max_entry_speed[left_end]);
      }

      // Left superblock: only forward pass to calculate max_left_exit
      float left_mm = cum_mm[left_end - 1];
      float left_a = cum_min_a[left_end - 1];
      float left_nominal = nominal[0];

      // Reverse pass
      if (max_left_exit == -1) {
        if (left_end == 1){
          // left is single block, use precalculated entry
          max_left_exit = entry_v[1];
        } else {
          // left is a super block, calculate its max exit speed
          max_left_exit = maxReachableSpeed(entry_v[0], left_mm, left_nominal, left_a, jerk_max);
        }
      }

      float v_junction_candidate = _MIN(max_left_exit, max_right_entry, left_nominal, right_nominal);

      if (right_len > 1) {
        // Check v_peak against min interior junction limit
        float right_v_peak = peakSpeed(entry_v[right_end], v_junction_candidate, right_a, jerk_max, right_mm, right_nominal);
        float right_min_jv = minVal(max_entry_speed, left_end, right_end);

        if (right_v_peak > right_min_jv && right_len > 1) {
          // Right group too agressive - need to split
          right_end = left_end + right_len / 2;
          continue;
        }
      }
      if (left_end > 1) {
        float left_v_peak = peakSpeed(entry_v[0], v_junction_candidate, left_a, jerk_max, left_mm, left_nominal);
        float left_min_jv = cum_max_entry_speed[left_end - 1];

        if (left_end > 1 && left_v_peak > left_min_jv) {
          // Left too aggressive - need to split
          uint8_t new_left_end = left_end / 2;
          right_end = left_end;
          left_end = new_left_end;
          max_left_exit = -1; // force recalculate
          continue;
        }
      }
      // Both groups valid
      left_exit_speed = v_junction_candidate;
      break;
    }


    // --- 5. Plan trajectory ---

    traj.plan_full(last_exit_speed, left_exit_speed, cum_min_a[left_end - 1], jerk_max, cum_mm[left_end - 1], nominal[0]);
    last_exit_speed = left_exit_speed;

    // Set up execution tracking
    orig_block_index = 0;
    orig_block_start_dist = 0;
    group_block_count = left_end;
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
  uint8_t currentBlockIndex() const { return orig_block_index; }

 private:
  /**
   * Compute the maximum speed reachable from v_from over a given distance
   * using a constant-jerk ramp. Uses binary search with cj_planRamp.
   */
  float maxReachableSpeed(float v_from, float total_mm,
                          float nominal, float a_max_val, float j_max_val) {
    if (total_mm <= 0.0f) return v_from;

    float v_trap = SQRT(v_from * v_from + 2.0f * a_max_val * total_mm);
    float hi = _MIN(nominal, v_trap);
    float lo = v_from;

    if (hi <= lo) return lo;

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
      if (hi - lo < 0.01f) break;
    }
    return lo;
  }

  ConstantJerkTrajectoryGenerator traj;
  uint8_t orig_block_index = 0;
  uint8_t group_block_count = 0;
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;
  float last_exit_speed = 0;
};
