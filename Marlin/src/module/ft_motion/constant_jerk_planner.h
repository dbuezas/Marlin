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
 * - Ignores block->entry_speed and block->exit_speed entirely
 * - Uses block->vmax_junction as junction speed ceiling (geometric, valid)
 * - Runs its own jerk-aware reverse pass on all visible blocks enforcing
 *   zero acceleration at junctions
 * - Merges compatible consecutive blocks into a single S-curve to relax
 *   the zero-acceleration-at-junctions limitation
 *
 * ─── Notation ───
 *
 *   Blocks are indexed 0..N-1. Each block i has:
 *     mm[i]             distance (mm)
 *     accel[i]          max acceleration (mm/s²)
 *     nominal[i]        max cruise speed (mm/s)
 *     vmax_junction[i]  geometric junction speed ceiling at entry of block i
 *
 *   A "superblock" [a..b) treats blocks a..b-1 as one segment with:
 *     distance  = sum(mm[a..b))
 *     accel     = min(accel[a..b))        (conservative)
 *     nominal   = nominal[a]              (all blocks in group share this)
 *
 *   The merge compatibility check ensures max(accel)/min(accel) ≤ CJP_MERGE_AMAX_RATIO
 *   (1.1). Using min(accel) is conservative: the superblock allows less speed
 *   than per-block pass would, so superblock-feasible ⇒ per-block feasible.
 *
 * ─── Key functions and their properties ───
 *
 *   maxReachableSpeed(v_from, dist, nominal, a_max, j_max) → v_to:
 *     The max speed reachable from v_from over dist under jerk/accel limits.
 *     Monotone in dist: more distance → higher or equal v_to.
 *     Monotone in v_from: higher start → higher or equal v_to.
 *     Monotone in a_max: higher accel limit → higher or equal v_to.
 *
 *   peakSpeed(v_entry, v_exit, a_max, j_max, dist, nominal) → v_peak:
 *     The peak velocity of an S-curve with given boundary speeds over dist.
 *     Monotone in v_entry and v_exit: raising either raises v_peak.
 *     Monotone in dist: more distance → higher or equal v_peak.
 *     Monotone in a_max: higher accel limit → higher or equal v_peak.
 *
 *   cj_planRamp(v_start, v_peak, j, a_max, decel, ...) → ramp_dist:
 *     Distance consumed by a 3-phase ramp between v_start and v_peak.
 *     Monotone in (v_peak - v_start): larger speed change → more distance.
 *     Symmetric: accel and decel ramps consume the same distance
 *     (time-reversed velocity profile has equal area).
 *     This means maxReachableSpeed(v_a, dist) ≥ v_b ⟹ decel from v_b
 *     to v_a also fits in dist. Critical for plan_full feasibility.
 *
 * ─── plan_full feasibility ───
 *
 *   plan_full(v0, v1, a_max, j, dist, nominal) requires:
 *     cj_planRamp(min(v0,v1), max(v0,v1), j, a_max) ≤ dist
 *   i.e. the ramp between the two boundary speeds must fit in dist.
 *
 *   The planner must ensure both directions:
 *     maxReachableSpeed(v0, dist, ...) ≥ v1   — accel direction
 *     maxReachableSpeed(v1, dist, ...) ≥ v0   — decel direction (by ramp symmetry)
 *   The accel direction is ensured by forward-pass construction.
 *   The decel direction is checked explicitly: if
 *     maxReachableSpeed(v_junction, left_mm, ...) < left_entry_speed
 *   the junction is infeasible and the groups are split.
 *
 * ─── Backward pass ───
 *
 *   max_safe_entry[i] = max speed at which block i can be entered such that
 *   blocks [i..N) can decelerate to a stop by block N, respecting all
 *   junction ceilings, jerk, and accel limits.
 *
 *   Computed right-to-left:
 *     max_safe_entry[N] = 0 (the exit of block N-1)
 *     max_safe_entry[i] = min(
 *                            maxReachableSpeed(max_safe_entry[i+1], mm[i], ...),
 *                            vmax_junction[i]
 *                         )
 *
 *   Property (backward-monotone):
 *     max_safe_entry over [i..N) ≤ max_safe_entry over [i..N+k)
 *     Adding blocks to the tail can only raise or maintain the safe entry
 *     speed, because the extra distance provides more room to decelerate.
 *
 * ─── Superblock backward pass ───
 *
 *   For a superblock [a..b) with exit speed v_exit:
 *     max_entry = min(
 *                    maxReachableSpeed(v_exit, sum(mm[a..b)), ...),
 *                    vmax_junction[a]
 *                 )
 *
 *   Both left and right superblocks use min(accel), which is conservative:
 *   per-block backward pass with individual accel[i] ≥ min(accel) would
 *   allow equal or higher speeds. So superblock-feasible ⇒ per-block feasible.
 *
 * ─── Merge algorithm ───
 *
 *   The block buffer is partitioned into left=[0..L), right=[L..R), and tail=[R, N):
 *   - Left and right are each superblocks (same nominal, close accel)
 *   - Right superblock's max_entry gives a better exit ceiling for left
 *     than individual block backward pass (more decel distance)
 *   - v_junction = min(max_left_exit, max_right_entry, nominals)
 *   - The tail are the remaining individual blocks.
 *
 *   Validation: the S-curve peak within each superblock must not exceed
 *   any interior junction ceiling. For right=[L..R):
 *     peakSpeed(v_junction, max_safe_entry[R], ...) ≤ min(vmax_junction[L+1..R))
 *   For left=[0..L):
 *     peakSpeed(left_entry, v_junction, ...) ≤ min(vmax_junction[1..L))
 *
 *   On failure, binary-split the offending group and retry
 *    - If both sides violate internal junctions, it's preferred to split the right
 *      side, which may reduce v_junction and make the left side valid.
 *    - If the left is split, it's second half becomes the new right.
 *
 * ─── Cross-cycle guarantees ───
 *
 *   Each cycle emits left=[0..L) and remembers right=[L..R).
 *   Next cycle, the old right blocks are the smallest possible new left group.
 *
 *   Stored between cycles:
 *     min_left_size  = len(old right)      — guarantee (1)
 *     max_safe_exit  = max_safe_entry[R]   — guarantee (2)
 *
 *   Let V_j = junction speed chosen in previous cycle,
 *       V_e = max_safe_entry[R] from previous cycle.
 *
 *   (1) Decel feasibility: left_entry_speed = V_j (previous exit).
 *       The previous right superblock [L..R) could decelerate from V_j
 *       to V_e over sum(mm[L..R)) with min(accel[L..R)).
 *       Now those same blocks are the left group, also using min(accel)
 *       over the same blocks — identical parameters.
 *       The tail beyond is the same or longer (new blocks may arrive),
 *       so by backward-monotone, max_safe_entry[R] ≥ V_e.
 *       min_left_size prevents splitting left below len(old right),
 *       preserving the distance over which V_j was validated.
 *
 *   (2) Interior junction feasibility: previous cycle validated
 *       peakSpeed(V_j, V_e, right_a, ...) ≤ min interior vmax_junction
 *       where right_a = min(accel) of the right group.
 *       The new left group uses cum_min_a = max(accel) of those same blocks,
 *       which is ≤ right_a * CJP_MERGE_AMAX_RATIO (bounded by merge check).
 *       In the current cycle, the new right group may be longer, so
 *       max_right_entry may exceed V_e. Since peakSpeed is monotone
 *       in exit speed, a higher exit could raise the peak above the
 *       interior junction limits. When splitting can't resolve this
 *       (left_end == min_left_size, right_len == 1), we fall back to
 *       max_safe_exit (= V_e), which reproduces approximately the same
 *       peak (slightly higher due to cum_min_a ≥ right_a, bounded by 1.1x).
 */

/**
 * Compute the peak velocity of an S-curve over dist with given boundary speeds.
 * Monotone in v_entry, v_exit, and dist: raising any of them raises v_peak.
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
    min_left_size = 1;
    max_safe_exit = 1e10f;
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
      float v_reachable = maxReachableSpeed(max_safe_entry[i + 1], mm[i], nominal[i], accel[i], jerk_max);
      max_safe_entry[i] = _MIN(v_reachable, vmax_junction[i]);
    }
    float left_entry_speed = traj.getExitSpeed();

    // Left-compatible group: cumulative superblock parameters for [0..i+1).
    //   cum_mm[i]             = sum(mm[0..i+1))            — superblock distance
    //   cum_min_a[i]          = max(accel[0..i+1))         — within CJP_MERGE_AMAX_RATIO of min
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
        cum_min_a[i] = new_a_max;
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
      const bool have_right = left_end < block_count;

      // Right superblock [left_end..right_end) parameters.
      // Only valid when have_right; guarded to avoid out-of-bounds reads.
      const float right_mm = have_right ? sumDist(mm, left_end, right_end) : 0;
      const float right_a = have_right ? minVal(accel, left_end, right_end) : 0;
      const float right_nominal = have_right ? nominal[left_end] : 0;

      // max_right_entry: max speed right superblock can accept and still
      // decelerate to max_safe_entry[right_end] over right_mm.
      // Conservative: uses min(accel) so any per-block pass would allow ≥ this.
      float max_right_entry;
      if (!have_right) {
        max_right_entry = 0;  // no right side → left must stop
      } else {
        const float v_reach = maxReachableSpeed(max_safe_entry[right_end], right_mm, right_nominal, right_a, jerk_max);
        max_right_entry = _MIN(v_reach, vmax_junction[left_end]);
      }

      // Left superblock [0..left_end): forward pass as single block
      float left_mm = cum_mm[left_end - 1];
      float left_a = cum_min_a[left_end - 1];
      float left_nominal = nominal[0];

      // max_left_exit: max speed left superblock can reach from left_entry_speed.
      // Capped by junction ceiling at left_end (entry of right/next group).
      float max_left_exit;
      {
        const float v_reach = maxReachableSpeed(left_entry_speed, left_mm, left_nominal, left_a, jerk_max);
        const float junction_cap = have_right ? vmax_junction[left_end] : 0;
        max_left_exit = _MIN(v_reach, junction_cap);
      }
      // Junction = min of what left can provide, what right can accept, and nominals.
      float v_junction_candidate = _MIN(max_left_exit, max_right_entry, left_nominal, right_nominal);

      // TODO: keep cache of max junction as long as left stays the same
      const float max_left_entry_from_candidate = maxReachableSpeed(v_junction_candidate, left_mm, left_nominal, left_a, jerk_max);
      bool valid_junction = left_entry_speed <= max_left_entry_from_candidate;

      // Validate right interior junctions:
      //   peakSpeed(v_junction, exit, right) ≤ min(vmax_junction[left_end+1..right_end))
      // Range [left_end+1, right_end) covers all interior block entries in right superblock.
      if (right_len > 1) {
        if (valid_junction) {
          float right_v_peak = peakSpeed(max_safe_entry[right_end], v_junction_candidate, right_a, jerk_max, right_mm, right_nominal);
          float right_min_internal_jv = minVal(vmax_junction, left_end + 1, right_end);
          valid_junction = right_v_peak <= right_min_internal_jv;
        }

        if (!valid_junction) {
          right_end = left_end + right_len / 2;  // split right, retry
          continue;
        }
      }

      // Validate left interior junctions:
      //   peakSpeed(left_entry, v_junction, left) ≤ min(vmax_junction[1..left_end))
      if (left_end > 1) {
        if (valid_junction) {
          float left_v_peak = peakSpeed(left_entry_speed, v_junction_candidate, left_a, jerk_max, left_mm, left_nominal);
          float left_min_jv = cum_vmax_junction[left_end - 1];
          valid_junction = left_v_peak <= left_min_jv;
        }

        if (!valid_junction){
          // Split left, but not below min_left_size — guarantee (1)
          if (left_end == min_left_size) {
            if (right_len > 1) {
              right_end = left_end + right_len / 2;  // try shrinking right first
              continue;
            }
            // Can't split either side. Fall back to max_safe_exit — guarantee (2):
            // previous cycle validated peak(V_j, max_safe_exit) ≤ interior junctions
            // for these same blocks, so max_safe_exit is known-safe.
            left_exit_speed = max_safe_exit;
            break;
          }
          right_end = left_end;
          left_end = _MAX(left_end / 2, min_left_size);
          continue;
        }
      }
      // Both superblocks pass interior junction checks
      left_exit_speed = v_junction_candidate;
      break;
    }


    // --- 5. Plan trajectory ---
    traj.plan_full(left_entry_speed, left_exit_speed, cum_min_a[left_end - 1], jerk_max, cum_mm[left_end - 1], nominal[0]);

    // Store right group state for next cycle's cross-cycle guarantees:
    //   min_left_size = right_len        → (1) preserves decel feasibility distance
    //   max_safe_exit = safe_entry[R]    → (2) known-valid exit for fallback
    min_left_size = right_len;
    max_safe_exit = max_safe_entry[right_end];

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
   * Max speed reachable from v_from over total_mm via jerk-limited ramp.
   * Monotone in v_from and total_mm: more of either → higher result.
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
  uint8_t group_buffer_consumed = 0;
  uint8_t min_left_size = 1;  // (1): previous right group size — floor for left splitting
  float max_safe_exit = 1e10f; // (2): previous right group exit — known-valid fallback
  float orig_block_start_dist = 0;
  float orig_block_end_dist = 0;
};
