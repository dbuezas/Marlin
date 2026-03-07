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

#if __has_include("../../inc/MarlinConfig.h")
  #include "../../inc/MarlinConfig.h"
#endif

#ifndef ENABLED
  #define ENABLED(V) true
#endif

#if ENABLED(FTM_CONSTANT_JERK)

#include "trajectory_constant_jerk.h"

bool ConstantJerkBlockPlanner::planNext(ConstantJerkTrajectoryGenerator& traj, float j_max) {
  float mm[BLOCK_BUFFER_SIZE];
  float nominal[BLOCK_BUFFER_SIZE];
  float accel[BLOCK_BUFFER_SIZE];
  float vmax_junction[BLOCK_BUFFER_SIZE + 1];  // max entry speed ceiling per block; [block_count]=0 (must stop)

  uint8_t block_count = 0;

  // Map from move-block index to buffer offset (for tracking consumed range)
  uint8_t buf_offset[BLOCK_BUFFER_SIZE]; // buf_offset[i] = buffer position of move block i

  // Look ahead at future blocks.
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
    SERIAL_ECHOLNPGM("CJ ERROR: block_count=", block_count);
    traj.reset();
    return false;
  }
  vmax_junction[block_count] = 0.0f;  // must stop after last block

  // Backward pass: max_safe_entry[i] = max feasible entry speed for [i..N)
  float max_safe_entry[BLOCK_BUFFER_SIZE + 1];
  max_safe_entry[block_count] = 0.0f;
  for (int8_t i = block_count - 1; i > 0; i--) {
    max_safe_entry[i] = maxReachableSpeed(max_safe_entry[i + 1], mm[i], _MIN(nominal[i], vmax_junction[i]), accel[i], j_max);
  }

  // Proposal B: carry both velocity and acceleration from previous block.
  // The previous exit state already happened — never cap or reset it.
  float v_left_entry = v_exit_stored;
  float a_left_entry = a_exit_stored;

  // Find max left-compatible extent: same nominal, accel ratio ≤ 1.1,
  // up to half the buffer when full, or all blocks when not full.
  // When the buffer isn't full, we've seen the end of the move queue,
  // so the left group can plan all the way to v_exit=0 without needing
  // a right group for braking.
  const uint8_t half = (block_count + 1) / 2;
  uint8_t max_left_compatible = 1;
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
      max_left_compatible++;
    }
  }

  #ifdef CJ_DEBUG
    printf("  planNext: v_entry=%.4f a_entry=%.4f block_count=%d max_left=%d prev_left=%d buffer_full=%d\n",
           v_left_entry, a_left_entry, block_count, max_left_compatible, prev_left_end, buffer_full);
    printf("  backward pass max_safe_entry:");
    for (uint8_t i = 0; i <= block_count; i++) printf(" [%d]=%.2f", i, max_safe_entry[i]);
    printf("\n");
    printf("  blocks: ");
    for (uint8_t i = 0; i < block_count; i++) printf(" [%d]{mm=%.2f nom=%.0f amax=%.0f vj=%.0f}", i, mm[i], nominal[i], accel[i], vmax_junction[i]);
    printf("\n");
  #endif

  // ─── Bottom-up left_end selection with right-side superblock braking ───
  //
  // For each candidate left_end, compute v_junction using the right-side
  // decel ramp (which allows a≠0 at interior boundaries). This replaces
  // the old v_cap = min(nominal, max_safe_entry[left_end]) which was too
  // conservative because the backward pass assumes a=0 at every boundary.

  // Helper: find right-compatible extent starting at left_end
  auto find_right_extent = [&](uint8_t le) -> uint8_t {
    if (le >= block_count) return le;
    uint8_t re = le + 1;
    float a_min_r = accel[le], a_max_r = accel[le];
    while (re < block_count) {
      if (nominal[re] != nominal[le]) break;
      float new_a_min = _MIN(a_min_r, accel[re]);
      float new_a_max = _MAX(a_max_r, accel[re]);
      if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
      a_min_r = new_a_min;
      a_max_r = new_a_max;
      re++;
    }
    return re;
  };

  // Helper: try a candidate left_end. Returns v_target on success, -1 on failure.
  // On success, traj is planned and ready.
  auto try_left_end = [&](uint8_t candidate) -> float {
    // Compute v_junction — the max safe exit speed for the left group.
    // Per prime directives: right-side superblock braking gives a higher
    // v_junction than the per-block backward pass (max_safe_entry) because
    // the right side is a continuous decel ramp with a≠0 at interior
    // boundaries. Use the MAX of backward pass and right-side braking,
    // capped by the geometric junction limit.
    float v_junction = max_safe_entry[candidate]; // floor
    {
      uint8_t right_end = find_right_extent(candidate);
      if (right_end > candidate) {
        float v_rside = maxSafeJunctionSpeed(mm, nominal, vmax_junction, accel,
                                              candidate, right_end,
                                              max_safe_entry[right_end], j_max);
        v_junction = _MAX(v_junction, v_rside);
      }
    }
    // Cap by geometric junction limit and nominal speed
    v_junction = _MIN(v_junction, _MIN(nominal[0], vmax_junction[candidate]));

    float dist_left = sumDist(mm, 0, candidate);
    float a_left = minVal(accel, 0, candidate);
    float v_target = maxReachableSpeed(v_left_entry, dist_left, v_junction, a_left, j_max);

    #ifdef CJ_DEBUG
    {
      uint8_t re_dbg = find_right_extent(candidate);
      float v_bpass = max_safe_entry[candidate];
      float v_rside_dbg = -1;
      if (re_dbg > candidate) {
        v_rside_dbg = maxSafeJunctionSpeed(mm, nominal, vmax_junction, accel,
                                            candidate, re_dbg,
                                            max_safe_entry[re_dbg], j_max);
      }
      printf("    try left_end=%d: right_end=%d v_bpass=%.4f v_rside=%.4f v_junction=%.4f v_target=%.4f dist_left=%.4f a_left=%.0f\n",
             candidate, re_dbg, v_bpass, v_rside_dbg, v_junction, v_target, dist_left, a_left);
    }
    #endif

    // Plan the left S-curve
    bool ok = traj.plan_full(v_left_entry, v_target, a_left, j_max, dist_left,
                              nominal[0], a_left_entry);
    if (!ok) return -1;

    // Check interior left junctions
    if (candidate > 1) {
      float dist_cum = 0;
      for (uint8_t k = 0; k + 1 < candidate; k++) dist_cum += mm[k];
      for (uint8_t k = candidate - 1; k >= 1; k--) {
        float v_at = traj.getVelocityAtDistance(dist_cum);
        float v_limit = _MIN(nominal[k], vmax_junction[k]);
        #ifdef CJ_DEBUG
          printf("    interior left junction[%d]: v_at=%.4f v_limit=%.4f at dist=%.4f → %s\n",
                 k, v_at, v_limit, dist_cum, (v_at > v_limit) ? "REJECT" : "OK");
        #endif
        if (v_at > v_limit) {
          return -1;
        }
        dist_cum -= mm[k - 1];
      }
    }

    return v_target;
  };

  // Try all feasible left_end candidates and pick the one with highest v_target.
  // Higher v_target = faster trajectory = better utilization of available distance.
  uint8_t best_left_end = 0;
  float best_v_target = -1;
  for (uint8_t c = 1; c <= max_left_compatible; c++) {
    float vt = try_left_end(c);
    if (vt >= 0 && vt > best_v_target) {
      best_left_end = c;
      best_v_target = vt;
    }
  }

  #ifdef CJ_DEBUG
    printf("  → best_left_end=%d best_v_target=%.4f max_left_compatible=%d\n",
           best_left_end, best_v_target, max_left_compatible);
  #endif

  // Phase 3: Handle left_end=0 (nothing feasible)
  if (best_left_end == 0) {
    // No feasible left group — use full_stop_fallback
    goto full_stop_fallback;
  }

  // Re-plan with best_left_end (may have been overwritten by later candidates)
  {
    if (try_left_end(best_left_end) < 0) {
      // Should not happen — it succeeded before. Fall back.
      goto full_stop_fallback;
    }
  }

  prev_left_end = best_left_end;
  prev_plan_blocks = best_left_end;

  if (false) {
    full_stop_fallback:
    // No feasible left group — plan a decel ramp from (v_entry, a_entry) to (v=0, a=0).
    // This continues the deceleration started by the previous cycle's superblock.
    // Use a_min over the blocks that the previous plan covered (prev_plan_blocks - 1
    // remaining after consuming 1 block), capped to what's currently visible.
    // This matches the a_min the original superblock used.
    cant_brake_count++;
    prev_left_end = 1;

    uint8_t a_span = _MIN(prev_plan_blocks > 0 ? prev_plan_blocks - 1 : block_count, block_count);
    if (a_span == 0) a_span = 1;
    float a_decel = minVal(accel, 0, a_span);

    // Plan a pure decel ramp using only the last 3 phases [-j, 0, +j].
    // This correctly handles a_entry on the decel side, unlike plan_full which
    // decomposes into accel+cruise+decel and loses the a_entry on the decel side.
    traj.plan_decel_only(v_left_entry, a_decel, j_max, a_left_entry);
    prev_plan_blocks = a_span;  // continue tracking the original plan's span
    #ifdef CJ_DEBUG
    {
      float d_plan = traj.getDistanceAtTime(traj.getTotalDuration());
      printf("  full_stop_fallback: planned decel from v=%.4f a=%.4f a_decel=%.4f a_span=%d prev_plan_blocks=%d over %.4f mm\n",
             v_left_entry, a_left_entry, a_decel, a_span, prev_plan_blocks, d_plan);
    }
    #endif
  }

  // Emit only 1 block: truncate trajectory to block 0's distance.
  // truncateToDistance updates v_exit and a_exit on the trajectory.
  // For normal single-block plans, dist_total == mm[0] so this is a no-op.
  // For multi-block superblocks or can't-brake decel, it cuts to block 0.
  #ifdef CJ_DEBUG
  {
    float pre_trunc_dur = traj.getTotalDuration();
    float pre_trunc_dist = traj.getDistanceAtTime(pre_trunc_dur);
    float pre_trunc_v_exit = traj.getVelocityAtTime(pre_trunc_dur);
    printf("  pre-truncate: dist=%.4f v_exit=%.4f duration=%.6f, truncating to %.4f\n",
           pre_trunc_dist, pre_trunc_v_exit, pre_trunc_dur, mm[0]);
  }
  #endif
  traj.truncateToDistance(mm[0]);

  v_exit_stored = traj.getExitSpeed();
  a_exit_stored = traj.getExitAccel();
  #ifdef CJ_DEBUG
    printf("  post-truncate: v_exit=%.4f a_exit=%.4f\n", v_exit_stored, a_exit_stored);
  #endif

  // Execution tracking: always consume 1 block
  orig_block_index = 0;
  orig_block_start_dist = 0;
  group_block_count = 1;
  group_buffer_consumed = buf_offset[0] + 1;
  orig_block_end_dist = mm[0];

  return true;
}

float ConstantJerkBlockPlanner::maxSafeJunctionSpeed(
    const float* mm, const float* nominal, const float* vmax_junction,
    const float* accel, uint8_t left_end, uint8_t right_end,
    float v_exit_right, float j_max) {

  if (right_end <= left_end) return v_exit_right;

  float dist_right = sumDist(mm, left_end, right_end);
  float a_right = minVal(accel, left_end, right_end);
  // Cap by both nominal and geometric junction limit at the boundary
  float v_geo_cap = _MIN(nominal[left_end], vmax_junction[left_end]);

  // Max junction speed from braking distance alone
  float v_hi = maxReachableSpeed(v_exit_right, dist_right, v_geo_cap, a_right, j_max);

  if (v_hi <= v_exit_right + 0.001f) return v_hi;

  // Check interior junctions at v_hi (common fast path)
  uint8_t n_interior = right_end - left_end - 1;
  if (n_interior == 0) return v_hi; // single-block right group, no interior junctions

  auto check_interior = [&](float v_junc) -> bool {
    float d_cum = 0;
    for (uint8_t k = 0; k < n_interior; k++) {
      d_cum += mm[left_end + k];
      float v_at_k = cj_decelVelocityAtDistance(v_junc, v_exit_right, j_max, a_right, d_cum);
      float limit = vmax_junction[left_end + k + 1];
      if (v_at_k > limit) return false;
    }
    return true;
  };

  if (check_interior(v_hi)) return v_hi;

  // Binary search: find max v_junction where all interior junctions pass
  float v_lo = v_exit_right;
  for (int i = 0; i < 20; i++) {
    float v_mid = 0.5f * (v_lo + v_hi);
    if (check_interior(v_mid))
      v_lo = v_mid;
    else
      v_hi = v_mid;
    if (v_hi - v_lo < 0.01f) break;
  }
  return v_lo;
}

float ConstantJerkBlockPlanner::maxReachableSpeed(float v_from, float dist_total,
                                                   float v_max, float a_max, float j_max) {
  float v_trap = SQRT(v_from * v_from + 2.0f * a_max * dist_total);
  float hi = _MIN(v_max, v_trap);

  if (hi <= v_from) return hi;
  if (cj_rampDist(v_from, hi, j_max, a_max) <= dist_total) return hi;

  // Initial guess: trapezoidal quadratic inverse
  //   dv^2 + B*dv + C = 0  where B = 2*v + am^2/j, C = 2*v*am^2/j - 2*s*am
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

float ConstantJerkBlockPlanner::minReachableSpeed(float v_from, float dist_total,
                                                    float a_max, float j_max) {
  if (v_from <= 0.0f || dist_total <= 0.0f) return v_from;

  // Tolerance: cj_rampDist has float precision ~0.001mm. Without tolerance,
  // borderline cases trigger the binary search which finds the WRONG (right)
  // intersection of the non-monotone ramp distance function, returning a
  // drastically overestimated minimum speed.
  if (cj_rampDist(0, v_from, j_max, a_max) <= dist_total + 0.01f) return 0;

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

#endif // FTM_CONSTANT_JERK
