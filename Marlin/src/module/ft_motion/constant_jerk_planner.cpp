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

  // Proposal B: carry both velocity and acceleration from previous block.
  // The previous exit state already happened — never cap or reset it.
  float v_left_entry = v_exit_stored;
  float a_left_entry = a_exit_stored;

  // Find max left-compatible extent: same nominal, accel ratio ≤ 1.1.
  uint8_t max_left_compatible = 1;
  {
    float a_min = accel[0], a_max_l = accel[0];
    for (uint8_t i = 1; i < block_count; i++) {
      if (nominal[i] != nominal[0]) break;
      float new_a_min = _MIN(a_min, accel[i]);
      float new_a_max = _MAX(a_max_l, accel[i]);
      if (new_a_max > new_a_min * CJP_MERGE_AMAX_RATIO) break;
      a_min = new_a_min;
      a_max_l = new_a_max;
      max_left_compatible++;
    }
  }

  // ─── Backward pass with integrated reuse check ───
  //
  // Compute max feasible entry speed for [i..N) assuming per-block (unmerged) planning.
  // NOT valid for merged trajectories — merging has more distance and can safely exceed these.
  //
  // When a previous trajectory is available for reuse, we check at the reuse junction
  // (i == reuse_left_end) whether the backward pass value has improved. If not, we can
  // stop the backward pass early and reuse the previous trajectory — skipping both the
  // remaining backward pass iterations AND the entire try_left_end loop.
  float max_safe_entry_to_unmerged_tail[BLOCK_BUFFER_SIZE + 1];
  max_safe_entry_to_unmerged_tail[block_count] = 0.0f;

  // Reuse candidate (0 = no reuse possible)
  uint8_t reuse_left_end = 0;
  if (prev_traj_state.valid && prev_left_end > 2
      && block_count >= prev_left_end - 1
      && prev_left_end - 1 >= max_left_compatible) {
    reuse_left_end = prev_left_end - 1;
  }

  bool reused = false;
  for (int8_t i = block_count - 1; i > 0; i--) {
    float safe;
    if (buffer_full && i == block_count - 1) {
      float cap = _MIN(nominal[i], vmax_junction[i]);
      safe = cj_maxSafeEntryForAnyExit(mm[i], cap, j_max, accel[i]);
    } else {
      safe = maxReachableSpeed(max_safe_entry_to_unmerged_tail[i + 1], mm[i], _MIN(nominal[i], vmax_junction[i]), accel[i], j_max);
    }
    max_safe_entry_to_unmerged_tail[i] = safe;

    // Check reuse at the junction point — stop backward pass early if unchanged
    if (reuse_left_end > 0 && i == (int8_t)reuse_left_end) {
      float v_junction_new = _MIN(safe, _MIN(nominal[0], vmax_junction[reuse_left_end]));
      if (v_junction_new <= prev_v_junction * 1.001f + 0.01f) {
        // Junction unchanged → reuse previous trajectory
        traj.restorePreTruncation(prev_traj_state);
        traj.advancePastDistance(prev_mm_block0);

        if (reuse_left_end > 1) {
          traj.savePreTruncation(prev_traj_state);
          prev_mm_block0 = mm[0];
        } else {
          prev_traj_state.valid = false;
        }

        traj.truncateToDistance(mm[0]);
        v_exit_stored = traj.getExitSpeed();
        a_exit_stored = traj.getExitAccel();

        prev_left_end = reuse_left_end;
        prev_v_junction = v_junction_new;

        orig_block_index = 0;
        orig_block_start_dist = 0;
        group_block_count = 1;
        group_buffer_consumed = buf_offset[0] + 1;
        orig_block_end_dist = mm[0];

        #ifdef CJ_DEBUG
          printf("  REUSE: left_end=%d v_junction=%.4f v_exit=%.4f a_exit=%.4f (stopped backward pass at i=%d of %d)\n",
                 reuse_left_end, v_junction_new, v_exit_stored, a_exit_stored, i, block_count);
        #endif
        reused = true;
        break;
      }
      #ifdef CJ_DEBUG
      else {
        printf("  reuse rejected at i=%d: v_junction_new=%.4f > prev_v_junction=%.4f * 1.001\n",
               i, v_junction_new, prev_v_junction);
      }
      #endif
      reuse_left_end = 0;  // Don't check again
    }
  }
  if (reused) return true;

  #ifdef CJ_DEBUG
    printf("  planNext: v_entry=%.4f a_entry=%.4f block_count=%d max_left=%d\n",
           v_left_entry, a_left_entry, block_count, max_left_compatible);
    printf("  backward pass max_safe_entry_to_unmerged_tail:");
    for (uint8_t i = 0; i <= block_count; i++) printf(" [%d]=%.2f", i, max_safe_entry_to_unmerged_tail[i]);
    printf("\n");
    printf("  blocks: ");
    for (uint8_t i = 0; i < block_count; i++) printf(" [%d]{mm=%.2f nom=%.0f amax=%.0f vj=%.0f}", i, mm[i], nominal[i], accel[i], vmax_junction[i]);
    printf("\n");
  #endif

  // ─── Left_end selection (largest feasible, backward pass only) ───
  //
  // For each candidate left_end, v_junction = max_safe_entry_to_unmerged_tail[left_end]
  // (backward pass with a=0 at boundaries). Plan the left S-curve and
  // check interior junctions against vmax_junction. Pick the LARGEST
  // feasible left_end — more blocks = more room for the S-curve = higher
  // v_peak in the early portion (block 0). Replanning each cycle from
  // actual (v, a) compensates for the conservative backward pass.

  // Helper: try a candidate left_end. Returns v_target on success, -1 on failure.
  // On success, traj is planned and ready.
  // If v_junction_override > 0, use it instead of the backward pass value.
  auto try_left_end = [&](uint8_t candidate, float v_junction_override = 0) -> float {
    float v_junction = v_junction_override > 0
      ? _MIN(v_junction_override, _MIN(nominal[0], vmax_junction[candidate]))
      : _MIN(max_safe_entry_to_unmerged_tail[candidate], _MIN(nominal[0], vmax_junction[candidate]));

    float dist_left = sumDist(mm, 0, candidate);
    float a_left = minVal(accel, 0, candidate);
    float v_target = maxReachableSpeed(v_left_entry, dist_left, v_junction, a_left, j_max, a_left_entry);

    #ifdef CJ_DEBUG
      printf("    try left_end=%d: v_junction=%.4f v_target=%.4f dist_left=%.4f a_left=%.0f\n",
             candidate, v_junction, v_target, dist_left, a_left);
    #endif

    // Plan the left S-curve
    bool ok = traj.plan_full(v_left_entry, v_target, a_left, j_max, dist_left,
                              nominal[0], a_left_entry);
    if (ok && candidate > 1) {
      // Merged trajectory: verify it actually reached v_target.
      // plan_full's can't-brake path returns true but exits at v >> v_target.
      // The backward pass guarantees v_target is safe for the unmerged tail
      // (assuming a=0 at the boundary). If the trajectory can't reach v_target,
      // the truncation at block 0 will produce a (v, a) state that the tail
      // can't handle. Reject and try a shorter merge.
      float v_actual_exit = traj.getVelocityAtTime(traj.getTotalDuration());
      if (v_actual_exit > v_target + 1.0f) {
        #ifdef CJ_DEBUG
          printf("    merge can't-brake: v_actual=%.4f > v_target=%.4f, rejecting candidate=%d\n",
                 v_actual_exit, v_target, candidate);
        #endif
        return -1;
      }
    }
    if (!ok) {
      if (candidate > 1) return -1;  // try shorter merge
      // Single block: can't reach v_target (S-curve decel asymmetry).
      float v_original_target = v_target;
      // Replan targeting v=0 — decel to 0 uses LESS distance, so it fits.
      // The planner's later truncation at mm[0] gives the best achievable exit.
      v_target = 0.0f;
      ok = traj.plan_full(v_left_entry, 0.0f, a_left, j_max, dist_left,
                           nominal[0], a_left_entry);
      if (!ok) {
        // plan_full(v=0) failed — block too short to fit even a full decel.
        // But with a_entry < 0, the +j absorption naturally exits at
        // v_absorbed = v_entry + a_entry*|a_entry|/(2*j). Try that directly.
        if (a_left_entry < 0.0f) {
          // The +j absorption naturally exits at v_absorbed after neutralizing a_entry.
          // Only use this if v_absorbed <= v_original_target (respects backward pass).
          // If v_absorbed > v_original_target, the absorption exits too fast for
          // the unmerged tail — don't use it.
          float v_absorbed = v_left_entry + a_left_entry * fabsf(a_left_entry) / (2.0f * j_max);
          // Allow tiny float-precision overshoot (0.01) but not more —
          // v_absorbed > v_original_target means the absorption exits too fast
          // for the unmerged tail.
          if (v_absorbed > 0.01f && v_absorbed <= v_original_target + 0.01f) {
            ok = traj.plan_full(v_left_entry, v_absorbed, a_left, j_max, dist_left,
                                 nominal[0], a_left_entry);
            if (ok) {
              v_target = v_absorbed;
              #ifdef CJ_DEBUG
                printf("    absorption exit: v_target=%.4f v_absorbed=%.4f (original=%.4f)\n",
                       v_target, v_absorbed, v_original_target);
              #endif
            }
          }
        }
        if (!ok) {
          // True last resort: plan_decel_only without distance constraint,
          // planner truncates to mm[0] afterward.
          traj.plan_decel_only(v_left_entry, a_left, j_max, a_left_entry);
          #ifdef CJ_DEBUG
            printf("    last-resort plan_decel_only: dur=%.6f dist=%.4f\n",
                   traj.getTotalDuration(), traj.getDistanceAtTime(traj.getTotalDuration()));
          #endif
          return 0.0f;
        }
      }

      // When a_entry < 0, plan_full(v_original_target) may fail but plan_full
      // succeeds for v_exit slightly above it (+j absorption uses less distance).
      // Bisect upward to find lowest feasible v_exit. Only when v=0 plan
      // under-exits (losing speed unnecessarily).
      if (a_left_entry < 0.0f && block_count > 1 && v_original_target > 0.01f) {
        float v_current_exit = traj.getVelocityAtTime(traj.getTotalDuration());
        float v_absorbed = v_left_entry + a_left_entry * fabsf(a_left_entry) / (2.0f * j_max);
        float v_limit = _MIN(nominal[0], vmax_junction[1]);
        float ve_hi = _MIN(v_absorbed, v_limit);
        if (ve_hi > v_original_target && ve_hi > 0.01f
            && v_current_exit < v_original_target - 0.01f) {
          float ve_lo = v_original_target, ve_best = 0.0f;
          for (int iter = 0; iter < 24; iter++) {
            float ve_mid = 0.5f * (ve_lo + ve_hi);
            if (traj.plan_full(v_left_entry, ve_mid, a_left, j_max, dist_left,
                                nominal[0], a_left_entry))
              { ve_best = ve_mid; ve_hi = ve_mid; }
            else
              ve_lo = ve_mid;
          }
          bool used = false;
          if (ve_best > 0.01f &&
              traj.plan_full(v_left_entry, ve_best, a_left, j_max, dist_left,
                              nominal[0], a_left_entry)) {
            float ve_actual = traj.getVelocityAtTime(traj.getTotalDuration());
            // Upward bisection finds ve_best ∈ [v_original_target, ve_hi].
            // ve_actual ≈ ve_best, so it's inherently above v_original_target.
            // Allow compound float imprecision (bisection + plan_full + query)
            // but not a real overshoot that would endanger the tail.
            if (ve_actual >= v_current_exit - 0.01f
                && ve_actual <= v_original_target + 0.05f) {
              v_target = ve_best;
              used = true;
              #ifdef CJ_DEBUG
                printf("    over-decel fix: v_target=%.4f exit=%.4f (was %.4f, v0_exit=%.4f)\n",
                       ve_best, ve_actual, v_original_target, v_current_exit);
              #endif
            }
          }
          if (!used) {
            // Restore v=0 plan (plan_full modifies state even on failure)
            traj.plan_full(v_left_entry, 0.0f, a_left, j_max, dist_left,
                            nominal[0], a_left_entry);
            v_target = 0.0f;
          }
        }
      }

      // Downward bisection: when plan_full(v_target) fails due to S-curve decel
      // asymmetry (decel to v>0 needs more distance than to v=0), find the highest
      // feasible v_exit in [0, v_original_target]. Works for any a_entry value.
      // Only triggers when v=0 plan under-exits (losing speed unnecessarily).
      if (v_target < 0.01f && block_count > 1 && v_original_target > 0.01f) {
        float v_current_exit = traj.getVelocityAtTime(traj.getTotalDuration());
        if (v_current_exit < v_original_target - 0.01f) {
          float ve_lo = 0.0f, ve_hi = v_original_target, ve_best = 0.0f;
          for (int iter = 0; iter < 24; iter++) {
            float ve_mid = 0.5f * (ve_lo + ve_hi);
            if (traj.plan_full(v_left_entry, ve_mid, a_left, j_max, dist_left,
                                nominal[0], a_left_entry))
              { ve_best = ve_mid; ve_lo = ve_mid; }
            else
              ve_hi = ve_mid;
          }
          bool used = false;
          if (ve_best > 0.01f &&
              traj.plan_full(v_left_entry, ve_best, a_left, j_max, dist_left,
                              nominal[0], a_left_entry)) {
            float ve_actual = traj.getVelocityAtTime(traj.getTotalDuration());
            float dur = traj.getTotalDuration();
            // Validate: exit must not have near-zero dip before the end
            float v_near = (dur > 0.001f) ? traj.getVelocityAtTime(dur - 0.001f) : ve_actual;
            // Downward bisection has ve_best ≤ v_original_target by construction.
            // Allow tiny float precision overshoot from plan_full.
            if (ve_actual > v_current_exit + 0.01f
                && ve_actual <= v_original_target + 0.01f
                && fabsf(v_near - ve_actual) < _MAX(0.2f, ve_actual * 0.5f)) {
              v_target = ve_best;
              used = true;
              #ifdef CJ_DEBUG
                printf("    downward bisect fix: v_target=%.4f exit=%.4f (was %.4f, v0_exit=%.4f)\n",
                       ve_best, ve_actual, v_original_target, v_current_exit);
              #endif
            }
          }
          if (!used) {
            // Restore v=0 plan (plan_full modifies state even on failure)
            traj.plan_full(v_left_entry, 0.0f, a_left, j_max, dist_left,
                            nominal[0], a_left_entry);
            v_target = 0.0f;
          }
        }
      }
    }

    // Check velocity at block 0 exit (the truncation point).
    // S-curve decel to v>0 can take MORE distance than to v=0, so plan_full
    // may truncate internally and exit faster than v_target.
    //
    // DO NOT add max_safe_entry_to_unmerged_tail[1] here. The block0 exit is part of a
    // truncated merged trajectory. On each subsequent cycle, as new blocks
    // arrive, the planner refines the trajectory with more lookahead and
    // can compute a higher safe exit. Capping at max_safe_entry_to_unmerged_tail[1] would
    // force every merged trajectory to exit at backward-pass speed, which
    // nullifies the entire merging algorithm — we'd end up with single
    // blocks at a=0 entry/exit. The whole point of merging is that the
    // merged trajectory has more distance available for decel than what
    // the per-block backward pass assumes.
    if (block_count > 1) {
      float v_at_block0 = (candidate > 1) ? traj.getVelocityAtDistance(mm[0])
                                            : traj.getVelocityAtTime(traj.getTotalDuration());
      float v_limit_block1 = _MIN(nominal[0], vmax_junction[1]);
      #ifdef CJ_DEBUG
        printf("    block0 exit check: v_at=%.4f v_limit=%.4f (vj=%.0f mse=%.2f buf_full=%d cand=%d) → %s\n",
               v_at_block0, v_limit_block1, vmax_junction[1], max_safe_entry_to_unmerged_tail[1], buffer_full, candidate,
               (v_at_block0 > v_limit_block1 + 0.01f) ? "REJECT" : "OK");
      #endif
      if (v_at_block0 > v_limit_block1 + 0.01f) {
        if (candidate == 1) {
          // Single block exit overshoots junction. Replan targeting v=0 —
          // decel to 0 uses less distance, so exit at mm[0] will be lower.
          traj.plan_full(v_left_entry, 0.0f, a_left, j_max, dist_left,
                          nominal[0], a_left_entry);
          #ifdef CJ_DEBUG
            printf("    replan v=0: exit=%.4f vs limit=%.4f\n",
                   traj.getVelocityAtTime(traj.getTotalDuration()), v_limit_block1);
          #endif
          return 0.0f;
        }
        return -1;
      }

    }

    // Check interior left junctions.
    // NOTE: only check vmax_junction here, NOT max_safe_entry_to_unmerged_tail. Interior
    // superblock velocities are allowed to exceed max_safe_entry_to_unmerged_tail — that's
    // the whole point of merging (more distance = higher v_peak).
    // max_safe_entry_to_unmerged_tail only constrains the v_junction TARGET of the merged
    // trajectory, not the intermediate truncated exits.
    if (candidate > 1) {
      float dist_cum = 0;
      for (uint8_t k = 0; k + 1 < candidate; k++) dist_cum += mm[k];
      for (uint8_t k = candidate - 1; k >= 1; k--) {
        float v_at = traj.getVelocityAtDistance(dist_cum);
        float v_limit = _MIN(nominal[k], vmax_junction[k]);
        #ifdef CJ_DEBUG
          printf("    interior left junction[%d]: v_at=%.4f v_limit=%.4f at dist=%.4f → %s\n",
                 k, v_at, v_limit, dist_cum, (v_at > v_limit + 0.01f) ? "REJECT" : "OK");
        #endif
        // Tolerance matches block0 exit check (line 176). Without it,
        // floating-point equality (e.g. v_at=27.0000 vs v_limit=27.0000)
        // rejects valid merges, forcing single-block fallback that
        // over-decelerates and causes unnecessary zero-speed touches.
        if (v_at > v_limit + 0.01f) {
          return -1;
        }
        dist_cum -= mm[k - 1];
      }
    }

    return v_target;
  };

  // Try from largest left_end downward — first feasible wins.
  // Larger left_end = more distance for the S-curve = higher v_peak at block 0.
  uint8_t best_left_end = 0;
  for (uint8_t c = max_left_compatible; c >= 1; c--) {
    float vt = try_left_end(c);
    if (vt >= 0) {
      best_left_end = c;
      break;
    }
  }

  // Fallback: the previous trajectory was heading toward prev_v_junction over
  // prev_left_end blocks. After consuming 1, prev_left_end - 1 blocks remain.
  // Re-targeting the same v_junction should be feasible since the previous
  // trajectory already proved it from the current (v, a).
  if (best_left_end == 0 && prev_left_end > 1) {
    uint8_t fallback = prev_left_end - 1;
    if (fallback <= block_count) {
      float vt = try_left_end(fallback, prev_v_junction);
      if (vt >= 0) best_left_end = fallback;
    }
  }

  #ifdef CJ_DEBUG
    printf("  → best_left_end=%d max_left_compatible=%d\n",
           best_left_end, max_left_compatible);
  #endif

  if (best_left_end == 0) {
    SERIAL_ECHOLNPGM("CJ ERROR: no feasible left_end, v=", v_left_entry, " a=", a_left_entry);
    traj.reset();
    return false;
  }

  prev_left_end = best_left_end;
  // Store the v_junction used for this plan so the next call can use it as fallback
  prev_v_junction = _MIN(max_safe_entry_to_unmerged_tail[best_left_end], _MIN(nominal[0], vmax_junction[best_left_end]));

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
    // Show what the merged trajectory predicts at block 0+1 boundary
    if (best_left_end > 1) {
      float d2 = mm[0] + mm[1];
      if (d2 <= pre_trunc_dist) {
        float v_at_d2 = traj.getVelocityAtDistance(d2);
        float t_at_d2 = traj.getTimeAtDistance(d2);
        float a_at_d2 = traj.getAccelerationAtTime(t_at_d2);
        float j_at_d2 = traj.getJerkAtTime(t_at_d2);
        printf("  merged predicts at block[0+1] (d=%.4f): v=%.4f a=%.4f j=%.1f\n",
               d2, v_at_d2, a_at_d2, j_at_d2);
      }
    }
  }
  #endif

  // Save pre-truncation trajectory for potential reuse in next planNext call.
  // Only save when merging (best_left_end > 1) — single blocks have nothing to reuse.
  if (best_left_end > 1) {
    traj.savePreTruncation(
      prev_traj_state);
    prev_mm_block0 = mm[0];
  } else {
    prev_traj_state.valid = false;
  }

  traj.truncateToDistance(mm[0]);

  v_exit_stored = traj.getExitSpeed();
  a_exit_stored = traj.getExitAccel();
  #ifdef CJ_DEBUG
    printf("  post-truncate: v_exit=%.4f a_exit=%.4f j_exit=%.1f\n",
           v_exit_stored, a_exit_stored, traj.getJerkAtTime(traj.getTotalDuration()));
  #endif

  // Execution tracking: always consume 1 block
  orig_block_index = 0;
  orig_block_start_dist = 0;
  group_block_count = 1;
  group_buffer_consumed = buf_offset[0] + 1;
  orig_block_end_dist = mm[0];

  return true;
}

float ConstantJerkBlockPlanner::maxReachableSpeed(float v_from, float dist_total,
                                                   float v_max, float a_max, float j_max,
                                                   float a_entry) {
  float v_trap = SQRT(v_from * v_from + 2.0f * a_max * dist_total);
  float hi = _MIN(v_max, v_trap);

  if (hi <= v_from) return hi;

  // ─── a_entry != 0: Newton + bisection using closed-form cj_rampDistWithA ───
  if (a_entry != 0.0f) {
    if (cj_rampDistWithA(v_from, hi, j_max, a_max, a_entry) <= dist_total) return hi;

    // Lower bound: minimum feasible v_peak for the accel ramp with a_entry.
    float v_lo = v_from + a_entry * fabsf(a_entry) / (2.0f * j_max);
    if (v_lo < 0.0f) v_lo = 0.0f;

    // Initial guess: midpoint (trapezoidal inverse is complex with a_entry)
    float v_peak = 0.5f * (v_lo + hi);

    // Bracketed Newton: maintains [v_lo, hi] bracket.
    // Uses Newton when step stays in bracket, bisects otherwise.
    for (int i = 0; i < 10; i++) {
      if (v_peak <= v_lo) v_peak = v_lo + 0.001f;
      const float f = cj_rampDistWithA(v_from, v_peak, j_max, a_max, a_entry) - dist_total;
      if (f <= 0.0f) v_lo = v_peak; else hi = v_peak;
      if (f <= 0.0f && f > -0.01f) break;
      if (hi - v_lo < 0.001f) break;
      const float fp = cj_rampDistWithADeriv(v_from, v_peak, j_max, a_max, a_entry);
      if (fp > 1e-10f) {
        v_peak -= f / fp;
        if (v_peak <= v_lo || v_peak >= hi) v_peak = 0.5f * (v_lo + hi);
      } else {
        v_peak = 0.5f * (v_lo + hi);
      }
    }
    return _MIN(v_lo, v_max);
  }

  // ─── a_entry == 0: fast path with closed-form cj_rampDist ───
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

  // Bracketed Newton: maintains [v_from, hi] bracket.
  float v_lo = v_from;
  for (int i = 0; i < 10; i++) {
    if (v_peak <= v_lo) v_peak = v_lo + 0.001f;
    const float f = cj_rampDist(v_from, v_peak, j_max, a_max) - dist_total;
    if (f <= 0.0f) v_lo = v_peak; else hi = v_peak;
    if (f <= 0.0f && f > -0.01f) break;
    if (hi - v_lo < 0.001f) break;
    const float fp = cj_rampDistDeriv(v_from, v_peak, j_max, a_max);
    if (fp > 1e-10f) {
      v_peak -= f / fp;
      if (v_peak <= v_lo || v_peak >= hi) v_peak = 0.5f * (v_lo + hi);
    } else {
      v_peak = 0.5f * (v_lo + hi);
    }
  }
  return v_lo;
}

#endif // FTM_CONSTANT_JERK
