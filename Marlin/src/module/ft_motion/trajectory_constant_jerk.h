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

#include "constant_jerk_planner.h"
#include "trajectory_generator.h"
#include <math.h>

/**
 * Constant-jerk (7-phase S-curve) trajectory generator.
 *
 * Phases: [+jerk, cruise_accel, -jerk, cruise_velocity, -jerk, cruise_decel, +jerk]
 * All junctions have a=0 (no non-zero boundary accelerations).
 * Uses binary search to find feasible peak velocity.
 */

// Simulate one phase of motion with constant jerk
static inline void cj_simulatePhase(float j_phase, float dt, float &v, float &a, float &dist) {
  if (dt <= 0.0f) return;
  dist += v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * j_phase * dt * dt * dt;
  v += a * dt + 0.5f * j_phase * dt * dt;
  a += j_phase * dt;
}

// Plan a 3-phase ramp between v_start and v_peak.
// Returns the distance consumed.
// a_start: initial acceleration (default 0). Only used for accel ramps (decel=false).
//   For accel: phases are [+j from a_start to a_peak, hold at a_peak, -j from a_peak to 0]
//   For decel: a_start is ignored (decel always starts from a=0 at v_peak)
static inline float cj_planRamp(float v_start, float v_peak, float j_max, float a_max,
                                bool decel, float &dt_jerk1, float &dt_hold, float &dt_jerk2,
                                float a_start = 0.0f) {
  float dv = v_peak - v_start;
  float a_peak_sq = j_max * dv + 0.5f * a_start * a_start;
  if (a_peak_sq < 0) {
    if (a_start < 0.0f) {
      // Float precision: a_start < 0 and v_peak ≈ v_start - a_start²/(2j).
      // a_peak_sq ≈ 0 but slightly negative. Clamp to degenerate case a_peak = 0:
      // the ramp just absorbs the existing negative acceleration.
      a_peak_sq = 0;
    } else {
      dt_jerk1 = dt_hold = dt_jerk2 = 0;
      return 0;
    }
  }
  float a_peak = SQRT(a_peak_sq);

  if (a_peak <= a_max) {
    dt_jerk1 = (a_peak - a_start) / j_max;
    dt_hold = 0;
    dt_jerk2 = a_peak / j_max;
  }
  else {
    dt_jerk1 = (a_max - a_start) / j_max;
    dt_jerk2 = a_max / j_max;
    float dv_no_hold = (2.0f * a_max * a_max - a_start * a_start) / (2.0f * j_max);
    dt_hold = _MAX(0.0f, (dv - dv_no_hold) / a_max);
  }

  float jk = decel ? -j_max : j_max;
  float v = decel ? v_peak : v_start;
  float a_v = decel ? 0.0f : a_start;
  float dist = 0;
  cj_simulatePhase(jk, dt_jerk1, v, a_v, dist);
  cj_simulatePhase(0, dt_hold, v, a_v, dist);
  cj_simulatePhase(-jk, dt_jerk2, v, a_v, dist);
  return dist;
}

// Plan a 3-phase decel ramp from (v_start, a_start) to (v_end, 0).
// Phases: [-j, 0, +j]. a_start <= 0 (already decelerating) or 0.
// Returns distance consumed. Sets dt_jerk1 (phase4), dt_hold (phase5), dt_jerk2 (phase6).
static inline float cj_planDecelRampWithA(
    float v_start, float v_end, float j_max, float a_max,
    float &dt_jerk1, float &dt_hold, float &dt_jerk2,
    float a_start = 0.0f) {
  const float dv = v_start - v_end;  // total velocity to shed
  // a_peak² = (a_start² + 2·j·dv) / 2
  float a_peak_sq = (a_start * a_start + 2.0f * j_max * dv) / 2.0f;
  if (a_peak_sq < 0.0f) a_peak_sq = 0.0f;
  float a_peak = SQRT(a_peak_sq);

  if (a_peak <= a_max) {
    // Triangular: no hold phase
    dt_jerk1 = (a_start + a_peak) / j_max;
    dt_hold = 0.0f;
    dt_jerk2 = a_peak / j_max;
  } else {
    // Trapezoidal: a_peak clamped to a_max
    a_peak = a_max;
    dt_jerk1 = (a_start + a_max) / j_max;
    dt_jerk2 = a_max / j_max;
    float dv_no_hold = (a_start * a_start - 2.0f * a_max * a_max) / (2.0f * j_max);
    float dv_hold = -dv - dv_no_hold;  // remaining dv for hold phase
    dt_hold = _MAX(0.0f, -dv_hold / a_max);
  }

  if (dt_jerk1 < 0.0f) dt_jerk1 = 0.0f;

  // Simulate to compute distance
  float v = v_start, a = a_start, dist = 0.0f;
  cj_simulatePhase(-j_max, dt_jerk1, v, a, dist);
  cj_simulatePhase(0,      dt_hold,  v, a, dist);
  cj_simulatePhase(j_max,  dt_jerk2, v, a, dist);
  return dist;
}

// Symmetric total ramp distance
static inline float cj_totalRampDist(float v_peak, float v_small, float v_large,
                                     float j_max, float a_max) {
  float dt_jerk1, dt_hold, dt_jerk2;
  float dist_accel = cj_planRamp(v_small, v_peak, j_max, a_max, false, dt_jerk1, dt_hold, dt_jerk2);
  float dist_decel = cj_planRamp(v_large, v_peak, j_max, a_max, true, dt_jerk1, dt_hold, dt_jerk2);
  return dist_accel + dist_decel;
}

// Closed-form ramp distance (O(1), no simulation).
// Equivalent to cj_planRamp but without computing phase durations.
//   Triangular (j*dv ≤ a_max²):  s = (v_s + v_p) * sqrt(dv / j)
//   Trapezoidal (j*dv > a_max²): s = (v_s + v_p) / 2 * (a_m/j + dv/a_m)
static inline float cj_rampDist(float v_start, float v_peak, float j_max, float a_max) {
  const float dv = v_peak - v_start;
  if (dv <= 0.0f) return 0.0f;
  if (j_max * dv <= a_max * a_max)
    return (v_start + v_peak) * SQRT(dv / j_max);
  else
    return (v_start + v_peak) * 0.5f * (a_max / j_max + dv / a_max);
}

// ds/d(v_peak) of cj_rampDist (v_start fixed, v_peak varies).
//   Triangular:  (3*v_p - v_s) / (2*sqrt(j*dv))
//   Trapezoidal: 0.5*(a_m/j + 2*v_p/a_m)
static inline float cj_rampDistDeriv(float v_start, float v_peak, float j_max, float a_max) {
  const float dv = v_peak - v_start;
  if (dv <= 0.0f) return 0.0f;
  if (j_max * dv <= a_max * a_max)
    return (3.0f * v_peak - v_start) / (2.0f * SQRT(j_max * dv));
  else
    return 0.5f * (a_max / j_max + 2.0f * v_peak / a_max);
}

// Distance from start of a 3-phase decel ramp (v_entry → v_exit) at which velocity = v_target.
// Phase boundaries (dist_endA, v_endA, dist_endB, v_endB) and dist_ramp_total precomputed.
// Requires: v_exit ≤ v_target ≤ v_entry.
static inline float cj_decelRampDistAtVelocity(
    float v_entry, float v_exit, float v_target,
    float j_max, float a_max,
    float dist_endA, float v_endA, float dist_endB, float v_endB,
    float dist_ramp_total) {
  if (v_target >= v_endA) {
    // Phase A: jerk=-j_max, a goes 0→-a_peak, v goes v_entry→v_endA
    // dist(v) = (2*v_entry + v) / 3 * sqrt(2*(v_entry - v) / j_max)
    const float dv = v_entry - v_target;
    return (dv <= 0.0f) ? 0.0f : (2.0f * v_entry + v_target) / 3.0f * SQRT(2.0f * dv / j_max);
  }
  if (v_target >= v_endB) {
    // Phase B: constant accel = -a_max, v goes v_endA→v_endB
    // dist(v) = dist_endA + (v_endA² - v²) / (2 * a_max)
    return dist_endA + (v_endA * v_endA - v_target * v_target) / (2.0f * a_max);
  }
  // Phase C: jerk=+j_max, a goes -a_peak→0, v goes v_endB→v_exit
  // By time-reversal from v_exit: dist_from_end = (2*v_exit + v)/3 * sqrt(2*(v - v_exit)/j_max)
  const float dv = v_target - v_exit;
  const float dist_from_end = (dv <= 0.0f) ? 0.0f : (2.0f * v_exit + v_target) / 3.0f * SQRT(2.0f * dv / j_max);
  return dist_ramp_total - dist_from_end;
}

// Velocity at distance d along a 3-phase decel ramp from v_entry to v_exit.
// The ramp has phases: [-j, 0, +j] (decel S-curve starting from a=0).
// d is measured from the start of the ramp.
// Returns v_entry if d <= 0, v_exit if d >= total ramp distance.
static inline float cj_decelVelocityAtDistance(
    float v_entry, float v_exit, float j_max, float a_max, float d) {
  if (d <= 0.0f) return v_entry;
  if (v_entry <= v_exit) return v_entry; // no decel needed

  // Plan the decel ramp to get phase durations
  float dt1, dt2, dt3;
  float dist_total = cj_planRamp(v_exit, v_entry, j_max, a_max, true, dt1, dt2, dt3);
  if (d >= dist_total) return v_exit;

  // Simulate through phases to find phase boundaries
  // Decel ramp: phase A = jerk -j_max, phase B = const accel, phase C = jerk +j_max
  // Starting from v_entry, a=0
  float v = v_entry, a = 0.0f, s = 0.0f;

  // Phase A: jerk = -j_max, duration dt1
  {
    float s_start = s;
    float v_start = v;
    float a_start = a;
    float jk = -j_max;
    float s_end = s_start + v_start * dt1 + 0.5f * a_start * dt1 * dt1
                  + (1.0f / 6.0f) * jk * dt1 * dt1 * dt1;
    if (d <= s_end && dt1 > 0.0f) {
      // Target is in phase A: solve s(t) = d for t, then compute v(t)
      // s(t) = s_start + v_start*t + 0.5*a_start*t² + (jk/6)*t³
      // Newton/bisection hybrid
      float t_lo = 0.0f, t_hi = dt1;
      float t = dt1 * ((d - s_start) / _MAX(s_end - s_start, 1e-12f));
      t = _MAX(t_lo, _MIN(t, t_hi));
      for (int i = 0; i < 16; i++) {
        float ft = s_start + v_start * t + 0.5f * a_start * t * t
                   + (1.0f / 6.0f) * jk * t * t * t - d;
        if (ft < 0.0f) t_lo = t; else t_hi = t;
        if (t_hi - t_lo < 1e-7f) break;
        float fp = v_start + a_start * t + 0.5f * jk * t * t;
        if (fp > 1e-6f) {
          t -= ft / fp;
          if (t < t_lo || t > t_hi) t = 0.5f * (t_lo + t_hi);
        } else {
          t = 0.5f * (t_lo + t_hi);
        }
      }
      return v_start + a_start * t + 0.5f * jk * t * t;
    }
    cj_simulatePhase(jk, dt1, v, a, s);
  }

  // Phase B: jerk = 0 (constant accel), duration dt2
  {
    float s_start = s;
    float v_start = v;
    float a_start = a;
    float s_end = s_start + v_start * dt2 + 0.5f * a_start * dt2 * dt2;
    if (d <= s_end && dt2 > 0.0f) {
      // Quadratic: s_start + v_start*t + 0.5*a_start*t² = d
      float ds = d - s_start;
      if (a_start == 0.0f) return v_start;
      float disc = v_start * v_start + 2.0f * a_start * ds;
      float t = (-v_start + SQRT(_MAX(0.0f, disc))) / a_start;
      t = _MAX(0.0f, _MIN(t, dt2));
      return v_start + a_start * t;
    }
    cj_simulatePhase(0, dt2, v, a, s);
  }

  // Phase C: jerk = +j_max, duration dt3
  {
    float s_start = s;
    float v_start = v;
    float a_start = a;
    float jk = j_max;
    // Same Newton/bisection as phase A
    float t_lo = 0.0f, t_hi = dt3;
    float s_end = s_start + v_start * dt3 + 0.5f * a_start * dt3 * dt3
                  + (1.0f / 6.0f) * jk * dt3 * dt3 * dt3;
    float t = dt3 * ((d - s_start) / _MAX(s_end - s_start, 1e-12f));
    t = _MAX(t_lo, _MIN(t, t_hi));
    for (int i = 0; i < 16; i++) {
      float ft = s_start + v_start * t + 0.5f * a_start * t * t
                 + (1.0f / 6.0f) * jk * t * t * t - d;
      if (ft < 0.0f) t_lo = t; else t_hi = t;
      if (t_hi - t_lo < 1e-7f) break;
      float fp = v_start + a_start * t + 0.5f * jk * t * t;
      if (fp > 1e-6f) {
        t -= ft / fp;
        if (t < t_lo || t > t_hi) t = 0.5f * (t_lo + t_hi);
      } else {
        t = 0.5f * (t_lo + t_hi);
      }
    }
    return v_start + a_start * t + 0.5f * jk * t * t;
  }
}

class ConstantJerkTrajectoryGenerator : public TrajectoryGenerator {
public:
  ConstantJerkTrajectoryGenerator() = default;

  // plan() delegates to the block planner. Defined in trajectory_constant_jerk.cpp.
  void plan(const float, const float, const float, const float, const float) override;

  // planRunout() resets planner sub-block state, then plans a zero-speed cruise.
  // Defined in trajectory_constant_jerk.cpp.
  void planRunout(const float duration) override;

  void setJerkMaxPtr(float* ptr) { jerk_max_ptr_ = ptr; }

  // Convenience: delegates to planner_.planNext(*this, j_max).
  // Defined in trajectory_constant_jerk.cpp.
  bool planNext(float j_max);

  // Plan a pure decel ramp from (v_entry, a_entry) to (v=0, a=0).
  // Uses only phases 4-6 [-j, 0, +j]; phases 0-3 are set to zero.
  // This is used by full_stop_fallback when continuing a deceleration that was
  // already in progress (a_entry may be negative).
  void plan_decel_only(float v_entry_in, float a_max_in, float j_max_in,
                       float a_entry_in = 0.0f) {
    v_entry = v_entry_in;
    v_exit = 0.0f;
    a_max = a_max_in;
    j_max = j_max_in;
    a_entry = a_entry_in;
    a_exit = 0.0f;

    float t5, t6, t7;
    dist_total = cj_planDecelRampWithA(v_entry, 0.0f, j_max, a_max, t5, t6, t7, a_entry);

    phase_dt[0] = phase_dt[1] = phase_dt[2] = phase_dt[3] = 0.0f;
    phase_dt[4] = t5;
    phase_dt[5] = t6;
    phase_dt[6] = t7;

    total_duration = t5 + t6 + t7;
    buildPhaseCache();
  }

  // Plan with explicit jerk and a_max (used by the block merging planner).
  // Jerk comes from cfg.jerk_max, passed through by the caller.
  // Returns false if infeasible (ramp between v0 and v1 exceeds distance).
  // a_entry_in: initial acceleration at v_entry (default 0). Non-zero when
  //   replanning mid-superblock with Proposal B (carry exit accel from previous block).
  bool plan_full(float v_entry_in, float v_exit_in,
                 float a_max_in, float j_max_in,
                 float dist_total_in, float v_nominal,
                 float a_entry_in = 0.0f) {
    v_entry = v_entry_in;
    v_exit = v_exit_in;
    a_max = a_max_in;
    j_max = j_max_in;
    dist_total = dist_total_in;
    a_entry = a_entry_in;
    a_exit = 0.0f;  // full S-curve always exits at a=0

    // ─── Decel-side fast path ───
    // When a_entry < 0 and v_exit <= v_entry, try placing a_entry on the decel
    // side directly. This avoids the velocity dip from the accel-side approach
    // (which applies +jerk to zero out a_entry before re-accelerating).
    if (a_entry < 0.0f && v_exit <= v_entry) {
      float dt5, dt6, dt7;
      float d_decel = cj_planDecelRampWithA(v_entry, v_exit, j_max, a_max, dt5, dt6, dt7, a_entry);
      // Use decel-side only when the ramp fits closely:
      // - Exact fit or slight overshoot only. Large overshoot + truncation would
      //   leave high exit velocity, breaking the planner's braking invariants.
      // - When d_decel << dist_total, can't add cruise (a≠0), so fall through.
      if (d_decel > 0.0f && d_decel >= dist_total - 0.01f && d_decel <= dist_total + 0.1f) {
        phase_dt[0] = phase_dt[1] = phase_dt[2] = phase_dt[3] = 0.0f;
        phase_dt[4] = dt5; phase_dt[5] = dt6; phase_dt[6] = dt7;
        total_duration = dt5 + dt6 + dt7;
        dist_total = d_decel;
        buildPhaseCache();
        if (d_decel > dist_total_in) {
          truncateToDistance(dist_total_in);
        }
        return true;
      }
      // d_decel < dist_total: can't add cruise at non-zero accel. Fall through
      // to accel-side approach which absorbs a_entry, cruises at v_valley, then decels.
    }

    // ─── Accel-side approach (standard) ───
    const float v_large = _MAX(v_entry, v_exit);

    // Minimum v_peak for the accel ramp with a_start = a_entry:
    //   a_entry > 0: need a_peak >= a_entry, so dv >= a_entry²/(2j), v_peak >= v_entry + a²/(2j)
    //   a_entry < 0: need a_peak² >= 0, so dv >= -a_entry²/(2j), v_peak >= v_entry - a²/(2j)
    // The accel ramp with a_entry < 0 naturally DECREASES velocity (a < 0 during the
    // continuation phase), so v_peak below v_entry is physically valid.
    const float v_peak_min_for_a = v_entry + a_entry * fabsf(a_entry) / (2.0f * j_max);

    // When a_entry < 0, v_peak can be below v_entry (accel ramp decreases velocity).
    // Only hard constraint: v_peak >= v_exit (decel ramp needs v_peak above v_exit).
    const float v_hard_floor = (a_entry >= 0.0f) ? v_large : v_exit;
    const float v_peak_floor = _MAX(v_hard_floor, v_peak_min_for_a);

    // Compute ramp distances using a_entry for accel side
    auto totalRampDistWithAEntry = [&](float v_peak_test) -> float {
      float dt1, dt2, dt3;
      float d_accel = cj_planRamp(v_entry, v_peak_test, j_max, a_max, false, dt1, dt2, dt3, a_entry);
      float d_decel = cj_planRamp(v_exit, v_peak_test, j_max, a_max, true, dt1, dt2, dt3);
      return d_accel + d_decel;
    };

    float v_peak = v_nominal;
    float min_dist_at_nominal = totalRampDistWithAEntry(v_nominal);
    if (min_dist_at_nominal > dist_total) {
      float v_peak_max = v_nominal;
      float v_peak_min = v_peak_floor;

      float minmimum_distance = totalRampDistWithAEntry(v_peak_floor);
      if (minmimum_distance > dist_total) {
        SERIAL_ECHOLNPGM("CJ ERROR: infeasible target:", dist_total,
          " minmimum_distance:", minmimum_distance,
          " v_entry:", v_entry,
          " v_exit:", v_exit,
          " j_max:", j_max,
          " a_max:", a_max
        );
        return false;

      } else if (dist_total - minmimum_distance > 0.001f) {
        for (int i = 0; i < 48; i++) {
          float v_mid = 0.5f * (v_peak_min + v_peak_max);
          float dist_mid = totalRampDistWithAEntry(v_mid);
          float overshoot = dist_mid - dist_total;
          if (overshoot > 0) {
            v_peak_max = v_mid;
          } else {
            v_peak_min = v_mid;
            if (-overshoot <= 0.001f) {
              break;
            }
          }
        }
      }
      v_peak = v_peak_min;
    }

    float t1, t2, t3, t4 = 0, t5, t6, t7;
    float dist_accel = cj_planRamp(v_entry, v_peak, j_max, a_max, false, t1, t2, t3, a_entry);
    float dist_decel = cj_planRamp(v_exit, v_peak, j_max, a_max, true, t5, t6, t7);

    float dist_ramps = dist_accel + dist_decel;
    const float requested_dist = dist_total;
    if (dist_ramps > dist_total) {
      // Ramps exceed requested distance (borderline infeasibility with a_entry).
      // Build the over-long trajectory, then truncate to fit.
      dist_total = dist_ramps;
    } else if (v_peak > 0.0f && dist_total > dist_ramps) {
      t4 = (dist_total - dist_ramps) / v_peak;
    }

    phase_dt[0] = t1; phase_dt[1] = t2; phase_dt[2] = t3;
    phase_dt[3] = t4;
    phase_dt[4] = t5; phase_dt[5] = t6; phase_dt[6] = t7;

    total_duration = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    buildPhaseCache();
    if (dist_total > requested_dist) {
      truncateToDistance(requested_dist);
    }
    return true;
  }

  // Truncate the trajectory to cover only the first `d` mm of distance.
  // After truncation, getTotalDuration() returns the time to reach `d`,
  // and v_exit/a_exit reflect the state at that point.
  // The phase cache remains valid — we just stop sampling earlier.
  // Used by the block planner to emit only block 0 of a multi-block superblock.
  void truncateToDistance(float d) {
    if (d >= dist_total) return;
    const float t = getTimeAtDistance(d);
    // Query v and a from phase cache (before modifying total_duration)
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    v_exit = phase_start_v[ph] + phase_start_a[ph] * dt + 0.5f * phaseJerk(ph) * dt * dt;
    a_exit = phase_start_a[ph] + phaseJerk(ph) * dt;
    dist_total = d;
    total_duration = t;
  }

  float getDistanceAtTime(const float t) const override {
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return dist_total;
    return rawDistanceAtTime(t);
  }

  float getTotalDuration() const override { return total_duration; }

  float getVelocityAtTime(const float t) const {
    if (t <= 0.0f) return v_entry;
    if (t >= total_duration) return v_exit;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    return phase_start_v[ph] + phase_start_a[ph] * dt + 0.5f * phaseJerk(ph) * dt * dt;
  }

  float getAccelerationAtTime(const float t) const {
    if (t <= 0.0f) return a_entry;
    if (t >= total_duration) return a_exit;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    return phase_start_a[ph] + phaseJerk(ph) * dt;
  }

  float getJerkAtTime(const float t) const {
    if (t <= 0.0f || t >= total_duration) return 0.0f;
    const int ph = findPhase(t);
    return phaseJerk(ph);
  }

  // Get velocity at a given distance along the trajectory.
  // Uses Newton's method for jerk phases, quadratic formula for constant-accel phases.
  float getVelocityAtDistance(const float d) const {
    if (d <= 0.0f) return v_entry;
    if (d >= dist_total) return v_exit;
    const int ph = findPhaseByDist(d);
    const float delta_s = d - phase_start_pos[ph];
    const float v_ph = phase_start_v[ph];
    const float a_ph = phase_start_a[ph];
    const float jk = phaseJerk(ph);

    if (jk == 0.0f) {
      // Constant accel: delta_s = v0*t + 0.5*a*t²
      if (a_ph == 0.0f) return v_ph;  // cruise phase
      // t = (-v0 + sqrt(v0² + 2*a*delta_s)) / a
      const float disc = v_ph * v_ph + 2.0f * a_ph * delta_s;
      const float t = (-v_ph + SQRT(_MAX(0.0f, disc))) / a_ph;
      return v_ph + a_ph * t;
    }

    // Jerk phase: find t such that s(t) = delta_s, then return v(t).
    // s(t) = v0*t + 0.5*a0*t² + (jk/6)*t³
    // Hybrid Newton/bisection: Newton when f' is healthy, bisection when f'≈0.
    const float jk6 = (1.0f / 6.0f) * jk;
    const float ph_end_pos = (ph < 6) ? phase_start_pos[ph + 1] : dist_total;
    const float phase_dist = ph_end_pos - phase_start_pos[ph];
    float t_lo = 0.0f, t_hi = phase_dt[ph];
    float t = (phase_dist > 0.0f) ? phase_dt[ph] * (delta_s / phase_dist) : 0.0f;
    for (int i = 0; i < 16; i++) {
      const float f = v_ph * t + 0.5f * a_ph * t * t + jk6 * t * t * t - delta_s;
      // Update bracket
      if (f < 0.0f) t_lo = t; else t_hi = t;
      if (t_hi - t_lo < 1e-7f) break;
      // f' = velocity at t, always ≥ 0
      const float fp = v_ph + a_ph * t + 0.5f * jk * t * t;
      if (fp > 1e-6f) {
        t -= f / fp;                                  // Newton step
        if (t < t_lo || t > t_hi) t = 0.5f * (t_lo + t_hi); // escaped bracket → bisect
      } else {
        t = 0.5f * (t_lo + t_hi);                    // f'≈0 → bisect
      }
    }
    return v_ph + a_ph * t + 0.5f * jk * t * t;
  }

  float getExitSpeed() const { return v_exit; }
  float getExitAccel() const { return a_exit; }
  float getEntryAccel() const { return a_entry; }

  // Find time at which the trajectory reaches distance d.
  // Bisection on rawDistanceAtTime.
  float getTimeAtDistance(float d) const {
    if (d <= 0.0f) return 0.0f;
    if (d >= dist_total) return total_duration;
    float lo = 0.0f, hi = total_duration;
    for (int i = 0; i < 32; i++) {
      const float mid = (lo + hi) * 0.5f;
      if (rawDistanceAtTime(mid) < d) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  void reset() override {
    v_entry = v_exit = 0.0f;
    a_entry = a_exit = 0.0f;
    a_max = j_max = dist_total = 0.0f;
    for (int i = 0; i < 7; ++i) {
      phase_dt[i] = 0.0f;
      phase_start_time[i] = 0.0f;
      phase_start_pos[i] = 0.0f;
      phase_start_v[i] = 0.0f;
      phase_start_a[i] = 0.0f;
    }
    total_duration = 0.0f;
  }

  // Delegate accessors to the planner
  ConstantJerkBlockPlanner& planner() { return planner_; }
  uint8_t bufferConsumed() const { return planner_.bufferConsumed(); }
  uint8_t blockCount() const { return planner_.blockCount(); }
  uint16_t cantBrakeCount() const { return planner_.cantBrakeCount(); }
  uint8_t currentBlockIndex() const { return planner_.currentBlockIndex(); }

private:
  // Raw distance-at-time without view offsets (used internally and by getTimeAtDistance)
  float rawDistanceAtTime(const float t) const {
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return dist_total;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    const float v = phase_start_v[ph];
    const float a = phase_start_a[ph];
    const float jk = phaseJerk(ph);
    return phase_start_pos[ph] + v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jk * dt * dt * dt;
  }

  void buildPhaseCache() {
    float v = v_entry, a = a_entry, dist = 0.0f, t = 0.0f;
    for (int i = 0; i < 7; ++i) {
      phase_start_time[i] = t;
      phase_start_pos[i] = dist;
      phase_start_v[i] = v;
      phase_start_a[i] = a;
      cj_simulatePhase(phaseJerk(i), phase_dt[i], v, a, dist);
      t += phase_dt[i];
    }
  }

  int findPhase(float t) const {
    for (int i = 0; i < 7; ++i)
      if (t < phase_start_time[i] + phase_dt[i]) return i;
    return 6;
  }

  int findPhaseByDist(float d) const {
    for (int i = 0; i < 6; ++i)
      if (d < phase_start_pos[i + 1]) return i;
    return 6;
  }

  float phaseJerk(int phase) const {
    switch (phase) {
      case 0: return j_max;
      case 2: return -j_max;
      case 4: return -j_max;
      case 6: return j_max;
      default: return 0.0f;
    }
  }

  ConstantJerkBlockPlanner planner_;   // Owned by value
  float* jerk_max_ptr_ = nullptr;

  float v_entry = 0, v_exit = 0;
  float a_entry = 0;  // initial acceleration (non-zero for Proposal B replanning)
  float a_exit = 0;   // exit acceleration (non-zero after truncateToDistance)
  float a_max = 0, j_max = 0, dist_total = 0;
  float total_duration = 0;
  float phase_dt[7] = {};
  float phase_start_time[7] = {};
  float phase_start_pos[7] = {};
  float phase_start_v[7] = {};
  float phase_start_a[7] = {};
};
