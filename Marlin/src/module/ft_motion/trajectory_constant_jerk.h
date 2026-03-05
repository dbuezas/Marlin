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
static inline float cj_planRamp(float v_start, float v_peak, float j_max, float a_max,
                                bool decel, float &dt_jerk1, float &dt_hold, float &dt_jerk2) {
  float dv = v_peak - v_start;
  float a_peak_sq = j_max * dv;
  if (a_peak_sq < 0) {
    dt_jerk1 = dt_hold = dt_jerk2 = 0;
    return 0;
  }
  float a_peak = SQRT(a_peak_sq);

  if (a_peak <= a_max) {
    dt_jerk1 = a_peak / j_max;
    dt_hold = 0;
    dt_jerk2 = a_peak / j_max;
  }
  else {
    dt_jerk1 = a_max / j_max;
    dt_jerk2 = a_max / j_max;
    float dv_no_hold = (a_max * a_max) / j_max;
    dt_hold = _MAX(0.0f, (dv - dv_no_hold) / a_max);
  }

  float jk = decel ? -j_max : j_max;
  float v = decel ? v_peak : v_start;
  float a_v = 0, dist = 0;
  cj_simulatePhase(jk, dt_jerk1, v, a_v, dist);
  cj_simulatePhase(0, dt_hold, v, a_v, dist);
  cj_simulatePhase(-jk, dt_jerk2, v, a_v, dist);
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

class ConstantJerkTrajectoryGenerator : public TrajectoryGenerator {
public:
  ConstantJerkTrajectoryGenerator() = default;

  // plan() override is unused — the CJ block planner calls plan_full() directly.
  // Kept to satisfy the pure virtual interface.
  void plan(const float, const float, const float, const float, const float) override {}

  // Plan with explicit jerk and a_max (used by the block merging planner).
  // Jerk comes from cfg.jerk_max, passed through by the caller.
  // Returns false if infeasible (ramp between v0 and v1 exceeds distance).
  bool plan_full(float v_entry_in, float v_exit_in,
                 float a_max_in, float j_max_in,
                 float dist_total_in, float v_nominal) {
    v_entry = v_entry_in;
    v_exit = v_exit_in;
    a_max = a_max_in;
    j_max = j_max_in;
    dist_total = dist_total_in;

    const float v_small = _MIN(v_entry, v_exit);
    const float v_large = _MAX(v_entry, v_exit);

    float min_dist_at_nominal = cj_totalRampDist(v_nominal, v_small, v_large, j_max, a_max);
    float v_peak = v_nominal;
    if (min_dist_at_nominal > dist_total) {
      float v_peak_max = v_nominal;
      float v_peak_min = v_large;

      float minmimum_distance = cj_totalRampDist(v_large, v_small, v_large, j_max, a_max);
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
          float dist_mid = cj_totalRampDist(v_mid, v_small, v_large, j_max, a_max);
          float overshoot = dist_mid - dist_total;
          if (overshoot > 0) {
            v_peak_max = v_mid;
          } else {
            v_peak_min = v_mid;
            if (-overshoot <= 0.001f) {
              // Undershoot peak and cruise for 0.001mm instead of wasting cpu
              break;
            }
          }
        }
      }
      v_peak = v_peak_min;
    }

    float t1, t2, t3, t4 = 0, t5, t6, t7;
    float dist_accel = cj_planRamp(v_entry, v_peak, j_max, a_max, false, t1, t2, t3);
    float dist_decel = cj_planRamp(v_exit, v_peak, j_max, a_max, true, t5, t6, t7);

    float dist_ramps = dist_accel + dist_decel;
    if (v_peak > 0.0f && dist_total > dist_ramps)
      t4 = (dist_total - dist_ramps) / v_peak;

    phase_dt[0] = t1; phase_dt[1] = t2; phase_dt[2] = t3;
    phase_dt[3] = t4;
    phase_dt[4] = t5; phase_dt[5] = t6; phase_dt[6] = t7;

    total_duration = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    buildPhaseCache();
    return true;
  }

  void planRunout(const float duration) override {
    reset();
    // Cruise at zero speed for the entire duration (same as trapezoidal)
    phase_dt[3] = duration;
    total_duration = duration;
    buildPhaseCache();
  }

  float getDistanceAtTime(const float t) const override {
    if (t <= 0.0f) return 0.0f;
    if (t >= total_duration) return dist_total;
    const int ph = findPhase(t);
    const float dt = t - phase_start_time[ph];
    const float v = phase_start_v[ph];
    const float a = phase_start_a[ph];
    const float jk = phaseJerk(ph);
    return phase_start_pos[ph] + v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * jk * dt * dt * dt;
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
    if (t <= 0.0f || t >= total_duration) return 0.0f;
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

  void reset() override {
    v_entry = v_exit = 0.0f;
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

private:
  void buildPhaseCache() {
    float v = v_entry, a = 0.0f, dist = 0.0f, t = 0.0f;
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

  float v_entry = 0, v_exit = 0;
  float a_max = 0, j_max = 0, dist_total = 0;
  float total_duration = 0;
  float phase_dt[7] = {};
  float phase_start_time[7] = {};
  float phase_start_pos[7] = {};
  float phase_start_v[7] = {};
  float phase_start_a[7] = {};
};
