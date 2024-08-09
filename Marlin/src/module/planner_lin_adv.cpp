/**
 * planner_lin_adv.cpp
 *
 * Compute and buffer movement commands for linear advance
 */

#include "planner_lin_adv.h"

int32_t calc_match_slope(la_block_t* la_blocks, float_t p0_delta, float_t v_delta, float_t a, float_t a_inv, float_t t_limit, bool reverse, float_t t_offset) {
  float v_sign = v_delta < 0 ? -1 : 1;
  v_delta = fabs(v_delta);
  p0_delta *= v_sign;

  float_t t_x = v_delta * a_inv;
  float_t p_x = 0.5 * v_delta * t_x;
  int32_t dir = p_x > -p0_delta ? 1 : -1;
  float_t dist_to_cover = p_x + dir * p0_delta;
  if (dist_to_cover < 0) return -1;
  float_t half_time_to_cover_dist = sqrtf(dist_to_cover * a_inv);
  float_t t_a_change = half_time_to_cover_dist + dir * t_x;
  if (t_a_change < 0) return -1;
  float_t t1 = t_a_change + half_time_to_cover_dist;
  dir *= v_sign;
  v_delta *= v_sign;
  if (reverse) {
    if (t_offset - t1 > t_limit) return -1;
    la_blocks[0] = { t_offset - t1, -v_delta, 0 };
    la_blocks[1] = { t_offset - t_a_change, -dir * a * t_a_change, -dir };
    la_blocks[2] = { t_offset, 0, dir };
    return 3;
  } else {
    if (t1 > t_limit) return -1;
    la_blocks[0] = { t_a_change, dir * a * t_a_change, dir };
    la_blocks[1] = { t1, v_delta, -dir };
    return 2;
  }
}

int32_t join(la_block_t* la_blocks, int8_t len1, int8_t len2) {
  if (len1 == -1 || len2 == -1) return -1;
  if (len1 > 0 && len2 > 0 && la_blocks[len1 - 1].t > la_blocks[len1].t) return -1;
  return len1 + len2;
}

int32_t calc_junc(la_block_t* la_blocks, float_t v0, float_t v1, float_t a_inv, float t_v_change, float_t t_limit) {
  float_t v_delta = v1 - v0;
  int32_t dir = v_delta > 0 ? 1 : -1;
  float_t half_time_to_reach_v = dir * v_delta * a_inv * 0.5;
  if (t_v_change + half_time_to_reach_v > t_limit) return -1;
  la_blocks[0] = { t_v_change - half_time_to_reach_v, v0, 0 };
  la_blocks[1] = { t_v_change + half_time_to_reach_v, v1, dir };
  return 2;
}

int32_t calc_best_effort(la_block_t* la_blocks, float_t p0_delta, float_t a, float_t a_inv, float t_end) {
  float_t v1 = a * t_end * 0.25 + p0_delta / t_end;
  float_t dt1 = v1 * a_inv;
  float_t dt2 = t_end * 0.5;
  float_t dt3 = dt2 - dt1;
  float_t v2 = v1 - dt2 * a;

  la_blocks[0] = { dt1, v1, 1 };
  la_blocks[1] = { dt1 + dt2, v2, -1 };
  la_blocks[2] = { dt1 + dt2 + dt3, 0, 1 };
  return 3;
}

enum class Part {
    START,
    L_SLOPE,
    CRUISE,
    R_SLOPE
};

int8_t solve(Part curr, la_block_t* la_blocks, int8_t len, float t_acc_before, float t_deac_start, float t_end, float_t p0_delta, float_t v_target, float_t a, float_t a_inv) {
  if (len == -1) return -1;
  switch (curr) {
    case Part::START: {
      // START -> L_SLOPE
      int8_t len2 = calc_match_slope(la_blocks, p0_delta, v_target, a, a_inv, t_acc_before, false, 0);
      len2 = solve(Part::L_SLOPE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
      if (len2 > -1) return len2;

      // START -> CRUISE
      float_t p1_delta = p0_delta + v_target * t_acc_before;
      len2 = calc_match_slope(la_blocks, p1_delta, 0, a, a_inv, t_deac_start, false, 0);
      len2 = solve(Part::CRUISE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
      if (len2 > -1) return len2;

      // START -> R_SLOPE
      p1_delta = p0_delta + v_target * (t_acc_before + t_deac_start);
      len2 = calc_match_slope(la_blocks, p1_delta, -v_target, a, a_inv, t_end, false, 0);
      len2 = solve(Part::R_SLOPE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
      if (len2 > -1) return len2;

      // START -> END
      float_t decel_duration = t_end - t_deac_start;
      float_t p_delta_end = p0_delta + v_target * (t_acc_before - decel_duration);
      return calc_best_effort(la_blocks, p_delta_end, a, a_inv, t_end);
      break;
    }
    case Part::L_SLOPE: {
      // L_SLOPE -> CRUISE
      int8_t len2 = calc_junc(&la_blocks[len], v_target, 0, a_inv, t_acc_before, t_end);
      if (len2 > -1) {
        len2 = join(la_blocks, len, len2);
        if (len2 == -1) {
          // no time to go from l_slope => cruise, so there won't be time to reach r_slope either
          // l_slop => end means breaking even before so that won't work wither. Backtrack now
          break;
        }
        len2 = solve(Part::CRUISE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
        if (len2 > -1) return len2;
      }

      // L_SLOPE -> R_SLOPE
      float_t t_v_change = (t_acc_before + t_deac_start) * 0.5;
      len2 = calc_junc(&la_blocks[len], v_target, -v_target, a_inv, t_v_change, t_end);
      len2 = join(la_blocks, len, len2);
      len2 = solve(Part::R_SLOPE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
      if (len2 > -1) return len2;

      // L_SLOPE -> END
      float p1_delta = 2 * v_target * (t_end - t_deac_start);
      len2 = calc_match_slope(&la_blocks[len], p1_delta, -v_target, a, a_inv, t_end, true, t_end);
      len2 = join(la_blocks, len, len2);
      if (len2 > -1) return len2;
      break;
    }
    case Part::CRUISE: {
      // CRUISE -> R_SLOPE
      int8_t len2 = calc_junc(&la_blocks[len], 0, -v_target, a_inv, t_deac_start, t_end);
      len2 = join(la_blocks, len, len2);
      len2 = solve(Part::R_SLOPE, la_blocks, len2, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
      if (len2 > -1) return len2;
      
      // CRUISE -> END
      float_t p1_delta = v_target * (t_end - t_deac_start);
      len2 = calc_match_slope(&la_blocks[len], p1_delta, 0, a, a_inv, t_deac_start, true, t_end);
      len2 = join(la_blocks, len, len2);
      if (len2 > -1) return len2;
      break;
    }
    case Part::R_SLOPE: {
      // R_SLOPE -> END
      int8_t len2 = calc_match_slope(&la_blocks[len], 0, v_target, a, a_inv, t_end, true, t_end);
      len2 = join(la_blocks, len, len2);
      if (len2 > -1) return len2;
      break;
    }
  }
  return -1;
}
float t_from_dist(float v0, float dist, float acc){
  return (-v0 + sqrtf(v0 * v0 + 2 * acc * dist)) / acc;
}
float v_from_dist(float v0, float d, float acc) {
  // Calculate the final velocity using the formula v_f = sqrt(v_0^2 + 2Ad)
  return sqrtf(v0 * v0 + 2 * acc * d);
}


int8_t computeProfile(float_t last_exit_speed, block_t* block, float_t k, float_t e_acc_steps_s2_in_x_units, la_block_t* la_blocks) {
  if (block->accelerate_before < 0 || block->accelerate_before > block->step_event_count || block->accelerate_before > block->decelerate_start || block->decelerate_start < 0 || block->decelerate_start > block->step_event_count) {
    SERIAL_ECHOLNPGM("&BAD BLOCK!!!");
    return -1;
  }

  float acc = block->acceleration_steps_per_s2;
  
  float cruise_rate = v_from_dist(block->initial_rate, block->accelerate_before, acc);
  // float t_acc_before = t_from_dist(block->initial_rate, block->accelerate_before, acc);
  // float t_cruise = (block->decelerate_start - block->accelerate_before) / block->nominal_rate;
  // float t_deac_start = t_acc_before + t_cruise;
  // float t_end = t_deac_start + t_from_dist(block->nominal_rate, block->step_event_count - block->accelerate_before, -acc);

  float t_acc_before = (cruise_rate - float(block->initial_rate)) / acc;
  float t_cruise = float(block->decelerate_start - block->accelerate_before) / cruise_rate;
  float t_deac_start = t_acc_before + t_cruise;
  float t_end = t_deac_start + (cruise_rate - float(block->final_rate)) / acc;

  float_t p0_target = block->initial_rate * k;
  float_t p0 = last_exit_speed * k; 
  float_t p0_delta = p0_target - p0;
  float_t v_target = acc * k;
  float_t a = e_acc_steps_s2_in_x_units;
  float_t a_inv = 1.0f / a;
  int8_t len = solve(Part::START, la_blocks, 0, t_acc_before, t_deac_start, t_end, p0_delta, v_target, a, a_inv);
  la_blocks[len] = {float(UINT32_MAX), 0, 0};
  len++;
  for (int32_t i = 1; i < len; i++) {
      if (la_blocks[i - 1].t > la_blocks[i].t) SERIAL_ECHOLNPGM("§la_block is broken!!!");
  }
  return len;
}
