/**
 * planner_lin_adv.cpp
 *
 * Compute and buffer movement commands for linear advance
 */
#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(LA_ZERO_SLOWDOWN)

#include "planner.h"
#include "planner_lin_adv_struct.h"


int8_t computeProfile(float_t last_exit_speed, block_t* block, float_t k, float_t eAccMax_steps_per_s2, la_block_t* la_blocks);

#endif // LA_ZERO_SLOWDOWN
