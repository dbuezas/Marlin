/**
 * planner_lin_adv.cpp
 *
 * Compute and buffer movement commands for linear advance
 */

#pragma once
#include "../inc/MarlinConfig.h"

typedef struct LABlock{
  float_t t;
  float_t v;
  int32_t d;
} la_block_t;
