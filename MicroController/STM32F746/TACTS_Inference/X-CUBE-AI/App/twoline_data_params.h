/**
  ******************************************************************************
  * @file    twoline_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Nov 17 00:05:06 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef TWOLINE_DATA_PARAMS_H
#define TWOLINE_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_TWOLINE_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_twoline_data_weights_params[1]))
*/

#define AI_TWOLINE_DATA_CONFIG               (NULL)


#define AI_TWOLINE_DATA_ACTIVATIONS_SIZES \
  { 2560, }
#define AI_TWOLINE_DATA_ACTIVATIONS_SIZE     (2560)
#define AI_TWOLINE_DATA_ACTIVATIONS_COUNT    (1)
#define AI_TWOLINE_DATA_ACTIVATION_1_SIZE    (2560)



#define AI_TWOLINE_DATA_WEIGHTS_SIZES \
  { 335120, }
#define AI_TWOLINE_DATA_WEIGHTS_SIZE         (335120)
#define AI_TWOLINE_DATA_WEIGHTS_COUNT        (1)
#define AI_TWOLINE_DATA_WEIGHT_1_SIZE        (335120)



#define AI_TWOLINE_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_twoline_activations_table[1])

extern ai_handle g_twoline_activations_table[1 + 2];



#define AI_TWOLINE_DATA_WEIGHTS_TABLE_GET() \
  (&g_twoline_weights_table[1])

extern ai_handle g_twoline_weights_table[1 + 2];


#endif    /* TWOLINE_DATA_PARAMS_H */
