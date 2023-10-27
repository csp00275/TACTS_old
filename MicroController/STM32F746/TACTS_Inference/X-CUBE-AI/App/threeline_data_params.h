/**
  ******************************************************************************
  * @file    threeline_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Oct 27 12:59:16 2023
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

#ifndef THREELINE_DATA_PARAMS_H
#define THREELINE_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_THREELINE_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_threeline_data_weights_params[1]))
*/

#define AI_THREELINE_DATA_CONFIG               (NULL)


#define AI_THREELINE_DATA_ACTIVATIONS_SIZES \
  { 1664, }
#define AI_THREELINE_DATA_ACTIVATIONS_SIZE     (1664)
#define AI_THREELINE_DATA_ACTIVATIONS_COUNT    (1)
#define AI_THREELINE_DATA_ACTIVATION_1_SIZE    (1664)



#define AI_THREELINE_DATA_WEIGHTS_SIZES \
  { 99600, }
#define AI_THREELINE_DATA_WEIGHTS_SIZE         (99600)
#define AI_THREELINE_DATA_WEIGHTS_COUNT        (1)
#define AI_THREELINE_DATA_WEIGHT_1_SIZE        (99600)



#define AI_THREELINE_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_threeline_activations_table[1])

extern ai_handle g_threeline_activations_table[1 + 2];



#define AI_THREELINE_DATA_WEIGHTS_TABLE_GET() \
  (&g_threeline_weights_table[1])

extern ai_handle g_threeline_weights_table[1 + 2];


#endif    /* THREELINE_DATA_PARAMS_H */
