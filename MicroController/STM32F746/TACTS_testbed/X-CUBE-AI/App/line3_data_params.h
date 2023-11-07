/**
  ******************************************************************************
  * @file    line3_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon Oct 23 16:11:37 2023
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

#ifndef LINE3_DATA_PARAMS_H
#define LINE3_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_LINE3_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_line3_data_weights_params[1]))
*/

#define AI_LINE3_DATA_CONFIG               (NULL)


#define AI_LINE3_DATA_ACTIVATIONS_SIZES \
  { 1664, }
#define AI_LINE3_DATA_ACTIVATIONS_SIZE     (1664)
#define AI_LINE3_DATA_ACTIVATIONS_COUNT    (1)
#define AI_LINE3_DATA_ACTIVATION_1_SIZE    (1664)



#define AI_LINE3_DATA_WEIGHTS_SIZES \
  { 99600, }
#define AI_LINE3_DATA_WEIGHTS_SIZE         (99600)
#define AI_LINE3_DATA_WEIGHTS_COUNT        (1)
#define AI_LINE3_DATA_WEIGHT_1_SIZE        (99600)



#define AI_LINE3_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_line3_activations_table[1])

extern ai_handle g_line3_activations_table[1 + 2];



#define AI_LINE3_DATA_WEIGHTS_TABLE_GET() \
  (&g_line3_weights_table[1])

extern ai_handle g_line3_weights_table[1 + 2];


#endif    /* LINE3_DATA_PARAMS_H */
