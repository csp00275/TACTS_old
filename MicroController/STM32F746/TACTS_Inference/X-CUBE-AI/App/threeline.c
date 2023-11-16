/**
  ******************************************************************************
  * @file    threeline.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Nov 17 00:05:15 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "threeline.h"
#include "threeline_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_threeline
 
#undef AI_THREELINE_MODEL_SIGNATURE
#define AI_THREELINE_MODEL_SIGNATURE     "63a2bd4fd4f9e0725f1860760b89c4a7"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Fri Nov 17 00:05:15 2023"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_THREELINE_N_BATCHES
#define AI_THREELINE_N_BATCHES         (1)

static ai_ptr g_threeline_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_threeline_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_0_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 36, AI_STATIC)
/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  dense_85_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)
/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  dense_85_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)
/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  dense_86_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)
/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  dense_86_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)
/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  dense_87_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)
/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  dense_87_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)
/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  dense_88_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)
/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  dense_88_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)
/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  dense_89_dense_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)
/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  dense_85_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18432, AI_STATIC)
/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  dense_85_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)
/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  dense_86_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 65536, AI_STATIC)
/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  dense_86_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)
/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  dense_87_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4096, AI_STATIC)
/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  dense_87_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)
/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  dense_88_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)
/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  dense_88_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)
/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  dense_89_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)
/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  dense_89_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4, AI_STATIC)
/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  input_0_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 36, 1, 1), AI_STRIDE_INIT(4, 4, 4, 144, 144),
  1, &input_0_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  dense_85_dense_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &dense_85_dense_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  dense_85_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &dense_85_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  dense_86_dense_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_86_dense_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  dense_86_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_86_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  dense_87_dense_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &dense_87_dense_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  dense_87_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &dense_87_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  dense_88_dense_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &dense_88_dense_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  dense_88_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &dense_88_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  dense_89_dense_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &dense_89_dense_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  dense_85_dense_weights, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 36, 512, 1, 1), AI_STRIDE_INIT(4, 4, 144, 73728, 73728),
  1, &dense_85_dense_weights_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  dense_85_dense_bias, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &dense_85_dense_bias_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  dense_86_dense_weights, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 512, 128, 1, 1), AI_STRIDE_INIT(4, 4, 2048, 262144, 262144),
  1, &dense_86_dense_weights_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  dense_86_dense_bias, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_86_dense_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  dense_87_dense_weights, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 128, 32, 1, 1), AI_STRIDE_INIT(4, 4, 512, 16384, 16384),
  1, &dense_87_dense_weights_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  dense_87_dense_bias, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &dense_87_dense_bias_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  dense_88_dense_weights, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 32, 8, 1, 1), AI_STRIDE_INIT(4, 4, 128, 1024, 1024),
  1, &dense_88_dense_weights_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  dense_88_dense_bias, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &dense_88_dense_bias_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  dense_89_dense_weights, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 8, 4, 1, 1), AI_STRIDE_INIT(4, 4, 32, 128, 128),
  1, &dense_89_dense_weights_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  dense_89_dense_bias, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &dense_89_dense_bias_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_89_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_88_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_89_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_89_dense_weights, &dense_89_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_89_dense_layer, 4,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_89_dense_chain,
  NULL, &dense_89_dense_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_88_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_88_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_88_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_88_layer, 3,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_88_chain,
  NULL, &dense_89_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_88_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_87_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_88_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_88_dense_weights, &dense_88_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_88_dense_layer, 3,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_88_dense_chain,
  NULL, &dense_88_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_87_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_87_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_87_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_87_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_87_chain,
  NULL, &dense_88_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_87_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_86_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_87_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_87_dense_weights, &dense_87_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_87_dense_layer, 2,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_87_dense_chain,
  NULL, &dense_87_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_86_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_86_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_86_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_86_layer, 1,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_86_chain,
  NULL, &dense_87_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_86_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_85_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_86_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_86_dense_weights, &dense_86_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_86_dense_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_86_dense_chain,
  NULL, &dense_86_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_85_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_85_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_85_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_85_layer, 0,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_85_chain,
  NULL, &dense_86_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_85_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_85_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_85_dense_weights, &dense_85_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_85_dense_layer, 0,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_85_dense_chain,
  NULL, &dense_85_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 356144, 1, 1),
    356144, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 2560, 1, 1),
    2560, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_THREELINE_IN_NUM, &input_0_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_THREELINE_OUT_NUM, &dense_89_dense_output),
  &dense_85_dense_layer, 0, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 356144, 1, 1),
      356144, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 2560, 1, 1),
      2560, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_THREELINE_IN_NUM, &input_0_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_THREELINE_OUT_NUM, &dense_89_dense_output),
  &dense_85_dense_layer, 0, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/


/******************************************************************************/
AI_DECLARE_STATIC
ai_bool threeline_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_threeline_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_0_output_array.data = AI_PTR(g_threeline_activations_map[0] + 368);
    input_0_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 368);
    
    dense_85_dense_output_array.data = AI_PTR(g_threeline_activations_map[0] + 512);
    dense_85_dense_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 512);
    
    dense_85_output_array.data = AI_PTR(g_threeline_activations_map[0] + 512);
    dense_85_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 512);
    
    dense_86_dense_output_array.data = AI_PTR(g_threeline_activations_map[0] + 0);
    dense_86_dense_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 0);
    
    dense_86_output_array.data = AI_PTR(g_threeline_activations_map[0] + 512);
    dense_86_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 512);
    
    dense_87_dense_output_array.data = AI_PTR(g_threeline_activations_map[0] + 0);
    dense_87_dense_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 0);
    
    dense_87_output_array.data = AI_PTR(g_threeline_activations_map[0] + 128);
    dense_87_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 128);
    
    dense_88_dense_output_array.data = AI_PTR(g_threeline_activations_map[0] + 0);
    dense_88_dense_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 0);
    
    dense_88_output_array.data = AI_PTR(g_threeline_activations_map[0] + 32);
    dense_88_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 32);
    
    dense_89_dense_output_array.data = AI_PTR(g_threeline_activations_map[0] + 0);
    dense_89_dense_output_array.data_start = AI_PTR(g_threeline_activations_map[0] + 0);
    
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool threeline_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_threeline_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    dense_85_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_85_dense_weights_array.data = AI_PTR(g_threeline_weights_map[0] + 0);
    dense_85_dense_weights_array.data_start = AI_PTR(g_threeline_weights_map[0] + 0);
    
    dense_85_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_85_dense_bias_array.data = AI_PTR(g_threeline_weights_map[0] + 73728);
    dense_85_dense_bias_array.data_start = AI_PTR(g_threeline_weights_map[0] + 73728);
    
    dense_86_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_86_dense_weights_array.data = AI_PTR(g_threeline_weights_map[0] + 75776);
    dense_86_dense_weights_array.data_start = AI_PTR(g_threeline_weights_map[0] + 75776);
    
    dense_86_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_86_dense_bias_array.data = AI_PTR(g_threeline_weights_map[0] + 337920);
    dense_86_dense_bias_array.data_start = AI_PTR(g_threeline_weights_map[0] + 337920);
    
    dense_87_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_87_dense_weights_array.data = AI_PTR(g_threeline_weights_map[0] + 338432);
    dense_87_dense_weights_array.data_start = AI_PTR(g_threeline_weights_map[0] + 338432);
    
    dense_87_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_87_dense_bias_array.data = AI_PTR(g_threeline_weights_map[0] + 354816);
    dense_87_dense_bias_array.data_start = AI_PTR(g_threeline_weights_map[0] + 354816);
    
    dense_88_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_88_dense_weights_array.data = AI_PTR(g_threeline_weights_map[0] + 354944);
    dense_88_dense_weights_array.data_start = AI_PTR(g_threeline_weights_map[0] + 354944);
    
    dense_88_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_88_dense_bias_array.data = AI_PTR(g_threeline_weights_map[0] + 355968);
    dense_88_dense_bias_array.data_start = AI_PTR(g_threeline_weights_map[0] + 355968);
    
    dense_89_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_89_dense_weights_array.data = AI_PTR(g_threeline_weights_map[0] + 356000);
    dense_89_dense_weights_array.data_start = AI_PTR(g_threeline_weights_map[0] + 356000);
    
    dense_89_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_89_dense_bias_array.data = AI_PTR(g_threeline_weights_map[0] + 356128);
    dense_89_dense_bias_array.data_start = AI_PTR(g_threeline_weights_map[0] + 356128);
    
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/


AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_threeline_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_THREELINE_MODEL_NAME,
      .model_signature   = AI_THREELINE_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 89716,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_bool ai_threeline_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_THREELINE_MODEL_NAME,
      .model_signature   = AI_THREELINE_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 89716,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x0,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}

AI_API_ENTRY
ai_error ai_threeline_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_threeline_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_error ai_threeline_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
    ai_error err;
    ai_network_params params;

    err = ai_threeline_create(network, AI_THREELINE_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE)
        return err;
    if (ai_threeline_data_params_get(&params) != true) {
        err = ai_threeline_get_error(*network);
        return err;
    }
#if defined(AI_THREELINE_DATA_ACTIVATIONS_COUNT)
    if (activations) {
        /* set the addresses of the activations buffers */
        for (int idx=0;idx<params.map_activations.size;idx++)
            AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
    }
#endif
#if defined(AI_THREELINE_DATA_WEIGHTS_COUNT)
    if (weights) {
        /* set the addresses of the weight buffers */
        for (int idx=0;idx<params.map_weights.size;idx++)
            AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
    }
#endif
    if (ai_threeline_init(*network, &params) != true) {
        err = ai_threeline_get_error(*network);
    }
    return err;
}

AI_API_ENTRY
ai_buffer* ai_threeline_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    ((ai_network *)network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}

AI_API_ENTRY
ai_buffer* ai_threeline_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    ((ai_network *)network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}

AI_API_ENTRY
ai_handle ai_threeline_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_threeline_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if (!net_ctx) return false;

  ai_bool ok = true;
  ok &= threeline_configure_weights(net_ctx, params);
  ok &= threeline_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_threeline_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_threeline_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_THREELINE_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

