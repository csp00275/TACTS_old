/*
 * ai.c
 *
 *  Created on: 2023. 10. 26.
 *      Author: 21310
 */

#include "ai.h"
#include "usart.h"
#include "stdio.h"
#include "vl53l0x_jh.h"

ai_handle twoLine = AI_HANDLE_NULL;
ai_handle threeLine = AI_HANDLE_NULL;
ai_handle fourLine = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
ai_u8 activations_two[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(32)
ai_u8 activations_three[AI_THREELINE_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(32)
ai_u8 activations_four[AI_FOURLINE_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
ai_float in_data_two[AI_TWOLINE_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data_two[AI_TWOLINE_OUT_1_SIZE];

AI_ALIGNED(32)
ai_float in_data_three[AI_THREELINE_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data_three[AI_THREELINE_OUT_1_SIZE];

AI_ALIGNED(32)
ai_float in_data_four[AI_FOURLINE_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data_four[AI_FOURLINE_OUT_1_SIZE];

/* Array of pointer to manage the model's input/output tensors */
ai_buffer *ai_input_two;
ai_buffer *ai_output_two;

ai_buffer *ai_input_three;
ai_buffer *ai_output_three;

ai_buffer *ai_input_four;
ai_buffer *ai_output_four;

int aiInit(void) {
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts_two[] = { activations_two };
  err = ai_twoline_create_and_init(&twoLine, acts_two, NULL);
  if (err.type != AI_ERROR_NONE) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "twoLine aiInit Error \n"), 100);
  };

  /* Reteive pointers to the model's input/output tensors */
  ai_input_two = ai_twoline_inputs_get(twoLine, NULL);
  ai_output_two = ai_twoline_outputs_get(twoLine, NULL);

  /* threeLine initialization */
   const ai_handle acts_three[] = { activations_three };
   err = ai_threeline_create_and_init(&threeLine, acts_three, NULL);
   if (err.type != AI_ERROR_NONE) {
       HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "threeLine aiInit Error \n"), 100);
   };
   ai_input_three = ai_threeline_inputs_get(threeLine, NULL);
   ai_output_three = ai_threeline_outputs_get(threeLine, NULL);

   /* fourLine initialization */
   const ai_handle acts_four[] = { activations_four };
   err = ai_fourline_create_and_init(&fourLine, acts_four, NULL);
   if (err.type != AI_ERROR_NONE) {
       HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "fourLine aiInit Error \n"), 100);
   };
   ai_input_four = ai_fourline_inputs_get(fourLine, NULL);
   ai_output_four = ai_fourline_outputs_get(fourLine, NULL);

  return 0;
}

int aiRun(const ai_float *in_data, ai_float *out_data) {
  ai_i32 n_batch;

  /* 1 - Update IO handlers with the data payload */
  ai_input_two[0].data = AI_HANDLE_PTR(in_data);
  ai_output_two[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_twoline_run(twoLine, &ai_input_two[0], &ai_output_two[0]);
  if (n_batch != 1) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiRun Error \n"), 100);
  };

  return 0;
}
