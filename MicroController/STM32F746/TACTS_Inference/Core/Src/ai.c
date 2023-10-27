/*
 * ai.c
 *
 *  Created on: 2023. 10. 26.
 *      Author: 21310
 */

#include "ai.h"
#include "usart.h"
#include "stdio.h"

ai_handle twoLine = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
ai_float in_data[AI_TWOLINE_IN_1_SIZE];

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
ai_float out_data[AI_TWOLINE_OUT_1_SIZE];

/* Array of pointer to manage the model's input/output tensors */
ai_buffer *ai_input;
ai_buffer *ai_output;

int aiInit(void) {
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_twoline_create_and_init(&twoLine, acts, NULL);
  if (err.type != AI_ERROR_NONE) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiInit Error \n"), 100);
  };

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_twoline_inputs_get(twoLine, NULL);
  ai_output = ai_twoline_outputs_get(twoLine, NULL);

  return 0;
}

int aiRun(const ai_float *in_data, ai_float *out_data) {
  ai_i32 n_batch;

  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_twoline_run(twoLine, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiRun Error \n"), 100);
  };

  return 0;
}
