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

ai_handle allLine = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
#if NUM_SENSOR == 24
AI_ALIGNED(32)
ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
ai_float in_data[AI_TWOLINE_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data[AI_TWOLINE_OUT_1_SIZE];

#elif NUM_SENSOR == 36
AI_ALIGNED(32)
ai_u8 activations[AI_THREELINECV_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
ai_float in_data[AI_THREELINECV_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data[AI_THREELINECV_OUT_1_SIZE];
#elif
AI_ALIGNED(32)
ai_u8 activations[AI_FOURLINE_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
ai_float in_data[AI_FOURLINE_IN_1_SIZE];
AI_ALIGNED(32)
ai_float out_data[AI_FOURLINE_OUT_1_SIZE];
#endif

ai_buffer *ai_input;
ai_buffer *ai_output;

int aiInit(void) {
    ai_error err;
    const ai_handle acts[] = { activations };
    err = ai_twoline_create_and_init(&allLine, acts, NULL);
    if (err.type != AI_ERROR_NONE) {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiInit Error \n"), 100);
    }

#if NUM_SENSOR == 24
    ai_input = ai_twoline_inputs_get(allLine, NULL);
    ai_output = ai_twoline_outputs_get(allLine, NULL);

#elif NUM_SENSOR == 36
    ai_input = ai_threelinecv_inputs_get(allLine, NULL);
    ai_output= ai_threelinecv_outputs_get(allLine, NULL);
#elif NUM_SENSOR == 48
    ai_input = ai_fourline_inputs_get(allLine, NULL);
    ai_output = ai_fourline_outputs_get(allLine, NULL);

#else
    HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Invalid NUM_SENSOR \n"), 100);
    return -1;
#endif
    return 0;
}


int aiRun(const ai_float *in_data, ai_float *out_data) {
    ai_i32 n_batch;

    // 1 - Update IO handlers with the data payload
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].data = AI_HANDLE_PTR(out_data);

#if NUM_SENSOR == 24
    n_batch = ai_twoline_run(allLine, &ai_input[0], &ai_output[0]);
#elif NUM_SENSOR == 36
    n_batch = ai_threelinecv_run(allLine, &ai_input[0], &ai_output[0]);
#elif NUM_SENSOR == 48
    n_batch = ai_fourline_run(allLine, &ai_input[0], &ai_output[0]);
#else
    HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Invalid NUM_SENSOR \n"), 100);
    return -1;
#endif
    if (n_batch != 1) {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiRun Error \n"), 100);
        return -1;
    };
    return 0;
}

