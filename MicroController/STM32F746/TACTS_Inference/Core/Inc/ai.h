/*
 * ai.h
 *
 *  Created on: 2023. 10. 26.
 *      Author: 21310
 */

#ifndef INC_AI_H_
#define INC_AI_H_

#include "twoLine.h"
#include "twoLine_data.h"

/* Global handle to reference the instantiated C-model */
extern ai_handle twoLine;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
extern ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
extern ai_float in_data[AI_TWOLINE_IN_1_SIZE];

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
extern ai_float out_data[AI_TWOLINE_OUT_1_SIZE];

/* Array of pointer to manage the model's input/output tensors */
extern ai_buffer *ai_input;
extern ai_buffer *ai_output;

int aiInit(void);
int aiRun(const ai_float *in_data, ai_float *out_data);



#endif /* INC_AI_H_ */
