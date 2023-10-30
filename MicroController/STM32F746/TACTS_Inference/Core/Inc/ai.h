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
#include "threeLine.h"
#include "threeLine_data.h"
#include "fourLine.h"
#include "fourLine_data.h"

/* Global handle to reference the instantiated C-model */
extern ai_handle twoLine;
extern ai_handle threeLine;
extern ai_handle fourLine;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
extern ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(32)
extern ai_u8 activations_three[AI_THREELINE_DATA_ACTIVATIONS_SIZE];

AI_ALIGNED(32)
extern ai_u8 activations_four[AI_FOURLINE_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
extern ai_float in_data_two[AI_TWOLINE_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data_two[AI_TWOLINE_OUT_1_SIZE];

AI_ALIGNED(32)
extern ai_float in_data_three[AI_THREELINE_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data_three[AI_THREELINE_OUT_1_SIZE];

AI_ALIGNED(32)
extern ai_float in_data_four[AI_FOURLINE_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data_four[AI_FOURLINE_OUT_1_SIZE];



/* Array of pointer to manage the model's input/output tensors */
extern ai_buffer *ai_input_two;
extern ai_buffer *ai_output_two;

extern ai_buffer *ai_input_three;
extern ai_buffer *ai_output_three;

extern ai_buffer *ai_input_four;
extern ai_buffer *ai_output_four;

int aiInit(void);
int aiRun(const ai_float *in_data, ai_float *out_data);



#endif /* INC_AI_H_ */
