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
#include "vl53l0x_jh.h"


extern ai_handle allLine;

/* Global c-array to handle the activations buffer */
#if NUM_SENSOR == 24
AI_ALIGNED(32)
extern ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
extern ai_float in_data[AI_TWOLINE_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data[AI_TWOLINE_OUT_1_SIZE];

#elif NUM_SENSOR == 36
AI_ALIGNED(32)
extern ai_u8 activations[AI_THREELINECV_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
extern ai_float in_data[AI_THREELINECV_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data[AI_THREELINECV_OUT_1_SIZE];

#elif
AI_ALIGNED(32)
extern ai_u8 activations[AI_FOURLINE_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
extern ai_float in_data[AI_FOURLINE_IN_1_SIZE];
AI_ALIGNED(32)
extern ai_float out_data[AI_FOURLINE_OUT_1_SIZE];

#endif

extern ai_buffer *ai_input;
extern ai_buffer *ai_output;

int aiInit(void);
int aiRun(const ai_float *in_data, ai_float *out_data);



#endif /* INC_AI_H_ */
