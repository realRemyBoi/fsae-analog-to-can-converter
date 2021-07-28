/*
 * filter.c
 *
 *  Created on: Feb 3, 2018
 *      Author: ryan
 */

#include "filter.h"

void InitLPFilterObject(LPFilter * filter, float freq, float samplingTime) {
	filter->freq = freq;	// rad/s
	filter->samplingTime = samplingTime; //s
	filter->prevInput = 0;
}

float LP_Filter(LPFilter * filter, float input, float samplingTime) {
	filter->samplingTime = samplingTime;
	filter->a = 1/(1 + filter->freq * filter->samplingTime);
	float out = filter->prevInput * filter->a + input * (1-filter->a);
	filter->prevInput = out;
	return out;
}
