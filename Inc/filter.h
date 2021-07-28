/*
 * filter.h
 *
 *  Created on: Feb 3, 2018
 *      Author: ryan
 */

#ifndef FILTER_H_
#define FILTER_H_

typedef struct {
	float samplingTime;
	float freq;
	float a;
	float prevInput;
} LPFilter;

void InitLPFilterObject(LPFilter * filter, float freq, float samplingTime);
float LP_Filter(LPFilter * filter, float signal, float samplingTime);
#endif /* FILTER_H_ */
