#ifndef TEST_H
#define TEST_H

#include <stdint.h>

void Test_Init(void);
void Test_StartCurrentStep(void);
void Test_StopCurrentStep(void);
void Test_StartSpeedCapture(void);
void Test_StopSpeedCapture(void);
void Test_Step1kHz(void);

#endif
