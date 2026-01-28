#ifndef CONTROL_SIMPLE_H
#define CONTROL_SIMPLE_H

#include <stdint.h>

typedef enum
{
    CONTROL_MODE_TORQUE_COMP = 0,
    CONTROL_MODE_SPEED = 1,
} ControlMode;

void ControlSimple_Init(void);
void ControlSimple_Step(int32_t enc_count);
void ControlSimple_Enable(uint8_t on);
void ControlSimple_SetMode(ControlMode mode);
void ControlSimple_SetSpeedRefRpm(float rpm);

void FilterWindow(uint16_t n);
void ControlSimple_SetCurrentRefmA(float mA);
void ControlSimple_EnableCurrentOverride(uint8_t on);
float ControlSimple_GetSpeedRpm(void);
float ControlSimple_GetCurrentRefmA(void);
float ControlSimple_GetCurrentRefmAClamped(void);
float ControlSimple_GetCurrentPIIntegrator(void);
float ControlSimple_GetCurrentPwmCmd(void);
int32_t Control_GetSpeedOutMotorRpm(void);
int32_t Control_GetAccelFilt(void);
void CurrentLoopStep(int16_t cur_mA);
void LogPrint(void);

#endif

