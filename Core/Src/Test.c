#include "Test.h"
#include "control_simple.h"
#include "adc.h"
#include <stdio.h>

#define TEST_TICK_HZ 1000U

// Current step test parameters
#define STEP_MA_0 500
#define STEP_MA_1 0
#define STEP_MA_2 -2000
#define STEP_MA_3 -2000
#define STEP_MA_4 2000
#define STEP_HOLD_MS 2000U
#define STEP_REPEAT 10U

// Speed capture parameters (from old project)
#define SPEED_CAP_MIN_RPM -4000
#define SPEED_CAP_MAX_RPM 4000
#define SPEED_CAP_STEP_RPM 200
#define SPEED_CAP_HOLD_TICKS 15000U
#define SPEED_CAP_DISCARD_TICKS  1000U     //5000U
#define SPEED_CAP_STEP_COUNT (((SPEED_CAP_MAX_RPM - SPEED_CAP_MIN_RPM) / SPEED_CAP_STEP_RPM) + 1)

static uint8_t g_step_enabled = 0;
static uint16_t g_step_tick = 0;
static uint8_t g_step_idx = 0;
static uint8_t g_step_round = 0;

static uint8_t g_speed_cap_enabled = 0;
static uint8_t g_speed_cap_started = 0;
static uint16_t g_speed_cap_step_idx = 0;
static uint32_t g_speed_cap_ticks = 0;
static uint8_t g_speed_cap_discard = 1;
static float g_speed_cap_sum_rpm = 0.0f;
static float g_speed_cap_sum_iref = 0.0f;
static float g_speed_cap_sum_imeas = 0.0f;
static uint32_t g_speed_cap_sum_count = 0;

static int current_step_value(uint8_t idx)
{
    switch (idx)
    {
    case 0: return STEP_MA_0;
    case 1: return STEP_MA_1;
    case 2: return STEP_MA_2;
    case 3: return STEP_MA_3;
    default: return STEP_MA_4;
    }
}

static float speed_capture_target_rpm(uint16_t step_idx)
{
    int32_t rpm = SPEED_CAP_MIN_RPM + ((int32_t)step_idx * SPEED_CAP_STEP_RPM);
    return (float)rpm;
}

void Test_Init(void)
{
    g_step_enabled = 0;
    g_step_tick = 0;
    g_step_idx = 0;
    g_step_round = 0;

    g_speed_cap_enabled = 0;
    g_speed_cap_started = 0;
    g_speed_cap_step_idx = 0;
    g_speed_cap_ticks = 0;
    g_speed_cap_discard = 1;
    g_speed_cap_sum_rpm = 0.0f;
    g_speed_cap_sum_iref = 0.0f;
    g_speed_cap_sum_imeas = 0.0f;
    g_speed_cap_sum_count = 0;
}

void Test_StartCurrentStep(void)
{
    g_step_enabled = 1;
    g_step_tick = 0;
    g_step_idx = 0;
    g_step_round = 0;
    ControlSimple_EnableCurrentOverride(1);
}

void Test_StopCurrentStep(void)
{
    g_step_enabled = 0;
    ControlSimple_EnableCurrentOverride(0);
}

void Test_StartSpeedCapture(void)
{
    g_speed_cap_enabled = 1;
    g_speed_cap_started = 0;
    g_speed_cap_step_idx = 0;
    g_speed_cap_ticks = 0;
    g_speed_cap_discard = 1;
    g_speed_cap_sum_rpm = 0.0f;
    g_speed_cap_sum_iref = 0.0f;
    g_speed_cap_sum_imeas = 0.0f;
    g_speed_cap_sum_count = 0;
    ControlSimple_EnableCurrentOverride(0);
}

void Test_StopSpeedCapture(void)
{
    g_speed_cap_enabled = 0;
    g_speed_cap_started = 0;
    ControlSimple_SetSpeedRefRpm(0.0f);
    ControlSimple_SetMode(CONTROL_MODE_TORQUE_COMP);
}

void Test_Step1kHz(void)
{
    if (g_step_enabled)
    {
        if (++g_step_tick >= STEP_HOLD_MS)
        {
            g_step_tick = 0;
            g_step_idx++;
            if (g_step_idx > 4)
            {
                g_step_idx = 0;
                if (++g_step_round >= STEP_REPEAT)
                {
                    Test_StopCurrentStep();
                }
            }
        }
        if (g_step_enabled)
        {
            int step_mA = current_step_value(g_step_idx);
            ControlSimple_SetCurrentRefmA((float)step_mA);
        }
    }

    if (g_speed_cap_enabled)
    {
        if (!g_speed_cap_started)
        {
            g_speed_cap_started = 1;
            g_speed_cap_step_idx = 0;
            g_speed_cap_ticks = 0;
            g_speed_cap_discard = 1;
            ControlSimple_SetMode(CONTROL_MODE_SPEED);
            ControlSimple_SetSpeedRefRpm(speed_capture_target_rpm(g_speed_cap_step_idx));
            printf("# SPEED_STEP,%u,%.0f,%u\r\n",
                   (unsigned)g_speed_cap_step_idx,
                   speed_capture_target_rpm(g_speed_cap_step_idx),
                   (unsigned)SPEED_CAP_DISCARD_TICKS);
        }
        else
        {
            g_speed_cap_ticks++;
            if (g_speed_cap_discard)
            {
                if (g_speed_cap_ticks >= SPEED_CAP_DISCARD_TICKS)
                {
                    g_speed_cap_discard = 0;
                    g_speed_cap_sum_rpm = 0.0f;
                    g_speed_cap_sum_iref = 0.0f;
                    g_speed_cap_sum_imeas = 0.0f;
                    g_speed_cap_sum_count = 0;
                }
            }
            else
            {
                g_speed_cap_sum_rpm += ControlSimple_GetSpeedRpm();
                g_speed_cap_sum_iref += ControlSimple_GetCurrentRefmA();
                g_speed_cap_sum_imeas += (float)ADCS.Cur[0];
                g_speed_cap_sum_count++;
            }

            if (g_speed_cap_ticks >= SPEED_CAP_HOLD_TICKS)
            {
                float avg_rpm = 0.0f;
                float avg_iref = 0.0f;
                float avg_imeas = 0.0f;
                if (g_speed_cap_sum_count > 0)
                {
                    avg_rpm = g_speed_cap_sum_rpm / (float)g_speed_cap_sum_count;
                    avg_iref = g_speed_cap_sum_iref / (float)g_speed_cap_sum_count;
                    avg_imeas = g_speed_cap_sum_imeas / (float)g_speed_cap_sum_count;
                }
                printf("SPEED_CAP,%.2f,%.2f,%.1f,%.1f\r\n",
                       speed_capture_target_rpm(g_speed_cap_step_idx),
                       avg_rpm,
                       avg_iref,
                       avg_imeas);

                g_speed_cap_ticks = 0;
                g_speed_cap_step_idx++;
                g_speed_cap_discard = 1;

                if (g_speed_cap_step_idx >= SPEED_CAP_STEP_COUNT)
                {
                    printf("# SPEED_DONE\r\n");
                    Test_StopSpeedCapture();
                }
                else
                {
                    ControlSimple_SetSpeedRefRpm(speed_capture_target_rpm(g_speed_cap_step_idx));
                    printf("# SPEED_STEP,%u,%.0f,%u\r\n",
                           (unsigned)g_speed_cap_step_idx,
                           speed_capture_target_rpm(g_speed_cap_step_idx),
                           (unsigned)SPEED_CAP_DISCARD_TICKS);
                }
            }
        }
    }
}
