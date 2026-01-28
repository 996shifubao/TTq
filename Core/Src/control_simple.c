#include "control_simple.h"
#include "moto_task.h"
#include "adc.h"
#include "friction_lut.h"
#include <math.h>
#include <stdio.h>
#include "main.h"

#define CONTROL_HZ 1000.0f
#define CONTROL_DT (1.0f / CONTROL_HZ)

#define PWM_HZ 80000.0f
#define CURRENT_LOOP_DIV 1
#define CURRENT_LOOP_DT (1.0f / (PWM_HZ / CURRENT_LOOP_DIV))

#define ENCODER_CPR 80000.0f
#define GEAR_RATIO 30.0f

#define I_MAX_A 2.0f
#define SPD_DEADBAND_RPM 3.0f
#define FRIC_GAIN 1.33f
#define SPEED_FILTER_MAX 1  //50
#define ACCEL_FILTER_N 150
#define TORQUE_GAIN 0.0f

typedef struct
{
    float  kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} PIController;

static uint8_t g_enabled = 0;
static ControlMode g_mode = CONTROL_MODE_TORQUE_COMP;
static float g_speed_ref_rpm = 0.0f;

static int32_t g_last_enc = 0;
static float g_speed_out = 0.0f;
static float g_last_speed_rpm = 0.0f;
static float g_last_speed_rpm_prev = 0.0f;
static float g_accel_rpm_s = 0.0f;
static float g_accel_filt_rpm_s = 0.0f;
static float g_accel_signed_rpm_s = 0.0f;
static float g_accel_signed_filt_rpm_s = 0.0f;
static uint8_t g_comp_enable = 1;
static float g_speed_filt_buf[SPEED_FILTER_MAX];
static uint16_t g_speed_filt_n = 20;
static uint16_t g_speed_filt_idx = 0;
static uint16_t g_speed_filt_count = 0;
static float g_speed_filt_sum = 0.0f;
static float g_accel_filt_buf[ACCEL_FILTER_N];
static uint16_t g_accel_filt_idx = 0;
static uint16_t g_accel_filt_count = 0;
static float g_accel_filt_sum = 0.0f;
static float g_accel_signed_filt_buf[ACCEL_FILTER_N];
static uint16_t g_accel_signed_filt_idx = 0;
static uint16_t g_accel_signed_filt_count = 0;
static float g_accel_signed_filt_sum = 0.0f;
static float g_last_i_ref_mA = 0.0f;
static int32_t g_last_enc_report = 0;
static volatile float g_i_ref_A = 0.0f;
static volatile float g_last_pwm_cmd = 0.0f;

extern PIController g_cur_pi;


static uint8_t g_override_enable = 0;
static float g_override_i_ref_A = 0.0f;

static PIController g_speed_pi = {
    .kp = 0.01f,    // 0.02
    .ki = 0.5f,     // 0.5
    .integrator = 0.0f,
    .out_min = -I_MAX_A,
    .out_max = I_MAX_A,
};

 PIController g_cur_pi = {
    .kp = 800.0f,     //300     800
    .ki = 200000.0f,   //20000    40000
    .integrator = 0.0f,
    .out_min = -MaxPWM,
    .out_max = MaxPWM,
};

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static void accel_filter_reset(void)
{
    g_accel_filt_idx = 0;
    g_accel_filt_count = 0;
    g_accel_filt_sum = 0.0f;
    for (uint16_t i = 0; i < ACCEL_FILTER_N; i++)
    {
        g_accel_filt_buf[i] = 0.0f;
    }
    g_accel_signed_filt_idx = 0;
    g_accel_signed_filt_count = 0;
    g_accel_signed_filt_sum = 0.0f;
    for (uint16_t i = 0; i < ACCEL_FILTER_N; i++)
    {
        g_accel_signed_filt_buf[i] = 0.0f;
    }
}

static float accel_filter_update(float x)
{
    if (ACCEL_FILTER_N <= 1)
    {
        return x;
    }

    if (g_accel_filt_count < ACCEL_FILTER_N)
    {
        g_accel_filt_count++;
    }
    else
    {
        g_accel_filt_sum -= g_accel_filt_buf[g_accel_filt_idx];
    }

    g_accel_filt_buf[g_accel_filt_idx] = x;
    g_accel_filt_sum += x;
    g_accel_filt_idx++;
    if (g_accel_filt_idx >= ACCEL_FILTER_N)
    {
        g_accel_filt_idx = 0;
    }

    return g_accel_filt_sum / (float)g_accel_filt_count;
}

static float accel_signed_filter_update(float x)
{
    if (ACCEL_FILTER_N <= 1)
    {
        return x;
    }

    if (g_accel_signed_filt_count < ACCEL_FILTER_N)
    {
        g_accel_signed_filt_count++;
    }
    else
    {
        g_accel_signed_filt_sum -= g_accel_signed_filt_buf[g_accel_signed_filt_idx];
    }

    g_accel_signed_filt_buf[g_accel_signed_filt_idx] = x;
    g_accel_signed_filt_sum += x;
    g_accel_signed_filt_idx++;
    if (g_accel_signed_filt_idx >= ACCEL_FILTER_N)
    {
        g_accel_signed_filt_idx = 0;
    }

    return g_accel_signed_filt_sum / (float)g_accel_signed_filt_count;
}

static void speed_filter_reset(void)
{
    g_speed_filt_idx = 0;
    g_speed_filt_count = 0;
    g_speed_filt_sum = 0.0f;
    for (uint16_t i = 0; i < g_speed_filt_n; i++)
    {
        g_speed_filt_buf[i] = 0.0f;
    }
}

static float speed_filter_update(float x)
{
    if (g_speed_filt_n <= 1)
    {
        return x;
    }

    if (g_speed_filt_count < g_speed_filt_n)
    {
        g_speed_filt_count++;
    }
    else
    {
        g_speed_filt_sum -= g_speed_filt_buf[g_speed_filt_idx];
    }

    g_speed_filt_buf[g_speed_filt_idx] = x;
    g_speed_filt_sum += x;
    g_speed_filt_idx++;
    if (g_speed_filt_idx >= g_speed_filt_n)
    {
        g_speed_filt_idx = 0;
    }

    return g_speed_filt_sum / (float)g_speed_filt_count;
}

static float pi_update(PIController *pi, float err, float dt)
{
    float p = pi->kp * err;
    pi->integrator += pi->ki * err * dt;
    pi->integrator = clampf(pi->integrator, pi->out_min, pi->out_max);
    return clampf(p + pi->integrator, pi->out_min, pi->out_max);
}

static float pi_update_aw(PIController *pi, float err, float dt)
{
    float p = pi->kp * err;
    float i_next = pi->integrator + pi->ki * err * dt;
    float u_unsat = p + i_next;
    float u_sat = clampf(u_unsat, pi->out_min, pi->out_max);
    if ((u_unsat == u_sat) ||
        ((u_sat >= pi->out_max) && (err < 0.0f)) ||
        ((u_sat <= pi->out_min) && (err > 0.0f)))
    {
        pi->integrator = clampf(i_next, pi->out_min, pi->out_max);
    }
    return u_sat;
}

static float interp_table(const float *x, const float *y, int len, float xq)
{
    if (xq <= x[0])
    {
        return y[0];
    }
    if (xq >= x[len - 1])
    {
        return y[len - 1];
    }
    int idx = 0;
    while (idx < (len - 1) && xq > x[idx + 1])
    {
        idx++;
    }
    float t = (xq - x[idx]) / (x[idx + 1] - x[idx]);
    return y[idx] + t * (y[idx + 1] - y[idx]);
}

static float friction_compensate_mA(float speed_rpm)
{
    float abs_rpm = fabsf(speed_rpm);
    if (abs_rpm < SPD_DEADBAND_RPM)
    {
        return 0.0f;
    }
    float dir = (speed_rpm >= 0.0f) ? 1.0f : -1.0f;
    if (dir > 0.0f)
    {
        return FRIC_GAIN * dir * interp_table(fric_pos_spd_rpm, fric_pos_mA, FRIC_POS_LEN, abs_rpm);
    }
    return FRIC_GAIN * dir * interp_table(fric_neg_spd_rpm, fric_neg_mA, FRIC_NEG_LEN, abs_rpm);
}

void ControlSimple_Init(void)
{
    g_enabled = 1;
    g_mode = CONTROL_MODE_TORQUE_COMP;
    g_speed_ref_rpm = 0.0f;
    g_last_enc = 0;
    g_speed_out = 0.0f;
    g_last_speed_rpm = 0.0f;
    g_last_speed_rpm_prev = 0.0f;
    g_accel_rpm_s = 0.0f;
    g_accel_filt_rpm_s = 0.0f;
    g_accel_signed_rpm_s = 0.0f;
    g_accel_signed_filt_rpm_s = 0.0f;
    g_speed_pi.integrator = 0.0f;
    g_cur_pi.integrator = 0.0f;
    g_i_ref_A = 0.0f;
    g_override_enable = 0;
    g_override_i_ref_A = 0.0f;
    accel_filter_reset();
    FilterWindow(1);
}

void ControlSimple_Enable(uint8_t on)
{
    g_enabled = on ? 1U : 0U;
    if (!g_enabled)
    {
        g_speed_pi.integrator = 0.0f;
        g_cur_pi.integrator = 0.0f;
        g_i_ref_A = 0.0f;
        g_override_enable = 0;
        g_override_i_ref_A = 0.0f;
        moto_single_run(MotoA, ABS, 0);
    }
}

void ControlSimple_SetMode(ControlMode mode)
{
    g_mode = mode;
    g_speed_pi.integrator = 0.0f;
    g_cur_pi.integrator = 0.0f;
    g_i_ref_A = 0.0f;
}

void ControlSimple_SetSpeedRefRpm(float rpm)
{
    g_speed_ref_rpm = rpm;
}

void ControlSimple_SetCurrentRefmA(float mA)
{
    g_override_i_ref_A = mA * 0.001f;
}

void ControlSimple_EnableCurrentOverride(uint8_t on)
{
    g_override_enable = on ? 1U : 0U;
}

float ControlSimple_GetSpeedRpm(void)
{
    return g_last_speed_rpm;
}

float ControlSimple_GetCurrentRefmA(void)
{
    return g_last_i_ref_mA;
}

float ControlSimple_GetCurrentRefmAClamped(void)
{
    return g_i_ref_A * 1000.0f;
}

float ControlSimple_GetCurrentPIIntegrator(void)
{
    return g_cur_pi.integrator;
}

float ControlSimple_GetCurrentPwmCmd(void)
{
    return g_last_pwm_cmd;
}

int32_t Control_GetSpeedOutMotorRpm(void)
{
    return (int32_t)(g_last_speed_rpm * 100.0f);
}

int32_t Control_GetAccelFilt(void)
{
    return (int32_t)g_accel_filt_rpm_s;
}

void FilterWindow(uint16_t n)
{
    if (n < 1)
    {
        n = 1;
    }
    if (n > SPEED_FILTER_MAX)
    {
        n = SPEED_FILTER_MAX;
    }
    g_speed_filt_n = n;
    speed_filter_reset();
}

void ControlSimple_Step(int32_t enc_count)
{
		 float W =0.06f;
    if (!g_enabled)
    {
        g_last_pwm_cmd = 0.0f;
        return;
    }

    int32_t delta = enc_count - g_last_enc;
    g_last_enc = enc_count;

    float speed_m = ((float)delta) * (2.0f * 3.1415926f) * CONTROL_HZ / ENCODER_CPR;
    float speed_out = speed_m / GEAR_RATIO;
    g_speed_out = speed_out;
    g_last_enc_report = enc_count;

    float speed_rpm_raw = speed_out * GEAR_RATIO * (60.0f / (2.0f * 3.1415926f));
    float speed_rpm = speed_filter_update(speed_rpm_raw);
    g_last_speed_rpm_prev = g_last_speed_rpm;
    g_last_speed_rpm = speed_rpm;
    float speed_abs = fabsf(g_last_speed_rpm);
    float speed_abs_prev = fabsf(g_last_speed_rpm_prev);
    g_accel_rpm_s = (speed_abs - speed_abs_prev) / CONTROL_DT;
    g_accel_filt_rpm_s = accel_filter_update(g_accel_rpm_s);
    g_accel_signed_rpm_s = (g_last_speed_rpm - g_last_speed_rpm_prev) / CONTROL_DT;
    g_accel_signed_filt_rpm_s = accel_signed_filter_update(g_accel_signed_rpm_s);

    float i_ref_A = 0.0f;
    float i_ref_mA = 0.0f;

    if (g_override_enable)
    {
        i_ref_A = clampf(g_override_i_ref_A, -I_MAX_A, I_MAX_A);
        i_ref_mA = i_ref_A * 1000.0f;
    }
    else if (g_mode == CONTROL_MODE_SPEED)
    {
        float err_rpm = g_speed_ref_rpm - speed_rpm;
        i_ref_A = pi_update(&g_speed_pi, err_rpm, CONTROL_DT);
        i_ref_A = clampf(i_ref_A, -I_MAX_A, I_MAX_A);
        i_ref_mA = i_ref_A * 1000.0f;
    }
    else
    {
        if (g_accel_filt_rpm_s < -1.0f )
        {
            g_comp_enable = 1;

        }
        else if (g_accel_filt_rpm_s > 1.0f)
        {
            g_comp_enable = 1;
        }

        if (g_comp_enable)
        {
            i_ref_mA = friction_compensate_mA(speed_rpm);
            i_ref_mA += TORQUE_GAIN * g_accel_signed_filt_rpm_s;
					  i_ref_A = i_ref_mA * 0.001f;
            i_ref_A = clampf(i_ref_A, -I_MAX_A, I_MAX_A);
					
//										  i_ref_mA = g_last_speed_rpm*W;
//					             i_ref_A = i_ref_mA * 0.001f;
        }
    }

    g_last_i_ref_mA = i_ref_mA;
    g_i_ref_A = i_ref_A;
}

void CurrentLoopStep(int16_t cur_mA)
{
    static uint8_t div = 0;
    if (!g_enabled)
    {
        g_last_pwm_cmd = 0.0f;
        return;
    }
    if (++div < CURRENT_LOOP_DIV)
    {
        return;
    }
    div = 0;

//    if (fabsf(g_i_ref_A) < 0.001f)
//    {
//        g_cur_pi.integrator = 0.0f;
//        moto_single_run(MotoA, ABS, 0);
//        return;
//    }

    float i_meas_A = ((float)cur_mA) * 0.001f;
    float err_A = g_i_ref_A - i_meas_A;
    float pwm_cmd = pi_update_aw(&g_cur_pi, err_A, CURRENT_LOOP_DT);
    g_last_pwm_cmd = pwm_cmd;

    if (fabsf(pwm_cmd) < 1.0f)
    {
        g_last_pwm_cmd = 0.0f;
        moto_single_run(MotoA, ABS, 0);
        return;
    }  
    if (pwm_cmd >= 0.0f)
    {
        moto_single_run(MotoA, Forward, (uint16_t)fabsf(pwm_cmd));

    }
    else
    {
        moto_single_run(MotoA, Back, (uint16_t)fabsf(pwm_cmd));

    }
}

void LogPrint(void)
{
    if (!g_enabled)
    {
        g_last_pwm_cmd = 0.0f;
        return;
    }
//    printf("t_ms=%lu,enc=%ld,spd_rpm=%.2f,iref_mA=%.1f,imeas_mA=%d\r\n",
//           (unsigned long)HAL_GetTick(),
//					 (long)g_last_enc_report,
//           g_last_speed_rpm,
//           g_last_i_ref_mA,
//           (int)ADCS.Cur[0]);

				 printf("t_ms=%lu,speed_ref_rpm=%ld,spd_rpm=%.2f,iref_mA=%.1f,imeas_mA=%d\r\n",
		 (unsigned long)HAL_GetTick(),
		 (long)g_speed_ref_rpm,
		 g_last_speed_rpm,
		 g_last_i_ref_mA,
		 (int)ADCS.Cur[0]);
					 
}





