#ifndef __MOTO_TASK_H
#define __MOTO_TASK_H

#include "stm32H7xx.h"

#define MaxPWM 2999

#define U0H_PWM(x);   {TIM1->CCR1=x;}
#define V0H_PWM(x);   {TIM1->CCR2=x;}
#define W0H_PWM(x);   {TIM1->CCR3=x;}

#define U1H_PWM(x);   {TIM8->CCR1=x;}
#define V1H_PWM(x);   {TIM8->CCR2=x;}
#define W1H_PWM(x);   {TIM8->CCR3=x;}

typedef enum 
{
	Back = 0,
	Forward = 1,
	ABS = 2,
	STOP = 3,
}MotoRunDir;

typedef enum
{
	MotoA = 0,
	MotoB = 1,
	MotoC = 2,
}moto;

void moto_single_run(uint8_t moto,MotoRunDir dir,uint16_t pwm);

#define	Clr_G4   GPIOG->BSRR=(1ul<<4 )<<16;


#endif
