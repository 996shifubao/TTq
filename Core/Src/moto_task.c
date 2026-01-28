#include "moto_task.h"
#include <stdio.h>

uint8_t MotoAdir=0;        // 电机旋转方向反转
uint8_t MotoBdir=0;        
uint8_t MotoCdir=0;

void moto_single_run(uint8_t moto,MotoRunDir dir,uint16_t pwm) // 
{
	  if(moto==MotoA)
		{
			  
				if((dir==Forward && MotoAdir==0) || (dir==Back && MotoAdir==1)) // U0H->V0L ADCIN8   AD1V
				{ 
					  V0H_PWM(pwm);
					  W0H_PWM(0);
				}
				else if((dir==Forward && MotoAdir==1) || (dir==Back && MotoAdir==0)) // V0H->U0L ADCIN14
				{
						V0H_PWM(0);   // U0H -> 0
						W0H_PWM(pwm); // V0H
				}
				else if(dir==ABS)
				{
					  V0H_PWM(0);
					  W0H_PWM(0);
				}
				else if(dir==STOP)
				{
					  V0H_PWM(MaxPWM);
					  W0H_PWM(MaxPWM);
				}
		} 
		else if(moto==MotoB)
		{
			  
				if((dir==Forward && MotoBdir==0) || (dir==Back && MotoBdir==1)) // W0H->U1L ADCIN15
				{ 
						
            W1H_PWM(0);   // W0H -> 0
						U0H_PWM(pwm); // U1H
				}
				else if((dir==Forward && MotoBdir==1) || (dir==Back && MotoBdir==0)) // U1H->W0L  ADCIN9
				{
						U0H_PWM(0);   // U1H -> 0
						W1H_PWM(pwm); // W0H
				}
				else if(dir==ABS)
				{
					  W1H_PWM(0);
					  U0H_PWM(0);
				}
				else if(dir==STOP)
				{
					  W1H_PWM(MaxPWM);
					  U0H_PWM(MaxPWM);
				}
		}
		else if(moto==MotoC)
		{
			  
				if((dir==Forward && MotoCdir==0) || (dir==Back && MotoCdir==1)) // V1H->W1L  ADCIN4
				{ 
						V1H_PWM(0);   // V1H -> 0
						U1H_PWM(pwm); // W1H
				}
				else if((dir==Forward && MotoCdir==1) || (dir==Back && MotoCdir==0)) // W1H->V1L  ADCIN10
				{
						
            U1H_PWM(0);   // W1H -> 0
						V1H_PWM(pwm); // V1H
				}
				else if(dir==ABS) // 
				{
					  U1H_PWM(0);
					  V1H_PWM(0);
				}
				else if(dir==STOP) 
				{
					  U1H_PWM(MaxPWM);
					  V1H_PWM(MaxPWM);
				}
		}
}