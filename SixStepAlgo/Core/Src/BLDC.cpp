#include "BLDC.h"


#define SQRT3   1.732
#define SQRT2   1.414
#define InvSqrt3  1.0/SQRT3
#define twiceInvSqrt3  2.0*InvSqrt3
#define Vd  12.0
#define InvSqrt3rateVd		(float)InvSqrt3/Vd
#define twiceInvSqrt3rateVd		(float)twiceInvSqrt3/Vd
BLDC* ptrBLDC = NULL;

#define Ts 1.0/20000

extern uint8_t hall_state;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
double _duty=0.1;
extern TIM_HandleTypeDef htim1;
//float hall_state_degree[6] = {240 , 300 , 0,60 , 120 , 180  };
double hall_state_degree[6] = {0,300,240,180,120 , 60 };

BLDC::BLDC()
{
	ptrBLDC = this;
//	spaceVec = (spaceVector){0,0}; // Amplitude - teta


}
void BLDC::SVPWM()
{
		float Ang_R = 0;
		double Pa=0 , Pb=0 ,Pc=0;
		Ang_R = fmod(ElAngle, 60) * RAD2DEG;
//		float Ta = (((float)Mag * cos(Ang_R)) / Mod_Index) - ((((float)Mag * sin(Ang_R)) / Mod_Index) * (1.0 / sqrt(3.0)));
//     float   Tb = (((float)Mag * sin(Ang_R)) / Mod_Index) * (2.0 / sqrt(3.0));
//      float  Tz = 1 - (Ta + Tb);
//		double	Ta = ((getUs() * sin((PI/3)-Ang_R))/Mod_Index);
//		double Tb = ((getUs() * sin(Ang_R))/Mod_Index);
//		double Tz = 1.0 - (Ta + Tb);
	
		if (ElAngle>=0 && ElAngle<60)
		{
				double	Ta = (Vstator.alfa)/Vd - InvSqrt3rateVd*Vstator.beta;
				double Tb = twiceInvSqrt3rateVd*Vstator.beta;
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2 + Ta + Tb;
				Pb = Tz/2 + Tb;
				Pc = Tz/2;
		}
		else if ( ElAngle >= 60 && ElAngle < 120 )
		{
				double	Ta = (Vstator.alfa)/Vd + InvSqrt3rateVd*Vstator.beta;
				double Tb = InvSqrt3rateVd*Vstator.beta -(Vstator.alfa)/Vd;
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2 + Ta;
				Pb = Tz/2 + Ta + Tb;
				Pc = Tz/2;
		}
		else if (ElAngle >= 120 && ElAngle < 180 )
		{
				double	Ta = twiceInvSqrt3rateVd*(Vstator.beta);
				double Tb = -InvSqrt3rateVd*Vstator.beta -(Vstator.alfa)/Vd;
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2;
				Pb = Tz/2 + Ta + Tb;
				Pc = Tz/2 + Tb;
		}
		else if (ElAngle >= 180 && ElAngle < 240) 
		{
				double	Ta = - ((Vstator.alfa)/Vd - InvSqrt3rateVd*Vstator.beta);
				double Tb = -twiceInvSqrt3rateVd*Vstator.beta;
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2;
				Pb = Tz/2 + Ta;
				Pc = Tz/2 + Ta + Tb;
		}
		else if( ElAngle >= 240 && ElAngle < 300) 
		{
				double	Ta = - ((Vstator.alfa)/Vd + InvSqrt3rateVd*Vstator.beta);
				double Tb = -( InvSqrt3rateVd*Vstator.beta -(Vstator.alfa)/Vd);
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2 + Tb;
				Pb = Tz/2;
				Pc = Tz/2 + Ta + Tb;
		}
		else if( ElAngle >= 300 && ElAngle < 360) 
		{
				double	Ta = -twiceInvSqrt3rateVd*(Vstator.beta);
				double Tb = -(-InvSqrt3rateVd*Vstator.beta -(Vstator.alfa)/Vd);
				double Tz = 1.0 - (Ta + Tb);
				Pa = Tz/2 + Ta + Tb;
				Pb = Tz/2;
				Pc = Tz/2 + Ta;
		}
		else
		{
			Error.svpwm.bits.wrongElAngle = 1;
		}
		pwmDuties.pwm_highside_1 = Vref+(Vref*(Pa-0.5)*_duty);
		pwmDuties.pwm_highside_1 = pwmDuties.pwm_highside_1>TIM1->ARR?TIM1->ARR:(pwmDuties.pwm_highside_1);
		pwmDuties.pwm_highside_2 = Vref+(Vref*(Pb-0.5)*_duty);
		pwmDuties.pwm_highside_2 = pwmDuties.pwm_highside_2>TIM1->ARR?TIM1->ARR:pwmDuties.pwm_highside_2;
		pwmDuties.pwm_highside_3 = Vref+(Vref*(Pc-0.5)*_duty);
		pwmDuties.pwm_highside_3 = pwmDuties.pwm_highside_3>TIM1->ARR?TIM1->ARR:pwmDuties.pwm_highside_3;
		TIM1->CCR1 = pwmDuties.pwm_highside_1;
		TIM1->CCR2 =  pwmDuties.pwm_highside_2;
		TIM1->CCR3 = pwmDuties.pwm_highside_3;
}


void BLDC::SVPWM_2()
{
	float Ta = 0;
	float Tb =0;
	float Tc =0;
	float Va = 0;
	float Vb =0;
	float Vc =0;
	float condition1 = (-SQRT3*Vstator.alfa + Vstator.beta);
	float condition2 = (SQRT3*Vstator.alfa + Vstator.beta);
	
//	if (Vstator.beta>0) Sector.N.Bits.A = 1; else Sector.N.Bits.A = 0;
//	if (-condition1>0) Sector.N.Bits.B = 1; else Sector.N.Bits.B = 0;
//	if (condition2>0) Sector.N.Bits.C = 1; else Sector.N.Bits.C = 0;
//	if(Sector.N.value == 0 ) Error.svpwm.bits.wrongElAngle = 1;
	if (ElAngle>=0 && ElAngle<60)
		{
			Sector.N.value = 3;
		}
		else if ( ElAngle >= 60 && ElAngle < 120 )
		{
			Sector.N.value = 1;
		}
		else if (ElAngle >= 120 && ElAngle < 180 )
		{
			Sector.N.value = 5;
		}
		else if (ElAngle >= 180 && ElAngle < 240) 
		{
			Sector.N.value = 4;
		}
		else if( ElAngle >= 240 && ElAngle < 300) 
		{
			Sector.N.value = 6;
		}
		else if( ElAngle >= 300 && ElAngle < 360) 
		{
			Sector.N.value = 2;
		}
	 X = (condition1) / (SQRT2*Vd);
	 Y = (condition2) / (SQRT2*Vd);
	 Z = 2*(Vstator.beta) / (SQRT2*Vd);
	switch(Sector.N.value)
	{
		case 0:
		{
			break;
		}
		case 1: 
		{
			float T1 = Z, T2 =Y; 
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Tb;
			Vb = Ta;
			Vc = Tc;
			break;
		}
		case 2: 
		{
			float T1 = Y, T2 =-X; 
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Ta;
			Vb = Tc;
			Vc = Tb;
			break;
		}
		case 3:    
		{
			float T1 = -Z, T2 =X;
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Ta;
			Vb = Tb;
			Vc = Tc;
			break;
		}
		case 4:
		{
			float T1 = -X, T2 =Z; 
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Tc;
			Vb = Tb;
			Vc = Ta;
			break;
		}
		case 5:    
		{
			float T1 = X, T2 = -Y; 
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Tc;
			Vb = Ta;
			Vc = Tb;
			break;
		}
		case 6: 
		{
			float T1 = -Y, T2 = -Z; 
			Ta = (1 - T1 -T2)/4;
			Tb = (Ta - T1)/2;
			Tc = (Tb - T2)/2;
			Va = Tb;
			Vb = Tc;
			Vc = Ta;
			break;
		}
		case 7: 
		{
			break;
		}
	}
	
	Va = Vref+(Vref*(Va-0.5)*_duty);
	Va = Va>(TIM1->ARR-1) ? (TIM1->ARR-1) : Va;
	
	Vb = Vref+(Vref*(Vb-0.5)*_duty);
	Vb = Vb>(TIM1->ARR-1) ? (TIM1->ARR-1) : Vb;
	
	Vc = Vref+(Vref*(Vc-0.5)*_duty);
	Vc = Vc>(TIM1->ARR-1) ? (TIM1->ARR-1) : Vc;
	svpwm2_Va =Va;
	svpwm2_Vb =Vb;
	svpwm2_Vc =Vc;
////	
	TIM1->CCR1 = Va;
	TIM1->CCR2 = Vb;
	TIM1->CCR3 = Vc;
}
void BLDC::FOC_Task()
{
	Istator = FOC.ClarkeTransformation(Iphase);
	Irotor = FOC.ParkTransformation(Istator , ElAngle);
	Vrotor.q = PID_step(Irotor.pid_q);
	Vrotor.d = PID_step(Irotor.pid_d);
	Vstator = FOC.InverseParkTransformation(Vrotor,ElAngle);
	//spaceVec	=	FOC.calcRefSpaceVector(Vstator);
}
void BLDC::setUs(double value)
{
	if(value>1.1)
	{
		spaceVec.Amplitude = 1.1;
	}
	else if(value<0.0)
	{
		spaceVec.Amplitude = 0.0;
	}
	else
	{
		spaceVec.Amplitude = value;
	}
}

void BLDC::setSpaceVecTeta(double value)
{
	if(value>360)
	{
		spaceVec.Amplitude = value - 360;
	}
	else if(value<0)
	{
		spaceVec.Amplitude = 360 + value;
	}
	else
	{
		spaceVec.Amplitude = value;
	}
}
//void BLDC::setElAngle(double value)
//{
//	if(value>360)
//	{
//		ElAngle = value - 360;
//	}
//	else if(value<0)
//	{
//		ElAngle = 360 + value;
//	}
//	else
//	{
//		ElAngle = (double)value;
//	}
//	return;
//}
double BLDC:: getUs( )
{
	return spaceVec.Amplitude;
}

double BLDC::getSpaceVecTeta( )
{
	return spaceVec.Teta;
}


double BLDC::PID_step(PID pid)
{
	float error = pid.ref - *pid.sensValue;
	// ... Devam edecek ....
	
	return pid.kp *error;
}
uint8_t firstHallChange = 0;

extern "C" void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(firstHallChange != 0)
	{
		ptrBLDC->speedElapsedTick = TIM2->CCR1;
		ptrBLDC->speedCalcDone = 1;
	}
  /* USER CODE END TIM2_IRQn 0 */
	
  HAL_TIM_IRQHandler(&htim2);
	
  /* USER CODE BEGIN TIM2_IRQn 1 */
//	hall_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) | (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)<<1) 
//									| (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)<<2);
//	switch(hall_state)
//	{
//		case 0: ptrBLDC->ElAngle=0.0;		 break;
//		case 1: ptrBLDC->ElAngle=hall_state_degree[5];	 break;
//		case 2: ptrBLDC->ElAngle=hall_state_degree[3];   break;
//		case 3: ptrBLDC->ElAngle=hall_state_degree[4]; 	 break;
//		case 4: ptrBLDC->ElAngle=hall_state_degree[1];  	 break;
//		case 5:	ptrBLDC->ElAngle=hall_state_degree[0];  	 break;
//		case 6: ptrBLDC->ElAngle=hall_state_degree[2];   break;
//		case 7: ptrBLDC->ElAngle=0.0; 		 break;
//	}
	
//	ptrBLDC->SVPWM();
	firstHallChange = 1;
//		ptrBLDC->SVPWM();
		
  /* USER CODE END TIM2_IRQn 1 */
}


/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
//		static uint8_t offsetAdcTuneIndx=0;
//			if(offsetAdcTuneIndx>1)
//			{
//				float avrgOffset_a = 0;
//				float avrgOffset_b = 0;
//				float avrgOffset_c = 0;
//				for(int i=0;i<5;i++)
//				{
//					avrgOffset_a += ptrBLDC->focMeasures[i].Ia;
//					avrgOffset_b += ptrBLDC->focMeasures[i].Ib;
//					avrgOffset_c += ptrBLDC->focMeasures[i].Ic;
//				}
//				ptrBLDC->Ia_avg =avrgOffset_a / 5;
//				ptrBLDC->Ib_avg =avrgOffset_b / 5;
//				ptrBLDC->Ic_avg =avrgOffset_c / 5;
//				
//				ptrBLDC->Iphase.a = (float)(ptrBLDC->Ia_offset - ptrBLDC->Ia_avg)*1.0;
//				ptrBLDC->Iphase.b = ptrBLDC->Ia_offset - ptrBLDC->Ib_avg;
//				ptrBLDC->Iphase.c = ptrBLDC->Ia_offset - ptrBLDC->Ic_avg;
//				ptrBLDC->FOC_Task();
//				ptrBLDC->SVPWM();
//			}
//			else
//			{
//				if(offsetAdcTuneIndx!=0)
//				{
//					float avrgOffset = 0;
//					for(int i=0;i<5;i++)
//					{
//						avrgOffset += ptrBLDC->focMeasures[i].Ia;
//					}
//					avrgOffset /= 5;
//					ptrBLDC->Ia_offset += avrgOffset;
//				}
//				offsetAdcTuneIndx++;
//				if(offsetAdcTuneIndx == 2)
//				{
//					ptrBLDC->Ia_offset /= 1;

//				}
//				
//			}
				/* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}


void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
	hall_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) | (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)<<1) 
									| (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)<<2);

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */
	ptrBLDC->focMeasures[0].Ia = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
	ptrBLDC->focMeasures[0].Ib = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
	ptrBLDC->focMeasures[0].Ic = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
	static uint8_t offsetAdcTuneIndx=0;
	if(offsetAdcTuneIndx<(6+5))
	{
		if(offsetAdcTuneIndx>5)
		{
			static float avrgOffset = 0;
			ptrBLDC->Ia_offset += ptrBLDC->focMeasures[0].Ia;
		
			ptrBLDC->Ib_offset += ptrBLDC->focMeasures[0].Ib;
			
			ptrBLDC->Ic_offset += ptrBLDC->focMeasures[0].Ic;
		}
		offsetAdcTuneIndx++;
		
		if(offsetAdcTuneIndx == 11)
		{
			ptrBLDC->Ia_offset /= 5;
			ptrBLDC->Ib_offset /= 5;
			ptrBLDC->Ic_offset /= 5;
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
		}
	}
	else
	{
		switch(hall_state)
		{
			case 0: ptrBLDC->ElAngle=0.0;		 break;
			case 1: ptrBLDC->ElAngle=hall_state_degree[5];	 break;
			case 2: ptrBLDC->ElAngle=hall_state_degree[3];   break;
			case 3: ptrBLDC->ElAngle=hall_state_degree[4]; 	 break;
			case 4: ptrBLDC->ElAngle=hall_state_degree[1];  	 break;
			case 5:	ptrBLDC->ElAngle=hall_state_degree[0];  	 break;
			case 6: ptrBLDC->ElAngle=hall_state_degree[2];   break;
			case 7: ptrBLDC->ElAngle=0.0; 		 break;
		}
		ptrBLDC->Iphase.a = (float)(ptrBLDC->Ia_offset - ptrBLDC->focMeasures[0].Ia)*1.0;
		ptrBLDC->Iphase.b = (ptrBLDC->Ib_offset - ptrBLDC->focMeasures[0].Ib);
//		ptrBLDC->Iphase.c = ptrBLDC->Ic_offset - ptrBLDC->focMeasures[0].Ic;
		ptrBLDC->Iphase.c = - ptrBLDC->Iphase.a - ptrBLDC->Iphase.b;
		ptrBLDC->FOC_Task();
				if(ptrBLDC->switchingMode==0)
		ptrBLDC->SVPWM();
				else
		ptrBLDC->SVPWM_2();	
	}
  /* USER CODE END ADC1_IRQn 1 */
}
