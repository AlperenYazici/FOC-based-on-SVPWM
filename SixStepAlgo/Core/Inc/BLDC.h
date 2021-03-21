#include "stm32f3xx_it.h"
#include "main.h"
#include "math.h"
#include "string.h"

#define PI 3.14159
#define RAD2DEG		(PI/180.0) 
#define Mod_Index 0.5774
#define MagRatioModIndex (float)Mag/Mod_Index
#define Vref 1200.0
#define Mod_Index_2 1.0/1.732


typedef struct PID
{
	float kp;
	float ki;
	float kd;
	float ref;
	float* sensValue;
}PID;

typedef struct adcMeasures{
	uint32_t Ia;
	uint32_t Ib;
	uint32_t Ic;
	uint32_t vbus;
	uint32_t temperature;
	}adcMeasures;

typedef struct AlfaBeta
{
	float alfa;
	float beta;
}AlfaBeta;

typedef struct qd
{
	float q = 0;
	float d = 0;
	PID pid_q = { 0.07,0.01,0.0 , 600.0 , &q };
	PID pid_d ={ 0.07,0.01,0.0 , 0.0 , &d };
}qd;

typedef struct ab
{
	float a;
	float b;
	float c;
}ab;

typedef struct spaceVector
{
	double Amplitude = 1.0;
	double Teta = 0;
}spaceVector;

typedef struct Bldc_Error
{
	union
	{
		uint16_t value;
		struct
		{
			uint8_t wrongElAngle : 1;
		}bits;
	}svpwm;
}Bldc_Error;

class SinusVectorControl
{
	public:
	AlfaBeta ClarkeTransformation(ab I_in)
	{
		AlfaBeta I_out;
		I_in.c = - (I_in.a + I_in.b );
		I_out.alfa = I_in.a;
		I_out.beta =	(I_in.a + 2*I_in.b)/sqrt(3.0);
		return I_out;
	}
	qd ParkTransformation(AlfaBeta I_in , double ElAngle)
	{
		qd I_out;
//		ElAngle = fmod(ElAngle,60);
		ElAngle = ElAngle * PI / 180 ;
		I_out.d	=	cos(ElAngle)*I_in.alfa+sin(ElAngle)*I_in.beta;
		I_out.q	=	-sin(ElAngle)*I_in.alfa+cos(ElAngle)*I_in.beta;
		return I_out;
	}
	AlfaBeta InverseParkTransformation(qd V_in, double ElAngle)
	{
//		ElAngle = fmod(ElAngle,60);
		ElAngle = ElAngle * PI / 180 ;
		AlfaBeta V_out;
		V_out.alfa	=	cos(ElAngle)*V_in.d-sin(ElAngle)*V_in.q;
		V_out.beta	=	sin(ElAngle)*V_in.d+cos(ElAngle)*V_in.q;
		return V_out;
	}
	spaceVector calcRefSpaceVector(AlfaBeta V_in)
	{
		spaceVector V_out;
		V_out.Amplitude	=	sqrt(V_in.alfa*V_in.alfa+V_in.beta*V_in.beta);
		V_out.Amplitude = V_out.Amplitude>1.1?1.1:V_out.Amplitude;
		V_out.Teta	= atan( (V_in.beta/V_in.alfa) )*180/PI	;
	
		return V_out;
	}
	
};

typedef struct mosfetDuty
{
	uint32_t pwm_highside_1;
	uint32_t pwm_highside_2;
	uint32_t pwm_highside_3;
}mosfetDuty;

typedef struct vector_sector
{
	union
	{
		uint8_t value;
		struct
		{
			uint8_t A :1;
			uint8_t B :1;
			uint8_t C :1;
			uint8_t  :5;
		}Bits;
		
	}N;
	
}vector_sector;

class BLDC
{
	
	public:
		Bldc_Error Error;
	uint8_t switchingMode = 0 ;
	uint8_t speedCalcDone = 0;
	uint32_t speedElapsedTick = 0;
		vector_sector Sector;
	mosfetDuty pwmDuties;
	ab Iphase;
	adcMeasures focMeasures[5];
	AlfaBeta Istator;
	qd Irotor;
	ab Vphase;
	AlfaBeta Vstator;
	qd Vrotor;
	float Ia_offset;
	float Ib_offset;
	float Ic_offset;
	float Ia_avg;
	float Ib_avg;
	float Ic_avg;
	float X ;
	float Y ;
	float Z ;
	BLDC();
	spaceVector spaceVec;
	public:
	float Vd=12;
	float ElAngle=0;
	SinusVectorControl FOC;
	void SVPWM();
	void SVPWM_2();
	void setUs(double value);
	void setSpaceVecTeta(double value);
	double getUs( );
	double getSpaceVecTeta();
	void FOC_Task();
	double PID_step(PID pid);
	uint32_t svpwm2_Va;
	uint32_t svpwm2_Vb;
	uint32_t svpwm2_Vc;
//	void setElAngle(double value);
};
