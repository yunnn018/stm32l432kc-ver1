/*
 * control1.c
 *
 *  Created on: May 9, 2025
 *      Author: PESP
 */

#include "stm32l4xx_hal.h"
#include <control1.h>
#include <math.h>
#include <tim.h>


//volatile int16_t    Iq_ref;
//const int wm_ref;
//const int Iq_ref_ini;

#define BATCH_DATA_LEN 32
uint32_t DataBuffer[BATCH_DATA_LEN];
uint16_t IADC1;
uint16_t IADC2;

#define SCALE			32768
#define SCALE_SHIFT		15
#define Ls	((uint16_t)(0.00059 * SCALE))
/****************** 延時函數 ******************/
void delay_ms(uint16_t time)
{
   uint16_t i=0;
   while(time--)
   {
      i=12000;
      while(i--) ;
   }
}

Triangles Tri_Function(uint16_t theta)
{
    Triangles output;
    uint16_t index;
    uint8_t quadrant;
    int16_t sin_val, cos_val;

    theta %= 10000;  // 角度一圈
    quadrant = theta / 2500;  // 判斷象限

    // 每象限對應 312 點（最大 index = 311）
    index = (theta % 2500) * 311 / 2500;
    if (index >= 311) index = 311;

    switch (quadrant) {
        case 0:  // 0° ~ 90°
            sin_val =  sintable[index];
            cos_val =  sintable[311 - index];
            break;
        case 1:  // 90° ~ 180°
            sin_val =  sintable[311 - index];
            cos_val = -sintable[index];
            break;
        case 2:  // 180° ~ 270°
            sin_val = -sintable[index];
            cos_val = -sintable[311 - index];
            break;
        case 3:  // 270° ~ 360°
        default:
            sin_val = -sintable[311 - index];
            cos_val =  sintable[index];
            break;
    }

    output.sin = sin_val;
    output.cos = cos_val;
    return output;
}

Currents ADC_AverageValue()
{
		Currents avg;
		uint32_t sum1=0, sum2=0;
		for(uint8_t i=0;i<BATCH_DATA_LEN;i += 2)
		{
			sum1 += DataBuffer[i];     // Channel5 (IADC1)
			sum2 += DataBuffer[i + 1]; // Channel6 (IADC2)
		}
		avg.IADC1 = (sum1 >> 4);
		avg.IADC2 = (sum2 >> 4);
		return avg;
}

#define OFFSET_WINDOW 50
static int32_t offset_sum1 = 0, offset_sum2 = 0;
static int16_t offset_buf1[OFFSET_WINDOW], offset_buf2[OFFSET_WINDOW];
static uint16_t offset_index = 0, offset_count = 0;
static int16_t Ioffset_temp, Ioffset_prev = 0;
static int16_t adc1_diff = 0, adc2_diff = 0;
static int16_t adc1max = 0, adc2max = 0;
static int16_t adc1min = 0, adc2min = 0;
static int16_t Ioffset1, Ioffset2, adc_offset1, adc_offset2;

Currents Update_Current_Offset(Currents *adc_avg,Currents *Idq,uint16_t msecCnt)
{
	Currents Ioffset;

    if (offset_count >= OFFSET_WINDOW) {
        offset_sum1 -= offset_buf1[offset_index];
        offset_sum2 -= offset_buf2[offset_index];
    }else {
    	offset_count++;
    }

    offset_buf1[offset_index] = adc_avg->IADC1;
    offset_buf2[offset_index] = adc_avg->IADC2;
    offset_sum1 += adc_avg->IADC1;
    offset_sum2 += adc_avg->IADC2;
    offset_index = (offset_index + 1) % OFFSET_WINDOW;

    /*** offset變動時重置極值 ***/
	Ioffset_temp = offset_sum1 / offset_count;
    if (abs(Ioffset_temp - Ioffset_prev) >10){
    	adc1max = 0; adc1min = 0;
    	adc2max = 0; adc2min = 0;
    }
    Ioffset_prev = Ioffset_temp;

    /*** 決定 offset 來源 ***/
    if (offset_count < OFFSET_WINDOW) {
        // 初期靜態offset
    	Ioffset1 = offset_sum1 / offset_count;
    	Ioffset2 = offset_sum2 / offset_count;
        Ioffset.IADC1 = Ioffset1;
        Ioffset.IADC2 = Ioffset2;
    } else {
        // 極值中點法補償偏差
    	adc_offset1 = (adc1max + adc1min) >> 1;
    	adc_offset2 = (adc2max + adc2min) >> 1;
        Ioffset.IADC1 = Ioffset1 + adc_offset1;
        Ioffset.IADC2 = Ioffset2 + adc_offset2;
        if (Idq->iq < 200){
            Ioffset.IADC1 = Ioffset1;
            Ioffset.IADC2 = Ioffset2;
        }
    }
    /*** 極值中點法 ***/
    if (msecCnt > 3000){  //馬達啟動過程再偵測極值，避免錯誤
        adc1_diff = adc_avg->IADC1 - Ioffset1;
        adc2_diff = adc_avg->IADC2 - Ioffset2;
        if (adc1_diff > adc1max) adc1max = adc1_diff;
        if (adc1_diff < adc1min) adc1min = adc1_diff;
        if (adc2_diff > adc2max) adc2max = adc2_diff;
        if (adc2_diff < adc2min) adc2min = adc2_diff;

    }

    return Ioffset;
}

Currents Get_Phase_Currents(Currents *adc_avg, Currents *ioffset)
{
	Currents iab;

	iab.ia = adc_avg->IADC1-ioffset->IADC1;
	iab.ib = adc_avg->IADC2-ioffset->IADC2;
	return iab;
}

/****************** 座標轉換配置 ******************/
Currents Clarke(Currents *iab) {
    Currents output;

    // 小馬達 adc 軸實際為 -abc 軸，因此需將讀值加負號
    output.ialpha = -iab->ia;
    output.ibeta = ((-iab->ia - 2 * iab->ib) * 1182) >> 11;

    return output;
}

Currents Park(Currents *ialphabeta, uint16_t theta)
{
	Currents output;
	Triangles sincos;
	sincos = Tri_Function(theta);

	output.id = (ialphabeta->ialpha * sincos.cos + ialphabeta->ibeta * sincos.sin) >> 11;	 // Id = Ialpha*cos + Ibeta*sin
	output.iq = (-ialphabeta->ialpha * sincos.sin + ialphabeta->ibeta * sincos.cos) >> 11; // Iq = -Ialpha*sin + Ibeta*cos

	return output;
}

Voltages Rev_Park(Voltages vdq, uint16_t theta)
{
	Voltages output;
	Triangles sincos;
	sincos = Tri_Function(theta);

	output.valpha = (vdq.vd * sincos.cos - vdq.vq * sincos.sin) >> 11;	// VAlpha = Vd*cos - Vq*sin
	output.vbeta = (vdq.vd * sincos.sin + vdq.vq * sincos.cos) >> 11;	// VBeta = Vd*sin + Vq*cos

	return output;
}

/****************** SVPWM 配置 ******************/
#define SQRT_3		1.732051
#define T			(2400 * 4)	//PWM開關週期，T等於4倍ARR的值
#define T_SQRT3     (uint16_t)(T * SQRT_3)	//根號3*開關週期
#define SECTOR_1	(uint32_t)3			//扇區1~6
#define SECTOR_2	(uint32_t)1
#define SECTOR_3	(uint32_t)5
#define SECTOR_4	(uint32_t)4
#define SECTOR_5	(uint32_t)6
#define SECTOR_6	(uint32_t)2

void CALC_SVPWM(Voltages *Stat_Volt_Input)
{
	uint8_t A=0, B=0, C=0, N=0;
	int32_t U1, U2, U3, wUAlpha, wUBeta;
	uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0;

    wUAlpha = Stat_Volt_Input->valpha * T_SQRT3;
    wUBeta  = Stat_Volt_Input->vbeta * T;

    U1 =  wUBeta;
    U2 = (-wUBeta + wUAlpha) >> 1;
    U3 = (-wUBeta - wUAlpha) >> 1;

    if(U1 > 0) A = 1;
    else A = 0;
    if(U2 > 0) B = 1;
    else B = 0;
    if(U3 > 0) C = 1;
    else C = 0;
    N = 4*C + 2*B + A;

    switch(N)
    {
    	case SECTOR_1:
    	case SECTOR_4:
    		hTimePhA = (T >> 3) + ((T + U2 + U1) >> 14);
    		hTimePhB = hTimePhA - (U2 >> 13);
    		hTimePhC = hTimePhB - (U1 >> 13);
    		break;
    	case SECTOR_2:
    	case SECTOR_5:
    		hTimePhB = (T >> 3) + ((T-U2-U3) >> 14);
    		hTimePhA = hTimePhB + (U2 >> 13);
    		hTimePhC = hTimePhA + (U3 >> 13);
    		break;
    	case SECTOR_3:
    	case SECTOR_6:
    		hTimePhB = (T >> 3) + ((T+U1+U3) >> 14);
    		hTimePhC = hTimePhB - (U1 >> 13);
    		hTimePhA = hTimePhC - (U3 >> 13);
    		break;
    	default:
    		break;
    }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, hTimePhA);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, hTimePhB);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, hTimePhC);

}


/****************** PID 配置 ******************/
void PID_Init(PID_Struct *Torque, PID_Struct *Flux, PID_Struct *Speed, PID_Struct *PLL_w, PID_Struct *Q)
{
	//轉矩q軸
	Torque->Kp = 50;			//比例調節		50
	Torque->Kp_div = 1000;  	//比例係數因子
	Torque->Ki = 1;				//積分調節		2
	Torque->Ki_div = 10000; 	//積分係數因子
	Torque->Integral = 0;		//積分累積值總和
	Torque->Upper_Limit = 1920;  //總輸出上限，根據PWM的ARR去設定，建議0.8倍才不會到過調製區
	Torque->Lower_Limit = -1920; //總輸出下限

	//磁通d軸
	Flux->Kp = 50; //	50
	Flux->Kp_div = 1000;
	Flux->Ki = 1;	//	2
	Flux->Ki_div = 10000;
	Flux->Integral = 0;
	Flux->Upper_Limit = 1920;
	Flux->Lower_Limit = -1920;

	//速度環
	Speed->Kp = 500; //500
	Speed->Kp_div = 1000; //1000
	Speed->Ki = 25; //25
	Speed->Ki_div = 10000; //10000
	Speed->Integral = 0;
	Speed->Upper_Limit = 150;
	Speed->Lower_Limit = -150;

	PLL_w->Kp = 500; //500
	PLL_w->Kp_div = 1000; //1000
	PLL_w->Ki = 8;  //8
	PLL_w->Ki_div = 10000; //10000
	PLL_w->Integral = 0;
	PLL_w->Upper_Limit = 3000;
	PLL_w->Lower_Limit = -3000;

	Q->Kp = 1500; //2000
	Q->Kp_div = 1000;
	Q->Ki = 20; //40
	Q->Ki_div = 1000;
	Q->Integral = 0;
	Q->Upper_Limit = 100;
	Q->Lower_Limit = -300;
}

//祖傳
int16_t PID_Regulator(int16_t Ref, int16_t Feedback, PID_Struct *PID)
{
	int16_t Error;
	int32_t P,I,Output;
	int32_t Integral_Limit = 2000000000;

	Error = Ref - Feedback;
	P = (int32_t)(PID->Kp * Error);

    Output = (P / PID->Kp_div) + (PID->Integral / PID->Ki_div);

    // Anti-Windup，只有在輸出未飽和時才允許積分
    if (PID->Ki != 0 && Output < PID->Upper_Limit && Output > PID->Lower_Limit)
    {
        I = (int32_t)(PID->Ki * Error);
        PID->Integral += I;

        // 限制積分範圍，防止 Integral 爆掉
        if (PID->Integral > Integral_Limit)
            PID->Integral = Integral_Limit;
        else if (PID->Integral < -Integral_Limit)
            PID->Integral = -Integral_Limit;
    }

	Output = (P / PID->Kp_div)+ (PID->Integral / PID->Ki_div); //避免控制結果太快達到上下限
	PID->Output = Output;
	if(Output >= PID->Upper_Limit)		//對 PI控制結果做限幅
		return(PID->Upper_Limit);
	else if(Output <= PID->Lower_Limit)
		return(PID->Lower_Limit);
	else {
		return((int16_t)Output);
	}
}

int16_t Qinner(Voltages Vdq, Currents *Idq, int16_t we)
{
	int32_t temp1, temp2, Q_temp, temp3, idq_mag2;
	int16_t Q;

	temp1 = Vdq.vq * Idq->id;
	temp2 = Vdq.vd * Idq->iq;
	Q_temp = temp1 - temp2;
	idq_mag2 = (Idq->id * Idq->id + Idq->iq * Idq->iq);
	temp3 = (we * Ls * idq_mag2) >> SCALE_SHIFT;
	Q = (Q_temp - temp3) >> 11;

	return Q;
}

