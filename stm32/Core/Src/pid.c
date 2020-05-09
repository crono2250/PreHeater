/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"
#include "pid.h"
#include "tmp75.h"

uint8_t fault = 0;						// 異常検出フラグ (0:正常, 1:異常
volatile  uint8_t PI_Control;				// PI制御実行・停止フラグ (1=実行)

    // double PI_P_val = 0;
    // double PI_I_val = 0;
    // double PI_D_val = 0;
	
volatile	double PI_Diff[2];					// 温度偏差情報 (変数)
volatile	double PI_integral;						// 積分情報(変数)
//	double PI_def = 0;							// 温度誤差情報 (変数)
//	double PI_ILN = 0;							// 積分情報 (変数)
//	double PI_Diff = 0;							// 微分情報 (変数)

//----------------
//100mSec Tick
/*

if (PI_Control)
{
    PI_Temp_Control();
} else
{
    PI_Diff[0] = 0;                     // PI関連パラメータクリア
    PI_Diff[1] = 0;
    PI_integral = 0;
}

*/

//----------------------------------------------------------------
// PI温度制御
//----------------------------------------------------------------
void PI_Temp_Control(uint8_t PI_ACT, uint16_t PI_Set_Temp, uint16_t PI_NowTemp)
{
    double temp, p, i, d, PI_def, set_temp;
    HAL_StatusTypeDef status = HAL_OK;

    if(PI_ACT)
    {
        if (PI_Set_Temp > PI_Temp_Max)              // 設定温度リミッタ
        {
            set_temp = PI_Temp_Max;
        } else
        {
            set_temp = (double)PI_Set_Temp;
        }
        
        temp = 1.0 * (double)PI_NowTemp;       // 温度取得
        temp = temp + temperature_offset;           // 温度補正
        PI_Diff[0] = PI_Diff[1];
        PI_Diff[1] = set_temp - temp;               // 偏差

        if(PI_Diff[1] >= PI_START_CONTROL_TEMPDIFF)
        {
        	PWM_SetValue = PI_PWM_Max;
            PWM_OUT(PWM_SetValue);                    // 設定温度と現在温度の差が制御開始温度差を超えている場合、ヒータ全開でintegralの加算は行わない。
            return;
        }

        PI_integral += (PI_Diff[0] + PI_Diff[1])/2.0 * DT;
        
        p = PI_P * PI_Diff[1];						// P
        i = PI_I * PI_integral;						// I
        d = PI_D * (PI_Diff[1] - PI_Diff[0]) / DT;	// D

        if(i > PI_I_Max){							// 積分値最大・最小リミッタ
            i = PI_I_Max;
        } else if(i < PI_I_MIN){
            i = PI_I_MIN;
        }

        // PI_P_val = p;
        // PI_I_val = i;
        // PI_D_val = d;

        PI_def = p + i + d;							// PID合成

        if(PI_def > PI_PWM_Max){					// PWM出力値最大・最小リミッタ
            PI_def = PI_PWM_Max;
        } else if(PI_def < PI_PWM_Min){
            PI_def = PI_PWM_Min;
        }

        PWM_SetValue = (uint16_t)PI_def;
        PWM_OUT(PWM_SetValue);

//        if(status != HAL_OK)	PWM_OUT(0);         //異常応答の場合はPWM止める
    } else
    {
        PI_Diff[0] = 0;                     // PI関連パラメータクリア
        PI_Diff[1] = 0;
//        PI_integral = 0;
        PWM_SetValue = 0;
        PWM_OUT(PWM_SetValue);
    }

	return;
}
