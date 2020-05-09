

#define PI_PWM_Max 99
#define PI_PWM_Min 0x0

#define PI_I_Max 10000 // 積分リミッタ上限値
#define PI_I_MIN -10000 // 積分リミッタ下限値

#define DT 0.15           // 制御周期 (150mSec)

#define PI_P 0.5         // PI P定数
#define PI_I 0.05      // PI I定数
#define PI_D 0.01           // PI D定数

#define temperature_offset 0             // 温度キャリブレーション値
#define PI_START_CONTROL_TEMPDIFF   500   // PI制御開始温度差 (5.00degC)

#define PI_Temp_Max 12000				// 最大温度:120℃

volatile	uint16_t	PWM_SetValue;

void PI_Temp_Control(uint8_t PI_ACT, uint16_t PI_Set_Temp, uint16_t PI_NowTemp);
