/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

#define TMP75_DATA_BUFFER_SIZE 1000
//#define TMP75_I2C_TIMEOUT 10000
#define TMP75_I2C_TIMEOUT 10
#define TMP75_DEVICE_ADDRESS 0b10010000 //device address is written on back side of your display


 #define TMP75_REG_TEMPERATURE_R        0x00
 #define TMP75_REG_CONFIG_RW            0x01
 #define TMP75_REG_T_LOW_RW             0x02
 #define TMP75_REG_T_HIGH_RW             0x03

// ### Configuration Reg ###
// D7 : OS (1 = OneShot)
// D6:D5 : Converter Resolution
//   00 : 9bit(0.5degC, 27.5degC), 01 : 10bit(0.25degC, 55mSec), 10 : 11bit(0.125degC, 110mSec), 11 : 12bit(0.0625degC, 220mSec)
// D4:D3 : Fault Queue
//   00 : Faults=1, 01 : Faults=2, 10 : Faults=4, 11 : Faults=6
// D2 : Polarity of ALARM pin
//   0 : Fault = Low, 1 : Fault = High
// D1 : Thermostat Mode
//   0 : Comparator Mode, 1 : Interrupt Mode
// D0 : Shutdown Mode
//   0 : Continuous Acquision, 1 : Shutdown (1-Shot Acquision)

//#define TMP75_Config 0b10100001		//(OneShot, 10bit(0.25degC, 55mSec), Faults=1, Fault = Low, Comparator Mode, Shutdown)
#define TMP75_Config 0b11000101		//(OneShot, 11bit(0.125degC, 110mSec), Faults=1, Fault = Low, Comparator Mode, Shutdown)

volatile uint16_t TMP75_NowTemperature;

//----------------------------------------------------------------
// 関数プロトタイプ宣言
//----------------------------------------------------------------
HAL_StatusTypeDef TMP75_INIT(void);
HAL_StatusTypeDef TMP75_STARTACQ(void);
uint16_t TMP75_GETTEMP(void);
