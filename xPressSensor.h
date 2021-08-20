#ifndef __PRESSURE_TS2203_H__
#define __PRESSURE_TS2203_H__

#if defined (__PRESSURE_TS2203__)

#include "cmsis.h"
#include "cmsis_os.h"
#include "hal_trace.h"

#ifdef __cplusplus
extern "C" {
#endif
	
#include "Key_Handle_Middle.h"


#define BTN_PRESSED_PIN_LEVEL   (0)
#define BTN_RELEASED_PIN_LEVEL  (1)

#define PRESS_PIN_INT_MESSAGE_ID 				1
#define PRESS_SET_FIFO_MESSAGE_ID 			    2
#define PRESS_SET_POW_MODE_MESSAGE_ID 		    3
#define PRESS_SET_SENSITIVITY_MESSAGE_ID 		4 //sensitivity

#define NEXTINPUT_I2C_ADDRESS 	0x4a
#define NEXTINPUT_ID            0x20


#define TS2203_LOG_ENABLE 
#ifdef  TS2203_LOG_ENABLE 
#define ts2203_TRACE(num,str, ...)       TRACE(num,"[TS2203]" str, ##__VA_ARGS__)
#else
#define ts2203_TRACE(str, ...)
#endif

void xPressSensor_PowerUp(void);
void xPressSensor_PowerDown(void);

#ifdef __PRESSURE_TS2203__

uint8_t TS2203_UpdateByownself(void);

void Touch_PressMessageSend(uint8_t           key_vlaue,uint8_t key_status);

typedef struct{
	uint8_t sensor_spp_enable;
	//-----------xPressSensor
	uint8_t pressSensor_SppEnable;
	uint8_t pressSensor_AdcEnable;
	uint8_t pressSensor_KeyEnable;
	uint8_t pressSensor_TimerSet; //support the setting of user
	//-----------xTouchSensor
	uint8_t touchSensor_SppEnable;
	uint8_t touchSensor_TimerSet; //support the setting of user
	uint8_t otherSensor_SppEnable;

}SensorSpp_Info;

extern SensorSpp_Info xSensorSpp_Info;
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif


