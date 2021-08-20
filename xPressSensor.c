
#if defined (__PRESSURE_TS2203__)
#include "cmsis.h"
#include "cmsis_os.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "hal_pressure.h"
#include "hal_trace.h"
#include "tgt_hardware.h"
#include "twi_master.h"
#include "sensor_thread.h"
#include "sensor_ctrl.h"
#include "app_gesture_manage.h"
#include "resources.h"
#include "apps.h"
#include "plat_types.h"
#include "hal_i2c.h"
#include "twi_hw_master.h"
#include "TS2203_adapter.h"
#include "cs_press_m6x_driver.h"
#include "app_xiaoai_spp.h"
#include "audio_prompt_sbc.h"
#include "cqueue.h"
#include "codec_sbc.h"
#include "bluetooth.h"
#include "rtx_lib.h"
#include "pressSurenSensor_Customer.h"
//#include  "aw8680x.h"

#define xPressSensor_RoundRobin_Time                  (0U)	
static void TS2203_gpiote_init(char edge_rise);
#define SIMULATION_KEY_LONGPRESS_THRESHOLD          2000            /**uint: ms*/
#define SIMULATION_KEY_CHECK_INTERVAL_MS            50              /**uint: ms*/
#define SIMULATION_KEY_PERIODIC_TIME_MS             500             /**uint: ms*/
#define SIMULATION_INTERRUPT_MODEx
#define DU86_UPDAMAXCOUNT                           (10)

static hal_pressure_event_type_enum simulation_key = HAL_PRESSURE_EVENT_NONE;
static uint32_t press_start_tick = 0;
static hal_pressure_gesture_cb_func TS2203_simulation_key_callback = NULL;
#define PRESSURE_WORK_ON           0
#define PRESSURE_TURN_OFF          1
static uint32_t releasedTimeCnt[2] = {0};
static uint32_t pressedTimeCnt[2] = {0};

static uint32_t xPressSensor_KeyDown[2];
static uint32_t xPressSensor_KeyUp[2];

osMutexDef(TS2203_Device_MutextID);
osMutexId ts2203_mutextid;
static void debug_TS2203_timehandler(void const *param);
osTimerDef (DEBUG_TS2203_TIMER, debug_TS2203_timehandler);
static osTimerId debug_TS2203_timer = NULL;

static void TS2203_timehandler(void const *param);
osTimerDef (APP_TS2203_TIMER, TS2203_timehandler);
static osTimerId app_TS2203_timer = NULL;
static void report_press_event_timehandler(void const *param);
osTimerDef (REPORT_PRESS_EVENT_TIMER, report_press_event_timehandler);
static osTimerId report_press_event_timer = NULL;
SensorSpp_Info xSensorSpp_Info;//add on 6-9
bool xPressSensor_Long_Key=false;


static void debug_TS2203_timehandler(void const *param)
{
	CS_SENSOR_DATA_Def	sensor_data;
	CS_CALIBRATION_RESULT_Def sensor_checkdata;	
	uint8_t _spp_data[64];
	osMutexWait((osMutexId)ts2203_mutextid, osWaitForever);

    #if 0//open it when enter the factory spp mode 
	if(xSensorSpp_Info.pressSensor_SppEnable==(uint8_t)1){
		_spp_data[0]=0x12;
		_spp_data[1]=0x12;
		_spp_data[2]=0x12;
		_spp_data[3]=0x12;
		#if defined(__MI_XIAOAI_SURPPORT__)
		app_xiaoai_spp_send((uint8_t*)_spp_data,(uint16_t)8);
		#endif
	}
	#endif

	if(xPressureEnable==true){

		if(xPressSensor_InitEnable==true){
			cs_press_read_sensor_data_init();	
			xPressSensor_InitEnable=false;
		}
	
		//handle the noise data when gather it 
		if(0==cs_press_read_sensor_data(&sensor_data)){
			xPressureSensor_GatherData[0]=sensor_data.rawdata;
			xPressureSensor_GatherData[1]=xPressureSensor_Offerset;
			app_product_pressuresensor_active();//add by hjw 	
		    //ts2203_TRACE(4,"jw----->presssensor_gather rawdata: %2d ,arith_rawdata: %d,energy_data: %d,diff_data: %d",sensor_data.rawdata,sensor_data.arith_rawdata,sensor_data.energy_data,sensor_data.diff_data);
			_spp_data[0]= (uint8_t)((sensor_data.rawdata>>8)&0xff);
			_spp_data[1]= (uint8_t)(sensor_data.rawdata&0xff);
			_spp_data[2]= (uint8_t)((sensor_data.diff_data>>8)&0xff);
			_spp_data[3]= (uint8_t)((sensor_data.diff_data>>8)&0xff);	
			
			if(xPressSppEnable==true){
				sprintf((char*)_spp_data, "raw: %d arith:%denergy: %d diff: %d end",sensor_data.rawdata,sensor_data.arith_rawdata,sensor_data.energy_data,sensor_data.diff_data);  			
				app_xiaoai_spp_send((uint8_t*)_spp_data,(uint16_t)64);
			}
		

		}else{

			if(cs_press_read_sensor_data(&sensor_data)==0){
				xPressureSensor_GatherData[0]=sensor_data.rawdata;
				xPressureSensor_GatherData[1]=xPressureSensor_Offerset;
				app_product_pressuresensor_active();//add by hjw 	
				//ts2203_TRACE(4,"jw----->presssensor_gather rawdata: %2d ,arith_rawdata: %d,energy_data: %d,diff_data: %d",sensor_data.rawdata,sensor_data.arith_rawdata,sensor_data.energy_data,sensor_data.diff_data);
			}else{
				//ts2203_TRACE(0,"jw-----> xpresssensure data is not read!");	
			}
		}

	}else{
		switch(xPressureSensor_testMode){
			case xPressSensor_TestMode_None:
			break;
			case xPressSensor_TestMode_Start:
				cs_press_calibration_enable();
			break;
			case xPressSensor_TestMode_End:
				cs_press_calibration_disable();				
				//cs_press_calreset();
				xPressureCheckModeEnable=false;
			break;
			case xPressSensor_TestMode_Process:
				cs_press_calibration_check(&sensor_checkdata);
				xPressureSensor_CheckData[0]=(short)sensor_checkdata.calibration_progress;
				xPressureSensor_CheckData[1]=(short)sensor_checkdata.calibration_factor;
				xPressureSensor_CheckData[2]=(short)sensor_checkdata.press_adc_1st;
				xPressureSensor_CheckData[3]=(short)sensor_checkdata.press_adc_2nd;
				xPressureSensor_CheckData[4]=(short)sensor_checkdata.press_adc_3rd;
			  	//ts2203_TRACE(2,"jw-----calibration_progress:  %2x ,calibration_factor: %d",sensor_checkdata.calibration_progress, sensor_checkdata.calibration_factor);
			  	//ts2203_TRACE(3,"jw-----press_adc_1st: %d, press_adc_2nd: %d,press_adc_3rd: %d",sensor_checkdata.press_adc_1st, sensor_checkdata.press_adc_2nd,sensor_checkdata.press_adc_3rd);
			  	app_product_pressuresensor_active();//add by hjw 	
			break;
		}
	}
	osMutexRelease((osMutexId)ts2203_mutextid);    
}

extern void app_product_test_uart_key_status_changed(bool key_status);
static uint16_t xInter_CurentDownTime=0;
static uint16_t xInter_CurentUpTime=0;
static uint8_t  xInter_ShortKeyEnable=0;
static void TS2203_timehandler(void const *param)
{
	uint8_t _spp_data[8];

#ifdef TWS_PROMPT_SYNC
	#undef TWS_PROMPT_SYNC
#endif

	if(audio_prompt_is_playing_ongoing()) return ;

	//if(xSWitch_Proess_Enable==true) return;
	

	switch(xInter_ShortKeyEnable){

		case 1:
		simulation_key = HAL_PRESSURE_EVENT_CLICK;
		ts2203_TRACE(0,"jw----->  HAL_PRESSURE_EVENT_CLICK");
		break;

		case 2:
		simulation_key = HAL_PRESSURE_EVENT_DOUBLE_CLICK;
		ts2203_TRACE(0,"jw----->  HAL_PRESSURE_EVENT_DOUBLE_CLICK");	
		break;

		case 3:
		simulation_key = HAL_PRESSURE_EVENT_THREE_CLICK;
		ts2203_TRACE(0,"jw----->  HAL_PRESSURE_EVENT_THREE_CLICK");
		break;

		//case 4:
		//simulation_key = HAL_PRESSURE_EVENT_THREE_CLICK;
		//ts2203_TRACE(0,"jw----->  HAL_PRESSURE_EVENT_THREE_CLICK");
		//break;

		default:
		break;

	}

	osMutexWait((osMutexId)ts2203_mutextid, osWaitForever);
	if(xSensorSpp_Info.pressSensor_SppEnable==(uint8_t)1){
		_spp_data[0]=(uint8_t)0x12;
		_spp_data[1]=(uint8_t)simulation_key;
		_spp_data[2]=(uint8_t)12;//((sensor_data.diff_data>>8)&0xff);
		_spp_data[3]=(uint8_t)12;//(sensor_data.diff_data&0xff);
		app_xiaoai_spp_send((uint8_t*)_spp_data,(uint16_t)8);
	}
	osMutexRelease((osMutexId)ts2203_mutextid);

	#if defined(__PROJ_K73__) || defined(__PROJ_K75__)
	if(TS2203_simulation_key_callback != NULL)
	{
		if((simulation_key==HAL_PRESSURE_EVENT_CLICK)||(simulation_key==HAL_PRESSURE_EVENT_DOUBLE_CLICK)||(simulation_key==HAL_PRESSURE_EVENT_THREE_CLICK))
		{
			TS2203_simulation_key_callback((uint8_t)simulation_key, 0, 0);
		}
	}
	simulation_key = HAL_PRESSURE_EVENT_NONE;
	#endif

	xInter_ShortKeyEnable=0;

}

bool USER_LONG_KEY_ENABLE=false;

static void report_press_event_timehandler(void const *param)
{
	uint8_t pin_status = 0;
	xInter_ShortKeyEnable=0;

	if(audio_prompt_is_playing_ongoing()) return ;

#define TWS_PROMPT_SYNC 

	pin_status = hal_gpio_pin_get_val(app_pressure_int_cfg.pin);
	if(pin_status==BTN_PRESSED_PIN_LEVEL){
		#if defined(__PROJ_K73__) || defined(__PROJ_K75__)
		simulation_key = HAL_PRESSURE_EVENT_LONGPRESS;
		if(TS2203_simulation_key_callback != NULL)
		{
			TS2203_simulation_key_callback((uint8_t)simulation_key, 0, 0);
		}
		simulation_key = HAL_PRESSURE_EVENT_NONE;
		#endif
		ts2203_TRACE(0,"\r\n   report_press_event_timehandler ----: long key");
		xPressSensor_Long_Key=true;
	}

}

/********************************************************************************/
static void xPressSensor_MCU_thread(void const *argument);
osThreadDef(xPressSensor_MCU_thread, osPriorityNormal, 1,1024*4, "xPressSensor_MCU_thread");
osThreadId xPressSensor_MCU_thread_tid = NULL; 

typedef struct{
	uint8_t		shortkey;
	uint8_t		longkey;
	uint16_t	messget_cnt;
	bool        key_tone;
}xPressSensor_Frame_t;
#define  xPressSuren_Mail_Main (20U)
osMailQDef(xPressSure_Mcu_Mail, xPressSuren_Mail_Main, xPressSensor_Frame_t);
static osMailQId xPressSure_Mcu_Mail_id = NULL;
xPressSensor_Frame_t xPrssSensor_Key;
/********************************************************************************/

void xPressSure_Sensor_Init(void)
{
	if(xPressSensor_MCU_thread_tid == NULL){
		xPressSensor_MCU_thread_tid = osThreadCreate(osThread(xPressSensor_MCU_thread), NULL);
	}
	if (xPressSure_Mcu_Mail_id == NULL){
		xPressSure_Mcu_Mail_id = osMailCreate(osMailQ(xPressSure_Mcu_Mail), NULL);
		if(xPressSure_Mcu_Mail_id!=NULL){
		//ts2203_TRACE(0," xPressSensor_MCU_MailPut init ");
		}
	}
}

#define xSensor_PutMailTimeOut  (100)
osStatus xPressSensor_MCU_MailPut(xPressSensor_Frame_t key)
{
	osStatus status=osOK;
	xPressSensor_Frame_t *msg_p = NULL;
	msg_p = (xPressSensor_Frame_t*)osMailAlloc(xPressSure_Mcu_Mail_id,0);
	if(NULL!=msg_p){
		*msg_p=(xPressSensor_Frame_t)key;
		if(xPressSure_Mcu_Mail_id!=NULL){
		status = osMailPut(xPressSure_Mcu_Mail_id, msg_p);
			if(osOK==status){
				//ts2203_TRACE(0," xPressSensor_MCU_press success ");
			}else{
				ts2203_TRACE(0," xPressSensor_MCU_press error ");
			}
	    }
	}
	return status;
} 

static void xPressSensor_MCU_thread(void const *argument)
{
	osEvent  evt;	
    osStatus status=osError;
    xPressSensor_Frame_t *msg_p = NULL;
	while(1)
	{			
		evt=osMailGet(xPressSure_Mcu_Mail_id,osWaitForever);
		if(osEventMail==evt.status){
			msg_p=(xPressSensor_Frame_t*)evt.value.p;
			if((msg_p->longkey==(uint8_t)1U)&&(msg_p->shortkey==(uint8_t)0U)){
				if(osTimerIsRunning(report_press_event_timer)){
					osTimerStop(report_press_event_timer);
				}
				osTimerStart(report_press_event_timer, xPressSensor_Response_LongTime);
			}
			if((msg_p->shortkey!=(uint8_t)0U)&&(msg_p->longkey==(uint8_t)0U)){
				if(osTimerIsRunning(app_TS2203_timer)){
					osTimerStop(app_TS2203_timer);
				}
				osTimerStart(app_TS2203_timer,xPressSensor_Response_Time);
			}

			///add 
			if((msg_p->key_tone==1)&&(USER_LONG_KEY_ENABLE==true)){
				msg_p->key_tone=0;
				//Key_Tone_Play();
				USER_LONG_KEY_ENABLE=false;
			}
			////

			status=osMailFree(xPressSure_Mcu_Mail_id,msg_p);
			if(status==osOK){
				//ts2203_TRACE(0," xPressSensor_MCU_thread OK ");
			}
		}else{
				ts2203_TRACE(0," xPressSensor_MCU_thread Error");
		}
		osDelay(xPressSensor_RoundRobin_Time);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


static uint32_t interrup_inter[2]={0,0};

static void TS2203_int_handle_process(enum HAL_GPIO_PIN_T pin)
{
	uint32_t up_time=0;
	uint32_t down_time=0;
	uint32_t inter=0;
    uint8_t pin_status = 0;
    //if(xSWitch_Proess_Enable==true) return ;
    interrup_inter[1]=osKernelGetSysTimerCount();
    inter=TICKS_TO_MS(interrup_inter[1]);
    inter=inter-TICKS_TO_MS(interrup_inter[0]);
    inter=(uint32_t)(inter/10);


    pin_status = !hal_gpio_pin_get_val(app_pressure_int_cfg.pin);
    if (BTN_RELEASED_PIN_LEVEL == pin_status) 
	{
		#if defined(__PROJ_K73__) || defined(__PROJ_K75__)
		xPressSensor_KeyDown[1]=osKernelGetSysTimerCount();
		down_time=TICKS_TO_MS(xPressSensor_KeyDown[1]);
		down_time=down_time-TICKS_TO_MS(xPressSensor_KeyDown[0]);
		down_time=(uint32_t)(down_time/10);

		if(xPressSure_Mcu_Mail_id!=NULL){
			xPrssSensor_Key.shortkey=0;
			xPrssSensor_Key.longkey=1;
			xPressSensor_MCU_MailPut(xPrssSensor_Key);
		}

		if(xPressSensor_Long_Key==false){
			if(xPressSure_Mcu_Mail_id!=NULL){
				if(USER_LONG_KEY_ENABLE==false){
					xPrssSensor_Key.key_tone=1;
					USER_LONG_KEY_ENABLE=true;
					xPressSensor_MCU_MailPut(xPrssSensor_Key);
				}
			}
		}
		xPressSensor_KeyDown[0]=osKernelGetSysTimerCount();
		#endif
    }
	else
	{
		#if defined(__PROJ_K73__) || defined(__PROJ_K75__)
		xPressSensor_KeyUp[1]=osKernelGetSysTimerCount();
		up_time=(TICKS_TO_MS(xPressSensor_KeyUp[1]));
		up_time=up_time-TICKS_TO_MS(xPressSensor_KeyDown[1]);
		up_time=(uint32_t)(up_time/10);
		
        if(TICKS_TO_MS(hal_timer_get_passed_ticks(xInter_CurentUpTime, xInter_CurentDownTime))<=xPressSensor_Response_Time){
        	if((xPressSensor_Long_Key==false)||(xLongSwitch_Enable==true)){
        		if((++xInter_ShortKeyEnable)&&(inter>5)){
        		   if(xInter_ShortKeyEnable<=3){        
        		   }        		  
        		}
        		
        	}

        }
	
		if(xPressSure_Mcu_Mail_id!=NULL){
			xPrssSensor_Key.shortkey++;
			xPrssSensor_Key.longkey=0;
			xPressSensor_MCU_MailPut(xPrssSensor_Key);
		}

		if(xPressSure_Mcu_Mail_id!=NULL){
			if(USER_LONG_KEY_ENABLE==false){
				xPrssSensor_Key.key_tone=1;
				USER_LONG_KEY_ENABLE=true;
				xPressSensor_MCU_MailPut(xPrssSensor_Key);
			}
		}
		xPressSensor_KeyUp[0]=osKernelGetSysTimerCount();
		xPressSensor_Long_Key=false;
		#endif
	}

	 interrup_inter[0]=osKernelGetSysTimerCount();
}

#if 0
static void TS2203_event_process(enum HAL_GPIO_PIN_T pin)
{
    SENSOR_MESSAGE_BLOCK msg;
    msg.mod_id = SENSOR_MODUAL_PRESS;
    msg.msg_body.message_id = PRESS_PIN_INT_MESSAGE_ID;
	msg.msg_body.message_Param0 = (uint8_t)pin;
    sensor_mailbox_put(&msg);
}
#endif


static void TS2203_int_handler(enum HAL_GPIO_PIN_T pin)
{
	

	//if(xSWitch_Proess_Enable==false){
		Key_Tone_Play();
	//}

	if(BTN_RELEASED_PIN_LEVEL==hal_gpio_pin_get_val(app_pressure_int_cfg.pin)){

		TS2203_gpiote_init(BTN_PRESSED_PIN_LEVEL);	
	}else{
	
		TS2203_gpiote_init(BTN_RELEASED_PIN_LEVEL);	
	}
	
	TS2203_int_handle_process(pin);		

}

static void TS2203_gpiote_init(char edge_rise)
{
    struct HAL_GPIO_IRQ_CFG_T gpiocfg;
    gpiocfg.irq_enable = true;
    gpiocfg.irq_debounce = true;
    #ifdef SIMULATION_INTERRUPT_MODE
	gpiocfg.irq_type = HAL_GPIO_IRQ_TYPE_LEVEL_SENSITIVE;/*HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE*/;
    #else
    gpiocfg.irq_type = HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE;/*HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE*/;
    #endif
    gpiocfg.irq_handler = TS2203_int_handler;
    if(edge_rise == 1) {
        gpiocfg.irq_polarity = HAL_GPIO_IRQ_POLARITY_HIGH_RISING;
    } else {
        gpiocfg.irq_polarity = HAL_GPIO_IRQ_POLARITY_LOW_FALLING;
    }
    hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_pressure_int_cfg, 1);
    hal_gpio_pin_set_dir(app_pressure_int_cfg.pin, HAL_GPIO_DIR_IN, 1);// 0 hjw 
    hal_gpio_setup_irq(app_pressure_int_cfg.pin, &gpiocfg);
    hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_pressure_reset_cfg, 1);
    hal_gpio_pin_set_dir(app_pressure_reset_cfg.pin, HAL_GPIO_DIR_OUT, 1);// 0 hjw 
    hal_gpio_pin_set(app_pressure_reset_cfg.pin);
}

void xPressSensor_PowerUp(void)
{
	hal_gpio_pin_set(app_pressure_reset_cfg.pin);
}

void xPressSensor_PowerDown(void)
{
	hal_gpio_pin_clr(app_pressure_reset_cfg.pin);
}

static int TS2203_handle_process(SENSOR_MESSAGE_BODY *msg_body)
{
    switch(msg_body->message_id)
    {
        case PRESS_PIN_INT_MESSAGE_ID:
        {
			TS2203_int_handle_process(msg_body->message_Param0);
            break;
        }
        default:
            break;
    }
    return 0;
}



static int32_t TS2203_init(void)
{
	CS_FW_INFO_Def fw_info;
	short   _offset=0;
	CS_SENSOR_DATA_Def	sensor_data={0,0,0,0};
	
  	int32_t ret=0;

	sensor_set_threadhandle(SENSOR_MODUAL_PRESS, TS2203_handle_process);
    TS2203_gpiote_init(BTN_PRESSED_PIN_LEVEL);
	memset(releasedTimeCnt, 0, sizeof(releasedTimeCnt));
	memset(pressedTimeCnt, 0, sizeof(pressedTimeCnt));
	press_start_tick = hal_sys_timer_get();
    app_TS2203_timer = osTimerCreate (osTimer(APP_TS2203_TIMER), osTimerOnce, NULL);
#if defined (__TS2203_PRESSURE__MULTIFUNC__)
    delay_timer = osTimerCreate(osTimer(DELAY_TIMER), osTimerOnce, NULL);
#endif
    report_press_event_timer = osTimerCreate (osTimer(REPORT_PRESS_EVENT_TIMER), osTimerOnce, NULL);
    debug_TS2203_timer = osTimerCreate (osTimer(DEBUG_TS2203_TIMER), osTimerPeriodic, NULL);
	ts2203_mutextid=osMutexCreate((osMutex(TS2203_Device_MutextID)));
	osMutexRelease((osMutexId)ts2203_mutextid);
	TS2203_UpdateByownself();
	cs_press_read_sensor_data_init();

	//enter the factory
	cs_press_read_sensor_data(&sensor_data);
	cs_press_read_sensor_offset(&_offset);
	cs_press_read_fw_info(&fw_info);
	xPressFac_ADC_NIOSE=(short)sensor_data.rawdata;
 	xPressFac_ADC_OFFSET=(short)_offset;
	xPressFac_ADC_softid=(short)fw_info.fw_version;
	ts2203_TRACE(3,"\r\n ----id:%x,adc:%x,offset:%x",xPressFac_ADC_softid,xPressFac_ADC_NIOSE,xPressFac_ADC_OFFSET);
  	//end the factory

    osTimerStart(debug_TS2203_timer,200);//200 cancel by hjw on 4-5 400
    cs_press_write_press_level(xPressSensor_PressLevel);
    xPressureEnable=false;
    xPressSensor_InitEnable=false; //
    xPressSure_Sensor_Init();
    return ret;
}

/*******************************************************************
*Fcuntion: TS2203_UpdateByownself
*Parame:none
*Return:None
*******************************************************************/
uint8_t TS2203_UpdateByownself(void)
{
  	char    _upret=0;
  	short   _offset=0;

	CS_FW_INFO_Def fw_info;
	cs_press_read_fw_info(&fw_info);
    cs_press_read_sensor_offset(&_offset);
	ts2203_TRACE(3, "\r\n --------------->manufacturer_offset befor is %x , module_id is %x ,fw_version is %x",fw_info.manufacturer_id,fw_info.module_id ,fw_info.fw_version);
	osMutexWait((osMutexId)ts2203_mutextid, osWaitForever);
	xPressureSensor_Offerset=(short)_offset;
	xPressSensor_McuSoftwareID=(unsigned short)fw_info.fw_version;
 	osMutexRelease((osMutexId)ts2203_mutextid);
	if((fw_info.module_id==0x68)&&(fw_info.fw_version==0x301)){
		ts2203_TRACE(0,"\r\n ----> pressure sensor have the new vsersion ,not need to update !");
		ts2203_TRACE(1, "\r\n --------------->test presssensor  offerset is %d ",_offset);
		osMutexWait((osMutexId)ts2203_mutextid, osWaitForever);
		xPressureSensor_Offerset=(short)_offset;
		xPressSensor_McuSoftwareID=(unsigned short)fw_info.fw_version;
		osMutexRelease((osMutexId)ts2203_mutextid);
		return 0;
	}
	    cs_press_reset_ic();
		_upret=TS2203_SoftwareUpdate();// need about 10s 
		cs_press_read_fw_info(&fw_info);
		ts2203_TRACE(3, "\r\n --------------->manufacturer_offset after is %x , module_id is %x ,fw_version is %x",fw_info.manufacturer_id,fw_info.module_id ,fw_info.fw_version);
		if(_upret==0){
			cs_press_read_sensor_offset(&_offset);
			osMutexWait((osMutexId)ts2203_mutextid, osWaitForever);
			xPressureSensor_Offerset=_offset;
			xPressSensor_McuSoftwareID=fw_info.fw_version;
			osMutexRelease((osMutexId)ts2203_mutextid);
			ts2203_TRACE(0,"\r\n ----> pressure sensor update successful! ");
		}else{
			for(int i=0;i<DU86_UPDAMAXCOUNT;i++){
				_upret=TS2203_SoftwareUpdate();// need about 10s 	
				if(_upret==0)
					break;
			}
			ts2203_TRACE(0,"\r\n ----> pressure sensor update Failed!");
		}
	return 0;
}


static int32_t TS2203_set_simulation_key_callback(hal_pressure_gesture_cb_func cb)
{
    TS2203_simulation_key_callback = cb;
    //ts2203_TRACE(0, "\r\n --------------->hjw callback");
	return 0;
}

static int32_t TS2203_get_pressure_key_status(uint8_t *sts)
{
	uint8_t pinLevel;

	pinLevel = hal_gpio_pin_get_val(app_pressure_int_cfg.pin);
	*sts = (BTN_PRESSED_PIN_LEVEL == pinLevel)?1:2; // 1-pressed 2-released
	if(*sts == 2){		
		return 0;
	}else{
		return 1;
	}
}


/*----------------------------------------------------------
** Function name:  is_TS220300_work_status
** Intput :  None
** Output :  None
** Function describe:None
** 
** Modification history:
**    2020/05/19 16:40, yunhui@chen.tiinlab.com create this function
*---------------------------------------------------------*/
#if 0
static int8_t is_TS2203_work_status(void)
{
	return (work_status);
}
#endif
/*****************************************************************
*Function:Enter the mode of sleep 
*
*
*
******************************************************************/
int32_t TS2203_Entern_StandbyMode(void)
{
	char _sta=0;
	//_sta=cs_press_set_device_sleep();
	//ts2203_TRACE(0,"\r\n jw--> pessSensor_Entern_StandbyMode");
	return (int32_t)_sta;
}

int32_t TS2203_Exit_StandbyMode(void)
{
	char _sta=0;
	//_sta=cs_press_set_device_wakeup();
	//ts2203_TRACE(0,"\r\n jw--> pessSensor_Entern_WorkMode");
	return (int32_t)_sta;
}

const hal_pressure_ctrl_t TS2203_pressure_ctrl =
{
	TS2203_init,
	NULL,/*TS2203_get_chip_id*/
	NULL,/*TS2203_reset*/
	NULL,//check esd
	TS2203_set_simulation_key_callback,
	NULL,
    TS2203_Exit_StandbyMode,
	TS2203_Entern_StandbyMode,/*TS2203_enter_standby_mode_status*/
	NULL,/*TS2203_enter_normal_mode_status*/
	NULL,
	NULL,/*TS2203_enter_standby_mode_status*/
	TS2203_get_pressure_key_status,	
};


#ifdef __SENSOR_TS2203_SUPPORT__
void Touch_PressMessageSend(uint8_t           key_vlaue,uint8_t key_status)
{  
}
#endif

#endif

