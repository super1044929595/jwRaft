#include "Key_Handle_Middle.h"


//private 
//---------------------------------------------------------------------
#define TOUCH_FILETR_LENGTH              (64U)  // the used is only 20 
static  unsigned char   xTouch_Filter_Value[TOUCH_FILETR_LENGTH];
static  unsigned char   xTouch_Filter_Status[TOUCH_FILETR_LENGTH];
static  unsigned char   xTouch_WCnt=0;
static  unsigned char   xTouch_RCnt=0;
static  unsigned char   xTouch_cnt=0;
static  unsigned short  xTouch_Pre=0;

static unsigned char	Press_StoreFull(void);
static unsigned char    Press_StoreZero(void);

//---------------------------------------------------------------------


unsigned char	Press_StoreFull(void)
{	
    if((xTouch_WCnt==xTouch_RCnt)&&(xTouch_cnt == TOUCH_FILETR_LENGTH))
    {	        
        return 1;
    }
    else {
        return 0;
    }
}

unsigned char    Press_StoreZero(void)
{	
    if((0== xTouch_cnt)&&(xTouch_WCnt== xTouch_RCnt))
    {	
        return 1;	
    }
    else
        return 0;
}
//public method
void   Press_KeyValueInit(void)
{	
    xTouch_WCnt=(unsigned char)0;
    xTouch_RCnt=(unsigned char)0;
    xTouch_cnt =(unsigned char)0;
    memset((unsigned char*)xTouch_Filter_Value,0,sizeof(xTouch_Filter_Value));		
    memset((unsigned char*)xTouch_Filter_Status,0,sizeof(xTouch_Filter_Status));
}

unsigned char 	Press_KeyValueWrite(unsigned char prevalue,unsigned char prestatus)
{	
    //unsigned short _key_pre;	
    if(Press_StoreFull())
    {	
        return 1;
    }
    #if 1 //cancel by user when not need the difference of the same button value 	
    XTOUCH_KEYVALUE_SENDMESSAGE(_key_pre,prevalue,prestatus);
 	
    if(_key_pre==xTouch_Pre)	
        return 1;
    #endif	
    xTouch_Filter_Value[xTouch_WCnt]=(unsigned char)prevalue;		
    xTouch_Filter_Status[xTouch_WCnt]= (unsigned char)prestatus;
    XTOUCH_KEYVALUE_SENDMESSAGE(xTouch_Pre,prevalue,prestatus);
    ++xTouch_WCnt;
    xTouch_WCnt=(unsigned char)(xTouch_WCnt%TOUCH_FILETR_LENGTH);	
    ++xTouch_cnt ;
 
    _key_pre=
    return 0;
}

unsigned short  Press_KeyValueRead(void)
{	
    unsigned short _key_pre;  
    if(Press_StoreZero())
    {
        return 1;
    }		

    XTOUCH_KEYVALUE_SENDMESSAGE(_key_pre,xTouch_Filter_Value[xTouch_RCnt],xTouch_Filter_Status[xTouch_RCnt]);
    xTouch_Filter_Value[xTouch_RCnt] = (unsigned char)0;
    xTouch_Filter_Status[xTouch_RCnt]= (unsigned char)TOUCH_PRESS_UNKNOW;
    ++xTouch_RCnt;
    xTouch_RCnt =(unsigned char)(xTouch_RCnt%TOUCH_FILETR_LENGTH);
    --xTouch_cnt;	
    //DF100_TRACE(1,"--------->hjw press relase %d!",xTouch_cnt);
    return _key_pre;
}
