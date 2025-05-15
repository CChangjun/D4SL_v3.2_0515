#line 1 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\D4SLv3.1\\CHANG_cust_d4sl\\Tick_Handler.h"
#ifndef _TICK_HANDLER_H
#define _TICK_HANDLER_H
typedef struct _sys_tick_t
{
    bool f_ms1;

    uint16_t cnt_ms2;
    uint16_t cnt_ms5;
    uint16_t cnt_ms10;
    uint16_t cnt_ms25;
    uint16_t cnt_ms50;
    uint16_t cnt_ms100;
    uint16_t cnt_ms250;
    uint16_t cnt_ms500;
    uint16_t cnt_ms750;
    uint16_t cnt_sec1;

    union
    {
        uint16_t wf;
        struct
        {
            uint16_t ms2 : 1;
            uint16_t ms5 : 1;
            uint16_t ms10 : 1;
            uint16_t ms25 : 1;

            uint16_t ms50 : 1;
            uint16_t ms100 : 1;
            uint16_t ms250 : 1;
            uint16_t ms500 : 1;

            uint16_t ms750 : 1;
            uint16_t sec1 : 1;
            uint16_t sec2 : 1;
            uint16_t sec3 : 1;

            uint16_t sec4 : 1;
            uint16_t sec5 : 1;
            uint16_t sec10 : 1;
            uint16_t min1 : 1;
        } bf;
    } flag;
} sys_tick_t;

sys_tick_t gSysTick;
void Tick_Handle(void);


void Tick_Handle(void)
{
    gSysTick.flag.wf = 0;
    if( ++gSysTick.cnt_ms2 == 2 )
    {
        gSysTick.flag.bf.ms2 = true;
        gSysTick.cnt_ms2 = 0;
    }

    if( ++gSysTick.cnt_ms5 == 5 )
    {
        gSysTick.flag.bf.ms5 = true;
        gSysTick.cnt_ms5 = 0;
    }

    if( ++gSysTick.cnt_ms10 == 10 )
    {
        gSysTick.flag.bf.ms10 = true;
        gSysTick.cnt_ms10 = 0;
    } 

    if( ++gSysTick.cnt_ms25 == 25 )
    {
        gSysTick.flag.bf.ms25 = true;
        gSysTick.cnt_ms25 = 0;
    }

    if( ++gSysTick.cnt_ms50 == 50 )
    {
        gSysTick.flag.bf.ms50 = true;
        gSysTick.cnt_ms50 = 0;
    }

    if( ++gSysTick.cnt_ms100 == 100 )
    {    
        gSysTick.flag.bf.ms100 = true;
        gSysTick.cnt_ms100 = 0;
    }  

    if( ++gSysTick.cnt_ms500 == 500 )
    {
        gSysTick.flag.bf.ms500 = true;
        gSysTick.cnt_ms500 = 0;
    }  

    if( ++gSysTick.cnt_sec1 == 1000 )
    {
        gSysTick.flag.bf.sec1 = true;
        gSysTick.cnt_sec1 = 0;
    }  
}


#endif