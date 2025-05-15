/* 230710,  D4SL 6Ch Wireless

    로타리 스위치 with 8Ch MUX  --- Done
    Input read, with 16Ch MUX   --- Done OPEN, LOCK, KEY 3Type
    무선으로 데이터 송신        --- Done




 * */

/*
// 시퀀스
    도어의 열림과 잠김을 상위로 보고
    락/언락의 지령을 수신하여 처리

// 참고사항
    D4SL을 메인으로 사용하였으나 제품의 결선방식 선택으로 타사 제품도 사용가능함

// To-do
    로타리 스위치 with 8Ch MUX  --- Done
    Input read, with 16Ch MUX   --- Done OPEN, LOCK, KEY 3Type
    무선으로 데이터 송신        --- Done

// Trouble Shooting
    펌웨어상 특이사항 없음
    

*/

// 231209   : 5.5G 실험실 장비 테스트용, 기존의 ESP32핀맵 에러 수정한 하드웨어(ver,231124)
// 240221   : CV_DOOR_Interface_3.ino;
//              -> Ap rev25에 맞춰 수정진행
//              -> 240318; AP와 해제시 모든 도어 오픈
// 240718   : CV_DOOR_Interface_4.ino;
//              -> PING-PONG루틴 개선버젼으로 수정

#include <esp_now.h>
#include <WiFi.h>

#include "common.h"
#include "config.h"
#include "wlDMS3_v2.h"
#include "Tick_Handler.h"
///////////////////////// TIMER INTERRUPT /////////////////////////////////////
// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
#define _TIMERINTERRUPT_LOGLEVEL_     4
#define TIMER0_INTERVAL_MS            1
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ESP32TimerInterrupt.h"
/////////////////////////////////////////////////////////////////////////////////


ESP32Timer ITimer0(0);
sys_config sys_Param;

void Tick_Handle(void);





#define MAX_BUFFER      50

#define CHANNEL 1
#define NUMMASTERS 2
esp_now_peer_info_t masters[NUMMASTERS] = {};

// define pin number is internal IO Numper.
// Input
#define SENROT                  34      // reding 2EA rotary SW MUX 8Ch
#define SENINPUT1               36      // Open/Lock Input MUX 16Ch SCAN
#define SENINPUT2               39      // Interlock KEY Input MUX 16Ch SCAN

// Output       
#define X1292_1                 22
#define X1292_2                 21
#define X1292_3                 19
#define X1292_4                 18
#define X1292_5                 5
#define X1292_6                 17
#define X1292_7                 16
#define X1292_8                 4
#define X1500                   15
#define CONNECT                 23      // Wireless Connect

#define MUX_SEL0                32
#define MUX_SEL1                33
#define MUX_SEL2                25
#define MUX_SEL3                26
#define MUX_8EN                 13      // Read Rotary SW
#define MUX_16EN1               14      // Open, LOCK Detect
#define MUX_16EN2               12      // KEY Detect

#define SW                      0
#define LED                     2
#define WL_TX_PERIODE           500
#define PING_PERIODE            3000
#define PONG_PERIODE            5000
#define WIRELESS_PERIODE        0           // 일단은 0으로 비동기 동작; 추후 문제시 시간설정



// add, v2.1
#define DEFAULT_T               D4SL    // Type, D4SL
#define  _IO_USAGE              1       // IO memory 차지 공간 , 최대 4   
#define  _IO_PAGE               0       // 0일 경우 Page 미사용(etherCAT tx 사용), 1일경우 사용 (PLC에서 요청하는 page word 출력), SV, PV, P, I, D
#define  _SERIAL                0       // Serial page 크기, 기본 7word,  0은 SERIAL 미사용
#define UPDATE_TIME             500     // 무선 업데이트 주기, 단위 ms

#define Board                       "D4SL"        //CHL:temp...
#define VERSION                     "v3.1"      //CHL:temp...
#define Last_Changed_Date           "25/05/09"  //CHL:temp...

char cRx_Buf[MAX_BUFFER];
// add, v2.1
uint16_t lInBit1;    // 230810, Open 8EA, Lock 8EA
uint16_t lInBit2;    // 230810, Key 8EA
uint16_t inBit;      // 230810, from wireless Control, NOT Yet
uint16_t lWinput[ _IO_USAGE * (_IO_PAGE +1 )];
uint16_t lWoutput[ _IO_USAGE * (_IO_PAGE +1 )];

uint8_t iDEBUG = 0;

uint8_t iAddr;              // 로터리 스위치 읽은거
uint8_t iAddr1;             // 로터리스위치1 읽은거
uint8_t iAddr2;             // 로터리스위치2 읽은거
uint8_t iOpen[8];           // input signal
uint8_t iLock[8];           // input signal
uint8_t iKey[8];            // input signal
uint8_t iRelay[16];         // input signal
    
uint32_t lTimerREF;
uint32_t lTimer1;
uint32_t lTimer2;
uint32_t lTimer3;

bool f_led_toggle = false;           // CHL...


void Set_GPIO();
void Mux_Sel_8ch(uint8_t Ch);
void Mux_Sel_16ch(uint8_t Ch);
void Read_Rotary();
// void IO_Scan();
// void IO_Wireless_Command();


void CommandProcess();

#if 1
void set_board_Version(void)
{
    Serial.printf("\n");
    Serial.printf("[MSG]BOARD INFO::%s::VERSION:%s\r\n",Board,VERSION);//CHL:temp...
    Serial.printf("[MSG]LAST CHANGED DATE...[%s]\n",Last_Changed_Date);
    Serial.printf("\n");
}
#endif 


void Set_GPIO()
{
    // Input Setting
    pinMode(SENROT, INPUT);
    pinMode(SENINPUT1, INPUT);
    pinMode(SENINPUT2, INPUT);

    pinMode(SENROT, INPUT);

    // Output Setting
    pinMode(X1292_1, OUTPUT);
    pinMode(X1292_2, OUTPUT);
    pinMode(X1292_3, OUTPUT);
    pinMode(X1292_4, OUTPUT);
    pinMode(X1292_5, OUTPUT);
    pinMode(X1292_6, OUTPUT);
    pinMode(X1292_7, OUTPUT);
    pinMode(X1292_8, OUTPUT);
    pinMode(X1500, OUTPUT);

    pinMode(CONNECT, OUTPUT);
    pinMode(MUX_8EN, OUTPUT);
    pinMode(MUX_16EN1, OUTPUT);
    pinMode(MUX_16EN2, OUTPUT);
    pinMode(MUX_SEL0, OUTPUT);
    pinMode(MUX_SEL1, OUTPUT);
    pinMode(MUX_SEL2, OUTPUT);
    pinMode(MUX_SEL3, OUTPUT);
    //pinMode(KEY_BUILTIN, INPUT);
    pinMode(LED, OUTPUT);

    Serial.println("Set_GPIO");
}

void Mux_Sel_8ch(uint8_t Ch)
{
    switch (Ch)
    {   // S2, S1, S0
        case 0:        digitalWrite(MUX_SEL2, LOW);         digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 1:        digitalWrite(MUX_SEL2, LOW);         digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);       break;
        case 2:        digitalWrite(MUX_SEL2, LOW);         digitalWrite(MUX_SEL1, HIGH);       digitalWrite(MUX_SEL0, LOW);        break;
        case 3:        digitalWrite(MUX_SEL2, LOW);         digitalWrite(MUX_SEL1, HIGH);       digitalWrite(MUX_SEL0, HIGH);       break;
        case 4:        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 5:        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);       break;
        case 6:        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);       digitalWrite(MUX_SEL0, LOW);        break;
        case 7:        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);       digitalWrite(MUX_SEL0, HIGH);       break;
        default:
            break;
    }
    delay(10);
}

void Mux_Sel_16ch(uint8_t Ch)
{
    switch (Ch)
    {   // S3, S2, S1, S0
        case 0:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 1:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 2:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 3:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 4:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 5:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 6:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 7:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 8:        digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 9:        digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 10:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 11:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 12:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 13:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 14:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 15:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        default:
            break;
    }
  //  delay(10);
}

void Read_Rotary()
{
    uint8_t i = 0;
    iAddr1 = 0;
    iAddr2 = 0;

    digitalWrite(MUX_8EN, HIGH);
    delay(100);

    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr1 += 1;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr1 += 2;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr1 += 4;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr1 += 8;
    i++;

    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr2 += 1;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr2 += 2;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr2 += 4;
    i++;
    Mux_Sel_8ch(i);
    if (!digitalRead(SENROT))
        iAddr2 += 8;

    digitalWrite(MUX_8EN, LOW);

    iAddr = (iAddr2 * 16) + iAddr1;
    Serial.printf( "iAddr : %d, Addr1 : %d, Addr2 : %d\n", iAddr, iAddr1, iAddr2 );
}

void IO_Scan2()
{
    // SENINPUT1, MUX_16EN1, Open1 ~ Open8, Lock1 ~ Lock8
    // SENINPUT2, MUX_16EN2, Key1 ~ Key8, Use Y4 ~Y11
    static bool en_set=false;
    uint8_t j = 0, i=0;
    static uint8_t cnt_open=0, cnt_close=1;

    static uint8_t status=0;

    switch( status )
    {
        case 0:
            digitalWrite(MUX_16EN1, HIGH);
            status = 1;
        break;

        case 1:
            if( cnt_open >= 15 ) cnt_open = 0;
            Mux_Sel_16ch(cnt_open); // 0,2,4,6,8,10,12,14
            status = 2;
        break;

        case 2:
            if (!digitalRead(SENINPUT1))
            {
                bitWrite(lInBit1, cnt_open, 1);
            }
            else
            {
                bitWrite(lInBit1, cnt_open, 0);
            }   
            cnt_open += 2;
            status = 3;     
        break;

        case 3:
            if( cnt_close >= 16 ) cnt_close = 1;
            Mux_Sel_16ch(cnt_close); // 1,3,5,7,9,11,13,15
            status = 4;        
        break;

        case 4:
            if (!digitalRead(SENINPUT1))
            {
                bitWrite(lInBit1, cnt_close, 1);
            }
            else
            {
                bitWrite(lInBit1, cnt_close , 0);
            }   
            
            //15번까지 읽었으면 처음부터 시작
            if( (cnt_close>=15) && (cnt_open>=14) )
            {
                cnt_open = 0;
                cnt_close = 1;
                status = 5; 
            }else
            {
                cnt_close += 2;   
                status = 1;  
            }
        break;

        case 5:
            digitalWrite(MUX_16EN1, LOW);
            status = 0;
        break;
    }

}

void CommandProcess()
{
#if 0
    uint8_t irxlength = 0;
    uint8_t icksum = 0;
    uint16_t ltemp;
    char ctemp[4];
    uint8_t itemp;

    // cRx_Buf[RXBUF_SIZE];
    if (Serial.available() > 0 )
    {
        irxlength = Serial.readBytesUntil(NULL, cRx_Buf, MAX_BUFFER);

        if( cRx_Buf[0] == '1' && cRx_Buf[1] == '0' )            digitalWrite(X1292_1, LOW);
        else if( cRx_Buf[0] == '1' && cRx_Buf[1] == '1' )       digitalWrite(X1292_1, HIGH);
        else if( cRx_Buf[0] == '2' && cRx_Buf[1] == '0' )       digitalWrite(X1292_2, LOW);
        else if( cRx_Buf[0] == '2' && cRx_Buf[1] == '1' )       digitalWrite(X1292_2, HIGH);
        else if( cRx_Buf[0] == '3' && cRx_Buf[1] == '0' )       digitalWrite(X1292_3, LOW);
        else if( cRx_Buf[0] == '3' && cRx_Buf[1] == '1' )       digitalWrite(X1292_3, HIGH);
        else if( cRx_Buf[0] == '4' && cRx_Buf[1] == '0' )       digitalWrite(X1292_4, LOW);
        else if( cRx_Buf[0] == '4' && cRx_Buf[1] == '1' )       digitalWrite(X1292_4, HIGH);
        else if( cRx_Buf[0] == '5' && cRx_Buf[1] == '0' )       digitalWrite(X1292_5, LOW);
        else if( cRx_Buf[0] == '5' && cRx_Buf[1] == '1' )       digitalWrite(X1292_5, HIGH);
        else if( cRx_Buf[0] == '6' && cRx_Buf[1] == '0' )       digitalWrite(X1292_6, LOW);
        else if( cRx_Buf[0] == '6' && cRx_Buf[1] == '1' )       digitalWrite(X1292_6, HIGH);
        else if( cRx_Buf[0] == '7' && cRx_Buf[1] == '0' )       digitalWrite(X1292_7, LOW);
        else if( cRx_Buf[0] == '7' && cRx_Buf[1] == '1' )       digitalWrite(X1292_7, HIGH);
        else if( cRx_Buf[0] == '8' && cRx_Buf[1] == '0' )       digitalWrite(X1292_8, LOW);
        else if( cRx_Buf[0] == '8' && cRx_Buf[1] == '1' )       digitalWrite(X1292_8, HIGH);
        else if( cRx_Buf[0] == '9' && cRx_Buf[1] == '0' )       digitalWrite(X1500, LOW);
        else if( cRx_Buf[0] == '9' && cRx_Buf[1] == '1' )       digitalWrite(X1500, HIGH);

        for( uint8_t i = 0; i < MAX_BUFFER; i++ )
        {   // buffer Clear
            cRx_Buf[i] = 0;
        }
    }
#endif
}

void IO_All_Open(void)
{
    digitalWrite(X1292_1, HIGH);
    digitalWrite(X1292_2, HIGH);
    digitalWrite(X1292_3, HIGH);
    digitalWrite(X1292_4, HIGH);
    digitalWrite(X1292_5, HIGH);
    digitalWrite(X1292_6, HIGH);
    digitalWrite(X1292_7, HIGH);
    digitalWrite(X1292_8, HIGH);
    digitalWrite(X1500, HIGH); 
}

void IO_All_Close(void)
{
    digitalWrite(X1292_1, LOW);
    digitalWrite(X1292_2, LOW);
    digitalWrite(X1292_3, LOW);
    digitalWrite(X1292_4, LOW);
    digitalWrite(X1292_5, LOW);
    digitalWrite(X1292_6, LOW);
    digitalWrite(X1292_7, LOW);
    digitalWrite(X1292_8, LOW);
    digitalWrite(X1500, LOW);
}

void WirelessCommandProcess() //1ms loop
{
    static uint16_t timeout=0;
    static uint16_t read_buf[20]={0,};

    #if FUNC_WL_REPAIRING_DELAY
    static bool paired_start=false, paired_end=false;
    static uint16_t paried_cnt=0;
    #endif

    uint8_t cmd=0;
    uint8_t iPage=0;
    uint8_t iCommand=0;
    bool send_data=false;

    uint8_t i;

    #if TEST_RESPONSE_DELAY
    static uint32_t ready_cnt=0;
    static uint32_t ready_max;

    if( sys_Param.channel_WL>8 ) 
    {
        ready_max=8;    
    }else ready_max=sys_Param.channel_WL;

    ready_max = ready_max*1000;
    #endif

    if( !WL.Stat_Paring ) //? ap에서 보낸 req를 받으면 stat_pairing set
    {
        timeout = 0;

        #if TEST_RESPONSE_DELAY
        ready_cnt=0;
        #endif
        
        #if FUNC_WL_REPAIRING_DELAY
            paired_end=true;
            if( paired_start )
            {
                if ( gSysTick.flag.bf.ms100 )  // 타임아웃 &&
                {
                    Serial.printf("[MSG]PAIRED DISCONNECT::WAITING... %d\n\r",paried_cnt);
                } 

                if( ++paried_cnt>5000 ) //끊어졌을때 5초간 대기
                {
                    paried_cnt=0;
                    paired_start=0;
                    Serial.printf("[MSG]PAIRED DISCONNECT::WAIT OUT\n\r");
                }else
                {
                    return;
                }
            }
            paried_cnt=0;
            paired_start=0;
        #endif
        digitalWrite(LED, LOW);
        digitalWrite(CONNECT, LOW);
        IO_All_Open(); //all 'key'LED ON //RELEASE?

        if ( gSysTick.flag.bf.sec1 )
        {
            Serial.printf("[MSG]UN PAIRING...\n\r");
        }    

        return;
    }
    //:CHL
    digitalWrite(CONNECT, HIGH); //* Release Unlock 상태 의미 

    #if FUNC_WL_REPAIRING_DELAY
    if( paired_start==0 )   //처음 연결시 1초간 대기
    {
        #if TEST_RESPONSE_DELAY
        if( ready_cnt<ready_max )
        {
            ready_cnt++;
            return;
        }
        #endif

        if( ++paried_cnt>1000 )
        {
            Serial.printf("[MSG]PAIRED CONNECT, WAIT OUT\n\r");
            paired_start=1;
            paired_end=0;
            paried_cnt=0;
        }else
        {
            if( gSysTick.flag.bf.ms100 )
            {
                Serial.printf("[MSG]PAIRED CONNECT, WAIT %d\n\r",paried_cnt);
            }
        }
    }else
    {
        if( paired_end )
        {
            paired_start=0;
            paired_end=0;
            paried_cnt=0;
        }
    }
    #endif

    if( WL.req_io_get ) //recv_cb에서 FSM 
    {
        WL.req_io_get = 0;
        timeout = 0;

        lWoutput[0] = lInBit1;
#if 0
        output_toggle(LED);
#endif
        //& AP 데이터 수신부 
        WL.io_get_data(read_buf,2); //^* read buf에 데이터 담아옴 
       //read buf에 rx buf .. 
        inBit = read_buf[0] & 0xFFFF;                                              //^  lInBit1 == lWoutput[0] == mux -> 현재 d4sl에서 open,lock 상태 
        Serial.printf("Read Mux : 0X%04X, From PLC : 0x%04x\r\n", lInBit1, inBit); //^* inBit -> plc에서 output.같은 

        #if FUNC_WL_REPAIRING_DELAY
        if( !paired_start )
        {   
            //& AP로 송신 
            WL.io_set_data(PEER_IO_GET,lWoutput, 2); // 뒷수자 단위 1바이트, 16비트 보내려면 2써야함
            return;
        }
        #endif


        if( inBit > 0 )
        {
            for( i = 0; i < 16; i++ )
            {
                iRelay[i] = bitRead(inBit, i); //^ 몇 번 켜라 꺼라 명령을 읽고 해당 H/L 제어
            }

            #if 1
            if( iRelay[0] == 1 )       digitalWrite(X1292_1, HIGH);
            else if( iRelay[0] == 0 )  digitalWrite(X1292_1, LOW);
            if( iRelay[1] == 1 )       digitalWrite(X1292_2, HIGH);
            else if( iRelay[1] == 0 )  digitalWrite(X1292_2, LOW);
            if( iRelay[2] == 1 )       digitalWrite(X1292_3, HIGH);
            else if( iRelay[2] == 0 )  digitalWrite(X1292_3, LOW);
            if( iRelay[3] == 1 )       digitalWrite(X1292_4, HIGH);
            else if( iRelay[3] == 0 )  digitalWrite(X1292_4, LOW);
            if( iRelay[4] == 1 )       digitalWrite(X1292_5, HIGH);
            else if( iRelay[4] == 0 )  digitalWrite(X1292_5, LOW);
            if( iRelay[5] == 1 )       digitalWrite(X1292_6, HIGH);
            else if( iRelay[5] == 0 )  digitalWrite(X1292_6, LOW);
            if( iRelay[6] == 1 )       digitalWrite(X1292_7, HIGH);
            else if( iRelay[6] == 0 )  digitalWrite(X1292_7, LOW);
            if( iRelay[7] == 1 )       digitalWrite(X1292_8, HIGH);
            else if( iRelay[7] == 0 )  digitalWrite(X1292_8, LOW);
            #else
            digitalWrite(X1292_1, HIGH);
            #endif
            
            if( iRelay[8] == 1 )       digitalWrite(X1500, HIGH);
            else if( iRelay[8] == 0 )  digitalWrite(X1500, LOW);
        }
        else //^ inbit가 0이면 모두 끄는 것 
        {
            #if FUNC_WL_ALL_RELEASE
            IO_All_Close();
            #endif
        }
        //! TEST POINT 
        WL.io_set_data(PEER_IO_GET,lWoutput, 2); // 뒷수자 단위 1바이트, 16비트 보내려면 2써야함
    }else if( WL.req_io_set )
    {
        WL.req_io_set = 0;
        timeout = 0;

        WL.io_set_data(PEER_IO_SET,lWoutput, 2); // 뒷수자 단위 1바이트, 16비트 보내려면 2써야함
    }

    if (gSysTick.flag.bf.sec1) // 타임아웃 &&
    {
        if (++timeout > 10)
        {
#if FUNC_WL_REPAIRING_DELAY
            paried_cnt = 0;
#endif
            timeout = 0;
            WL.delete_peer(); // 타임아웃되면 AP 페어링 해제
        }
    }

    if( !WL.req_io_get || !WL.req_io_set ) //^ 모두 reset되니까 일단 둘다 걸어줌 하나만 해ㅐ도ㅇㅋ
    {
        f_led_toggle=true;
    }
}



// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR TimerHandler0(void *timerNo)
{
    gSysTick.f_ms1 = 1;
    return true;
    ///////////////////////////////////////////////////////////
}
void App_Init_Sys_Param(void)
{
    // 채널을 alias 번호에 맵핑시키는 방법 : 하드웨어 추가후에는 변경해야 함
    if ((iAddr < 0x10) || (iAddr > 0xDF))
        iAddr = 0xE0;

    // 할당 범위 : 16 - 31 = 1, 16개씩
    sys_Param.addr_WL = iAddr;
    sys_Param.channel_WL = iAddr / 16;
    sys_Param.group_WL = SYS_SETUP_WL_GROUP; // 하드웨어 변경 전까지는 그룹은 여기서 하드코딩으로 수정해서 펌웨어 만들어야 함
}

void App_Init_Wifi(void)
{
    WL.setType(DEFAULT_T);
    WL.setGroup(sys_Param.group_WL);
    WL.setChannel(sys_Param.channel_WL);
    WL.setAddr(sys_Param.addr_WL);
    if (WL.init(_IO_USAGE, _IO_PAGE, _SERIAL) == true)
    {
        Serial.printf("[MSG]SYSTEM INFO::WIRELESS::Group = %d\r\n", sys_Param.group_WL);
        Serial.printf("[MSG]SYSTEM INFO::WIRELESS::Channel = %d\r\n", sys_Param.channel_WL);
        Serial.printf("[MSG]SYSTEM INFO::WIRELESS::Address = %d\r\n", sys_Param.addr_WL);
    }

    WL.setDebug_Msg(2); //level 
}

void setup()
{
    memset(&sys_Param, 0, sizeof(sys_Param));
    Serial.begin(230400);
    //Serial.begin(115200);
    Serial.setTimeout(100);
    while(!Serial);
    set_board_Version();
    
    Set_GPIO(); // Configue Pin I/O Mode
    Read_Rotary();
    
    App_Init_Sys_Param();
    App_Init_Wifi();

    lInBit1 = 0;
    lInBit2 = 0;
    inBit = 0;
    
	if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
	{
		Serial.print(F("Starting  ITimer0 OK\r\n"));
		//Serial.println(millis());
	}
    //digitalWrite(MUX_16EN1, HIGH);

}

void loop()
{
    if (gSysTick.f_ms1) 
    {
        gSysTick.f_ms1 = 0;
        Tick_Handle();
        IO_Scan2();
        WirelessCommandProcess();
    }else; //end of f_ms1;
    
    if( gSysTick.flag.bf.ms500 )
    {
        if( f_led_toggle == true)
        {
            f_led_toggle=false;
            output_toggle(LED);
        }
    }


}




