
/* Rev3.2
* 사용방법
* 1. init
* - setType, setAddr, setChannel, setGroup, Wl.init() : 예제 코드 참조
* 
* 2. wifi reset
* - Address, Channel, Group 설정후 아래 순서로 reset 진행 필수수
* - WL.delete_peer()
* - call init function
*
* 3. Debug Log 활성화
* - setDebug_Msg : 0=None, 1=Standard Debug Level, 2=Detail Mode
* 4. Timeout
* delete_peer() 함수 call
*/

/* - 초기화 함수 예제
void App_Init_Wifi(void)
{
  WL.setType(DEFAULT_T);
  WL.setAddr(sys_main_param.func.wl_addr);
  WL.setChannel(sys_main_param.func.wl_channel);
  WL.setGroup(sys_main_param.func.wl_group);
  if( WL.init(_IO_USAGE, _IO_PAGE, _SERIAL) == true) 
  {
    Serial.printf( "[MSG]SYSTEM INFO::WIRELESS::Group = %d\r\n",sys_main_param.func.wl_group);
    Serial.printf( "[MSG]SYSTEM INFO::WIRELESS::Channel = %d\r\n",sys_main_param.func.wl_channel);
    Serial.printf( "[MSG]SYSTEM INFO::WIRELESS::Address = %d\r\n",sys_main_param.func.wl_addr);
  } 
}
*/

/* Reset 예제
     if( sys_main_param.ap_reset )
    {
      sys_main_param.ap_reset = 0;

      WL.delete_peer();

      App_Init_Wifi();
      Serial.printf("[MSG]LOOP::AP Reset\r\n");
    }
 */

/* Debug Log 활성화
  WL.setDebug_Msg(0);
 */

#ifndef WIRELESS_H
#define WIRELESS_H

// included header files
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "common.h"
#include "Tick_Handler.h"
//////////////////////////////////////

// 무선 Network config
#define MAX_PEER                    16              // peer 최대수, 페어링 비트가 16개임
#define MAX_CHANNEL                 14              // 2.4GHz channel 한국/중국 1~13
#define MIN_RSSI                    -90             // 0에 가까운게 좋음

#define AP                          "AP" //CHL:temp...
#define VERSION                     "v3.1"//CHL:temp...

// 부품 Type and id define
#define DIW_FLOW                    0x01
#define CDA_FLOW                    0x02
#define SMART_DAMPER                0x03
#define X_RAY                       0x04
#define D40A                        0x05
#define D4SL                        0x06
#define TIC                         0x07
#define LMFC                        0x08
#define MANOMETER                   0x09
#define LCT                         0x0A    //GH : Add

// Reqeuset & response HEX
#define AP_PAIRING_REQ              0x01      
#define AP_PAIRING_CANCEL           0x02
#define AP_IO_GET                   0x03      
#define AP_IO_SET                   0x04
#define AP_SERIAL_SET               0x05
#define AP_SERIAL_GET               0x06

#define PEER_PAIRING_OK             0x81
#define PEER_PAIRING_CANCEL         0x82
#define PEER_IO_GET                 0x83
#define PEER_IO_SET                 0x84
#define PEER_SERIAL_SET             0x85      
#define PEER_SERIAL_GET             0x86

// #define PEER_SERIAL_EXCHANGE_RESP 0xE
// recv/send buffer in words
#define MAX_PAGE                    10
#define MAX_IO                      6
#define MAX_SERIAL_DATA             7

typedef enum _WIFI_PACKET_T
{
	WIFI_PACKET_GROUP 	= 0,
	WIFI_PACKET_CHANNEL,
	WIFI_PACKET_COMMAND,
	WIFI_PACKET_TYPE,
	WIFI_PACKET_ADDRESS,
	WIFI_PACKET_LENGTH,
	WIFI_PACKET_DATA,

	WIFI_PACKET_MAX = 50,
} WIFI_PACKET_T;

typedef struct
{
    uint8_t io_usage; // IO영역 점유 word
    uint8_t io_page;  // IO영역 Page수(4bit, 1~15, 0은 미사용), io_page>1, tx영역에서 request하는 date 씀;
    uint8_t serial;   // serial page 수 0 이면 미사용
} ecat_t;

// promiscuous_rx call back 함수, recev packet 분석, RSSI값확보
typedef struct
{
    unsigned frame_ctrl : 16;
    unsigned duration_id : 16;
    unsigned sequence_ctrl : 16;
    uint8_t addr1[6]; /* src address */
    uint8_t addr2[6]; /* recv address */
    uint8_t addr3[6]; /* broadcast address */
    uint8_t addr4[6]; /* non */
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0];
} wifi_ieee80211_packet_t;

// class 객체
class wireless
{
public:

    /* Public 변수; 클래스.xxxxxxx 로 사용가능 */
    bool Stat_Paring;
    uint16_t typeID;
    uint8_t peer_type;
    uint8_t peer_addr; //디바이스의 어드레스 설정 값
    uint8_t peer_channel;
    uint8_t bFlagOnce;      // AP_PAIRING_REQ 연속 진입 방지;

    uint8_t AP_Mac[6]; // AP Address

    uint8_t Debug_Msg;

    //uint8_t ap_cancel_ok;

    uint8_t req_io_get;    
    uint8_t req_io_set;   
    uint8_t req_serial_get;   
    uint8_t req_serial_set;    
    uint8_t serial_len;
    uint8_t ap_group;
    uint8_t wl_channel;
    uint8_t get_channel;
    wifi_second_chan_t second_ch;


    wireless() : peer_addr(0), peer_type(0), AP_failure_flag(false)
    { // 생성자1  : defalut id
        send_p = &wireless::send_cb;
        recv_p = &wireless::recv_cb;
        rx_p = &wireless::promiscuous_rx_cb;
    }

    wireless(uint8_t peer_type, uint8_t peer_addr) : peer_type(peer_type), peer_addr(peer_addr), AP_failure_flag(false)
    { // 생성자2  : id 매개변수
        send_p = &wireless::send_cb;
        recv_p = &wireless::recv_cb;
        rx_p = &wireless::promiscuous_rx_cb;
    }

    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // 세마포어
    SemaphoreHandle_t sem_tx;
    SemaphoreHandle_t sem_ack;
    SemaphoreHandle_t sem_rx;
    SemaphoreHandle_t sem_ack2;
    SemaphoreHandle_t sem_ex;

    bool init(uint8_t _IO_USAGE, uint8_t _IO_PAGE, uint8_t _SERIAL); // 초기화 setup() 함수내 멤버함수 호출

    uint8_t getID()
    {
        // 현 ID 값 확인, setID 에서 처리해둔 값 리턴
        Serial.printf("getID();     peer_addr     : %3d(0x%02x)\n", peer_addr, peer_addr);
        return peer_addr;
    };

    uint8_t getType()
    {
        Serial.printf("getType();   peer_type   : %3d(0x%02x)\n", peer_type, peer_type);
        return peer_type;
    }; // Peer_type 값, type은 컴파일시 설정

    uint8_t getPeerChannel(void)
    {
        return peer_channel;
    };

    bool set_pairing(const uint8_t *mac_addr, uint16_t channel);

    bool set_channel(uint8_t ch); // 채널변경 1 ~12

    int getRSSI()
    {
        return RSSI;
    }; // RSSI Return

    void setAddr(uint8_t addr) // call after init(), pairing 상태에서 Id가 변경 시 기존 AP와 Pairing 취소함,
    {
        peer_addr = addr;
       // Serial.printf("setAddr();     peer_addr     : %3d(0x%02x)\n", peer_addr, peer_addr);
    };

    void setChannel(uint8_t channel) // call after init(), pairing 상태에서 Id가 변경 시 기존 AP와 Pairing 취소함,
    {
        wl_channel = channel;
       // Serial.printf("setAddr();     peer_addr     : %3d(0x%02x)\n", peer_addr, peer_addr);
    };    

    void setGroup(uint8_t group) // call after init(), pairing 상태에서 Id가 변경 시 기존 AP와 Pairing 취소함,
    {
        ap_group = group;
       // Serial.printf("setAddr();     peer_addr     : %3d(0x%02x)\n", peer_addr, peer_addr);
    };    

    void setType(uint8_t type)
    {
        peer_type = type;
      //  Serial.printf("setType();   peer_type   : %3d(0x%02x)\n", peer_type, peer_type);
    };

    void setECAT(uint8_t _io_usage, uint8_t _io_page, uint8_t _serial)
    {
        ecat.io_usage = _io_usage;
        ecat.io_page = _io_page;
        ecat.serial = _serial;
    };

    void setDebug_Msg(uint8_t level)
    {
        Debug_Msg = level;
    }

    uint8_t get_query(const uint8_t *_data)
    {
        return _data[0];
    };

    uint8_t get_index(const uint8_t *_data)
    {
        return _data[1];
    };


    void send_data(uint8_t cmd, const void *tx_data, uint8_t len)
    {
        // send query and index with tx data

        esp_err_t result;
        uint8_t buff[8 * MAX_PAGE], tx_len;
        word_big_endian_t crc16;

        if ( cmd!=0 )
        {
            tx_len = 8 + len;
            //CRC 체크
    
            buff[0] = ap_group;
            buff[1] = peer_channel;//cmd;
            buff[2] = cmd;//peer_channel;
            buff[3] = peer_type;
            buff[4] = peer_addr;
            buff[5] = tx_len;

            if( len ) memcpy(&buff[6], tx_data, tx_len);
            crc16.flag.wd = crc16_modbus(CRC16_MODBUS_INIT_CODE, buff, tx_len-2);

            buff[tx_len-2] = crc16.flag.bf.low;
            buff[tx_len-1] = crc16.flag.bf.hi;

            result = esp_now_send(AP_Mac, (const uint8_t *)buff, tx_len);

            if(Debug_Msg > 1)
            {
                Serial.printf("[MSG]WIFI::SEND DATA::CRC LOW=0x%02x, HIGH=0x%02x\r\n",crc16.flag.bf.low, crc16.flag.bf.hi);
            }
        }
    };

    void delete_peer()
    {
        esp_now_del_peer(AP_Mac);
        memset(AP_Mac, 0, 6);

        if(Debug_Msg > 0)
        {
            Serial.println("[MSG]WIFI::DELETE PEER\r\n");
        }

        Stat_Paring = 0;
    };

    // call back member function
    void send_cb(const uint8_t *des_addr, esp_now_send_status_t status);
    void recv_cb(const uint8_t *src_mac, const uint8_t *data, int len);
    void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type); // wifi recev callback 함수, rssi확보


    // Call back member function pointer return
    void (wireless::*send_p)(const uint8_t *, esp_now_send_status_t);
    void (wireless::*recv_p)(const uint8_t *, const uint8_t *, int);
    void (wireless::*rx_p)(void *, wifi_promiscuous_pkt_type_t);

    void io_set_data(uint8_t cmd, void *data, uint8_t len ) // AP로 일방송신, len 1당 1바이트
    {
        memcpy((uint8_t *)rx_ioBuff, (uint16_t *)data, len); // 보낼데이터를 rx_ioBuff에 담고 이후 send_data에서 처리
        send_data(cmd, &rx_ioBuff, len );   // AP rev24 Test용; 2페이지 한번에전송   
    };

    void io_get_data(void *data, uint8_t len) // AP로부터 수신, len 1당 1바이트
    {
        memcpy((uint8_t *)data, (uint8_t *)tx_ioBuff, len); // 받은데이터를 data에 복사(data는 인자로 사용한 변수)
    };

    void serial_set_data(uint8_t cmd, void *data, uint8_t len) // AP로 송신, len 1당 1바이트
    {
        memcpy((uint8_t *)rx_sBuff, (uint8_t *)data, len);
        send_data(cmd, &rx_sBuff, len );
    };
    void serial_get_data(void *data, uint8_t len) // AP로부터 수신, len 1당 1바이트
    {
        memcpy((uint8_t *)data, (uint8_t *)tx_sBuff, len);
    };

    
    
    /* private 변수; 객체 내 에서만 사용가능; 외부에서 접근 불가 */
    private:
        uint16_t rx_ioBuff[MAX_IO];
        uint16_t tx_ioBuff[MAX_IO];
        // uint16_t rx_sBuff[MAX_PAGE][7];
        // uint16_t tx_sBuff[MAX_PAGE][7];
        uint16_t rx_sBuff[MAX_PAGE];
        uint16_t tx_sBuff[MAX_PAGE];
        uint8_t rx_nak_cnt = 0;
        uint8_t tx_nak_cnt = 0;
        // uint8_t broadcast_addr[6];
        // uint8_t AP_Mac[6]; // AP Address
        uint8_t sindex;
        int RSSI;
        bool AP_failure_flag;
        uint8_t channel;
        esp_now_peer_info_t peerInfo;
        bool pair_info_check(bool paired, uint8_t *buff);
        ecat_t ecat;
        bool duplicate_ID_flag = false;
}; // Wireless Class End

wireless WL; // WL로 객체 생성


// 콜백 함수 포인터
void send_CB(const uint8_t *des_addr, esp_now_send_status_t status)
{
    return ((WL.*WL.send_p)((const uint8_t *)des_addr, (esp_now_send_status_t)status));
}
void recv_CB(const uint8_t *src_mac, const uint8_t *data, int len)
{
    return ((WL.*WL.recv_p)((const uint8_t *)src_mac, (const uint8_t *)data, (int)len));
}
void rx_CB(void *buf, wifi_promiscuous_pkt_type_t type)
{
    return ((WL.*WL.rx_p)((void *)buf, (wifi_promiscuous_pkt_type_t)type));
}

void wireless::recv_cb(const uint8_t *src_mac, const uint8_t *data, int len)
{
    static uint8_t rx_buff[100];
    static uint8_t tx_buff[100];
    static uint8_t cmd = 0;    
    uint8_t tx_len=0, rx_len=len;
    bool re_pairing=false, send_wifi=false;   
    word_big_endian_t crc16; 

    memcpy( rx_buff, data, rx_len);
    if (!pair_info_check(Stat_Paring, rx_buff))
    {
        if(Debug_Msg > 1)
        {
            Serial.printf("[MSG]WIFI::CB::Not match pair info Group=0x%02x channel=0x%02x command : 0x%02x, Type=0x%02x, Addr=0x%02x \n\r", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]);
        }
        return;
    }else
    {
    }

    crc16.flag.wd = crc16_modbus(CRC16_MODBUS_INIT_CODE, rx_buff, rx_len-2);
    if( rx_buff[rx_len-2] != crc16.flag.bf.low )
    {
        if(Debug_Msg > 1)
        {
            Serial.printf("[MSG]WIFI::CB::CRC LOW BUF=0x%02x, CAL=0x%02x",rx_buff[rx_len-2] ,crc16.flag.bf.low );
        }
        return;
    }
    
    if( rx_buff[rx_len-1] != crc16.flag.bf.hi )
    {
        if(Debug_Msg > 1)
        {
            Serial.printf("[MSG]WIFI::CB::CRC LOW BUF=0x%02x, CAL=0x%02x",rx_buff[rx_len-1] ,crc16.flag.bf.hi );
        }
        return;
    } 

    cmd = rx_buff[WIFI_PACKET_COMMAND];

    if(Debug_Msg > 1)
    {
        Serial.printf("[MSG]WIFI::CB::Group=0x%02x channel=0x%02x command : 0x%02x, Type=0x%02x, Addr=0x%02x \n\r", rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3], rx_buff[4]);
    }
    memset(tx_buff, 0, sizeof(tx_buff));

    switch (cmd)
    {
        case AP_PAIRING_REQ :   
            //bFlagOnce : 앞 사람이 이거 왜 넣었는지 모르겠네...있어도 동작상 문제는 없지만,  추후에 삭제 예정..
            //이걸 재귀콜 한다고??????? 확인 후 삭제
            if( !bFlagOnce && !Stat_Paring )
            {
                bFlagOnce = 1;
                Stat_Paring = true;
                peer_channel = rx_buff[WIFI_PACKET_CHANNEL]; 
                
                memcpy(AP_Mac, src_mac, 6);
                set_pairing(AP_Mac, wl_channel );

                tx_buff[tx_len++] = ecat.io_usage;
                tx_buff[tx_len++] = ecat.io_page;
                tx_buff[tx_len++] = ecat.serial;

                send_data(PEER_PAIRING_OK, tx_buff, tx_len);
                if(Debug_Msg > 0)
                {
                    Serial.printf("[MSG]WIFI::CB::SEND PAIRING OK\r\n");
                }

                bFlagOnce = 0;
            }else if(Stat_Paring)
            {
                re_pairing = true;
            }
            
            if( re_pairing )
            {
                esp_now_del_peer(AP_Mac);
                memset(AP_Mac, 0, 6);

                tx_buff[tx_len++] = ecat.io_usage;
                tx_buff[tx_len++] = ecat.io_page;
                tx_buff[tx_len++] = ecat.serial;

                memcpy(AP_Mac, src_mac, 6);
                set_pairing(AP_Mac, wl_channel);
                
                send_data(PEER_PAIRING_OK, tx_buff, tx_len);
                
                Stat_Paring = true;
                if(Debug_Msg > 0)
                {
                    Serial.printf("[MSG]WIFI::CB::SEND RE-PAIRING OK\r\n");
                }
            }
        break;

        case AP_PAIRING_CANCEL:
            if(Debug_Msg > 0)
            {
                Serial.printf("[MSG]WIFI::CB::AP_CANCEL_PAIRING\r\n");
            }
            send_data(PEER_PAIRING_CANCEL, tx_buff, 0);
            esp_now_del_peer(AP_Mac);
            memset(AP_Mac, 0, 6);
            Stat_Paring = 0;
        break;

        case AP_IO_GET: 
            if( Stat_Paring )
            {
                WL.req_io_get = 1;        
                if( ecat.io_page==0 )
                {
                    memcpy(&tx_ioBuff, &rx_buff[WIFI_PACKET_DATA], 2);
                }else
                {
                    memcpy(&tx_ioBuff, &rx_buff[WIFI_PACKET_DATA], rx_len-8);
                }  
            } 
            if(Debug_Msg > 1)
            {
                Serial.printf("[MSG]WIFI::CB::AP_DATA_REQ\r\n");
            }
        break;
        
        case AP_IO_SET: 
            
            if( ecat.io_page && (rx_buff[WIFI_PACKET_DATA+1]&BIT7))
            {
                memcpy(&tx_ioBuff, &rx_buff[WIFI_PACKET_DATA], 2);
                WL.req_io_set = 1;
                if(Debug_Msg > 1)
                {
                    Serial.printf("[MSG]WIFI::CB::AP_DATA_SET::PAGED\r\n");
                }
            }else
            {
                if(Debug_Msg > 0)
                {
                    Serial.printf("[MSG]WIFI::CB::AP_DATA_SET::NONE PAGE\r\n");
                }
            }
        break;

        case AP_SERIAL_SET: //여기 스마트댐퍼
            WL.req_serial_set = 1;
            WL.serial_len = rx_len-8;
            memcpy(&tx_sBuff, &rx_buff[WIFI_PACKET_DATA], WL.serial_len);  
        break;

        case AP_SERIAL_GET: //여기 스마트댐퍼
            WL.req_serial_get = 1;
            WL.serial_len = rx_len-8;
            //Get Data시 serial pate 7word data는 peer에서 알아서 사용
            memcpy(&tx_sBuff, &rx_buff[WIFI_PACKET_DATA], WL.serial_len);  
        break;

        default :
        break;
    }
}

void wireless::send_cb(const uint8_t *des_addr, esp_now_send_status_t status)
{
}

void wireless::promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    if (memcmp(AP_Mac, hdr->addr2, 6) == 0)
    {
        RSSI = ppkt->rx_ctrl.rssi;
    }else
    {
        return;
    }
}

bool wireless::set_pairing(const uint8_t *mac_addr, uint16_t channel)
{
    esp_err_t paring_result;
    memcpy(&peerInfo.peer_addr, mac_addr, 6);   
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    paring_result = esp_now_add_peer(&peerInfo);


    if (paring_result == ESP_OK)
    {
        if(Debug_Msg > 0)
        {
            Serial.printf("[MSG]WIFI::CB::PAIRING REGISTER NEW\r\n");
        }
        return true;
    }
    else if (paring_result == ESP_ERR_ESPNOW_EXIST)
    {
        if(Debug_Msg > 0)
        {
            Serial.printf("[MSG]WIFI::CB::PAIRING REGISTER EXIST\r\n");
        }
        return true;
    }
    else
    {
        if(Debug_Msg > 0)
        {
                Serial.printf("[MSG]WIFI::CB::PAIRING REGISTER FAIL\r\n");
        }
        return false;
    }
}

bool wireless::pair_info_check(bool paired, uint8_t *buff)
{
    if( (ap_group==buff[WIFI_PACKET_GROUP]) && (peer_type==buff[WIFI_PACKET_TYPE]) && (peer_addr==buff[WIFI_PACKET_ADDRESS]) )
    {
        if( paired )
        {
            if( peer_channel==buff[WIFI_PACKET_CHANNEL] ) return true;
        }else
        {
            return true;
        }
    }
    return false;
}
bool wireless::init(uint8_t _IO_USAGE, uint8_t _IO_PAGE, uint8_t _SERIAL)
{
    setECAT(_IO_USAGE, _IO_PAGE, _SERIAL);
    WiFi.mode(WIFI_STA);
    sem_ack = xSemaphoreCreateBinary();
    sem_tx = xSemaphoreCreateBinary();
    sem_ack2 = xSemaphoreCreateBinary();
    sem_rx = xSemaphoreCreateBinary();
    sem_ex = xSemaphoreCreateBinary();

    #if 1
    if( esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40) == ESP_OK )
    {
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::ERR::setup bandwidth\r\n");
        return false;
    }   
    #endif

    if (esp_now_init() != ESP_OK)
        return false;
    if (esp_wifi_start() != ESP_OK)
        return false;


    #if 1
    esp_wifi_set_promiscuous(true);
    if( esp_wifi_set_channel(wl_channel, WIFI_SECOND_CHAN_NONE) == ESP_OK ) 
    {
        Serial.printf( "[MSG]WIFI::INIT::OK::setup channel=%d\r\n",wl_channel);
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::ERR::setup channel\r\n");
        esp_wifi_set_promiscuous(false);
        return false;
    }
    esp_wifi_set_promiscuous(false);
    #endif


    if (esp_wifi_set_promiscuous(true) != ESP_OK)
        return false;
    if (esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_5M_L) != ESP_OK)
        return false; // 1Mbps, 5Mbps, 11Mbps available
    if (esp_now_register_recv_cb(recv_CB) != ESP_OK)
        return false;
    if (esp_wifi_set_promiscuous_rx_cb(rx_CB) != ESP_OK)
        return false;
    if (esp_now_register_send_cb(send_CB) != ESP_OK)
        return false;
    // if (!pairing(broadcast_addr, 1))
    //     return false;

    if( esp_wifi_get_channel( &get_channel, &second_ch ) == ESP_OK )
    {
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::ERR::getup channel\r\n");
        return false;
    }

    typeID = 0;
    typeID = getType();
    typeID <<= 8;
    typeID += getID();
    
    Serial.printf("[MSG]WIFI::INIT:: TYPE/ID : %d(0x%04x)\n", typeID, typeID);

    delay(500);

    Serial.println("[MSG]WIFI::INIT::COMPELETE");

    memset(rx_ioBuff, 0, sizeof(rx_ioBuff));

    return true;
}

bool wireless::set_channel(uint8_t ch)
{
    bool rtn = false;
    esp_wifi_set_promiscuous(true);

    if (ch > 0 || ch <= MAX_CHANNEL)
    {
        if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK)
        {
            rtn = true;
        }else
        {
            Serial.println("[MSG]WIFI::SET::ERR::channel set");
        }
    }else
    {
        Serial.println("[MSG]WIFI::SET::ERR::channel over range");
    }
    esp_wifi_set_promiscuous(false);

    return rtn;
}


#endif
