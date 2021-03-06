//#define DEBUG_RXANY 1 を追加しないとhondaの拡張CANは取得できないので注意
#define DEBUG_RXANY 0
#define BCAN 0
#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>

SoftwareSerial receiver(5, 7); // RX, TX

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

// 車速PIN
const int WT_PIN = 6;

MCP_CAN CAN0(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(57600);

    // FK7のB-CANは125KBPS F-CANは500KBPS
    // 中華のCANボードは8MHz
#if(BCAN==1)
    while (CAN_OK != CAN0.begin(CAN_125KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
#else
    while (CAN_OK != CAN0.begin(CAN_500KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
#endif

    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(1000);
    }
    Serial.println("CAN BUS Shield init ok!");
    pinMode( WT_PIN, OUTPUT );
    receiver.begin(115200);
    
}

// UBX-ESF-MEAS 速度データのみ
unsigned char speedData[] = {
    0xB5, 0x62, 0x10, 0x02 // id
  , 0x0C, 0x00  // len
  , 0x00, 0x00, 0x00, 0x00 // time
//  , 0x10, 0x08  // flag 1件 FirstByte
//  , 0x01, 0x08  // flag 1件 Ext0のTimemark
  , 0x02, 0x08  // flag 1件 Ext1のTimemark
  , 0x00, 0x00  // id
  , 0x00, 0x00, 0x00, 0x0B // data speed
  , 0x04, 0x48}; // checksum

// UBX-ESF-MEAS リアTickのみ
unsigned char rearTickData[] = {
    0xB5, 0x62, 0x10, 0x02 // id
  , 0x10, 0x00  // len
  , 0x00, 0x00, 0x00, 0x00 // time
//  , 0x10, 0x10  // flag 2件 FirstByte
//  , 0x01, 0x10  // flag 2件 Ext0のTimemark
  , 0x02, 0x10  // flag 2件 Ext1のTimemark
  , 0x00, 0x00  // id
  , 0x00, 0x00, 0x00, 0x08 // data RL
  , 0x00, 0x00, 0x00, 0x09 // data RR
  , 0x04, 0x48}; // checksum

#if(BCAN==1)
float unit = 2.777;  // 0.01km/hから1mm/sに変換(10000 / 60 / 60)
#else
float unit = 637.0 * 4.0 / 60.0 / 60.0 * 10.0; // 60km/hで637RPMなので、1km/hあたりのRPSを算出して、パルスに変換（1回転4パルス）
#endif
unsigned char fwd = 0x00;
int cnt = 0;

void loop()
{
    unsigned char len = 0;
    unsigned char buf[255];

    if(CAN_MSGAVAIL == CAN0.checkReceive())            // check if data coming
    {
        CAN0.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

#if(BCAN==1)
        unsigned long canId = CAN0.getCanId();
        
        // 車速
        if(canId == 0x12F85950) {
            unsigned long ms = millis();

            // タイミングのパルス
            digitalWrite( WT_PIN, HIGH );
            delay(5); // 5ミリ秒で十分か不明
            digitalWrite( WT_PIN, LOW );
            
            //Serial.print("Get data from ID: ");
            //Serial.println(canId, HEX);
            int ck_a = 0;
            int ck_b = 0;

            int len = 8 + (4 * 1);

            // 0.01km/hから1mm/sに変換してセット
            unsigned long kmph = buf[3] << 8;
            kmph = kmph + buf[4];

            float work =  kmph * unit;
            unsigned long sp = work;
            
            speedData[14] = byte(sp);
            speedData[15] = byte(sp >> 8);
            speedData[16] = byte(sp >> 16);

            // ローカル時間を設定
            speedData[6] = byte(ms);
            speedData[7] = byte(ms >> 8);
            speedData[8] = byte(ms >> 16);
            speedData[9] = byte(ms >> 24);

            // CLASS ~ ペイロードまでを対象にチェックサムを算出
            for(int i = 2; i < 6 + len; i++){
              int hex2 = speedData[i];
              hex2 &= 0xFF;
              ck_a += hex2;
              ck_a &= 0xFF;
              ck_b += ck_a;
              ck_b &= 0xFF;
            }
            speedData[6+len] = ck_a;
            speedData[6+len+1] = ck_b;

            // レシーバに送信
            receiver.write(speedData, sizeof(speedData));
        }
#else
        unsigned long canId = CAN0.getCanId();

        //Serial.print("Get data from ID: ");
        //Serial.println(canId, HEX);
        
        // 車輪別車速
        if(canId == 0x01D0) {
            cnt++;
            // 50hzは高負荷すぎる可能性があるので10hzに調整する小細工
            if(cnt != 5)
              return;
            cnt = 0;
            unsigned long ms = millis();

            // タイミングのパルス
            digitalWrite( WT_PIN, HIGH );
            delay(5); // 5ミリ秒で十分か不明
            digitalWrite( WT_PIN, LOW );

            // INFO:u-center側で以下を設定する前提
            //      相対パルス
            //      サンプリング周期:10Hz
            //      スケール:0.0001
            
            //Serial.print("Get data from ID: ");
            //Serial.println(canId, HEX);
            int ck_a = 0;
            int ck_b = 0;

            int len = 8 + (4 * 2);

            // RL
            // 0.01km/hからパルスに変換してセット
            unsigned long kmph = (unsigned long)(buf[5] & 0xF4) >> 3;
            kmph = kmph + (buf[4] << 5);
            kmph = kmph + ((buf[3] & 0x03) << 13);

            // パルスへの変換
            // （unitは1km/s当たりのパルス数のため、10Hzであることも加味して、スケールで調整)
            kmph = kmph * unit;

            rearTickData[14] = byte(kmph);
            rearTickData[15] = byte(kmph >> 8);
            rearTickData[16] = byte(kmph >> 16) | fwd; // 前進後退はシフトで判断

            // RR
            // 0.01km/hからパルスに変換してセット
            unsigned long kmph2 = (unsigned long)(buf[7] & 0xF0) >> 4;
            kmph2 = kmph2 + (buf[6] << 4);
            kmph2 = kmph2 + ((buf[5] & 0x07) << 12);

            // パルスへの変換
            kmph2 = kmph2 * unit;

            rearTickData[18] = byte(kmph2);
            rearTickData[19] = byte(kmph2 >> 8);
            rearTickData[20] = byte(kmph2 >> 16) | fwd;

            // ローカル時間を設定
            rearTickData[6] = byte(ms);
            rearTickData[7] = byte(ms >> 8);
            rearTickData[8] = byte(ms >> 16);
            rearTickData[9] = byte(ms >> 24);

            // CLASS ~ ペイロードまでを対象にチェックサムを算出
            for(int i = 2; i < 6 + len; i++){
              int hex2 = rearTickData[i];
              hex2 &= 0xFF;
              ck_a += hex2;
              ck_a &= 0xFF;
              ck_b += ck_a;
              ck_b &= 0xFF;
            }
            rearTickData[6+len] = ck_a;
            rearTickData[6+len+1] = ck_b;

            // レシーバに送信
            receiver.write(rearTickData, sizeof(rearTickData));
        }
        // シフト
        else if(canId == 0x0191) {
            //Serial.print("Get data from ID: ");
            //Serial.print(canId, HEX);
            //Serial.print(" shift:");
            //Serial.println(buf[0], HEX);
            
            // シフト情報を退避
            fwd = (buf[0] & 0x02) << 6;
        }
#endif

    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
