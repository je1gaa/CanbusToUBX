//#define DEBUG_RXANY 1 を追加しないとhondaの拡張CANは取得できないので注意
#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>

SoftwareSerial receiver(5, 7); // RX, TX

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN0(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    // FK7は125KBPS 車種によって異なる
    // 中華のCANボードは8MHz
    while (CAN_OK != CAN0.begin(CAN_125KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(1000);
    }
    Serial.println("CAN BUS Shield init ok!");
    receiver.begin(115200);
}

// UBX-ESF-MEAS 速度データのみ
unsigned char speedData[] = {
    0xB5, 0x62, 0x10, 0x02 // id
  , 0x0C, 0x00  // len
  , 0x00, 0x00, 0x00, 0x00 // time
  , 0x10, 0x08  // flag
  , 0x00, 0x00  // id
  , 0x00, 0x00, 0x00, 0x0B // data speed
  , 0x04, 0x48}; // checksum

void loop()
{
    unsigned char len = 0;
    unsigned char buf[255];
    float unit = 2.777;  // 0.01km/hから1mm/sに変換(10000 / 60 / 60)

    if(CAN_MSGAVAIL == CAN0.checkReceive())            // check if data coming
    {
        CAN0.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned long canId = CAN0.getCanId();
        
        // 車速信号
        if(canId == 0x12F85950) {
            unsigned long ms = millis();
            
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
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
