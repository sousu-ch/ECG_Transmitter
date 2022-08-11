#include <Arduino.h>
#include <Wire.h>


#define SIG_IN 26    // 信号入力ピン
#define ADC_COMPE 14 // ADCエラッタパッチ選択（LOWなら補正無し）

//液晶関連
const uint8_t ADDRES_OLED = 0x3C;
const int SDA_OLED = 5;
const int SCL_OLED = 4;
const uint32_t Frequensy_OLED = 400000; // Max=400kHz

uint8_t oled_ram_buf[128][8] = {0};
//液晶関連ここまで


#define BOARD_LED 25 // 基板内蔵LED
#define CHECK_PIN 16 // 動作タイミングチェックピン
#define SIG_IN 26    // 信号入力ピン
#define OF_LED 15    // 過大入力表示LEDピン
#define UP_SW 9      // レンジUpボタン
#define DN_SW 10     // レンジDownボタン
#define ADC_COMPE 14 // ADCエラッタパッチ選択（LOWなら補正無し）
#define PX2 0        // 画面(R)原点
#define PY1 16       // 波形画面の下端
#define PY2 52       // スペクトル画面の下端(-50db)
#define NNN 256      // FFTのサンプル数
#define LCD_BUF_SIZE 128 // LCDの描画バッファ
#define DATA_BUF_SIZE  128// ADCで取得した生波形のバッファサイズ


//グローバル変数
uint16_t range = 8; // レンジ番号(3:100Hz, 4:200Hz, 5:50Hz, 6:1k, 7:2k, 8:5k, 9:10k, 10:20k, 11:50k)

uint8_t buf_no = 0;
uint8_t latest_buf_no = 0;
uint8_t last_data = 0;
uint8_t wave[129];


void setup()
{
    pinMode(CHECK_PIN, OUTPUT);       // 実行時間測定用
    pinMode(BOARD_LED, OUTPUT);       // pico内蔵LED
    pinMode(OF_LED, OUTPUT);          // オーバーフロー表示LED
    pinMode(UP_SW, INPUT_PULLUP);     // UPボタン
    pinMode(DN_SW, INPUT_PULLUP);     // Downボタン
    pinMode(ADC_COMPE, INPUT_PULLUP); // ADC補正指定ピン（HIGHで補正有り）



    // ADC初期化古いやつ。FIFOにするときに消すコード
    pinMode(ADC_COMPE, INPUT_PULLUP); // ADC補正指定ピン（HIGHで補正有り）
    analogReadResolution(12);         // ADCのフルスケールを8ビットに設定

    // ADCの初期化ここに必要★★★★

    Serial1.begin(115200);

    SSD1306_Init(); // OLED ssd1306 初期化
    delay(1000);
    Clear_Display_All();

}

void loop()
{

    int i;
    static uint8_t last_segment_no = 0;
    static uint8_t last_data = 0;
    uint8_t new_data = 0;
    static uint16_t count360 = 0;
    static uint8_t countupno = 1;

    int adcdata = analogRead(SIG_IN);
    new_data = round(adcdata) - 1768;
    // new_data =255;
    Serial1.println(new_data);
    if (new_data > 63)
        new_data = 10;

    //★★★★これを外すと描画でエラーになるぞ！謎！★★★★★
    if (new_data == 0)
        new_data = 1;

    koushin(last_data, new_data, last_segment_no);
    last_data = new_data;
    if (last_segment_no < 127)
    {
        last_segment_no++;
    }
    else
    {
        last_segment_no = 0;
    }
}

//******************************************
void SSD1306_Init()
{
    i2c_init(i2c0, 100 * 3 * 80); // EARLEPHILHOWER_PICOのライブラリは、 Wire.setClockでclockが変えられない(1.9.4)ため、sdkの方法でclockを変える必要がある

    Wire.begin();
    // Wire.begin(SDA_OLED, SCL_OLED); もとのソースにあったコード
    // Wire.setSDA(4);おっちゃんのデフォルトは4,5なので、これ書く必要なし。標準のRP2040の場合i2c1なのでここじゃないので注意
    // Wire.setSCL(5);
    Wire.setClock(Frequensy_OLED);
    delay(100);

    Wire.beginTransmission(ADDRES_OLED);
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command) follow Max=31byte
    Wire.write(0xAE);       // display off
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command) follow Max=31byte
    Wire.write(0xA8);       // Set Multiplex Ratio  0xA8, 0x3F
    Wire.write(0b00111111); // 64MUX
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0xD3);       // Set Display Offset 0xD3, 0x00
    Wire.write(0x00);
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0x40);       // Set Display Start Line 0x40
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0xA1);       // Set Segment re-map 0xA0/0xA1
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0xC0);       // Set COM Output Scan Direction 0xC0,/0xC8
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0xDA);       // Set COM Pins hardware configuration 0xDA, 0x02
    Wire.write(0b00010010);
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x81);       // Set Contrast Control 0x81, default=0x7F
    Wire.write(255);        // 0-255
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0xA4);       // Disable Entire Display On
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0xA6);       // Set Normal Display 0xA6, Inverse display 0xA7
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0xD5);       // Set Display Clock Divide Ratio/Oscillator Frequency 0xD5, 0x80
    Wire.write(0b10000000);
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x20);       // Set Memory Addressing Mode
    Wire.write(0x10);       // Page addressing mode
    Wire.endTransmission();
    Wire.beginTransmission(ADDRES_OLED);
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x22);       // Set Page Address
    Wire.write(0);          // Start page set
    Wire.write(7);          // End page set
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x21);       // set Column Address
    Wire.write(0);          // Column Start Address
    Wire.write(127);        // Column Stop Address
    Wire.write(0b00000000); // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x8D);       // Set Enable charge pump regulator 0x8D, 0x14
    Wire.write(0x14);
    Wire.write(0b10000000); // control byte, Co bit = 1 (1byte only), D/C# = 0 (command)
    Wire.write(0xAF);       // Display On 0xAF
    Wire.endTransmission();
}
//**************************************************
void Clear_Display_All()
{
    uint8_t i, j, k;

    for (i = 0; i < 8; i++)
    { // Page(0-7)
        Column_Page_Set(0, 127, i);

        for (j = 0; j < 16; j++)
        { // column = 8byte x 16
            Wire.beginTransmission(ADDRES_OLED);
            Wire.write(0b01000000); // control byte, Co bit = 0 (continue), D/C# = 1 (data)
            for (k = 0; k < 8; k++)
            { // continue to 31byte
                Wire.write(0x00);
            }
            Wire.endTransmission();
        }
        // delay(1);
    }

    for (i = 0; i < 128; i++)
    {
        for (j = 0; j < 8; j++)
        {
            oled_ram_buf[i][j] = 0;
        }
    }
}
//**************************************************
//******************************************
void Column_Page_Set(uint8_t x0, uint8_t x1, uint8_t page)
{
    Wire.beginTransmission(ADDRES_OLED);
    Wire.write(0b10000000);  // control byte, Co bit = 1 (1byte only), D/C# = 0 (command) Max=31byte
    Wire.write(0xB0 | page); // set page start address(B0～B7)
    Wire.write(0b00000000);  // control byte, Co bit = 0 (continue), D/C# = 0 (command)
    Wire.write(0x21);        // set Column Address
    Wire.write(x0);          // Column Start Address(0-127)
    Wire.write(x1);          // Column Stop Address(0-127)
    Wire.endTransmission();
}

void koushin(uint8_t y1, uint8_t y2, uint8_t last_segment_no)
{
    // uint8_t oled_ram_buf[2][8] = 0;
    uint8_t page_no_temp = 0; // 0 to 7の８ページある
    uint8_t latest_segment_no = 0;
    uint8_t page, ytemp;

    //描画するsegment_noを決める
    //ただし、最後のセグメントは描画しないので
    if (last_segment_no < 127)
    {
        latest_segment_no = last_segment_no + 1;
    }
    else
    {
        latest_segment_no = 0;
    }

    //
    if (y1 > 63)
        y1 = 63;
    if (y2 > 63)
        y2 = 63;

    for (page = 0; page < 8; page++)
    {
        oled_ram_buf[latest_segment_no][page] = 0x00; //一週前のデータは消しておく
    }

    //描画する点と点の間に線を引く
    if (y1 > y2)
    {
        //右下がりデータならー方向にカウントする
        for (ytemp = y1; ytemp >= y2; ytemp--)
        {
            page = floor(ytemp / 8);
            uint8_t sakaime = y1 - ((y1 - y2) >> 1);
            if (ytemp >= sakaime)
            {
                oled_ram_buf[last_segment_no][page] = oled_ram_buf[last_segment_no][page] | (0x01 << (ytemp % 8));
            }
            else
            {
                oled_ram_buf[latest_segment_no][page] = oled_ram_buf[latest_segment_no][page] | (0x01 << (ytemp % 8));
            }
        }
    }
    else
    {
        //右上がりデータなら＋方向にカウントする
        for (ytemp = y1; ytemp <= y2; ytemp++)
        {
            page = floor(ytemp / 8);
            uint8_t sakaime = y1 + ((y2 - y1) >> 1);
            if (ytemp < sakaime)
            {
                oled_ram_buf[last_segment_no][page] = oled_ram_buf[last_segment_no][page] | (0x01 << (ytemp % 8));
            }
            else
            {
                oled_ram_buf[latest_segment_no][page] = oled_ram_buf[latest_segment_no][page] | (0x01 << (ytemp % 8));
            }
        }
    }

    for (page = 0; page < 8; page++)
    {
        Column_Page_Set(last_segment_no, last_segment_no + 1, page); //セグメント128にも書いちゃってるけど、そこが問題あったら対策必要です。
        Wire.beginTransmission(ADDRES_OLED);
        // 2セグメントずつ送信することで高速化する。
        Wire.write(0b01000000); // control byte, Co bit = 0 (continue), D/C# = 1 (data)
        Wire.write(oled_ram_buf[last_segment_no][page]);
        Wire.write(oled_ram_buf[latest_segment_no][page]);
        Wire.endTransmission();
    }

    //最後に、次の描画segmentを更新しておく
    // last_segment_no = latest_segment_no;
}
