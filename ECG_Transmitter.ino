#include <Arduino.h>
#include <Wire.h>
#include "hardware/adc.h"

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
    
    //pinMode(ADC_COMPE, INPUT_PULLUP); // ADC補正指定ピン（HIGHで補正有り）
    analogReadResolution(8); // ADCのフルスケールを8ビットに設定
    Serial.begin(115200);
    //ADC初期化
    adc_gpio_init(SIG_IN);

    adc_init();
    //adc_select_input(SIG_IN);
    adc_fifo_setup(
        true,  // Write each comp1leted conversion to the sample FIFO
        false,  // Enable DMA data request (DREQ)
        0,     // DREQ (and IRQ) asserted when at least 1 sample present
        false, // We won't see the ERR bit because of 8 bit reads; disable.
        true   // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    //adc_set_clkdiv(23436.5); // 2048サンプル毎秒
    //adc_set_clkdiv(46874); // 1024サンプル毎秒
    adc_set_clkdiv(29999); // 1600サンプル毎秒 商用電源の50Hzの整数倍を選定している
    adc_run(true);

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
    int new_data_tmp = 0;
    static uint16_t count360 = 0;
    static uint8_t countupno = 1;
    static float adcbuf[64];
    static uint8_t adcbufcount = 0;
    static uint8_t adc_16_cycle_count = 0;
    float fadcdata;

    // ADC取り込み
    if(adc_fifo_get_level()>0)
    {
        adcbuf[adcbufcount] = (float)adc_fifo_get();
        adcbufcount++;
        if (adcbufcount > 63)
        {
            adcbufcount = 0;
        }
        adc_16_cycle_count++;
    }
    // 1600Hzサンプリング32カウントだから
    // 1600/32=50Hz周期で描画する。
    // 1フレーム128lineなので、(1/50)*128=2.5[sec]で１ページ描画する
    if (adc_16_cycle_count > 15)
    {
        adc_16_cycle_count = 0;

        //Serial.print(adcbuf[adcbufcount]);//生データ出力
        //Serial.print(",");

        // FIR処理
        fadcdata = (uint16_t)fir_LPF100(&adcbuf[0], adcbufcount);
        Serial.println(fadcdata);//シリアルモニター用出力

        new_data_tmp = round(fadcdata) / 2;
        if (new_data_tmp > (63+49))
        {
            new_data = 63;
        }
        else if (new_data_tmp <= 49)//OLEDの中心は64/2=32だが、下側が飽和しているので下にオフセットする
        {
            new_data = 1;
        }
        else
        {
            new_data = new_data_tmp - 49;
        }

//別で作ったから仮に消した
#if 0
        if (new_data > 63)
            new_data = 63;

        //★★★★これを外すと描画でエラーになるぞ！謎！★★★★★
        if (new_data == 0)
            new_data = 1;
#endif
        // Serial.println(new_data);

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

    //int adcdata = analogRead(SIG_IN);

}

//******************************************
float fir_LPF100(float *data, uint8_t count)
{
    float result=0;
    const float hm[64] = {
        0.004108952,
        0.001844843,
        -0.000663094,
        -0.003318615,
        -0.006012725,
        -0.008627203,
        -0.011038618,
        -0.013122699,
        -0.014758884,
        -0.015834915,
        -0.016251266,
        -0.015925276,
        -0.014794797,
        -0.012821218,
        -0.009991748,
        -0.006320846,
        -0.001850727,
        0.003349083,
        0.009183158,
        0.015532513,
        0.022257799,
        0.029203276,
        0.036201406,
        0.043077939,
        0.049657298,
        0.055768095,
        0.06124858,
        0.065951822,
        0.069750464,
        0.072540854,
        0.074246433,
        0.074820235,
        0.074246433,
        0.072540854,
        0.069750464,
        0.065951822,
        0.06124858,
        0.055768095,
        0.049657298,
        0.043077939,
        0.036201406,
        0.029203276,
        0.022257799,
        0.015532513,
        0.009183158,
        0.003349083,
        -0.001850727,
        -0.006320846,
        -0.009991748,
        -0.012821218,
        -0.014794797,
        -0.015925276,
        -0.016251266,
        -0.015834915,
        -0.014758884,
        -0.013122699,
        -0.011038618,
        -0.008627203,
        -0.006012725,
        -0.003318615,
        -0.000663094,
        0.001844843,
        0.004108952,
        0
    };

    const float hm100Hz[64] = {
        -0.004133652,
        -0.007892593,
        -0.010667762,
        -0.011959085,
        -0.011457967,
        -0.009106838,
        -0.005125729,
        5.12808E-18,
        0.005571444,
        0.010762627,
        0.014731671,
        0.016742719,
        0.016282374,
        0.013154322,
        0.007537836,
        -5.12808E-18,
        -0.008542881,
        -0.0169127,
        -0.023797315,
        -0.027904531,
        -0.0281241,
        -0.02367778,
        -0.014238136,
        5.12808E-18,
        0.018306174,
        0.039462966,
        0.06187302,
        0.083713593,
        0.1031217,
        0.118388899,
        0.12814322,
        0.131497004,
        0.12814322,
        0.118388899,
        0.1031217,
        0.083713593,
        0.06187302,
        0.039462966,
        0.018306174,
        5.12808E-18,
        -0.014238136,
        -0.02367778,
        -0.0281241,
        -0.027904531,
        -0.023797315,
        -0.0169127,
        -0.008542881,
        -5.12808E-18,
        0.007537836,
        0.013154322,
        0.016282374,
        0.016742719,
        0.014731671,
        0.010762627,
        0.005571444,
        5.12808E-18,
        -0.005125729,
        -0.009106838,
        -0.011457967,
        -0.011959085,
        -0.010667762,
        -0.007892593,
        -0.004133652,
        0};

    const float hm60Hz[64] = {
        0.009162457,
        0.007851843,
        0.006001986,
        0.00367648,
        0.000968027,
        -0.002004319,
        -0.005099264,
        -0.008158594,
        -0.011013505,
        -0.013491681,
        -0.015424819,
        -0.016656274,
        -0.017048484,
        -0.016489832,
        -0.014900624,
        -0.01223789,
        -0.008498774,
        -0.003722308,
        0.002010517,
        0.008578453,
        0.015823417,
        0.023555529,
        0.031559574,
        0.039602645,
        0.047442653,
        0.054837359,
        0.061553563,
        0.067376044,
        0.072115898,
        0.075617902,
        0.077766599,
        0.078490843,
        0.077766599,
        0.075617902,
        0.072115898,
        0.067376044,
        0.061553563,
        0.054837359,
        0.047442653,
        0.039602645,
        0.031559574,
        0.023555529,
        0.015823417,
        0.008578453,
        0.002010517,
        -0.003722308,
        -0.008498774,
        -0.01223789,
        -0.014900624,
        -0.016489832,
        -0.017048484,
        -0.016656274,
        -0.015424819,
        -0.013491681,
        -0.011013505,
        -0.008158594,
        -0.005099264,
        -0.002004319,
        0.000968027,
        0.00367648,
        0.006001986,
        0.007851843,
        0.009162457,
        0};

    const float hm30Hz[64] = {
        -0.004361021,
        -0.00352936,
        -0.002589728,
        -0.001545795,
        -0.000402311,
        0.000834925,
        0.002159103,
        0.003562449,
        0.005036292,
        0.006571129,
        0.008156706,
        0.00978211,
        0.011435862,
        0.01310602,
        0.01478029,
        0.016446137,
        0.018090903,
        0.019701925,
        0.021266658,
        0.02277279,
        0.024208362,
        0.025561886,
        0.026822453,
        0.027979839,
        0.029024607,
        0.0299482,
        0.030743019,
        0.031402507,
        0.031921207,
        0.032294816,
        0.032520231,
        0.032595579,
        0.032520231,
        0.032294816,
        0.031921207,
        0.031402507,
        0.030743019,
        0.0299482,
        0.029024607,
        0.027979839,
        0.026822453,
        0.025561886,
        0.024208362,
        0.02277279,
        0.021266658,
        0.019701925,
        0.018090903,
        0.016446137,
        0.01478029,
        0.01310602,
        0.011435862,
        0.00978211,
        0.008156706,
        0.006571129,
        0.005036292,
        0.003562449,
        0.002159103,
        0.000834925,
        -0.000402311,
        -0.001545795,
        -0.002589728,
        -0.00352936,
        -0.004361021,
        0};

    //FIRもどき。ダウンサンプリングもしちゃう。
    for (uint8_t k = 0; k < 64; k++)
    {
        result += hm30Hz[k] * data[count];
        count++;
        if(count>63)
        {
            count = 0;
        }
    }
    return result;
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
