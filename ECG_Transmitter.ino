#include <Arduino.h>
#include <Wire.h>
#include "hardware/adc.h"

#define SIG_IN 26               // 信号入力ピン
#define SEL_SW 28               // レンジセレクトスイッチ
#define HEART_BEAT_AVERAGE_NO 4 // 心拍計算のためにに何周期移動平均するか
#define OLED_OFFSET 110          // OLEDの中心に対して波形中心をどれだけオフセットするか。
//#define DEBUG_PRINT
#define ADC_SAMPLING_RATE 1600 // 50Hz用。1600Hzサンプリングの64T(=FIRタップ長)は50Hzの2周期に相当する
//#define ADC_SAMPLING_RATE 1920 // 60Hz用

//グローバル変数

// OLED関連
const uint8_t ADDRES_OLED = 0x3C;
const int SDA_OLED = 5;
const int SCL_OLED = 4;
const uint32_t Frequensy_OLED = 3000000; // Max=400kHz。実は、さらに数倍の実力があるようだ。オーバークロックで3Mに設定した。
// OLED関連ここまで

uint8_t oled_ram_buf[128][8] = {0}; //OLEDの表示データ用バッファ。よこ128segment,たて8page
uint8_t dig_oled_buf[18];           //現在の心拍数を記憶しておくバッファ。ECGで表示が上書きされるのの防止のため。
uint8_t heart_beat_draw_EN =0;      //心拍を検出したことを示すフラグ。このフラグがたったら画面上部に心拍検出マークを付ける
static uint8_t draw_cycle = 1;      // OLED描画周期。draw_buf_cycle_countに対して、draw_cycle周期でOLEDにデータを送信する。

void setup()
{
    pinMode(SEL_SW, INPUT_PULLUP);    // レンジセレクトスイッチ
    analogReadResolution(8); // ADCのフルスケールを8ビットに設定
    Serial.begin(115200);
    //ADC初期化
    adc_gpio_init(SIG_IN);

    adc_init();
    //adc_select_input(SIG_IN);

    // ADC取り込み設定。計算および描画処理直後はFIFOに蓄積され、最大8byteまでたまることがある。
    // FIFOサイズは最大4だが、ADC分解能を8bit設定にすることで8byteに拡張している。
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

    // 商用電源のの整数倍を選定している。29999は1600Hzサンプリングで、64T(=FIRタップ長)は50Hzの2周期に相当する
    adc_set_clkdiv(48000000 / ADC_SAMPLING_RATE - 1); 

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
    static float adcbuf[64];
    static float buf3[3];
    static uint8_t adcbufcount = 0;
    static uint8_t draw_buf_cycle_count = 0;
    static uint8_t sw_count = 0;
    float fadcdata;
    float ddadc;
    static uint8_t pt0 = 0;
    static uint8_t pt1 = 1;
    static uint8_t pt2 = 2;
    static uint16_t max_count = 0;
    static uint8_t ddadc_max_val = 0;
    static uint8_t ddadc_th = 0;
    static uint16_t heart_beat_bpm;
    static uint16_t heart_beat_T_sum = 0;
    static uint16_t heart_beat_T_buf[HEART_BEAT_AVERAGE_NO];
    static uint16_t heart_beat_T_buf_pt = 0;
    static uint16_t beseT = 16;
    static uint16_t ddadc_count = 0;

    static uint8_t sw_stat = 0;
    static uint8_t sw_HL = 0;
    static uint8_t draw_buf_cycle = 8;
    static uint8_t sw_timer_count = 0;
    static uint8_t view_stage = 0;

    static uint8_t draw_cycle_count = 0;
    static uint8_t adcbufcount_temp = 0;
    // ADC取り込み。計算および描画処理直後はFIFOに蓄積され、最大8byteまでたまることがある。
    // FIFOサイズは最大4だが、ADC分解能を8bit設定にすることで8byteに拡張している。
    if (adc_fifo_get_level() > 0)
    {
        adcbuf[adcbufcount] = (float)adc_fifo_get();
        adcbufcount_temp=adcbufcount;
        adcbufcount++;
        //FIRフィルタが64段なので、バッファも64個のリングバッファにしてある
        if (adcbufcount > 63)
        {
            adcbufcount = 0;
        }
        draw_buf_cycle_count++;
        sw_timer_count++;
        ddadc_count++;
        max_count++;
    }

    //スイッチ入力の監視
    //16カウントごと(1600Hzの場合T=10ms)にポートを読んで、LOWならON継続回数をカウントアップする。
    if (sw_timer_count > 15)
    {
        sw_timer_count = 0;
        sw_HL = digitalRead(SEL_SW);
        if (sw_HL == LOW)
        {
            if(sw_count<255) // uint8_t型なので、オーバーフロー対策
            {
                sw_count++;
            }
        }
        if (sw_HL == HIGH)
        {
            sw_count=0;
        }
        //周期切り替え。チャタリング防止で5(=50ms)にした。
        if (sw_count == 5)
        {
            draw_buf_cycle = heart_beat_T_sum / HEART_BEAT_AVERAGE_NO / 128 - 1;
            last_segment_no=0;
            //OLEDの表示時間およびその他処理遅延で最大8サイクル弱かかるので、最短は８。
            //39bpm相当：8、220bpm相当：46
            if (draw_buf_cycle < 7 && draw_buf_cycle > 45)
            {
                draw_buf_cycle = 15; //心拍が測定できていないとみなして、固定値にリセットする
            }
            draw_cycle = (draw_cycle + 1) % 4;

        }
    }

    // OLED 描画用バッファの更新周期。表示周期ではないことに注意。
    // ADC_SAMPLING_RATE Hzサンプリングのうち、draw_buf_cycleサイクルでデータ処理する
    if (draw_buf_cycle_count > draw_buf_cycle)
    {
        draw_buf_cycle_count = 0;

        // FIR処理
        fadcdata = (uint16_t)Fir_LPF(&adcbuf[0], adcbufcount);

#if defined(DEBUG_PRINT)
        for (i = 0; i < draw_buf_cycle; i++)
        {
            if (adcbufcount_temp>0)
            {
                adcbufcount_temp--;
            }
            else
            {
                adcbufcount_temp=63;
            }
        }

        for (i = 0; i < draw_buf_cycle+1; i++)
        {
            //if((uint16_t)adcbuf[adcbufcount_temp]==0)
            {
                Serial.print((uint16_t)adcbuf[adcbufcount_temp]); //シリアルモニター用出力。
            }
            adcbufcount_temp = (adcbufcount_temp + 1) % 64;

            if (i < draw_buf_cycle)
            {
                Serial.println(","); //シリアルモニター用出力
            }
        }
        Serial.print(","); //シリアルモニター用出力
#endif

        buf3[pt0] = fadcdata;
        ddadc=(-1)*((buf3[pt0] - buf3[pt2]) - (buf3[pt2] - buf3[pt1]));
        pt0 = (pt0 + 1) % 3;
        pt1 = (pt1 + 1) % 3;
        pt2 = (pt2 + 1) % 3;
        
        if (ddadc_max_val < (uint8_t)ddadc)
        {
            ddadc_max_val = (uint8_t)ddadc;
        }

        //1600Hzサンプリングの場合3200=2秒周期で閾値を更新する。
        //２秒あれば完全１心拍あるはず、という計算。
        if (max_count > ADC_SAMPLING_RATE*2)
        {
            max_count = 0;
            ddadc_th = ddadc_max_val >>1;
            ddadc_max_val = 0;
        }
        
        //心拍を２階微分した値をピーク記録し、その半分を閾値に心拍を検出する。
        //R波付近で何度も検出してしまうので、検出後400カウント(625ms)は検出しないようにする。(1600Hzの場合)
        //最高220bpmとすると、T=273ms=436カウントなので、最大0.91T検出しないことになる。
        if (ddadc > (float)ddadc_th && ddadc_count > (ADC_SAMPLING_RATE/4))
        {
            // HEART_BEAT_AVERAGE_NO周期の移動平均とする
            heart_beat_T_sum += ddadc_count;
            heart_beat_T_buf_pt = (heart_beat_T_buf_pt + 1) % HEART_BEAT_AVERAGE_NO;
            heart_beat_T_sum -= heart_beat_T_buf[heart_beat_T_buf_pt];
            heart_beat_T_buf[heart_beat_T_buf_pt] =ddadc_count;
            heart_beat_bpm = round(ADC_SAMPLING_RATE / (float)heart_beat_T_sum * 60 * HEART_BEAT_AVERAGE_NO); // 心拍数算出
            ddadc_count = 0;
            heart_beat_draw_EN=1;
            //心拍数表示
            Draw_Heart_beat(heart_beat_bpm, last_segment_no);
        }
        // Serial.print(max_count); //シリアルモニター用出力
        // Serial.print(",");       //シリアルモニター用出力
        // Serial.print(ddadc_max_val); //シリアルモニター用出力
        // Serial.print(",");           //シリアルモニター用出力
        // Serial.print(ddadc_th);      //シリアルモニター用出力
        // Serial.print(",");           //シリアルモニター用出力
        // Serial.print(ddadc_count);   //シリアルモニター用出力
        // Serial.print(",");           //シリアルモニター用出力
        // Serial.print(heart_beat_bpm); //シリアルモニター用出力
        // Serial.print(",");           //シリアルモニター用出力
        // Serial.print(ddadc);         //シリアルモニター用出力
        // Serial.print(",");   //シリアルモニター用出力

        Serial.println((uint16_t)fadcdata); //シリアルモニター用出力。draw_buf_cycle周期でフィルターしたデータ

        // draw_cycleはOLEDの描画列を進める周期である。
        draw_cycle_count++;
        if(draw_cycle_count>draw_cycle)
        {
            draw_cycle_count = 0;
        }

        //OLEDの縦軸が64ピクセルなので、スケールを合わせる。
        //心臓付近で測定した場合このぐらいがちょうどいいが、
        //両手の指先で検出するなどの場合、振幅がちいさくなるので、/2しないほうがいい。
        new_data_tmp = round(fadcdata);

        // OLEDの中心は64/2=32だが、下側が飽和しているので下にオフセットOLED_OFFSETしているする。
        // この値は２段目増幅の入力オフセット電圧に大きく左右されるため、
        // 使用したOPアンプにあわせて調整するのがよいだろう。
        if (new_data_tmp > (63 + (OLED_OFFSET)))
        {
            new_data = 63;
        }
        else if (new_data_tmp <= (OLED_OFFSET))
        {
            new_data = 1;
        }
        else
        {
            new_data = new_data_tmp - (OLED_OFFSET);
        }

        // Serial.println(new_data); //OEDに描画するデータ

        // draw_cycle>0の場合は前回描画列のバッファに追記塗りをする。
        // draw_cycle==0の場合だけ描画列を進めて、前回列と今回列間のバッファに追記塗し、両バッファデータをOEDに送る。
        if(draw_cycle_count>0)
        {
            Draw_ECG_A(last_data, new_data, last_segment_no);
        }
        else
        {
            Draw_ECG_B(last_data, new_data, last_segment_no);
            last_data = new_data;
            //OLEDの名列目まで描画するか。127が最大
            if (last_segment_no < 127)
            {
                last_segment_no++;
            }
            else
            {
                last_segment_no = 0;
            }
        }

    }


}

//******************************************
//波形をFIRフィルタにかける。
//係数は64段のカットオフ35Hzに設定にしてある
//******************************************
float Fir_LPF(float *data, uint8_t count)
{
    float result = 0;

#if 0
    //テスト用フィルタ-64段average
    const float avg64[64] = {
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625,
        0.015625};
    
    //テスト用フィルタ-カットオフ30Hz
    const float hm30Hz[64] =
        {-0.004361021,
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
        0.026822453
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

    //テスト用フィルタ-カットオフ30Hz
    const float hm40Hz[64] =
        {-0.010212238,
        -0.010684186,
        -0.01091653,
        -0.01088707,
        -0.010577422,
        -0.009973486,
        -0.009065833,
        -0.007850009,
        -0.006326764,
        -0.004502175,
        -0.002387679,
        1.96346E-18,
        0.002639013,
        0.005502659,
        0.008559739,
        0.011775013,
        0.015109721,
        0.018522189,
        0.021968491,
        0.025403162,
        0.028779944,
        0.032052559,
        0.035175487,
        0.038104743,
        0.040798627,
        0.043218441,
        0.045329163,
        0.047100053,
        0.04850519,
        0.049523927,
        0.050141249,
        0.050348041,
        0.050141249,
        0.049523927,
        0.04850519,
        0.047100053,
        0.045329163,
        0.043218441,
        0.040798627,
        0.038104743,
        0.035175487,
        0.032052559,
        0.028779944,
        0.025403162,
        0.021968491,
        0.018522189,
        0.015109721,
        0.011775013,
        0.008559739,
        0.005502659,
        0.002639013,
        1.96346E-18,
        -0.002387679,
        -0.004502175,
        -0.006326764,
        -0.007850009,
        -0.009065833,
        -0.009973486,
        -0.010577422,
        -0.01088707,
        -0.01091653,
        -0.010684186,
        -0.010212238,
        0};
#endif

    const float hm35Hz[64] =
        {-0.008581722,
        -0.008194836,
        -0.007621372,
        -0.006858064,
        -0.005904071,
        -0.004761053,
        -0.003433201,
        -0.001927242,
        -0.0002524,
        0.001579678,
        0.003555026,
        0.005657503,
        0.00786897,
        0.01016949,
        0.012537567,
        0.014950399,
        0.017384159,
        0.01981429,
        0.022215815,
        0.024563657,
        0.026832958,
        0.028999404,
        0.03103954,
        0.032931084,
        0.034653213,
        0.036186845,
        0.037514892,
        0.038622487,
        0.039497184,
        0.040129126,
        0.040511173,
        0.040639004,
        0.040511173,
        0.040129126,
        0.039497184,
        0.038622487,
        0.037514892,
        0.036186845,
        0.034653213,
        0.032931084,
        0.03103954,
        0.028999404,
        0.026832958,
        0.024563657,
        0.022215815,
        0.01981429,
        0.017384159,
        0.014950399,
        0.012537567,
        0.01016949,
        0.00786897,
        0.005657503,
        0.003555026,
        0.001579678,
        -0.0002524,
        -0.001927242,
        -0.003433201,
        -0.004761053,
        -0.005904071,
        -0.006858064,
        -0.007621372,
        -0.008194836,
        -0.008581722,
        0};

    // FIR convolution
    for (uint8_t k = 0; k < 64; k++)
    {
        result += hm35Hz[k] * data[count];
        count++;
        if (count > 63)
        {
            count = 0;
        }
    }
    return result;
}
//******************************************
void SSD1306_Init()
{
    // EARLEPHILHOWER_PICOのライブラリは、 Wire.setClockでclockが変えられなかったので、
    // sdkの方法でclockを変えていたが、今はWire.setClockでいいようだ。
    // i2c_init(i2c0, 100 * 3 * 80); //こんな書き方をする。

    Wire.begin();
    // Wire.begin(SDA_OLED, SCL_OLED); //ポートを変更する場合の書き方メモ
    // Wire.setSDA(4);//ポートを変更する場合の書き方メモ。
    // Wire.setSCL(5);//EARLEPHILHOWER_PICOのデフォルトは4,5なので、基本的には書く必要なし。標準のRP2040の場合i2c1なのでここじゃないので注意
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


//**************************************************
// 5byte数字フォント。
// 隙間がないので、自分で隙間を挿入する必要がある
//**************************************************
const uint8_t font_num_7[10][5] =
    {
        {0x7C, 0x8A, 0x92, 0xA2, 0x7C}, // 0
        {0x00, 0x42, 0xFE, 0x02, 0x00}, // 1
        {0x46, 0x8A, 0x92, 0x92, 0x62}, // 2
        {0x44, 0x82, 0x92, 0x92, 0x6C}, // 3
        {0x18, 0x28, 0x48, 0x88, 0xFE}, // 4
        {0xE4, 0xA2, 0xA2, 0xA2, 0x9C}, // 5
        {0x7C, 0x92, 0x92, 0x92, 0x4C}, // 6
        {0x80, 0x8E, 0x90, 0xA0, 0xC0}, // 7
        {0x6C, 0x92, 0x92, 0x92, 0x6C}, // 8
        {0x64, 0x92, 0x92, 0x92, 0x6C}, // 9
};



//**************************************************
// 心拍数をOLEDの右上に描画する
// ECG描画で消去されないように、グローバル変数に現在の心拍数を出力しておく。
// 心拍数はDraw_ECG_BでもECG描画対象列に相当する部分だけ描画されることになる
//**************************************************
void Draw_Heart_beat(uint16_t heart_beat_bpm, uint8_t last_segment_no)
{
    //心拍を３桁に分解
    uint8_t dig3[3];

    dig3[0] = heart_beat_bpm / 100;
    dig3[1] = (heart_beat_bpm - dig3[0]*100) / 10;
    dig3[2] = heart_beat_bpm % 10;

    uint8_t j=0;
    for(int digno=0;digno<3;digno++)
    {

        for (int i = 0; i < 5; i++)
        {
            dig_oled_buf[j] = font_num_7[dig3[digno]][i];
            j++;
        }
        dig_oled_buf[j] = 0;//最後の１回は非表示領域だが、コードを単純化するため0を書き込んでおく
        j++;
    }
    Column_Page_Set(111, 111 + 16, 7); //右上端に表示
    Wire.beginTransmission(ADDRES_OLED);
    // 17セグメント連続で書き込むことで高速化する
    Wire.write(0b01000000); // control byte, Co bit = 0 (continue), D/C# = 1 (data)
    for(int i=0;i<17;i++)
    {
        Wire.write(dig_oled_buf[i]);
    }
    Wire.endTransmission();
}

//**************************************************
// 描画タイミングより前だった場合に使う。
// エイリアシングを防止の対策。
// バッファに重ね塗りすることで描画けが最小化される。
//
// 実際にはADC読み出し回数に対してdraw_buf_cycle回に１回しか重ね塗りしていないので、
// それでもぬけがあるが、気にならないレベルなのでこのままにした。
// これ以上回数を増やすとFFTの計算時間も無視できなくなると思う。
//**************************************************
void Draw_ECG_A(uint8_t y1, uint8_t y2, uint8_t last_segment_no)
{
    uint8_t page_no_temp = 0; // 0 to 7の８ページある
    uint8_t page, ytemp;


    //描画する点と点の間に線を引く
    if (y1 > y2)
    {
        //右下がりデータならー方向にカウントする
        for (ytemp = y1; ytemp >= y2; ytemp--)
        {
            page = floor(ytemp / 8);
            oled_ram_buf[last_segment_no][page] = oled_ram_buf[last_segment_no][page] | (0x01 << (ytemp % 8));
        }
    }
    else
    {
        //右上がりデータなら＋方向にカウントする
        for (ytemp = y1; ytemp <= y2; ytemp++)
        {
            page = floor(ytemp / 8);
            oled_ram_buf[last_segment_no][page] = oled_ram_buf[last_segment_no][page] | (0x01 << (ytemp % 8));
        }
    }

}

//**************************************************
// 描画タイミングで使うほう。
// 前回値と今回値間に上書きでバッファに書き出し、OLEDに描画命令まで実行する。
// 点描画ではなく、線描画にすることでグラフの途切れを防止している。
// 心拍数表示を消さないために、描画対象列に相当する部分だけ心拍数を描画するようにしてある
//**************************************************
void Draw_ECG_B(uint8_t y1, uint8_t y2, uint8_t last_segment_no)
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

    //一周前のデータを消す
    for (page = 0; page < 8; page++)
    {
        oled_ram_buf[latest_segment_no][page] = 0x00; 
    }

    //心拍数を描画する
    for (int i = 0; i < 17; i++)
    {
        oled_ram_buf[111+i][7] = dig_oled_buf[i];//描画位置は右上にする
    }

    //描画周期を表示する
    for (int i = 0; i < 5; i++)
    {
        oled_ram_buf[i][7] = font_num_7[draw_cycle][i];
    }

    //心拍検出タイミングを描画する。
    if(heart_beat_draw_EN==1)
    {
        oled_ram_buf[latest_segment_no][7] = 0xf0;//描画位置は上端で、4ピクセルの線にした
    }
    heart_beat_draw_EN=0;

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
        if (last_segment_no == 127)
        {
            Column_Page_Set(last_segment_no, last_segment_no, page); //セグメント127専用
            Wire.beginTransmission(ADDRES_OLED);
            // 2セグメントずつ送信することで高速化する。
            Wire.write(0b01000000); // control byte, Co bit = 0 (continue), D/C# = 1 (data)
            Wire.write(oled_ram_buf[last_segment_no][page]);
            //本来latest_segment_noがゼロになっているので、そこに最新データを書く必要があるが、
            //描画が遅くなるので省略している。
        }
        else
        {
            Column_Page_Set(last_segment_no, last_segment_no + 1, page); //セグメント128には書かない
            Wire.beginTransmission(ADDRES_OLED);
            // 2セグメントずつ送信することで高速化する。
            Wire.write(0b01000000); // control byte, Co bit = 0 (continue), D/C# = 1 (data)
            Wire.write(oled_ram_buf[last_segment_no][page]);
            Wire.write(oled_ram_buf[latest_segment_no][page]);
        }
        Wire.endTransmission();
    }
}
