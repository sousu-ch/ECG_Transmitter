#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

//#define SCROLL_VIEW

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

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);;  // 0.96inch OLED SSD1306 使用時に選択
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // 1.3inch OLED SH1106 使用時に選択

//グローバル変数
uint16_t range = 8; // レンジ番号(3:100Hz, 4:200Hz, 5:50Hz, 6:1k, 7:2k, 8:5k, 9:10k, 10:20k, 11:50k)

uint8_t buf_no = 0;
uint8_t latest_buf_no = 0;
uint8_t last_data = 0;
uint8_t wave[129];

void readWave()
{
    if (buf_no > DATA_BUF_SIZE)
    {
        buf_no=0;
    }
    latest_buf_no = buf_no;
    last_data = wave[buf_no];          // LCDスクロール表示の前データ塗りつぶし用データ
    wave[buf_no] = analogRead(SIG_IN); // 波形データー取得
    buf_no++;
    // delayMicroseconds(3886);      // サンプリング周期調整
}

void setup()
{
    pinMode(CHECK_PIN, OUTPUT);       // 実行時間測定用
    pinMode(BOARD_LED, OUTPUT);       // pico内蔵LED
    pinMode(OF_LED, OUTPUT);          // オーバーフロー表示LED
    pinMode(UP_SW, INPUT_PULLUP);     // UPボタン
    pinMode(DN_SW, INPUT_PULLUP);     // Downボタン
    pinMode(ADC_COMPE, INPUT_PULLUP); // ADC補正指定ピン（HIGHで補正有り）

    Serial.begin(115200);     // Rp pico はシリアルの起動が失敗することがある
    analogReadResolution(8); // ADCのフルスケールを8ビットに設定

    u8g2.begin(); // (I2Cバス400kbpsで開始される)
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop(); // 左上を文字位置とする
    u8g2.clearBuffer();
    //u8g2.drawStr(0, 0, "Multi Range FFT v0.4");

    u8g2.sendBuffer();
    delay(1000);
}

void loop()
{
    uint8_t data=0;

    // 波形の読み取り
    digitalWrite(BOARD_LED, LOW); // ボード内蔵LED点灯
    digitalWrite(CHECK_PIN, LOW); // タイミング測定ピンHigh

#if defined(SCROLL_VIEW)
    readWave(); // 波形を読み取り

#else
    data = analogRead(SIG_IN)-100; // 波形データー取得

#endif
    digitalWrite(CHECK_PIN, HIGH); // タイミング測定ピンLow
    digitalWrite(BOARD_LED, HIGH); // ボード内蔵LED消灯

    showWaveform(data);
    //showBackGround(); // 目盛線等の背景を表示           (1.4ms)
    u8g2.sendBuffer();

    //delay(5);
}

void showWaveform(uint8_t new_y)
{
    static uint8_t last_x_axis=0;
    static uint8_t last_y = 0;
    static uint8_t new_x_axis;

    if (last_x_axis > 126)
    {
        new_x_axis = 0;
        //消去処理
        u8g2.clearDisplay();
            //描画処理
            u8g2.drawLine(new_x_axis, last_y, new_x_axis, new_y); // 波形プロット
    }
    else{
        new_x_axis = last_x_axis + 1;
        u8g2.drawLine(last_x_axis, last_y, new_x_axis, new_y); // 波形プロット
    }

    last_x_axis = new_x_axis;
    last_y = new_y;
}

void showWaveform_SCROLL_VIEW()
{ // 入力波形を表示
    int last_y, new_y;
    u8g2.setDrawColor(0); // 黒で書く
    uint16_t last_baf_no=0;





        last_y = last_data / 64;
        uint16_t byouga_no;
        byouga_no = latest_buf_no;
        for (int i = 0; i < 128; i += 1) //前回のを黒で塗りつぶす
        {

            new_y = (wave[byouga_no] / 64);
            //★たぶんここ描画位置X座標がずれる
            u8g2.drawLine(PX2 + i, last_y, PX2 + i + 1, new_y); // 波形プロット
            last_y = new_y;
            byouga_no++;
    }


    last_y = (wave[0]) / 64;
    u8g2.setDrawColor(1); // 白で書く
    byouga_no = last_baf_no;
    byouga_no = last_baf_no+2;
    for (int i = 2; i < 254; i += 2)
    {
        byouga_no += 2;
        if (byouga_no > 250)
        {
            byouga_no = 0;
        }

        new_y = PY1 - (wave[i + 2] / 64);
        //★たぶんここ描画位置X座標がずれる
        u8g2.drawLine(PX2 + i / 2, last_y, PX2 + i / 2 + 1, new_y); // 波形プロット
        last_y = new_y;
    }
}

void showBackGround()
{ // グラフの修飾（目盛他の作画）
    //　領域区分線
    u8g2.setDrawColor(1);          // 白で書く
    u8g2.drawVLine(0, 6, 6);       // 時間軸左端 縦線
    u8g2.drawVLine(63, 6, 6);      // 時間軸1/2
    u8g2.drawVLine(127, 6, 6);     // 時間軸右端
    u8g2.drawVLine(1, 8, 2);       // 時間軸左端 中心線
    u8g2.drawVLine(63 - 1, 8, 2);  // 時間軸1/2
    u8g2.drawVLine(63 + 1, 8, 2);  // 時間軸1/2
    u8g2.drawVLine(127 - 1, 8, 2); // 時間軸右端
    u8g2.drawHLine(PX2, PY2, 128); // スペクトル下端線

    // 周波数目盛（下の横軸）
    for (int xp = PX2; xp < 127; xp += 10)
    { // 等間隔目盛
        u8g2.drawVLine(xp, PY2 + 1, 2);
    }
    u8g2.drawBox(PX2, PY2 + 2, 2, 2);      // 0k太い目盛(2画素）
    u8g2.drawBox(PX2 + 49, PY2 + 2, 3, 2); // 10k太い目盛(3画素）
    u8g2.drawBox(PX2 + 99, PY2 + 2, 3, 2); // 20k太い目盛

    freqScale(); // レンジに対応した周波数目盛を表示

    //　スペクトルレベル目盛（縦軸）
    u8g2.setDrawColor(2); // 重なっても見えるようにXORで書く
    for (int y = PY2 - 10; y > 16; y -= 10)
    {                            // dB目盛線（横の点線）
        u8g2.drawHLine(0, y, 2); // ゼロの傍の目盛
        for (int x = 9; x < 127; x += 10)
        {
            u8g2.drawHLine(x, y, 3);
        }
    }

    for (int y = PY2 - 10; y > 16; y -= 10)
    { // (-60dB対応）交点マーク（+）の縦の線
        for (int x = 0; x < 110; x += 50)
        {
            u8g2.drawPixel(x, y - 1); // 縦線では交点が消えるので点で描く
            u8g2.drawPixel(x, y + 1);
        }
    }
    u8g2.setFont(u8g2_font_micro_tr); // 小さな3x5ドットフォントで、
    u8g2.setFontMode(0);
    u8g2.setDrawColor(1);
    u8g2.drawStr(117, 16, "0dB"); // スペクトル感度
    u8g2.drawStr(117, 26, "-20");
    u8g2.drawStr(117, 36, "-40");
    u8g2.drawStr(117, 45, "-60");
}

void freqScale()
{                                          // 周波数目盛表示
    u8g2.setFont(u8g2_font_mozart_nbp_tr); // 5x7ドットフォント
    u8g2.drawStr(0, 56, "0");              // 原点の0を表示
    switch (range)
    { // レンジ番号に応じた周波数を表示
    case 3:
        u8g2.drawStr(114, 0, "1s");  // サンプリング期間
        u8g2.drawStr(45, 56, "50");  // 1/2周波数
        u8g2.drawStr(92, 56, "100"); // フルスケール周波数
        break;
    case 4:
        u8g2.drawStr(102, 0, "0.5s");
        u8g2.drawStr(42, 56, "100");
        u8g2.drawStr(92, 56, "200");
        break;
    case 5:
        u8g2.drawStr(96, 0, "200ms");
        u8g2.drawStr(42, 56, "250");
        u8g2.drawStr(92, 56, "500");
        break;
    case 6:
        u8g2.drawStr(96, 0, "100ms");
        u8g2.drawStr(42, 56, "500");
        u8g2.drawStr(95, 56, "1k");
        break;
    case 7:
        u8g2.drawStr(102, 0, "50ms");
        u8g2.drawStr(45, 56, "1k");
        u8g2.drawStr(95, 56, "2k");
        break;
    case 8:
        u8g2.drawStr(102, 0, "20ms");
        u8g2.drawStr(41, 56, "2.5k");
        u8g2.drawStr(95, 56, "5k");
        break;
    case 9:
        u8g2.drawStr(102, 0, "10ms");
        u8g2.drawStr(45, 56, "5k");
        u8g2.drawStr(92, 56, "10k");
        break;
    case 10:
        u8g2.drawStr(108, 0, "5ms");
        u8g2.drawStr(42, 56, "10k");
        u8g2.drawStr(92, 56, "20k");
        break;
    case 11:
        u8g2.drawStr(108, 0, "2ms");
        u8g2.drawStr(42, 56, "25k");
        u8g2.drawStr(92, 56, "50k");
        break;
    default:
        break;
    }
}