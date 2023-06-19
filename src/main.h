#define LGFX_USE_V1
#include "lvgl.h"
#include <LovyanGFX.hpp>
#define screenWidth 320
#define screenHeight 480


class LGFX : public lgfx::LGFX_Device
{

lgfx::Panel_ST7796      _panel_instance;
lgfx::Bus_Parallel8 _bus_instance;   // 8ビットパラレルバスのインスタンス (ESP32のみ)
lgfx::Light_PWM     _light_instance;
lgfx::Touch_FT5x06           _touch_instance; // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436

public:
  LGFX(void)
  {
    { // バス制御の設定を行います。
      auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。
// 8ビットパラレルバスの設定
      //cfg.i2s_port = I2S_NUM_0;     // 使用するI2Sポートを選択 (I2S_NUM_0 or I2S_NUM_1) (ESP32のI2S LCDモードを使用します)
      cfg.freq_write = 20000000;    // 送信クロック (最大20MHz, 80MHzを整数で割った値に丸められます)
      cfg.pin_wr =  47;              // WR を接続しているピン番号
      cfg.pin_rd =  -1;              // RD を接続しているピン番号
      cfg.pin_rs = 0;              // RS(D/C)を接続しているピン番号
      cfg.pin_d0 = 9;              // D0を接続しているピン番号
      cfg.pin_d1 = 46;              // D1を接続しているピン番号
      cfg.pin_d2 = 3;              // D2を接続しているピン番号
      cfg.pin_d3 = 8;              // D3を接続しているピン番号
      cfg.pin_d4 = 18;              // D4を接続しているピン番号
      cfg.pin_d5 = 17;              // D5を接続しているピン番号
      cfg.pin_d6 = 16;              // D6を接続しているピン番号
      cfg.pin_d7 = 15;              // D7を接続しているピン番号
      _bus_instance.config(cfg);    // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
    }

    { // 表示パネル制御の設定を行います。
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。

      cfg.pin_cs           =    -1;  // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst          =    4;  // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy         =    -1;  // BUSYが接続されているピン番号 (-1 = disable)

      // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。

      cfg.panel_width      =   320;  // 実際に表示可能な幅
      cfg.panel_height     =   480;  // 実際に表示可能な高さ
      cfg.offset_x         =     0;  // パネルのX方向オフセット量
      cfg.offset_y         =     0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         =  true;  // データ読出しが可能な場合 trueに設定
      cfg.invert           = true;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false;  // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared       =  true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

// 以下はST7735やILI9163のようにピクセル数が可変のドライバで表示がずれる場合にのみ設定してください。
//    cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
//    cfg.memory_height    =   320;  // ドライバICがサポートしている最大の高さ

      _panel_instance.config(cfg);
    }

//*
    { // バックライト制御の設定を行います。（必要なければ削除）
      auto cfg = _light_instance.config();    // バックライト設定用の構造体を取得します。

      cfg.pin_bl = 45;              // バックライトが接続されているピン番号
      cfg.invert = false;           // バックライトの輝度を反転させる場合 true
      cfg.freq   = 44100;           // バックライトのPWM周波数
      cfg.pwm_channel = 7;          // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセットします。
    }
//*/

//*
    { // タッチスクリーン制御の設定を行います。（必要なければ削除）
      auto cfg = _touch_instance.config();

      cfg.x_min      = 0;    // タッチスクリーンから得られる最小のX値(生の値)
      cfg.x_max      = 319;  // タッチスクリーンから得られる最大のX値(生の値)
      cfg.y_min      = 0;    // タッチスクリーンから得られる最小のY値(生の値)
      cfg.y_max      = 479;  // タッチスクリーンから得られる最大のY値(生の値)
      cfg.pin_int    = 7;   // INTが接続されているピン番号
      cfg.bus_shared = true; // 画面と共通のバスを使用している場合 trueを設定
      cfg.offset_rotation = 0;// 表示とタッチの向きのが一致しない場合の調整 0~7の値で設定
// I2C接続の場合
      cfg.i2c_port = 1;      // 使用するI2Cを選択 (0 or 1)
      cfg.i2c_addr = 0x38;   // I2Cデバイスアドレス番号
      cfg.pin_sda  = 6;     // SDAが接続されているピン番号
      cfg.pin_scl  = 5;     // SCLが接続されているピン番号
      cfg.freq = 400000;     // I2Cクロックを設定

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  // タッチスクリーンをパネルにセットします。
    }
//*/
    setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};

// 準備したクラスのインスタンスを作成します。
//#include "main.h"




LGFX tft;



static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void my_touch_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouch(&touchX, &touchY);
    if (!touched) { data->state = LV_INDEV_STATE_REL; }
    else {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touchX;
      data->point.y = touchY;
    }
}


//static lv_style_t label_style;
//static lv_obj_t *headerLabel;
//static lv_obj_t * label;

