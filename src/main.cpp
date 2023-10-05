#include <Arduino.h>
#include <DS18B20.h>
#include <M5Unified.h>
#include <OneWire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_sntp.h>

#include "DSEG7ModernBold60.h"
#include "wifi_credentials.h"

#define FIRMWARE_VERSION "v1.0"
#define FIRMWARE_DATE    __DATE__

#define X_MARGIN                 40
#define DISPLAY_TOD_INTERVAL_MS  (1 * 1000)
#define TEMP_MEASURE_INTERVAL_MS (10 * 1000)
#define PAGE3_UPDATE_MS          (5 * 1000)
#define UPDATE_IND_RADIUS        8
#define UPDATE_IND_ON_MS         500  // Duration (ms) to display update indicator circle

// Thermometer sprite data
#define thermom_spr_wdth 200
#define thermom_spr_ht   25
#define thermom_spr_rad  5
// #define thermom_width    (thermom_spr_wdth - 10)
// #define thermom_height   (thermom_spr_ht - 10)

// Battery sprite / icon data
#define batt_spr_x       0   // Battery sprite X location on LCD
#define batt_spr_y       0   // Battery sprite Y location on LCD
#define batt_spr_wdth    37  // Battery sprite width
#define batt_spr_ht      20  // Battery sprite height
#define batt_rect_width  30
#define batt_rect_height 14
#define batt_button_wdth 4
#define batt_button_ht   6

// #define ONE_WIRE_BUS_PIN 26 // 8-pin DIP port
#define ONE_WIRE_BUS_PIN 33  // Grove 4-pin port

OneWire oneWire(ONE_WIRE_BUS_PIN);
DS18B20 sensor(&oneWire);
Preferences eeprom;

// ST7789V resolution 240 x 135
M5Canvas batt_sprite(&M5.Display);     // Sprite for battery icon and percentage text
M5Canvas thermom_sprite(&M5.Display);  // Sprite for battery icon and percentage text
m5::rtc_time_t RTCtime;
m5::rtc_date_t RTCdate;
time_t t;
struct tm *timeinfo;
unsigned long update_tod_ms = 0;
unsigned long update_temp_sensor_ms = 0;
unsigned long update_page3 = 0;
unsigned long update_indicator_ms = 0;
int display_page = 0;
int indicator_color_on = M5.Display.color565(100, 100, 100);
int indicator_color_off = M5.Display.color565(0, 0, 0);
float temp_correction = 0.0;

void sync_rtc_to_ntp(void);
void display_tod(void);
void flash_LED(uint8_t brightness_percent, uint8_t freq, int16_t flash_number);
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y);
uint16_t batt_percent_to_color(int32_t batt);
int temp_to_color(int temp);
void draw_bargraph_scale(void);
void display_splash_screen(void);

/* -----------------
  Setup function runs once
----------------- */
void setup() {
  setCpuFrequencyMhz(80);  // Hopefully ESP32 runs a bit cooler at 80MHz vs 240MHz

  auto cfg = M5.config();

  cfg.serial_baudrate = 115200;  // default=115200. if "Serial" is not needed, set it to 0.
  cfg.clear_display = true;      // default=true. clear the screen when begin.
  cfg.output_power = true;       // default=true. use external port 5V output.
  cfg.internal_imu = true;       // default=true. use internal IMU.
  cfg.internal_rtc = true;       // default=true. use internal RTC.
  cfg.internal_spk = false;      // default=true. use internal speaker.
  cfg.internal_mic = false;      // default=true. use internal microphone.
  cfg.led_brightness = 0;        // default= 0. system LED brightness (0=off / 255=max) (※ not NeoPixel)

  M5.begin(cfg);
  // M5.Power.Axp192.setEXTEN(false);

  ESP_LOGI("setup", "Booted!");

  M5.Display.setBrightness(128);
  M5.Display.setRotation(1);

  // Create battery icon sprite
  batt_sprite.createSprite(batt_spr_wdth, batt_spr_ht);
  thermom_sprite.createSprite(thermom_spr_wdth, thermom_spr_ht);

  // Sync ESP32 RTC to M5Stack RTC. Need to call this periodically to avoid drift of a few sec /day
  M5.Rtc.setSystemTimeFromRtc();

  // Initialise DS18B20 temperature sensor
  sensor.begin();

  // Read temperature offset from EEPROM
  eeprom.begin("correction", false);
  // eeprom.putFloat("correction", -5.5);
  // delay(10);
  temp_correction = eeprom.getFloat("correction", 0.0);
  eeprom.end();
  ESP_LOGI("", "temperature correction is: %.2f", temp_correction);

  flash_LED(20, 6, 2);
  display_splash_screen();
  int timeout = 0;
  // Timeout waiting for button press after 10 seconds
  while (!M5.BtnA.isPressed()) {
    M5.update();
    delay(100);
    if (timeout++ > 40) break;
  };
  M5.Display.clear();

  // Start with temperature display
  display_page = 1;
  draw_bargraph_scale();
}

/* -----------------
  Main Loop
----------------- */
void loop() {
  int width = M5.Display.width();
  int height = M5.Display.height();
  static int circ_x = width - 12;
  static int circ_y = 13;
  static bool display_once = true;
  int y = 0;
  int x = width / 2;
  int line_ht = 30;
  float temperature = 0.0;
  char txt[80] = "";

  M5.update();  // check buttons

  // M5Stick C/CPlus: BtnA, BtnB, BtnPWR
  if (M5.BtnA.isHolding()) {
    if (display_page == 0) {
      // Sync ESP32 RTC to NTP time
      M5.Display.clear();
      sync_rtc_to_ntp();
      M5.Display.clear();
      display_page = 0;  // Set to display TOD page
    } else if (display_page == 1) {
      // Calibrate temperature (adjust temp offset)
      M5.Display.clear();
      M5.Display.setTextDatum(top_center);
      M5.Display.setFont(&fonts::FreeSans12pt7b);
      M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
      x = width / 2;
      y = 5;
      M5.Display.drawString("Calibrate Temp.", x, y);

      x = 20;
      y = 40;
      M5.Display.setTextDatum(top_left);
      M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
      M5.Display.drawString("Offset:", x, y);

      x = 110;
      M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
      sprintf(txt, "%.2f", temp_correction);
      M5.Display.drawString(txt, x, y);

      while (M5.BtnA.isPressed()) {
        M5.update();
        delay(100);
      }

      delay(700);

      do {
        M5.update();
        delay(100);
        if (M5.BtnB.wasClicked()) {
          temp_correction += 0.2;
          sprintf(txt, "%.2f", temp_correction);
          M5.Display.drawString(txt, x, y);
          delay(300);
        } else if (M5.BtnPWR.wasClicked()) {
          temp_correction -= 0.2;
          sprintf(txt, "%.2f", temp_correction);
          M5.Display.drawString(txt, x, y);
          delay(300);
        }
      } while (!M5.BtnA.wasClicked());

      M5.Display.setTextDatum(top_center);
      M5.Display.setFont(&fonts::FreeSans12pt7b);
      M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
      x = width / 2;
      y += 30;
      M5.Display.drawString("Finished!", x, y);
      delay(1000);

      // Save to EEPROM
      eeprom.begin("correction", false);
      eeprom.putFloat("correction", temp_correction);
      eeprom.end();

      M5.Display.clear();
      display_page = 1;
      draw_bargraph_scale();
      M5.update();  // Clear any remaining clicks
    }
  }

  if (M5.BtnA.wasClicked()) {
    M5.Display.clear();
    update_tod_ms = 0;          // Trigger immediate page draw
    update_temp_sensor_ms = 0;  // Trigger immediate page draw
    display_page++;
    if (display_page > 3)
      display_page = 0;

    // Create thermometer bargraph scale
    if (display_page == 1) {
      draw_bargraph_scale();
    }
    if (display_page == 3)
      display_once = false;
  }

  // Erase update indicator
  if (millis() > update_indicator_ms)
    M5.Display.fillCircle(circ_x, circ_y, UPDATE_IND_RADIUS, indicator_color_off);

  switch (display_page) {
    case 0:
      if (millis() >= update_tod_ms) {
        update_tod_ms = millis() + DISPLAY_TOD_INTERVAL_MS;
        update_indicator_ms = millis() + UPDATE_IND_ON_MS;
        M5.Display.fillCircle(circ_x, circ_y, UPDATE_IND_RADIUS, indicator_color_on);
        display_tod();
      }
      break;

    case 1:
      if (millis() >= update_temp_sensor_ms) {
        update_temp_sensor_ms = millis() + TEMP_MEASURE_INTERVAL_MS;
        update_indicator_ms = millis() + UPDATE_IND_ON_MS;
        M5.Display.fillCircle(circ_x, circ_y, UPDATE_IND_RADIUS, indicator_color_on);

        if (sensor.isConnected()) {
          sensor.requestTemperatures();
          // wait until sensor is ready
          while (!sensor.isConversionComplete()) {};
          temperature = sensor.getTempC() + temp_correction;
          ESP_LOGI("", "DS18B20 Temperature 1= %.1f", temperature);

          x = width / 2;
          y = 60;
          sprintf(txt, "%.1f C", temperature);
          M5.Display.setFont(&DSEG7_Modern_Bold_60);
          M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Display.setTextDatum(top_center);
          M5.Display.setTextPadding(120);
          M5.Display.drawString(txt, x, y);
          M5.Display.drawCircle(x + 55, y + 15, 7);
          M5.Display.drawCircle(x + 55, y + 15, 6);

          // erase old fill
          thermom_sprite.clear();
          thermom_sprite.drawRoundRect(0, 0, thermom_spr_wdth, thermom_spr_ht, thermom_spr_rad, TFT_LIGHTGRAY);

          // Thermometer scale is 10 to 40°C ==> span of 30
          // Display 30 deg over x-span of 200 (thermom_width)
          int disp_temp = (int)round(temperature);
          if (disp_temp > 40) disp_temp = 40;
          if (disp_temp < 10) disp_temp = 10;

          // disp_temp = 38;  // for testing
          int32_t bar_width = (disp_temp - 10) * thermom_spr_wdth / (40 - 10);
          int bar_color = temp_to_color(disp_temp);
          thermom_sprite.fillRoundRect(1, 1, bar_width - 2, thermom_spr_ht - 2, thermom_spr_rad, bar_color);
          thermom_sprite.pushSprite(width / 2 - thermom_spr_wdth / 2 - 5, 2);

        } else {
          M5.Display.setTextDatum(top_center);
          M5.Display.setFont(&fonts::FreeSans12pt7b);
          M5.Display.setTextColor(TFT_RED, TFT_BLACK);
          M5.Display.clear();
          y = 20;
          M5.Display.drawString("DS18B20", x, y);
          M5.Display.drawString("Not connected", x, y + 25);
          ESP_LOGI("Error", "DS18B20 Not connected");
        }
      }

      break;

    case 2:
      if (millis() >= update_page3) {
        int32_t batt_percent = M5.Power.getBatteryLevel();
        uint16_t batt_color = 0;
        const int x_val = 90;

        update_page3 = millis() + PAGE3_UPDATE_MS;
        update_indicator_ms = millis() + UPDATE_IND_ON_MS;
        M5.Display.fillCircle(circ_x, circ_y, UPDATE_IND_RADIUS, indicator_color_on);

        M5.Display.setFont(&fonts::FreeSans12pt7b);
        M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
        M5.Display.setTextPadding(140);
        M5.Display.setTextDatum(top_left);
        x = 15;
        y = line_ht;
        M5.Display.drawString("Level:", x, y);

        batt_color = batt_percent_to_color(batt_percent);
        M5.Display.setTextColor(batt_color, TFT_BLACK);
        sprintf(txt, "%d%%", batt_percent);
        x = x_val;
        M5.Display.drawString(txt, x, y);

        x = 15;
        y += line_ht;
        M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
        M5.Display.drawString("State:", x, y);

        x = x_val;
        M5.Display.setTextColor(TFT_DARKGRAY, TFT_BLACK);
        if (M5.Power.isCharging())
          M5.Display.drawString("Charging", x, y);
        else if (M5.Power.getBatteryLevel() == 100)
          M5.Display.drawString("Full", x, y);
        else
          M5.Display.drawString("Discharge", x, y);

        x = 15;
        y += line_ht;
        M5.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
        M5.Display.drawString("USB:", x, y);
        sprintf(txt, "%.1f V", M5.Power.Axp192.getVBUSVoltage());
        x = x_val;
        M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Display.drawString(txt, x, y);

        disp_batt_symbol(width - 22, height / 2);
      }
      break;

    case 3:
      if (!display_once) {
        display_once = true;
        display_splash_screen();
      }
      break;

    default:
      break;
  }
}

/* -----------------
  Convert temperature to TFT color
  0-18 deg C = BLUE
  19-24 deg C = ORANGE
  25-40 deg C = RED
----------------- */
int temp_to_color(int temp) {
  switch (temp) {
    case -50 ... 15:
      return TFT_BLUE;
      break;

    case 16 ... 19:
      return TFT_SKYBLUE;  // Light Blue
      break;

    case 20 ... 22:
      return M5.Display.color24to16(0xff944d);  // Light Orange
      break;

    case 23 ... 25:
      return TFT_ORANGE;
      break;

    case 26 ... 150:
      return TFT_RED;
      break;

    default:
      return TFT_DARKGREY;
      break;
  }
}

/* -----------------
  Synchronise ESP32 Real Time Clock (RTC) to NTP, then
  update the CoreInk's external RTC chip to ESP32 RTC
----------------- */
void sync_rtc_to_ntp(void) {
  int width = M5.Display.width();
  int height = M5.Display.height();
  int line_ht = 30;
  int x = width / 2;
  int y = 0;

  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextPadding(0);
  M5.Display.setTextDatum(top_center);
  M5.Display.drawString("WiFi:", x, y);
  ESP_LOGI("sync TOD", "Connecting WiFi");

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }

  y += line_ht;
  M5.Display.drawString("Connected.", x, y);
  ESP_LOGI("sync TOD", "Connected");

  y += line_ht;
  M5.Display.drawString("Syncing clock", x, y);
  ESP_LOGI("sync TOD", "Syncing clock");

  // Sync to NTP
  y += line_ht;
  x = 40;
  configTzTime("ACST-9:30ACDT,M10.1.0,M4.1.0/3", "0.au.pool.ntp.org");  // ACST = Australian Central Standard Time (timezone)
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
    M5.Display.drawString("*", x, y);
    x += 15;
    delay(1000);
  }

  // Epoch time variable, i.e. number of seconds since 1-1-1970 UTC
  t = time(nullptr) + 1;  // Advance one second
  while (t > time(nullptr))
    ;                            // Synchronization in seconds
  timeinfo = localtime(&t);      // Convert epoch time to a "tm" structure
  M5.Rtc.setDateTime(timeinfo);  // Writes the date and time to the Core2's external RTC chip
  y += 15;
  x = width / 2;
  M5.Display.drawString("Clock sync'd", x, y);
  ESP_LOGI("sync TOD", "Clock sync'd");

  WiFi.disconnect(true);
  delay(1500);
}

/* -----------------
  Display Time Of Day
----------------- */
void display_tod(void) {
  char txt[80];
  int width = M5.Display.width();
  int height = M5.Display.height();
  int line_ht = 30;
  int y = 10;
  int x = width / 2;

  // Read RTC time of day
  t = time(nullptr);
  timeinfo = localtime(&t);

  M5.Lcd.setTextPadding(100);

  // Divider line
  M5.Display.drawLine(X_MARGIN, y, width - X_MARGIN, y, TFT_DARKGRAY);

  // Display date and time
  // https://cplusplus.com/reference/ctime/strftime/
  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Display.setTextDatum(top_center);

  // Day of week
  y += (line_ht - 14);
  strftime(txt, 80, "%A", timeinfo);
  M5.Display.drawString(txt, x, y);

  // Date
  y += line_ht;
  strftime(txt, 80, "%e-%b-%Y", timeinfo);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.drawString(txt, x, y);

  // Time
  y += line_ht;
  strftime(txt, 80, "%I:%M:%S %p", timeinfo);
  M5.Display.drawString(txt, x, y);

  // Divider line
  y += (line_ht + 5);
  M5.Display.drawLine(X_MARGIN, y, width - X_MARGIN, y, TFT_DARKGRAY);
}

/*
-----------------
  Flash the StickC LED
  brightness_percent - percentage brightness
  freq - freq in Hz to flash
  flash_number - number of flashes to do. -1 = forever
-----------------
*/
void flash_LED(uint8_t brightness_percent, uint8_t freq, int16_t flash_number) {
  uint32_t period = 500 / freq;  // Double the frequency of 1000ms period
  int16_t count = flash_number;
  uint8_t led_bright = ((brightness_percent * 0xFF) / 100);

  while (brightness_percent && (count > 0)) {
    M5.Power.setLed(led_bright);
    delay(period);
    M5.Power.setLed(0);
    delay(period);
    count--;
  };
}

/*
-----------------
  Display Core2 battery symbol, % charge, and voltage
  batt_x - X coordinate of battery sprite
  batt_y - Y coordinate of battery sprite
  batt_volt - LiPo voltage. Range is 3.7V to 4.2V
-----------------
*/
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y) {
  int32_t batt_percent = 0;
  int32_t batt_fill_length = 0;
  uint16_t fill_colour = TFT_DARKGRAY;
  uint16_t spr_x = 0;                // X-axis offset of battery icon and voltage text in sprite
  uint16_t spr_y = batt_spr_ht / 2;  // Y-axis offset of battery icon and voltage text in sprite

  // Read battery voltage
  batt_percent = M5.Power.getBatteryLevel();
  batt_fill_length = (batt_percent * batt_rect_width) / 100;

  // Clear the old values
  batt_sprite.fillSprite(TFT_BLACK);  // Clear the battery icon sprite

  fill_colour = batt_percent_to_color(batt_percent);

  // Draw the battery symbol outline
  spr_x = 0;
  spr_y = ((batt_spr_ht - batt_rect_height) / 2) - 1;
  batt_sprite.drawRect(spr_x, spr_y, batt_rect_width, batt_rect_height, TFT_LIGHTGRAY);

  // Draw the button on top of the battery - intentional gap from the main battery rectangle
  batt_sprite.fillRect(spr_x + batt_rect_width + 1, (batt_spr_ht - batt_button_ht) / 2, batt_button_wdth, batt_button_ht, TFT_LIGHTGRAY);

  // Erase the old battery level
  batt_sprite.fillRect(spr_x + 1, spr_y + 1, batt_rect_width - 2, batt_rect_height - 2, TFT_BLACK);

  // Draw the current battery level
  batt_sprite.fillRect(spr_x + 1, spr_y + 1, batt_fill_length - 2, batt_rect_height - 2, fill_colour);

  // Draw lighning bolt symbol
  if (M5.Power.isCharging()) {
    uint16_t cntre_x = spr_x + (batt_rect_width / 2);
    uint16_t cntre_y = spr_y + (batt_rect_height / 2) - 1;
    batt_sprite.fillTriangle(cntre_x - 15, cntre_y - 2, cntre_x, cntre_y, cntre_x + 2, cntre_y + 6, TFT_ORANGE);
    batt_sprite.fillTriangle(cntre_x + 15, cntre_y + 2, cntre_x, cntre_y, cntre_x - 2, cntre_y - 6, TFT_ORANGE);
  }

  // Display the sprite
  batt_sprite.pushRotateZoom(batt_x, batt_y, -90, 1.3, 1.3);
  // batt_sprite.pushSprite(batt_x, batt_y);
}

uint16_t batt_percent_to_color(int32_t batt) {
  uint16_t color = TFT_WHITE;

  if (batt < 20)
    color = TFT_RED;
  else if (batt >= 20 && batt < 50)
    color = TFT_ORANGE;
  else if (batt >= 50)
    color = TFT_DARKGREEN;

  return color;
}

void draw_bargraph_scale(void) {
  int y = 0;
  int x = M5.Display.width() / 2;
  char scale_txt[80] = "";

  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(top_center);
  M5.Display.setTextPadding(0);

  // Draw minor scale tick marks
  y = 30;
  const int num_ticks_minor = 18;
  for (int i = 0; i < (num_ticks_minor + 1); i++) {
    x = 16 + (i * thermom_spr_wdth / num_ticks_minor);
    M5.Display.drawLine(x, y - 4, x, y, TFT_DARKGREY);
  }

  // Draw major scale tick marks
  y = 36;
  const int num_ticks_major = 6;
  for (int i = 0; i < (num_ticks_major + 1); i++) {
    x = 16 + (i * thermom_spr_wdth / num_ticks_major);
    M5.Display.drawLine(x, y - 10, x, y, TFT_WHITE);
    sprintf(scale_txt, "%d", 10 + i * 5);
    M5.Display.drawString(scale_txt, x, y + 2);
  }
}

void display_splash_screen(void) {
  int x = 0;
  int y = 0;
  int line_ht = 35;
  char txt[80] = "";

  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setTextPadding(0);
  M5.Display.setTextDatum(top_center);
  x = M5.Display.width() / 2;
  y = 20;

  M5.Display.drawString("Thermometer LCD", x, y);

  y += line_ht;
  M5.Display.setFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.drawString("by Patrick Felstead", x, y);

  y += line_ht;
  M5.Display.setFont(&fonts::FreeSans9pt7b);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  sprintf(txt, "%s, %s", FIRMWARE_VERSION, FIRMWARE_DATE);
  M5.Display.drawString(txt, x, y);
}