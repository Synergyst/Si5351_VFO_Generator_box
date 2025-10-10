#include <Adafruit_SSD1306.h>   // Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_NeoPixel.h>  // WaveShare RP2040 Zero's WS2812 LED
#include <si5351.h>             // Etherkit https://github.com/etherkit/Si5351Arduino
#include <LittleFS.h>           // LittleFS for storing user-tunables
#include <Wire.h>               // I2C
#include <math.h>               // Standard math library
#include "string_switch.h"      // Switch-case for String/char*

// ==================== Defines ====================
#define FW_VERSION "1.1.4"     // Our firmware version
#define IF 10700               // Inter-Frequency in kHz
#define BAND_INIT 18           // Initial band preset
#define STEP_INIT 6            // Initial step size preset
#define XT_CAL_F 0             // XTAL ppm drift calibration amount
#define S_GAIN 505             // SM sensitivity: 101=500mv; 202=1v; 303=1.5v; 404=2v; 505=2.5v; 1010=5v (max)
#define SM_ADC A0              // Signal Meter ADC pin
#define LED_COUNT 1            // The WaveShare RP2040 Zero only has the one LED
#define LED_PIN 16             // WS2812 LED pin
#define rotLeft 14             // Rotary-left pin
#define rotRight 13            // Rotary-right pin
#define BAND_PIN 11            // Band selector push button pin
#define RX_TX_PIN 10           // RX / TX mode push button pin
#define TUNESTEP_PIN 9         // Tune step push button pin
#define WAIT_USB_SERIAL false  // Wait for a client to connect to USB serial connsole
#define BAUD 9600              // USB serial baud rate

// ==================== Tuner variables ====================
unsigned long freq, freqold, fstep;
long interfreq = IF, interfreqold = 0;
static long lastSentIF = -1;
long cal = XT_CAL_F;
byte encoder = 1;
byte stp, n = 1;
byte count, sigmeter, sigmeterOld;
bool sts = 0;
unsigned int period = 100;
unsigned long time_now = 0;

// ==================== Encoder state ====================
static const int8_t encTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};                           // encoder 4-position state table
static uint8_t encPrev = 0;  // encoder previous value
static int8_t encAccum = 0;  // encocer accumulator value

// ==================== Line Reader ====================-
static const size_t CMD_BUF_SZ = 255;  // USB serial buffer length
static char sUsbBuf[CMD_BUF_SZ];       // USB serial buffer
static size_t sUsbLen = 0;             // USB serial pending buffer input length

// ==================== UI scanner ====================
static const size_t UI_MAX_CUSTOM_SCAN = 255;  // Max custom scan list length
static bool uiScanOn = false;                  // Scanner off initially
static uint8_t uiScanSrc = 0;                  // 0 = hard list, 1 = custom list
static uint32_t uiScanDelayMs = 250;           // per-step delay (ms)
static uint32_t uiScanLastMs = 0;              // Last scan cycle in milliseconds
static size_t uiScanIdx = 0;                   // Current list index
static uint32_t uiScanCurrFreqHz = 0;          // Current scanner frequency in Hertz
static size_t uiCustomScanLen = 0;             // Length of custom scan list
static const uint32_t uiHardScanListMURS[] PROGMEM = {
  // MURS channels + old business band
  151820000UL,  // 151.820 MHz - MURS Calling (NFM)
  151880000UL,  // 151.880 MHz - MURS Safety (NFM)
  151940000UL,  // 151.940 MHz - MURS Emergency (NFM)
  154570000UL,  // 154.570 MHz - MURS Blue (WFM / old business band)
  154600000UL   // 154.600 MHz - MURS Green (WFM / old business band)
};
static const uint32_t uiHardScanListFM[] PROGMEM = {
  // Various FM broadcast station(s)
  96300000UL,   // 96.3 MHz -
  102500000UL,  // 102.5 MHz -
  103300000UL,  // 103.3 MHz -
  104100000UL,  // 104.1 MHz -
  106500000UL,  // 106.5 MHz -
  107700000UL,  // 107.7 MHz -
};
static const uint32_t uiHardScanListNOAA[] PROGMEM = {
  // NOAA WX station(s)
  162550000UL  // 162.550 MHz - NOAA WX Ch 7
};
static const uint32_t uiHardScanListCB[] PROGMEM = {
  // Citizens Band (40-channels)
  26965000UL,  // 26.965 MHz - CB Channel 1
  26975000UL,  // 26.975 MHz - CB Channel 2
  26985000UL,  // 26.985 MHz - CB Channel 3
  26995000UL,  // 26.995 MHz - CB Channel 4
  27005000UL,  // 27.005 MHz - CB Channel 5
  27015000UL,  // 27.015 MHz - CB Channel 6
  27025000UL,  // 27.025 MHz - CB Channel 7
  27035000UL,  // 27.035 MHz - CB Channel 8
  27055000UL,  // 27.055 MHz - CB Channel 9
  27065000UL,  // 27.065 MHz - CB Channel 10
  27075000UL,  // 27.075 MHz - CB Channel 11
  27085000UL,  // 27.085 MHz - CB Channel 12
  27095000UL,  // 27.095 MHz - CB Channel 13
  27105000UL,  // 27.105 MHz - CB Channel 14
  27115000UL,  // 27.115 MHz - CB Channel 15
  27125000UL,  // 27.125 MHz - CB Channel 16
  27135000UL,  // 27.135 MHz - CB Channel 17
  27145000UL,  // 27.145 MHz - CB Channel 18
  27155000UL,  // 27.155 MHz - CB Channel 19
  27165000UL,  // 27.165 MHz - CB Channel 20
  27175000UL,  // 27.175 MHz - CB Channel 21
  27185000UL,  // 27.185 MHz - CB Channel 22
  27205000UL,  // 27.205 MHz - CB Channel 23
  27215000UL,  // 27.215 MHz - CB Channel 24
  27225000UL,  // 27.225 MHz - CB Channel 25
  27255000UL,  // 27.255 MHz - CB Channel 26
  27265000UL,  // 27.265 MHz - CB Channel 27
  27275000UL,  // 27.275 MHz - CB Channel 28
  27285000UL,  // 27.285 MHz - CB Channel 29
  27295000UL,  // 27.295 MHz - CB Channel 30
  27305000UL,  // 27.305 MHz - CB Channel 31
  27315000UL,  // 27.315 MHz - CB Channel 32
  27325000UL,  // 27.325 MHz - CB Channel 33
  27335000UL,  // 27.335 MHz - CB Channel 34
  27345000UL,  // 27.345 MHz - CB Channel 35
  27355000UL,  // 27.355 MHz - CB Channel 36
  27365000UL,  // 27.365 MHz - CB Channel 37
  27375000UL,  // 27.375 MHz - CB Channel 38
  27385000UL,  // 27.385 MHz - CB Channel 39
  27405000UL   // 27.405 MHz - CB Channel 40
};
static const size_t uiHardScanLen = sizeof(uiHardScanListMURS) / sizeof(uiHardScanListMURS[0]);  // Hard scan list length
static uint32_t uiCustomScanList[UI_MAX_CUSTOM_SCAN];                                            // Custom scanner list

// ==================== Terminal TUI ====================
enum {
  TUI_DIRTY_NONE = 0,
  TUI_DIRTY_HEADER = 1 << 0,
  TUI_DIRTY_SCAN = 1 << 1,
  TUI_DIRTY_FREQ = 1 << 2,
  TUI_DIRTY_BARS = 1 << 3,
  TUI_DIRTY_LOG = 1 << 4,
  TUI_DIRTY_INPUT = 1 << 5,
  TUI_DIRTY_FULL = 1 << 15
};                                              // TUI mask
static bool tuiEnabled = false;                 // TUI master switch
static bool tuiAnsi = true;                     // ANSI sequences allowed
static bool tuiInAlt = false;                   // in alternate screen
static bool tuiDirtyFull = false;               // needs full redraw
static bool tuiDirtyDyn = false;                // needs dynamic-only update
static unsigned long tuiLastDraw = 0;           // last draw time in milliseconds
static uint16_t tuiRows = 48;                   // terminal rows (default)
static uint16_t tuiCols = 200;                  // terminal cols (default)
static uint16_t tuiLogTop = 24;                 // computed based on layout
static uint16_t tuiLogBottom = (48 - 2);        // last row used by log pane (rows-2)
static uint16_t tuiInputRow = 48;               // input row (bottom)
static bool gTermNoEcho = false;                // Disable echo in readLine so TUI can render the input itself
static uint16_t tuiPanelW = 60;                 // width of right-side status panel
static const size_t TUI_LOG_MAX = 200;          // log pane max log count
static const size_t TUI_LINE_MAX = 240;         // log pane max line count
static char tuiLog[TUI_LOG_MAX][TUI_LINE_MAX];  // log pane log buffer
static size_t tuiLogHead = 0;                   // next write
static size_t tuiLogCount = 0;                  // log pane log count
static uint16_t tuiDirtyMask = 0;               // mask value for TUI dirty

// ==================== Power/dBm sampling constants ====================
#define ADC_VREF 3.03f     // your original 3.03 reference
#define SM_INPUT_DIV 2.0f  // 2x opamp gain -> divide measured voltage by 2
#define DBM_SLOPE 40.0f    // dBm = slope * V + offset -> from your formula
#define DBM_OFFSET 0.0f    // dBm offset amount

// ==================== Power/dBm sampling state ====================
static float gSmVolt = 0.0f;       // input voltage at MCU pin (after Vref scaling and divider)
static float gDbmNow = -99.0f;     // instantaneous dBm (unsmoothed)
static float gDbmSmooth = -99.0f;  // smoothed dBm for display
static float gPowerW = 0.0f;       // power in watts (from smoothed dBm)
static uint8_t gPwrBar = 0;        // 0..14 bar for TTY/OLED
static char gPowerStr[16] = "";    // compact text power (uW/mW/W)

// ==================== Top-3 peaks during scan ====================
struct PeakEntry {
  uint16_t idx;                           // index into current scan list
  uint32_t freqHz;                        // cached frequency for convenience
  float dbm;                              // peak dBm observed for this index
  float watts;                            // computed watts for this index peak
  uint8_t valid;                          // 0/1
};                                        // Peak structure format
static PeakEntry gScanPeaks[3];           // Top peaks detected
static bool gScanPeaksNeedsSave = false;  // for future flash persistence

// ==================== I2C devices ====================
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);  // SSD1306 OLED mono display (128x64 resolution)
Si5351 si5351(0x60);                                          // Si531 programmable clock generator

// ==================== WS2812 LED ====================
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);  // WS2812 LED on WaveShare RP2040 Zero PCB

// ==================== Persistent storage using LittleFS ====================
struct CfgV1 {
  uint32_t magic;
  uint16_t ver;
  uint16_t reserved;
  uint32_t freq_hz;
  int32_t if_khz;
  uint8_t stp_idx;
  uint8_t band_count;
  uint8_t s_meter_x;
  uint8_t scan_src;  // 0=HARD, 1=CUSTOM
  uint32_t scan_delay_ms;
  uint16_t tty_rows;
  uint16_t tty_cols;
  uint8_t tty_ansi;  // 0/1
  uint8_t pad0;
  uint16_t custom_len;
  uint32_t custom_list[UI_MAX_CUSTOM_SCAN];
};                                                      // configuration file version 1 file format
static const char* CFG_PATH = "/vfo.cfg";               // Config location in flash
static const uint32_t CFG_MAGIC = 0x52504631;           // 'RPF1'
static const uint16_t CFG_VERSION = 1;                  // Configuration file format version
static bool sCfgLoaded = false;                         // is configuration file loaded
static bool sCfgDirty = false;                          // is configuration file dirty
static unsigned long sCfgDirtySince = 0;                // time in milliseconds since config was dirty
static unsigned long sCfgLastSave = 0;                  // time in milliseconds since config was last saved
static const unsigned long CFG_MIN_SAVE_GAP_MS = 1500;  // minimum gap in milliseconds between config writes
static const unsigned long CFG_DEBOUNCE_MS = 5000;      // delay after last change before saving config

// ==================== Peaks persistence (LittleFS) ====================
struct PeaksV1 {
  uint32_t magic;
  uint16_t ver;
  uint8_t scan_src;  // 0=HARD, 1=CUSTOM (metadata only)
  uint8_t reserved;
  PeakEntry entries[3];
};                                                        // peaks file version 1 file format
static const char* PEAKS_PATH = "/peaks.bin";             // peaks location in flash
static const uint32_t PEAKS_MAGIC = 0x504B5331;           // 'PKS1'
static const uint16_t PEAKS_VERSION = 1;                  // peaks file format version
static bool sPeaksLoaded = false;                         // is peaks file loaded
static bool sPeaksDirty = false;                          // is peaks file dirty
static unsigned long sPeaksDirtySince = 0;                // time in milliseconds since peaks were dirty
static unsigned long sPeaksLastSave = 0;                  // time in milliseconds since peaks were last saved
static const unsigned long PEAKS_MIN_SAVE_GAP_MS = 1500;  // minimum gap in milliseconds between peaks writes
static const unsigned long PEAKS_DEBOUNCE_MS = 5000;      // delay after last change before saving peaks

// ==================== Watchdog timer ====================
enum WDTResetReason : uint8_t {
  WDT_REASON_NONE = 0,
  WDT_REASON_TIMEOUT = 1,   // WATCHDOG_REASON_TIMER
  WDT_REASON_FORCE = 2,     // WATCHDOG_REASON_FORCE (software-forced reboot)
  WDT_REASON_MULTIPLE = 3,  // More than one flag set
  WDT_REASON_UNKNOWN = 255
};                                                   // Map the RP2040 WATCHDOG_REASON bitfield into a simple enum
static constexpr uint32_t WDT_TIMEOUT_ESCALATE = 3;  // After 3 consecutive TIMEOUT boots
constexpr uint32_t WDT_TIMEOUT_MS = 1750;            // Configure your watchdog timeout (milliseconds)

// ==================== Boot Animations ====================
void animateTextToCenterAndCompress(Adafruit_SSD1306& d, const char* text, int startX, int startY, uint16_t moveMs, uint16_t compressMs, float minScaleY, float maxStretchX);
void bootExplosion(Adafruit_SSD1306& d, uint16_t vibrateMs);

// ==================== Setup ====================
void setup() {
  // ==================== Serial init ====================
  Serial.begin(BAUD);
  if (WAIT_USB_SERIAL) {
    while (!Serial) {}
    delay(500);
  } else {
    delay(3000);
  }
  Serial.printf("\n\r\nRP2040Zero (BOOT):\tFW VER: %s\n\r", FW_VERSION);
  Serial.println("RP2040Zero (BOOT):\tStarting now..");

  // ==================== Watchdog init ====================
  // Read raw reason flags and whether it was a watchdog-caused reboot
  Serial.println("RP2040Zero (WDT):\tInitializing WDT...");
  uint32_t raw_reason = watchdog_hw->reason;      // Bitfield from hardware
  bool caused_by_wdt = watchdog_caused_reboot();  // Convenience boolean
  Serial.print("RP2040Zero (WDT):\twatchdog_hw->reason = 0x");
  Serial.println(raw_reason, HEX);
  if (caused_by_wdt) {
    WDTResetReason reason = classify_wdt_reason(raw_reason);
    handle_wdt_reason_switch(reason);
  } else {
    Serial.println("RP2040Zero (WDT):\tPrevious reset was not from Watchdog (power-on, reset pin, etc.).");
  }
  watchdog_update();
  // Start the watchdog; pause_on_debug=true so it won’t reset while halted in a debugger
  watchdog_enable(WDT_TIMEOUT_MS, /*pause_on_debug=*/true);
  Serial.printf("RP2040Zero (WDT):\tWatchdog enabled with %d ms timeout.\n", WDT_TIMEOUT_MS);
  watchdog_update();

  // ==================== I2C init ====================
  Serial.println("RP2040Zero (BOOT):\tReached I2C init");
  Wire.setClock(1000000);
  Wire.begin();
  watchdog_update();

  // ==================== Display init ====================
  Serial.println("RP2040Zero (BOOT):\tReached display init");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D /*0x3C*/);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();
  display.setTextSize(1);
  watchdog_update();

  // ==================== I2C detection ====================
  Serial.println("RP2040Zero (BOOT):\tReached I2C device detection");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("RP2040 I2C device(s):");
  display.setCursor(0, 12);
  Serial.print("RP2040Zero (BOOT):\tDetected I2C device(s): ");
  int i2cDevsFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    watchdog_update();
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (i2cDevsFound >= 1) {
        Serial.print(", 0x");
        Serial.print(addr, HEX);
      } else {
        Serial.print("0x");
        Serial.print(addr, HEX);
      }
      display.print("0x");
      display.print(addr, HEX);
      display.print(" ");
      display.display();
      i2cDevsFound++;
    }
  }
  Serial.println();
  delay(1000);
  watchdog_update();

  // ==================== LED init ====================
  Serial.println("RP2040Zero (BOOT):\tReached WS2812 init");
  strip.begin();
  strip.show();
  strip.setBrightness(64);
  strip.setPixelColor(0, 64, 64, 64);
  strip.show();
  watchdog_update();

  // ==================== Tuner init ====================
  Serial.println("RP2040Zero (BOOT):\tReached tuner init");
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  watchdog_update();

  // ==================== Rotary init ====================
  Serial.println("RP2040Zero (BOOT):\tReached rotary encoder init");
  pinMode(rotLeft, INPUT_PULLUP);
  pinMode(rotRight, INPUT_PULLUP);
  {
    uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
    uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
    encPrev = (a << 1) | b;
    encAccum = 0;
  }
  watchdog_update();

  // ==================== Button init ====================
  Serial.println("RP2040Zero (BOOT):\tReached button pin state init");
  pinMode(TUNESTEP_PIN, INPUT_PULLUP);
  pinMode(BAND_PIN, INPUT_PULLUP);
  pinMode(RX_TX_PIN, INPUT_PULLUP);
  watchdog_update();

  // ==================== Tuner variables init ====================
  Serial.println("RP2040Zero (BOOT):\tReached tuner variables init");
  count = BAND_INIT;
  bandpresets();
  stp = STEP_INIT;
  setstep();
  watchdog_update();

  // ==================== Config persistent ====================
  // Note: We do not auto-enable the TUI at boot even if saved; user can TTY ON.
  // Saved ttyRows/ttyCols/ANSI are already applied to globals.
  Serial.println("RP2040Zero (BOOT):\tReached config file persistence");
  cfgInitAndLoad();
  watchdog_update();

  // ==================== Peaks persistence ====================
  Serial.println("RP2040Zero (BOOT):\tReached peak file persistence");
  peaksInitAndLoad();
  watchdog_update();

  // ==================== Boot Animations ====================
  Serial.println("RP2040Zero (BOOT):\tReached boot animations");
  display.setCursor(0, 0);
  animateTextToCenterAndCompress(display, "VFO Generator", 25, 0, 150, 150, 0.3f, 0.025f);
  watchdog_update();
  bootExplosion(display, 400);
  watchdog_update();

  // ==================== Boot process finished ====================
  Serial.println("RP2040Zero (BOOT):\tReached boot finalization stage");
  delay(750);
  // WS2812 WRGB OLED now indicates successful boot
  strip.setBrightness(32);
  strip.setPixelColor(0, 64, 0, 32);
  strip.show();
  Serial.print("RP2040Zero (BOOT):\tRunning loop now..\n\rRP2040Zero (BOOT):\tHELP | ? | H for help dialog\n\r> ");
  watchdog_update();

  // ==================== Boot process finished ====================
  wdt_mark_boot_ok();  // <-- reset consecutive TIMEOUT counter on a clean boot
}
// ==================== Loop ====================
void loop() {
  watchdog_update();  // ==================== Feed the watchdog regularly to prevent a reset ====================

  // ==================== Pass USB serial data into our command handler ====================
  if (readLine(Serial, sUsbBuf, sUsbLen, CMD_BUF_SZ)) {
    handleCommand(sUsbBuf, Serial);
  }
  watchdog_update();

  cfgTick();  // ==================== Flush pending config saves with rate-limits to reduce wear ====================
  watchdog_update();
  peaksTick();  // ==================== Flush pending peaks saves with rate-limits to reduce wear ====================
  watchdog_update();
  pollEncoder();  // ==================== Rotary encoder polling ====================
  watchdog_update();
  uiScanTick();  // ==================== Scanner tick ====================
  watchdog_update();
  pwrTick();  // ==================== AD8307 meter tick ====================
  watchdog_update();
  tuiTick();  // ==================== Terminal UI tick ====================
  watchdog_update();

  // ==================== Frequency changed, retune now ====================
  if (freqold != freq) {
    time_now = millis();
    tunegen();
    freqold = freq;
    tuiMarkDirty();  // reflect changes to TUI
  }
  watchdog_update();

  // ==================== Inter-Frequency changed, retune now ====================
  if (interfreqold != interfreq) {
    time_now = millis();
    if (!uiScanOn && interfreq != lastSentIF) {  // don't push IF in scan
      lastSentIF = interfreq;
    }
    tunegen();  // early-returns when uiScanOn
    interfreqold = interfreq;
    tuiMarkDirty();  // reflect changes to TUI
  }
  watchdog_update();

  // ==================== SM changed, adjust SM now ====================
  if (sigmeterOld != sigmeter) {
    time_now = millis();
    sigmeterOld = sigmeter;
    tuiMarkDirty();  // reflect changes to TUI
  }
  watchdog_update();

  // ==================== Tune step size cycle button ====================
  if (digitalRead(TUNESTEP_PIN) == LOW) {
    time_now = (millis() + 175);
    setstep();
    delay(175);
  }
  watchdog_update();

  // ==================== Band preset cycle button ====================
  if (digitalRead(BAND_PIN) == LOW) {
    time_now = (millis() + 175);
    inc_preset();
    delay(175);
  }
  watchdog_update();

  // ==================== TX/RX button ====================
  if (digitalRead(RX_TX_PIN) == LOW) {
    time_now = (millis() + 175);
    sts = 1;
  } else sts = 0;
  watchdog_update();

  // ==================== Limit display ticks ====================
  static unsigned long lastDraw = 0;     // last time since redraw in milliseconds
  const unsigned long drawPeriod = 100;  // redraw time in milliseconds
  if (millis() - lastDraw >= drawPeriod) {
    lastDraw = millis();
    displayfreq();
    watchdog_update();
    layout();
    watchdog_update();
  }
}

// ==================== UI Scanner ====================
static inline size_t uiCurrentListLen() {
  return uiScanSrc ? uiCustomScanLen : uiHardScanLen;
}
static inline const uint32_t* uiCurrentListPtr() {
  return uiScanSrc ? uiCustomScanList : uiHardScanListMURS;
}
static inline const uint32_t* uiCurrentListPtr(const String& band) {
  // String overload
  STR_SWITCH(band)
  STR_CASE("FM") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListFM;
  }
  STR_CASE("NOAA") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListNOAA;
  }
  STR_CASE("CB") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListCB;
  }
  STR_DEFAULT {
    return uiScanSrc ? uiCustomScanList : uiHardScanListCB;
  }
  STR_SWITCH_END;
}
static inline const uint32_t* uiCurrentListPtr(const char* band) {
  // C-string overload
  STR_SWITCH(band)
  STR_CASE("FM") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListFM;
  }
  STR_CASE("NOAA") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListNOAA;
  }
  STR_CASE("CB") {
    return uiScanSrc ? uiCustomScanList : uiHardScanListCB;
  }
  STR_DEFAULT {
    return uiScanSrc ? uiCustomScanList : uiHardScanListCB;
  }
  STR_SWITCH_END;
}
static inline void uiScanUse(uint8_t src) {
  uiScanSrc = src ? 1 : 0;
}
static void uiScanStart() {
  if (uiCurrentListLen() == 0) return;
  uiScanOn = true;
  uiScanIdx = 0;
  uiScanLastMs = millis() - uiScanDelayMs;
  scanPeaksReset();  // start fresh peaks for this scan session
  tuiMarkDirty();    // update TUI panel
}
static void uiScanStop() {
  uiScanOn = false;
}
static void sendScanStatus() {
  if (uiScanOn) {
    Serial.println("Scanning");
  } else {
    Serial.println("Idling");
  }
}
static void uiScanTick() {
  if (!uiScanOn) return;
  if (sts) return;

  size_t len = uiCurrentListLen();
  if (len == 0) {
    uiScanOn = false;
    return;
  }
  uint32_t now = millis();
  if ((now - uiScanLastMs) >= uiScanDelayMs) {
    uiScanLastMs = now;
    const uint32_t* list = uiCurrentListPtr();
    uiScanCurrFreqHz = list[uiScanIdx];
    freq = uiScanCurrFreqHz;
    uiScanIdx = (uiScanIdx + 1) % len;
  }
}
static inline unsigned long uiDisplayFreqHz() {
  if (uiScanOn) {
    if (uiScanCurrFreqHz != 0) return uiScanCurrFreqHz;
    if (uiCurrentListLen() > 0) return uiCurrentListPtr()[0];
  }
  return freq;
}
static void drawScanUiOverlay() {
  if (!uiScanOn) return;
  display.setTextSize(1);
  display.setCursor(0, -2);
  display.print("s");
  size_t len = uiCurrentListLen();
  if (len > 0) {
    const int barX0 = 96;
    const int barW = 31;
    const int barY = 0;
    display.drawRect(barX0, barY, barW, 2, WHITE);
    int fillW = 0;
    if (uiScanIdx > 0) {
      float p = (float)uiScanIdx / (float)len;
      if (p > 1.0f) p = 1.0f;
      fillW = (int)((barW - 2) * p);
    }
    if (fillW > 0) display.fillRect(barX0 + 1, barY + 1, fillW, 1, WHITE);
  }
}

// ==================== Tuner ====================
void set_frequency(short dir) {
  if (encoder == 1) {
    if (dir == 1) freq = freq + fstep;
    if (freq >= 225000000UL) freq = 225000000UL;
    if (dir == -1) freq = freq - fstep;
    if (fstep == 1000000 && freq <= 1000000) freq = 1000000;
    else if (freq < 10000) freq = 10000;
  }
  if (encoder == 1) {
    if (dir == 1) n = n + 1;
    if (n > 42) n = 1;
    if (dir == -1) n = n - 1;
    if (n < 1) n = 42;
  }
}
void tunegen() {
  uint32_t ifHz = sts ? 0UL : (uint32_t)interfreq * 1000UL;
  uint64_t outCentiHz = ((uint64_t)freq + (uint64_t)ifHz) * 100ULL;
  si5351.set_freq(outCentiHz, SI5351_CLK0);
}
static void formatFreqSmart(uint32_t hz, char* out, size_t outSz) {
  // Formats Hz as "### Hz", "###.### kHz", or "###.### MHz" (trims trailing zeros)
  if (hz < 1000UL) {
    snprintf(out, outSz, "%lu Hz", (unsigned long)hz);
    return;
  }
  if (hz < 1000000UL) {  // kHz
    uint32_t k = hz / 1000UL;
    uint32_t rem = hz % 1000UL;  // Hz remainder -> fractional kHz
    if (rem == 0) {
      snprintf(out, outSz, "%lu kHz", (unsigned long)k);
    } else {
      char frac[4];
      snprintf(frac, sizeof(frac), "%03lu", (unsigned long)rem);  // 3 decimals
      int len = 3;
      while (len > 0 && frac[len - 1] == '0') frac[--len] = '\0';
      snprintf(out, outSz, "%lu.%s kHz", (unsigned long)k, frac);
    }
    return;
  }
  // MHz
  uint32_t m = hz / 1000000UL;
  uint32_t rem = hz % 1000000UL;  // 0..999999 Hz -> fractional MHz
  if (rem == 0) {
    snprintf(out, outSz, "%lu MHz", (unsigned long)m);
  } else {
    // Show kHz as the fractional part (3 decimals), trim trailing zeros
    uint32_t frac = rem / 1000UL;  // 0..999 kHz
    char frac3[4];
    snprintf(frac3, sizeof(frac3), "%03lu", (unsigned long)frac);
    int len = 3;
    while (len > 0 && frac3[len - 1] == '0') frac3[--len] = '\0';
    if (len == 0) {
      snprintf(out, outSz, "%lu MHz", (unsigned long)m);
    } else {
      snprintf(out, outSz, "%lu.%s MHz", (unsigned long)m, frac3);
    }
  }
}
void displayfreq() {
  unsigned long df = uiDisplayFreqHz();
  unsigned int m = df / 1000000UL;
  unsigned int k = (df % 1000000UL) / 1000UL;
  unsigned int h = (df % 1000UL) / 1UL;

  display.clearDisplay();
  display.setTextSize(1);

  char buffer[15] = "";
  //display.setCursor(5, 1);
  //sprintf(buffer, "%2d.%003d.%003d", m, k, h);
  if (m < 1) {
    display.setCursor(41, 1);
    sprintf(buffer, "%003d.%003d", k, h);
  } else if (m < 100) {
    display.setCursor(5, 1);
    sprintf(buffer, "%2d.%003d.%003d", m, k, h);
  } else if (m >= 100) {
    unsigned int h2 = (df % 1000UL) / 10UL;
    display.setCursor(5, 1);
    sprintf(buffer, "%2d.%003d.%02d", m, k, h2);
  }
  display.print(buffer);
}
void setstep() {
  switch (stp) {
    case 1:
      stp = 2;
      fstep = 1;
      break;
    case 2:
      stp = 3;
      fstep = 10;
      break;
    case 3:
      stp = 4;
      fstep = 100;
      break;
    case 4:
      stp = 5;
      fstep = 1000;
      break;
    case 5:
      stp = 6;
      fstep = 10000;
      break;
    case 6:
      stp = 7;
      fstep = 100000;
      break;
    case 7:
      stp = 8;
      fstep = 1000000;
      break;
    case 8:
      stp = 1;
      fstep = 10000000;
      break;
  }
}
void inc_preset() {
  count++;
  if (count > 21) count = 1;
  bandpresets();
  delay(50);
}
void bandpresets() {
  switch (count) {
    case 1: freq = 100000; break;
    case 2: freq = 800000; break;
    case 3: freq = 1800000; break;
    case 4: freq = 3650000; break;
    case 5: freq = 4985000; break;
    case 6: freq = 6180000; break;
    case 7: freq = 7200000; break;
    case 8: freq = 10000000; break;
    case 9: freq = 11780000; break;
    case 10: freq = 13630000; break;
    case 11: freq = 14100000; break;
    case 12: freq = 15000000; break;
    case 13: freq = 17655000; break;
    case 14: freq = 21525000; break;
    case 15: freq = 27025000; break;
    case 16: freq = 28400000; break;
    case 17: freq = 50000000; break;
    case 18: freq = 107700000; break;
    case 19: freq = 130000000; break;
    case 20: freq = 144000000; break;
    case 21: freq = 220000000; break;
  }
  stp = 4;
  setstep();
}
void layout() {
  display.setTextColor(WHITE);
  //display.drawLine(0, 20, 127, 20, WHITE);
  //display.drawLine(0, 43, 127, 43, WHITE);
  //display.drawLine(105, 24, 105, 39, WHITE);
  //display.drawLine(87, 24, 87, 39, WHITE);
  //display.drawLine(87, 48, 87, 63, WHITE);
  //display.drawLine(15, 55, 82, 55, WHITE);
  display.setTextSize(1);
  //display.setCursor(59, 23);
  //display.print("STEP");
  display.setCursor(51, 33);
  if (stp == 2) {
    display.print("  1Hz");
    display.drawLine(114, 17, 122, 17, WHITE);
  }
  if (stp == 3) {
    display.print(" 10Hz");
    display.drawLine(102, 17, 110, 17, WHITE);
  }
  if (stp == 4) {
    display.print(" 100Hz");
    display.drawLine(78, 17, 86, 17, WHITE);
  }
  if (stp == 5) {
    display.print(" 1kHz");
    display.drawLine(66, 17, 74, 17, WHITE);
  }
  if (stp == 6) {
    display.print("10kHz");
    display.drawLine(54, 17, 62, 17, WHITE);
  }
  if (stp == 7) {
    display.print("100kHz");
    display.drawLine(30, 17, 38, 17, WHITE);
  }
  if (stp == 8) {
    display.print(" 1MHz");
    display.drawLine(18, 17, 26, 17, WHITE);
  }
  if (stp == 1) {
    display.print(" 10MHz");
    display.drawLine(6, 17, 14, 17, WHITE);
  }

  /*display.setTextSize(1);
  display.setCursor(92, 48);
  display.print("IF:");
  display.setCursor(92, 57);
  display.print(interfreq);
  display.print("k");

  display.setTextSize(1);
  display.setCursor(110, 23);
  if (freq < 1000000) display.print("kHz");
  if (freq >= 1000000) display.print("MHz");
  display.setCursor(110, 33);
  if (interfreq == 0) display.print("VFO");
  if (interfreq != 0) display.print("L O");*/

  display.setCursor(91, 28);
  if (!sts) display.print("RX");
  if (!sts) interfreq = IF;
  if (sts) display.print("TX");
  if (sts) interfreq = 0;

  bandlist();
  drawbargraph();
  drawScanUiOverlay();
  display.display();
}
void bandlist() {
  /*display.setTextSize(2);
  display.setCursor(0, 25);
  if (count == 1) display.print("GEN");
  if (count == 2) display.print("MW");
  if (count == 3) display.print("160m");
  if (count == 4) display.print("80m");
  if (count == 5) display.print("60m");
  if (count == 6) display.print("49m");
  if (count == 7) display.print("40m");
  if (count == 8) display.print("31m");
  if (count == 9) display.print("25m");
  if (count == 10) display.print("22m");
  if (count == 11) display.print("20m");
  if (count == 12) display.print("19m");
  if (count == 13) display.print("16m");
  if (count == 14) display.print("13m");
  if (count == 15) display.print("11m");
  if (count == 16) display.print("10m");
  if (count == 17) display.print("6m");
  if (count == 18) display.print("WFM");
  if (count == 19) display.print("AIR");
  if (count == 20) display.print("2m");
  if (count == 21) display.print("1m");*/
  if (count == 1) interfreq = 0;
  else if (!sts) interfreq = IF;
}
void drawbargraph() {
  byte y = map(n, 1, 42, 1, 14);
  display.setTextSize(1);

  // show dBm and power (using smoothed shared state)
  display.setCursor(0, 46);
  display.print(gDbmSmooth, 1);  // dBm value, 1 decimal
  display.print("dBm ");
  display.print(gPowerStr);  // compact power string

  /*display.setCursor(0, 57);
  display.print("SM");
  switch (sigmeter) {
    case 14: display.fillRect(80, 58, 2, 6, WHITE);
    case 13: display.fillRect(75, 58, 2, 6, WHITE);
    case 12: display.fillRect(70, 58, 2, 6, WHITE);
    case 11: display.fillRect(65, 58, 2, 6, WHITE);
    case 10: display.fillRect(60, 58, 2, 6, WHITE);
    case 9: display.fillRect(55, 58, 2, 6, WHITE);
    case 8: display.fillRect(50, 58, 2, 6, WHITE);
    case 7: display.fillRect(45, 58, 2, 6, WHITE);
    case 6: display.fillRect(40, 58, 2, 6, WHITE);
    case 5: display.fillRect(35, 58, 2, 6, WHITE);
    case 4: display.fillRect(30, 58, 2, 6, WHITE);
    case 3: display.fillRect(25, 58, 2, 6, WHITE);
    case 2: display.fillRect(20, 58, 2, 6, WHITE);
    case 1: display.fillRect(15, 58, 2, 6, WHITE);
    default: display.fillRect(15, 58, 2, 6, WHITE);
  }*/
}

// ==================== Configuration file handlers ====================
static inline unsigned long stepIndexToFstep(uint8_t stpIdx) {
  // Helper: map step index -> step size (Hz), mirrors your setstep() table without cycling
  switch (stpIdx) {
    case 2: return 1UL;
    case 3: return 10UL;
    case 4: return 1000UL;
    case 5: return 5000UL;
    case 6: return 10000UL;
    case 7: return 100000UL;
    case 8: return 1000000UL;
    case 1: return 10000000UL;
    default: return 1000UL;  // safe default: 1 kHz
  }
}
static inline uint32_t clampHz(uint32_t hz) {
  if (hz < 10000UL) return 10000UL;
  if (hz > 225000000UL) return 225000000UL;
  return hz;
}
static inline int32_t clampIfKHz(int32_t ifk) {
  if (ifk < 0) return 0;
  if (ifk > 200000) return 200000;
  return ifk;
}
static inline uint32_t clampScanDelay(uint32_t ms) {
  if (ms < 1UL) return 1UL;
  if (ms > 10000UL) return 10000UL;
  return ms;
}
static inline uint16_t clampRows(uint16_t r) {
  if (r < 12) return 12;
  if (r > 80) return 80;
  return r;
}
static inline uint16_t clampCols(uint16_t c) {
  if (c < 40) return 40;
  if (c > 200) return 200;
  return c;
}
static void cfgApply(const CfgV1& c) {
  // Sanitize first
  uint32_t fHz = clampHz(c.freq_hz);
  int32_t ifKHz = clampIfKHz(c.if_khz);
  uint8_t stpIdx = (c.stp_idx >= 1 && c.stp_idx <= 8) ? c.stp_idx : 4;
  uint8_t bc = (c.band_count >= 1 && c.band_count <= 21) ? c.band_count : count;
  uint8_t smx = (c.s_meter_x >= 1 && c.s_meter_x <= 14) ? c.s_meter_x : sigmeter;
  uint8_t scanSrc = (c.scan_src ? 1 : 0);
  uint32_t scanDelay = clampScanDelay(c.scan_delay_ms);
  uint16_t cr = clampRows(c.tty_rows);
  uint16_t cc = clampCols(c.tty_cols);
  uint16_t clen = c.custom_len;
  if (clen > UI_MAX_CUSTOM_SCAN) clen = UI_MAX_CUSTOM_SCAN;

  // Apply to your globals
  freq = fHz;
  interfreq = ifKHz;
  stp = stpIdx;
  fstep = stepIndexToFstep(stpIdx);
  count = bc;
  sigmeter = smx;
  uiScanSrc = scanSrc;
  uiScanDelayMs = scanDelay;
  tuiAnsi = (c.tty_ansi != 0);
  tuiRows = cr;
  tuiCols = cc;

  uiCustomScanLen = clen;
  for (size_t i = 0; i < clen; ++i) {
    uiCustomScanList[i] = clampHz(c.custom_list[i]);
  }
}
static void cfgFillFromGlobals(CfgV1& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.ver = CFG_VERSION;
  c.freq_hz = clampHz(freq);
  c.if_khz = clampIfKHz(interfreq);
  c.stp_idx = stp;
  c.band_count = count;
  c.s_meter_x = (sigmeter >= 1 && sigmeter <= 14) ? sigmeter : 1;
  c.scan_src = uiScanSrc ? 1 : 0;
  c.scan_delay_ms = clampScanDelay(uiScanDelayMs);
  c.tty_rows = clampRows(tuiRows);
  c.tty_cols = clampCols(tuiCols);
  c.tty_ansi = tuiAnsi ? 1 : 0;
  c.custom_len = (uiCustomScanLen <= UI_MAX_CUSTOM_SCAN) ? uiCustomScanLen : UI_MAX_CUSTOM_SCAN;
  for (size_t i = 0; i < c.custom_len; ++i) c.custom_list[i] = clampHz(uiCustomScanList[i]);
}
static bool cfgLoad() {
  if (!LittleFS.begin()) {
    Serial.println("\rRP2040Zero (CFG):\tLittleFS mount failed\n\r> ");
    return false;
  }
  if (!LittleFS.exists(CFG_PATH)) {
    Serial.println("\rRP2040Zero (CFG):\tNo config file, using defaults\n\r> ");
    return false;
  }
  File f = LittleFS.open(CFG_PATH, "r");
  if (!f) {
    Serial.println("\rRP2040Zero (CFG):\tOpen failed\n\r> ");
    return false;
  }
  CfgV1 c;
  size_t got = f.read((uint8_t*)&c, sizeof(c));
  f.close();
  if (got < offsetof(CfgV1, custom_list)) {
    Serial.println("\rRP2040Zero (CFG):\tShort read\n\r> ");
    return false;
  }
  if (c.magic != CFG_MAGIC || c.ver != CFG_VERSION) {
    Serial.println("\rRP2040Zero (CFG):\tBad magic/version\n\r> ");
    return false;
  }
  cfgApply(c);
  sCfgLoaded = true;
  Serial.printf("\rRP2040Zero (CFG):\tLoaded configuration at: %s\n", CFG_PATH);
  return true;
}
static bool cfgSaveNow() {
  if (!LittleFS.begin()) {
    Serial.println("\rRP2040Zero (CFG):\tFS mount failed (save)\n\r> ");
    return false;
  }
  CfgV1 c;
  cfgFillFromGlobals(c);
  File f = LittleFS.open(CFG_PATH, "w");
  if (!f) {
    Serial.println("\rRP2040Zero (CFG):\tOpen for write failed\n\r> ");
    return false;
  }
  size_t want = sizeof(c);
  size_t wr = f.write((const uint8_t*)&c, want);
  f.flush();
  f.close();
  if (wr != want) {
    Serial.println("\rRP2040Zero (CFG):\tShort write\n\r> ");
    return false;
  }
  sCfgLastSave = millis();
  Serial.print("\rRP2040Zero (CFG):\tSaved\n\r> ");
  return true;
}
static void cfgMarkDirty() {
  sCfgDirty = true;
  sCfgDirtySince = millis();
}
static void cfgInitAndLoad() {
  (void)cfgLoad();  // okay if it fails; we keep defaults
  sCfgDirty = false;
  sCfgDirtySince = 0;
  sCfgLastSave = 0;
}
static void cfgTick() {
  if (!sCfgDirty) return;
  unsigned long now = millis();
  if ((now - sCfgDirtySince) >= CFG_DEBOUNCE_MS && (now - sCfgLastSave) >= CFG_MIN_SAVE_GAP_MS) {
    if (cfgSaveNow()) {
      sCfgDirty = false;
    }
  }
}

// ==================== Rotary Encoder ====================
inline void pollEncoder() {
  static uint32_t lastStepMs = 0;
  const uint8_t a = (digitalRead(rotRight) == LOW);
  const uint8_t b = (digitalRead(rotLeft) == LOW);
  const uint8_t curr = (a << 1) | b;
  const uint8_t idx = (encPrev << 2) | curr;
  const int8_t delta = encTable[idx];
  if (delta != 0) {
    encAccum += delta;
    if (encAccum >= 4 || encAccum <= -4) {
      if (millis() - lastStepMs >= 2) {  // small throttle if needed
        set_frequency(encAccum > 0 ? 1 : -1);
        lastStepMs = millis();
      }
      encAccum = 0;
    }
  }
  encPrev = curr;
}

// ==================== TUI section start ====================
static void sPrintf(Stream& s, const char* fmt, ...) {
  // Small printf helper for Streams
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  s.write((const uint8_t*)buf, strlen(buf));
}
// ==================== TUI layout utilities (right-side panel) ====================
static inline uint16_t tuiPanelX() {
  // 1-based column where the panel starts
  return (tuiCols > tuiPanelW) ? (tuiCols - tuiPanelW + 1) : 1;
}
static inline uint16_t tuiLogWidth() {
  // width available to the left-side console
  uint16_t px = tuiPanelX();
  if (px <= 2) return tuiCols;  // no space for a gap -> use full width
  return (uint16_t)(px - 2);    // keep 1-col gutter between log and panel
}
static void clearRegion(Stream& s, uint16_t r, uint16_t c, uint16_t w) {
  // Clear a rectangular region width 'w' starting at (row r, col c)
  if (w == 0) return;
  vtMove(s, r, c);
  for (uint16_t i = 0; i < w; ++i) s.write(' ');
}
// ==================== Simple VT100 helpers ====================
static void vt(Stream& s, const char* seq) {
  if (tuiAnsi) s.print(seq);
}
static void vtAltOn(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?1049h");
}
static void vtAltOff(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?1049l");
}
static void vtClear(Stream& s) {
  if (tuiAnsi) s.print("\x1B[2J\x1B[H");
}
static void vtHideCur(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?25l");
}
static void vtShowCur(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?25h");
}
static inline uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static void vtMove(Stream& s, uint16_t r, uint16_t c) {
  if (!tuiAnsi) return;
  if (tuiRows == 0 || tuiCols == 0) return;
  r = clamp16(r, 1, tuiRows);
  c = clamp16(c, 1, tuiCols);
  char buf[24];
  int n = snprintf(buf, sizeof(buf), "\x1B[%u;%uH", (unsigned)r, (unsigned)c);
  s.write((const uint8_t*)buf, (size_t)n);
}
static void vtWrapOff(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?7l");
}
static void vtWrapOn(Stream& s) {
  if (tuiAnsi) s.print("\x1B[?7h");
}
static void vtClrEol(Stream& s) {
  if (tuiAnsi) s.print("\x1B[K");
}
static void vtRevOn(Stream& s) {
  if (tuiAnsi) s.print("\x1B[7m");
}
static void vtRevOff(Stream& s) {
  if (tuiAnsi) s.print("\x1B[27m");
}
// ==================== Log pane (ring buffer) ====================
static void tuiLogLine(const char* line) {
  if (!line) return;
  strncpy(tuiLog[tuiLogHead], line, TUI_LINE_MAX - 1);
  tuiLog[tuiLogHead][TUI_LINE_MAX - 1] = '\0';
  tuiLogHead = (tuiLogHead + 1) % TUI_LOG_MAX;
  if (tuiLogCount < TUI_LOG_MAX) tuiLogCount++;
  tuiDirtyDyn = true;
}
static void tuiLogf(const char* fmt, ...) {
  char tmp[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  // Wrap to the visible console width on the left side (leave small margin for indent)
  const uint16_t logCols = tuiLogWidth();
  const uint16_t visCols = (logCols > 2) ? (logCols - 2) : logCols;

  const char* p = tmp;
  while (*p) {
    const char* nl = strchr(p, '\n');
    size_t segLen = nl ? (size_t)(nl - p) : strlen(p);

    size_t off = 0;
    while (off < segLen) {
      size_t take = segLen - off;
      if (take > visCols) take = visCols;

      char line[TUI_LINE_MAX];
      size_t copy = (take < sizeof(line) - 1) ? take : (sizeof(line) - 1);
      memcpy(line, p + off, copy);
      line[copy] = '\0';
      tuiLogLine(line);
      off += take;
    }

    if (!nl) break;
    if (segLen == 0) tuiLogLine("");
    p = nl + 1;
  }
}
// ==================== TUI draw helpers ====================
static void tuiComputeRows() {
  tuiInputRow = tuiRows;
  tuiLogBottom = (tuiRows >= 2) ? (tuiRows - 2) : tuiRows;
  tuiLogTop = (tuiRows >= 12) ? 12 : (tuiRows / 2);
}
static const char* stepToStr() {
  switch (stp) {
    case 2: return "1 Hz";
    case 3: return "10 Hz";
    case 4: return "1 kHz";
    case 5: return "5 kHz";
    case 6: return "10 kHz";
    case 7: return "100 kHz";
    case 8: return "1 MHz";
    case 1: return "10 MHz";
    default: return "?";
  }
}
static void drawHeader(Stream& s) {
  const uint16_t px = tuiPanelX();
  clearRegion(s, 1, px, tuiPanelW);
  vtMove(s, 1, px);
  vtRevOn(s);
  char fbuf[32];
  formatFreqSmart(uiDisplayFreqHz(), fbuf, sizeof(fbuf));
  // Trim text if longer than panel width
  char hdr[96];
  snprintf(hdr, sizeof(hdr), "  RP2040 VFO + VFO Scanner | %s | Mode: %s  ", fbuf, (sts ? "TX" : "RX"));
  size_t len = strlen(hdr);
  if (len > tuiPanelW) hdr[tuiPanelW] = '\0';
  s.print(hdr);
  vtRevOff(s);
}
static void drawScanRow(Stream& s) {
  const uint16_t px = tuiPanelX();
  clearRegion(s, 2, px, tuiPanelW);
  vtMove(s, 2, px);
  const size_t len = uiCurrentListLen();
  size_t idx = uiScanIdx;
  if (idx >= len && len > 0) idx = len - 1;
  char line[128];
  snprintf(line, sizeof(line), " Scan:%s Source:%s Length:%lu Index:%lu Delay:%lu ",
           uiScanOn ? "ON" : "OFF",
           uiScanSrc ? "CUSTOM" : "HARD",
           (unsigned long)len,
           (unsigned long)idx,
           (unsigned long)uiScanDelayMs);
  // Trim
  if (strlen(line) > tuiPanelW) line[tuiPanelW] = '\0';
  s.print(line);
}
static void drawFreqBlock(Stream& s) {
  const uint16_t px = tuiPanelX();

  // Big frequency line at row 4
  clearRegion(s, 4, px, tuiPanelW);
  vtMove(s, 4, px);
  unsigned long df = uiDisplayFreqHz();
  unsigned int m = df / 1000000UL;
  unsigned int k = (df % 1000000UL) / 1000UL;
  unsigned int h = (df % 1000UL);
  char big[64];
  if (m < 1) {
    snprintf(big, sizeof(big), "%03u.%03u kHz", k, h);
  } else if (m < 100) {
    snprintf(big, sizeof(big), "%2u.%03u.%03u MHz", m, k, h);
  } else {
    unsigned int h2 = (df % 1000UL) / 10UL;
    snprintf(big, sizeof(big), "%2u.%03u.%02u MHz", m, k, h2);
  }
  vtRevOn(s);
  // Trim
  char out[64];
  snprintf(out, sizeof(out), " %s ", big);
  if (strlen(out) > tuiPanelW) out[tuiPanelW] = '\0';
  s.print(out);
  vtRevOff(s);

  // Band at row 6
  clearRegion(s, 6, px, tuiPanelW);
  vtMove(s, 6, px);
  s.print("Band: ");
  switch (count) {
    case 1: s.print("GEN"); break;
    case 2: s.print("MW"); break;
    case 3: s.print("160m"); break;
    case 4: s.print("80m"); break;
    case 5: s.print("60m"); break;
    case 6: s.print("49m"); break;
    case 7: s.print("40m"); break;
    case 8: s.print("31m"); break;
    case 9: s.print("25m"); break;
    case 10: s.print("22m"); break;
    case 11: s.print("20m"); break;
    case 12: s.print("19m"); break;
    case 13: s.print("16m"); break;
    case 14: s.print("13m"); break;
    case 15: s.print("11m"); break;
    case 16: s.print("10m"); break;
    case 17: s.print("6m"); break;
    case 18: s.print("WFM"); break;
    case 19: s.print("AIR"); break;
    case 20: s.print("2m"); break;
    case 21: s.print("1m"); break;
    default: s.print("?"); break;
  }

  // IF and STEP lines (rows 7 and 8) for quick glance
  clearRegion(s, 7, px, tuiPanelW);
  vtMove(s, 7, px);
  sPrintf(s, "IF: %lu kHz  STEP: %s", (unsigned long)interfreq, stepToStr());
}
static void drawBars(Stream& s) {
  const uint16_t px = tuiPanelX();
  // TU bar (n mapped 1..42 -> 1..14)
  /*byte tu = map(n, 1, 42, 1, 14);
  clearRegion(s, 9, px, tuiPanelW);
  vtMove(s, 9, px);
  s.print("TU ");
  for (int i = 1; i <= 14; ++i) s.print(i <= tu ? '|' : '.');

  // SM bar (x 1..14)
  clearRegion(s, 10, px, tuiPanelW);
  vtMove(s, 10, px);
  s.print("SM ");
  for (int i = 1; i <= 14; ++i) s.print(i <= sigmeter ? '|' : '.');*/

  // PWR bar + numeric readout (dbm + compact power)
  clearRegion(s, 11, px, tuiPanelW);
  vtMove(s, 11, px);
  s.print("PWR ");
  for (int i = 1; i <= 14; ++i) s.print(i <= gPwrBar ? '|' : '.');

  // trailing text: "  -12.3dBm 123.45uW" etc.
  char tail[64];
  snprintf(tail, sizeof(tail), "  %.1fdBm %s", (double)gDbmSmooth, gPowerStr);
  s.print(tail);
}
static void drawLog(Stream& s) {
  const uint16_t logW = tuiLogWidth();

  // "Console:" label one row above the log
  if (tuiLogTop > 1) {
    clearRegion(s, tuiLogTop - 1, 1, logW);
    vtMove(s, tuiLogTop - 1, 1);
    s.print("Console:");
  }

  // Determine which lines to show (tail of ring buffer)
  const uint16_t visRows = (tuiLogBottom >= tuiLogTop) ? (tuiLogBottom - tuiLogTop + 1) : 0;
  size_t oldest = (tuiLogHead + TUI_LOG_MAX - tuiLogCount) % TUI_LOG_MAX;
  size_t start = (tuiLogCount > visRows)
                   ? (oldest + (tuiLogCount - visRows)) % TUI_LOG_MAX
                   : oldest;

  for (uint16_t r = 0; r < visRows; ++r) {
    size_t idx = (start + r) % TUI_LOG_MAX;
    clearRegion(s, tuiLogTop + r, 1, logW);
    if (r < tuiLogCount) {
      vtMove(s, tuiLogTop + r, 1);
      s.write(' ');  // small indent
      // Draw clipped line if necessary
      const char* line = tuiLog[idx];
      size_t len = strlen(line);
      if (len > (logW - 2)) len = (logW - 2);
      for (size_t i = 0; i < len; ++i) s.write((uint8_t)line[i]);
    }
  }
}
static void drawInput(Stream& s) {
  const uint16_t logW = tuiLogWidth();
  clearRegion(s, tuiInputRow, 1, logW);
  vtMove(s, tuiInputRow, 1);
  s.print("> ");
  size_t showLen = sUsbLen;
  if (showLen > (logW - 3)) showLen = (logW - 3);
  for (size_t i = 0; i < showLen; ++i) s.write((uint8_t)sUsbBuf[i]);
  // Put cursor at end of input
  vtMove(s, tuiInputRow, 3 + showLen);
}
static void tuiEnter() {
  if (!tuiAnsi) return;  // refuse to enter without ANSI
  if (tuiInAlt) return;
  vtAltOn(Serial);
  vtClear(Serial);
  vtHideCur(Serial);
  tuiInAlt = true;
  tuiComputeRows();
  tuiDirtyFull = true;
}
static void tuiExit() {
  if (!tuiInAlt) return;
  vtShowCur(Serial);
  vtAltOff(Serial);
  tuiInAlt = false;
}
static void tuiSetEnabled(bool on) {
  if (on == tuiEnabled) return;
  tuiEnabled = on;
  gTermNoEcho = tuiEnabled;
  if (tuiEnabled) {
    tuiEnter();
    tuiLogf("TUI enabled (%ux%u).", (unsigned)tuiRows, (unsigned)tuiCols);
  } else {
    tuiExit();
  }
}
static void tuiRedrawFull() {
  if (!tuiEnabled) return;
  vtHideCur(Serial);
  vtClear(Serial);
  drawHeader(Serial);
  drawScanRow(Serial);
  drawScanPeaks(Serial);
  //drawFreqBlock(Serial);
  drawBars(Serial);
  drawLog(Serial);
  drawInput(Serial);  // sets caret position
  vtShowCur(Serial);
  tuiDirtyFull = false;
  tuiDirtyDyn = false;
}
static void tuiRedrawDynamic() {
  if (!tuiEnabled) return;
  vtHideCur(Serial);
  // No vtClear here; dynamic should be partial updates only
  drawHeader(Serial);
  drawScanRow(Serial);
  drawScanPeaks(Serial);
  //drawFreqBlock(Serial);
  drawBars(Serial);
  drawLog(Serial);
  drawInput(Serial);  // sets caret position
  vtShowCur(Serial);
  tuiDirtyDyn = false;
}
static inline void tuiDirtySet(uint16_t m) {
  tuiDirtyMask |= m;
}
static inline void tuiDirtyClear(uint16_t m) {
  tuiDirtyMask &= ~m;
}
static void tuiRedrawDirty() {
  if (!tuiEnabled || tuiDirtyMask == 0) return;
  vtHideCur(Serial);

  if (tuiDirtyMask & TUI_DIRTY_FULL) {
    vtClear(Serial);
    drawHeader(Serial);
    drawScanRow(Serial);
    drawScanPeaks(Serial);
    //drawFreqBlock(Serial);
    drawBars(Serial);
    drawLog(Serial);
    drawInput(Serial);
    tuiDirtyMask = 0;
    vtShowCur(Serial);
    return;
  }

  if (tuiDirtyMask & TUI_DIRTY_HEADER) {
    drawHeader(Serial);
    tuiDirtyClear(TUI_DIRTY_HEADER);
  }
  if (tuiDirtyMask & TUI_DIRTY_SCAN) {
    drawScanRow(Serial);
    drawScanPeaks(Serial);
    tuiDirtyClear(TUI_DIRTY_SCAN);
  }
  /*if (tuiDirtyMask & TUI_DIRTY_FREQ) {
    drawFreqBlock(Serial);
    tuiDirtyClear(TUI_DIRTY_FREQ);
  }*/
  if (tuiDirtyMask & TUI_DIRTY_BARS) {
    drawBars(Serial);
    tuiDirtyClear(TUI_DIRTY_BARS);
  }
  if (tuiDirtyMask & TUI_DIRTY_LOG) {
    drawLog(Serial);
    tuiDirtyClear(TUI_DIRTY_LOG);
  }
  if (tuiDirtyMask & TUI_DIRTY_INPUT) {
    drawInput(Serial);
    tuiDirtyClear(TUI_DIRTY_INPUT);
  }

  vtShowCur(Serial);
}
static void tuiMarkDirty() {
  tuiDirtyDyn = true;
}
static void tuiTick() {
  if (!tuiEnabled) return;

  if (tuiDirtyFull) {
    tuiRedrawFull();
    return;
  }

  if (tuiDirtyDyn) {
    tuiRedrawDynamic();
    return;
  }

  // Nothing to update: do not touch the terminal at all.
}
static void drawScanPeaks(Stream& s) {
  const uint16_t px = tuiPanelX();
  // We'll render at rows 3..5
  for (int r = 3; r <= 5; ++r) clearRegion(s, r, px, tuiPanelW);

  if (!uiScanOn) {
    vtMove(s, 3, px);
    s.print("Peaks: (idle)");
    return;
  }

  vtMove(s, 3, px);
  s.print("Peaks:");
  // Up to 3 entries
  for (int i = 0; i < 3; ++i) {
    const int row = 3 + i;
    clearRegion(s, row, px, tuiPanelW);
    vtMove(s, row, px);
    if (!gScanPeaks[i].valid) {
      s.printf("%d: --", i + 1);
      continue;
    }
    char pw[24];
    formatPowerShort(gScanPeaks[i].dbm, gScanPeaks[i].watts, pw, sizeof(pw));
    // Example line: "1:[12] -23.4 dBm (45.67uW)  162.550 MHz"
    char fbuf[32];
    formatFreqSmart(gScanPeaks[i].freqHz, fbuf, sizeof(fbuf));
    s.printf("%d:[%u] %.1f dBm (%s)  %s",
             i + 1,
             (unsigned)gScanPeaks[i].idx,
             (double)gScanPeaks[i].dbm,
             pw,
             fbuf);
  }
}
// ==================== Improved terminal line reader (echo-aware) ====================
static inline bool isPrintableAscii(char c) {
  return c >= 0x20 && c <= 0x7E;
}
static void termEraseChars(Stream& s, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    s.write('\b');
    s.write(' ');
    s.write('\b');
  }
}
bool readLine(Stream& s, char* outBuf, size_t& inoutLen, size_t outSz) {
  while (s.available()) {
    char c = (char)s.read();

    if (c == '\n' || c == '\r') {
      if (c == '\r' && s.peek() == '\n') (void)s.read();
      size_t useLen = inoutLen;
      if (useLen >= outSz) useLen = outSz - 1;
      outBuf[useLen] = '\0';
      if (!gTermNoEcho) {
        s.write('\r');
        s.write('\n');
      }
      inoutLen = 0;
      if (tuiEnabled) tuiMarkDirty();
      return true;
    }

    if (c == 0x08 || c == 0x7F) {  // backspace/delete
      if (inoutLen > 0) {
        inoutLen--;
        if (!gTermNoEcho) termEraseChars(s, 1);
        if (tuiEnabled) tuiMarkDirty();
      } else {
        if (!gTermNoEcho) s.write('\a');
      }
      continue;
    }

    if (c == 0x15) {  // Ctrl-U
      if (!gTermNoEcho && inoutLen > 0) termEraseChars(s, inoutLen);
      inoutLen = 0;
      if (tuiEnabled) tuiMarkDirty();
      continue;
    }

    if (c == 0x17) {  // Ctrl-W
      if (inoutLen > 0) {
        size_t i = inoutLen;
        while (i > 0 && outBuf[i - 1] == ' ') i--;
        while (i > 0 && outBuf[i - 1] != ' ') i--;
        size_t toErase = inoutLen - i;
        inoutLen = i;
        if (!gTermNoEcho && toErase > 0) termEraseChars(s, toErase);
        if (tuiEnabled) tuiMarkDirty();
      }
      continue;
    }

    if (c == 0x03) {  // Ctrl-C
      if (!gTermNoEcho) {
        s.write('^');
        s.write('C');
        s.write('\r');
        s.write('\n');
      }
      outBuf[0] = '\0';
      inoutLen = 0;
      if (tuiEnabled) tuiMarkDirty();
      return true;
    }

    if (c == '\t') c = ' ';
    if (isPrintableAscii(c)) {
      if (inoutLen < (outSz - 1)) {
        outBuf[inoutLen++] = c;
        if (!gTermNoEcho) s.write((uint8_t)c);
        if (tuiEnabled) tuiMarkDirty();
      } else {
        if (!gTermNoEcho) s.write('\a');
      }
    }
  }
  return false;
}
// ==================== TUI section end ====================

// ==================== AD8307 metering ====================
static void updatePowerFromADC() {
  // Sample ADC and compute voltage at MCU pin
  uint16_t raw = analogRead(SM_ADC);
  float v_mcu = ((float)raw) * (ADC_VREF / 1023.0f);
  float v_in = v_mcu / SM_INPUT_DIV;  // actual input voltage after op-amp

  // dBm = 40*V - 40 (your original)
  float dbm = (DBM_SLOPE * v_in) + DBM_OFFSET;

  // Initialize smoothing once, then low-pass filter
  static bool inited = false;
  if (!inited) {
    gDbmSmooth = dbm;
    inited = true;
  } else {
    // Gentle smoothing to reduce flicker; tune alpha if needed
    const float alpha = 0.15f;
    gDbmSmooth = (1.0f - alpha) * gDbmSmooth + alpha * dbm;
  }

  gSmVolt = v_in;
  gDbmNow = dbm;

  // Power from smoothed dBm
  // mW = 10^(dBm/10); W = mW/1000
  float mW = powf(10.0f, gDbmSmooth / 10.0f);
  gPowerW = mW / 1000.0f;

  // Format power string like your OLED logic: uW for <=0 dBm, mW for <30 dBm, W for >=30 dBm
  if (gDbmSmooth <= 0.0f) {
    float uW = mW * 1000.0f;
    snprintf(gPowerStr, sizeof(gPowerStr), "%.2fuW", (double)uW);
  } else if (gDbmSmooth >= 30.0f) {
    snprintf(gPowerStr, sizeof(gPowerStr), "%.2fW", (double)gPowerW);
  } else {
    snprintf(gPowerStr, sizeof(gPowerStr), "%.2fmW", (double)mW);
  }

  // Map dBm to 0..14 bar. Example: -80..+10 dBm -> 0..14
  int bar = (int)roundf((gDbmSmooth + 80.0f) * (14.0f / 90.0f));
  if (bar < 0) bar = 0;
  if (bar > 14) bar = 14;
  gPwrBar = (uint8_t)bar;
}
static void pwrTick() {
  // Rate-limit ADC reads and mark TTY dirty if it changes
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if ((now - lastMs) < 100) return;  // ~10 Hz refresh
  lastMs = now;

  uint8_t prevBar = gPwrBar;
  int prevDbm10 = (int)roundf(gDbmSmooth * 10.0f);

  updatePowerFromADC();

  int newDbm10 = (int)roundf(gDbmSmooth * 10.0f);
  if (gPwrBar != prevBar || newDbm10 != prevDbm10) {
    // Ask TTY to update bars; OLED is redrawn on its own cadence
    tuiMarkDirty();
  }

  // record peaks while scanning
  if (uiScanOn) {
    size_t len = uiCurrentListLen();
    if (len > 0 && uiScanCurrFreqHz != 0) {
      // current index is the one we just set in uiScanTick (uiScanIdx already advanced)
      uint16_t currIdx = (uint16_t)((uiScanIdx + len - 1) % len);
      scanPeaksConsider(currIdx, uiScanCurrFreqHz, gDbmSmooth, gPowerW);
    }
  }
}
static void scanPeaksReset() {
  for (int i = 0; i < 3; ++i) {
    gScanPeaks[i].valid = 0;
    gScanPeaks[i].idx = 0;
    gScanPeaks[i].freqHz = 0;
    gScanPeaks[i].dbm = -999.0f;
    gScanPeaks[i].watts = 0.0f;
  }
  gScanPeaksNeedsSave = false;
}
static void peaksMaybePersist(const PeakEntry& e) {
  (void)e;           // not needed right now; we save the full 3-entry block
  peaksMarkDirty();  // schedule a debounced save of /peaks.bin
}
static void formatPowerShort(float dbm, float watts, char* out, size_t outSz) {
  // mirror OLED/TUI compact unit logic
  float mW = watts * 1000.0f;
  if (dbm <= 0.0f) {
    float uW = mW * 1000.0f;
    snprintf(out, outSz, "%.2fuW", (double)uW);
  } else if (dbm >= 30.0f) {
    snprintf(out, outSz, "%.2fW", (double)watts);
  } else {
    snprintf(out, outSz, "%.2fmW", (double)mW);
  }
}
static void scanPeaksConsider(uint16_t currIdx, uint32_t freqHz, float dbm, float watts) {
  // Insert/refresh into top-3 table if this reading qualifies.
  // Keeps unique indices, sorted by descending dBm.

  if (!isfinite(dbm)) return;  // Ignore obviously bad readings

  // 1) If already present, update if higher
  int present = -1;
  for (int i = 0; i < 3; ++i) {
    if (gScanPeaks[i].valid && gScanPeaks[i].idx == currIdx) {
      present = i;
      break;
    }
  }
  bool changed = false;

  if (present >= 0) {
    if (dbm > gScanPeaks[present].dbm) {
      gScanPeaks[present].dbm = dbm;
      gScanPeaks[present].watts = watts;
      gScanPeaks[present].freqHz = freqHz;
      changed = true;
    }
  } else {
    // 2) Not present: find slot (invalid or weakest)
    int slot = -1;
    for (int i = 0; i < 3; ++i)
      if (!gScanPeaks[i].valid) {
        slot = i;
        break;
      }
    if (slot < 0) {
      // all valid; replace weakest if this is better
      int weakest = 0;
      for (int i = 1; i < 3; ++i)
        if (gScanPeaks[i].dbm < gScanPeaks[weakest].dbm) weakest = i;
      if (dbm > gScanPeaks[weakest].dbm) slot = weakest;
    }
    if (slot >= 0) {
      gScanPeaks[slot].valid = 1;
      gScanPeaks[slot].idx = currIdx;
      gScanPeaks[slot].freqHz = freqHz;
      gScanPeaks[slot].dbm = dbm;
      gScanPeaks[slot].watts = watts;
      changed = true;
    }
  }

  // 3) Sort by descending dBm if changed
  if (changed) {
    for (int i = 0; i < 3; ++i) {
      for (int j = i + 1; j < 3; ++j) {
        if (gScanPeaks[j].valid && (!gScanPeaks[i].valid || gScanPeaks[j].dbm > gScanPeaks[i].dbm)) {
          PeakEntry tmp = gScanPeaks[i];
          gScanPeaks[i] = gScanPeaks[j];
          gScanPeaks[j] = tmp;
        }
      }
    }
    gScanPeaksNeedsSave = true;        // for future storage hook
    peaksMaybePersist(gScanPeaks[0]);  // optional: persist only top-1 on change, or the changed entry
    tuiMarkDirty();                    // refresh TUI to reflect new peaks
  }
}
static void peaksApply(const PeaksV1& p) {
  // copy entries into RAM table
  for (int i = 0; i < 3; ++i) gScanPeaks[i] = p.entries[i];
  sPeaksLoaded = true;
}
static void peaksFillFromGlobals(PeaksV1& p) {
  memset(&p, 0, sizeof(p));
  p.magic = PEAKS_MAGIC;
  p.ver = PEAKS_VERSION;
  p.scan_src = uiScanSrc ? 1 : 0;  // save current source as metadata
  for (int i = 0; i < 3; ++i) p.entries[i] = gScanPeaks[i];
}
static bool peaksLoad() {
  if (!LittleFS.begin()) {
    Serial.println("\rRP2040Zero (PEAKS):\tLittleFS mount failed");
    return false;
  }
  if (!LittleFS.exists(PEAKS_PATH)) {
    Serial.println("\rRP2040Zero (PEAKS):\tNo peaks file");
    return false;
  }
  File f = LittleFS.open(PEAKS_PATH, "r");
  if (!f) {
    Serial.println("\rRP2040Zero (PEAKS):\tOpen failed");
    return false;
  }
  PeaksV1 p;
  size_t got = f.read((uint8_t*)&p, sizeof(p));
  f.close();
  if (got < sizeof(PeaksV1) || p.magic != PEAKS_MAGIC || p.ver != PEAKS_VERSION) {
    Serial.println("\rRP2040Zero (PEAKS):\tBad file");
    return false;
  }
  peaksApply(p);
  Serial.printf("\rRP2040Zero (PEAKS):\tLoaded peaks history at: %s\n", PEAKS_PATH);
  return true;
}
static bool peaksSaveNow() {
  if (!LittleFS.begin()) {
    Serial.println("\rRP2040Zero (PEAKS):\tFS mount failed (save)");
    return false;
  }
  PeaksV1 p;
  peaksFillFromGlobals(p);
  File f = LittleFS.open(PEAKS_PATH, "w");
  if (!f) {
    Serial.println("\rRP2040Zero (PEAKS):\tOpen for write failed");
    return false;
  }
  size_t want = sizeof(p);
  size_t wr = f.write((const uint8_t*)&p, want);
  f.flush();
  f.close();
  if (wr != want) {
    Serial.println("\rRP2040Zero (PEAKS):\tShort write");
    return false;
  }
  sPeaksLastSave = millis();
  Serial.println("\rRP2040Zero (PEAKS):\tSaved");
  return true;
}
static void peaksMarkDirty() {
  sPeaksDirty = true;
  sPeaksDirtySince = millis();
}
static void peaksInitAndLoad() {
  (void)peaksLoad();  // ok to fail; keep RAM defaults
  sPeaksDirty = false;
  sPeaksDirtySince = 0;
  sPeaksLastSave = 0;
}
static void peaksTick() {
  if (!sPeaksDirty) return;
  unsigned long now = millis();
  if ((now - sPeaksDirtySince) >= PEAKS_DEBOUNCE_MS && (now - sPeaksLastSave) >= PEAKS_MIN_SAVE_GAP_MS) {
    if (peaksSaveNow()) sPeaksDirty = false;
  }
}

// ==================== Boot Animations ====================
static inline float easeOutCubic(float t) {
  return 1.0f - powf(1.0f - t, 3.0f);
}
static void drawBitmapScaled1(Adafruit_SSD1306& d,
                              const uint8_t* buf, int bw, int bh,
                              int cx, int cy, float sx, float sy) {
  if (bw <= 0 || bh <= 0 || sx <= 0.0f || sy <= 0.0f) return;

  const int dw = (int)fmaxf(1.0f, floorf(bw * sx + 0.5f));
  const int dh = (int)fmaxf(1.0f, floorf(bh * sy + 0.5f));
  const int x0 = cx - (dw / 2);
  const int y0 = cy - (dh / 2);

  const int rowBytes = (bw + 7) / 8;
  for (int dy = 0; dy < dh; ++dy) {
    const float srcYf = (float)dy / sy;
    int syi = (int)floorf(srcYf);
    if (syi < 0) syi = 0;
    else if (syi >= bh) syi = bh - 1;

    for (int dx = 0; dx < dw; ++dx) {
      const float srcXf = (float)dx / sx;
      int sxi = (int)floorf(srcXf);
      if (sxi < 0) sxi = 0;
      else if (sxi >= bw) sxi = bw - 1;

      const uint8_t byte = buf[syi * rowBytes + (sxi >> 3)];
      if (byte & (0x80 >> (sxi & 7))) {
        const int x = x0 + dx;
        const int y = y0 + dy;
        if ((unsigned)x < (unsigned)d.width() && (unsigned)y < (unsigned)d.height()) {
          d.drawPixel(x, y, WHITE);
        }
      }
    }
  }
}
void animateTextToCenterAndCompress(Adafruit_SSD1306& d,
                                    const char* text,
                                    int startX, int startY,
                                    uint16_t moveMs,
                                    uint16_t compressMs,
                                    float minScaleY = 0.35f,
                                    float maxStretchX = 1.15f) {
  if (!text || !*text) return;

  int16_t x1, y1;
  uint16_t w, h;
  d.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  if (w <= 0 || h <= 0) return;

  GFXcanvas1 canvas(w, h);
  canvas.fillScreen(0);
  canvas.setTextColor(1);
  canvas.setTextSize(1);
  canvas.setCursor(-x1, -y1);
  canvas.print(text);

  const uint8_t* buf = canvas.getBuffer();

  const int W = d.width();
  const int H = d.height();

  const int targetX = (W - (int)w) / 2;
  const int targetY = (H - (int)h) / 2;

  {
    const uint8_t dt = 16;
    for (uint16_t t = 0; t <= moveMs; t += dt) {
      float u = easeOutCubic(fminf(1.0f, (float)t / (float)moveMs));
      float xf = startX + (targetX - startX) * u;
      float yf = startY + (targetY - startY) * u;
      d.clearDisplay();
      d.drawBitmap((int)roundf(xf), (int)roundf(yf), buf, w, h, WHITE);
      d.display();
      delay(dt);
      yield();
      watchdog_update();
    }
  }
  {
    const int cx = targetX + (int)w / 2;
    const int cy = targetY + (int)h / 2;
    const uint8_t dt = 16;
    for (uint16_t t = 0; t <= compressMs; t += dt) {
      float u = easeOutCubic(fminf(1.0f, (float)t / (float)compressMs));
      float sy = 1.0f + (minScaleY - 1.0f) * u;
      float sx = 1.0f + (maxStretchX - 1.0f) * u;
      d.clearDisplay();
      drawBitmapScaled1(d, buf, w, h, cx, cy, sx, sy);
      d.display();
      delay(dt);
      yield();
      watchdog_update();
    }
  }
}
void bootExplosion(Adafruit_SSD1306& d, uint16_t vibrateMs = 350) {
  const int W = d.width();
  const int H = d.height();

  const float cx = W / 2.0f;
  const float cy = H / 2.0f;

  d.clearDisplay();
  const int finalR = 10;
  for (int r = 2; r <= finalR; r += 2) {
    d.clearDisplay();
    d.fillCircle((int)cx, (int)cy, r, WHITE);
    d.display();
    delay(35);
    watchdog_update();
  }

  {
    const uint8_t frameDt = 16;
    const float ampX = 1.2f;
    const float ampY = 1.0f;
    for (uint16_t t = 0; t < vibrateMs; t += frameDt) {
      float phase = (float)t * 0.06f;
      float dx = sinf(phase + 3.1f) * ampX + ((float)random(-10, 11)) * 0.03f;
      float dy = cosf(phase + 2.7f) * ampY + ((float)random(-10, 11)) * 0.03f;
      d.clearDisplay();
      d.fillCircle((int)roundf(cx + dx), (int)roundf(cy + dy),
                   finalR + ((t >> 4) & 1), WHITE);
      d.display();
      delay(frameDt);
      yield();
      watchdog_update();
    }
  }

  const int N = 1600;
  struct Particle {
    float x, y, vx, vy;
    uint16_t life;
  };
  Particle p[N];

  randomSeed((uint32_t)micros());
  const float startR = 8.0f;
  for (int i = 0; i < N; ++i) {
    float ang = (float)random(0, 6283) * 0.001f;
    float r = startR + (float)random(-200, 201) * 0.005f;
    float speed = 0.6f + (float)random(0, 140) * 0.01f;
    float ca = cosf(ang);
    float sa = sinf(ang);
    p[i].x = cx + r * ca;
    p[i].y = cy + r * sa;
    p[i].vx = ca * speed;
    p[i].vy = sa * speed;
    p[i].life = 110 + (uint16_t)random(0, 30);
    watchdog_update();
  }

  const float g = 0.039087f;
  const float drag = 0.9999996f;
  const int frames = 120;
  for (int frame = 0; frame < frames; ++frame) {
    d.clearDisplay();
    for (int i = 0; i < N; ++i) {
      if (p[i].life == 0) continue;
      int xi = (int)(p[i].x + 0.5f);
      int yi = (int)(p[i].y + 0.5f);
      if ((uint32_t)xi < (uint32_t)W && (uint32_t)yi < (uint32_t)H) {
        d.drawPixel(xi, yi, WHITE);
      }
      p[i].vx *= drag;
      p[i].vy = p[i].vy * drag + g;
      p[i].x += p[i].vx;
      p[i].y += p[i].vy;

      if (p[i].y > (H + 10) || p[i].x < -10 || p[i].x > (W + 10)) {
        p[i].life = 0;
      } else {
        --p[i].life;
      }
    }
    d.display();
    delay(16);
    yield();
    watchdog_update();
  }
}

// ==================== Command handlers ====================
static void sendHelpTo(Stream& out) {
  if (tuiEnabled) {
    // Optional: richer help that can write to any Stream or to TUI log
    out.print("RP2040Zero Help:");
    out.print("General:");
    out.print("  HELP | ? | H          - show this help");
    out.print("HW:");
    out.print("  BOOTSEL               - reboot to UF2 bootloader");
    out.print("  I2C?                  - scan I2C bus and print devices");
    out.print("Unimplemented HW buttons:");
    out.print("  SETSTEP               - set rotary knob step count");
    out.print("  INCBAND               - increment band preset");
    out.print("Tuning/state:");
    out.print("  FREQ?                 - query current frequency");
    out.print("  FREQ <Hz>             - set frequency (10 kHz .. 225 MHz)");
    out.print("  IF <kHz>              - set IF in kHz (0 .. 200000)");
    out.print("  SIGMETER <1..14>      - set S-meter bucket");
    out.print("Scanning:");
    out.print("  SCAN START            - start scanning current list");
    out.print("  SCAN STOP             - stop scanning");
    out.print("  SCAN?                 - show scanner status");
    out.print("  SCAN SRC HARD         - use hard-coded list (default: CB 40-ch)");
    out.print("  SCAN SRC CUSTOM       - use custom list");
    out.print("  SCAN DELAY <ms>       - set per-step delay (20..10000 ms)");
    out.print("  SCAN ADD <Hz>         - add a frequency to custom list");
    out.print("  SCAN CLEAR            - clear custom list");
    out.print("  SCAN LIST?            - show current scan list");
    out.print("Terminal UI:");
    out.print("  TTY ON                - enable terminal UI (ANSI/VT100)");
    out.print("  TTY OFF               - disable terminal UI");
    out.print("  TTY?                  - show TTY status");
    out.print("  TTY ANSI ON|OFF       - enable/disable ANSI usage");
    out.print("  TTY REDRAW            - force full redraw");
    out.print("  TTY SIZE <rows> <cols>- set terminal rows/cols (for layout)");
    out.print("Console editing:");
    out.print("  Enter (CR/LF)         - submit line");
    out.print("  Backspace/Delete      - erase last character");
    out.print("  Ctrl-U                - clear entire line");
    out.print("  Ctrl-W                - delete previous word");
    out.print("  Ctrl-C                - cancel current line");
    out.println();
  } else {
    out.println("RP2040Zero Help:");
    out.println("General:");
    out.println("  HELP | ? | H          - show this help");
    out.println("HW:");
    out.println("  BOOTSEL               - reboot to UF2 bootloader");
    out.println("  I2C?                  - scan I2C bus and print devices");
    out.println("Unimplemented HW buttons:");
    out.println("  SETSTEP               - set rotary knob step count");
    out.println("  INCBAND               - increment band preset");
    out.println("Tuning/state:");
    out.println("  FREQ?                 - query current frequency");
    out.println("  FREQ <Hz>             - set frequency (10 kHz .. 225 MHz)");
    out.println("  IF <kHz>              - set IF in kHz (0 .. 200000)");
    out.println("  SIGMETER <1..14>      - set S-meter bucket");
    out.println("Scanning:");
    out.println("  SCAN START            - start scanning current list");
    out.println("  SCAN STOP             - stop scanning");
    out.println("  SCAN?                 - show scanner status");
    out.println("  SCAN SRC HARD         - use hard-coded list (default: CB 40-ch)");
    out.println("  SCAN SRC CUSTOM       - use custom list");
    out.println("  SCAN DELAY <ms>       - set per-step delay (20..10000 ms)");
    out.println("  SCAN ADD <Hz>         - add a frequency to custom list");
    out.println("  SCAN CLEAR            - clear custom list");
    out.println("  SCAN LIST?            - show current scan list");
    out.println("Terminal UI:");
    out.println("  TTY ON                - enable terminal UI (ANSI/VT100)");
    out.println("  TTY OFF               - disable terminal UI");
    out.println("  TTY?                  - show TTY status");
    out.println("  TTY ANSI ON|OFF       - enable/disable ANSI usage");
    out.println("  TTY REDRAW            - force full redraw");
    out.println("  TTY SIZE <rows> <cols>- set terminal rows/cols (for layout)");
    out.println("Console editing:");
    out.println("  Enter (CR/LF)         - submit line");
    out.println("  Backspace/Delete      - erase last character");
    out.println("  Ctrl-U                - clear entire line");
    out.println("  Ctrl-W                - delete previous word");
    out.println("  Ctrl-C                - cancel current line");
    out.println();
  }
}
static void sendScanStatus(Stream& io) {
  // Existing sendHelpRP can remain; we’ll prefer sendHelpTo so help appears inside TUI log when active.
  const size_t len = uiCurrentListLen();
  char buf[32];
  formatFreqSmart(uiScanCurrFreqHz, buf, sizeof(buf));
  io.printf("Scan: %s, ", uiScanOn ? "ON" : "OFF");
  io.printf("Source: %s, ", uiScanSrc ? "CUSTOM" : "HARD");
  io.printf("Delay: %lu ms, ", (unsigned long)uiScanDelayMs);
  io.printf("Length: %lu, ", (unsigned long)len);
  io.printf("Index: %lu", (unsigned long)uiScanIdx);
  if (uiScanOn) {
    io.printf(", Current: %lu Hz (%s)\n", (unsigned long)uiScanCurrFreqHz, buf);
  } else {
    io.printf("\n");
  }
}
static void termPrompt() {
  // Helper: show prompt in plain mode
  if (!tuiEnabled) sPrintf(Serial, "\r> ");
}
void handleCommand(const char* line, Stream& io) {
  if (line[0] == '\0') return;

  // Echo typed command into TUI log so it's always visible in the console pane
  if (tuiEnabled) {
    tuiLogf("> %s\n", line);
  }

  const bool fromUSB = (&io == &Serial);

  // When TUI is ON, route printed outputs to the log instead of raw terminal
  // (we still do some sPrintf to Serial for critical actions or if TUI is OFF)
  auto outLine = [&](const char* s) {
    if (tuiEnabled) tuiLogf("%s", s);
    else sPrintf(Serial, "%s\n", s);
  };
  auto outPrintf = [&](const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (tuiEnabled) tuiLogf("%s", buf);
    else sPrintf(Serial, "%s", buf);
  };

  // ==================== Reboot to bootloader ====================
  if (!strcmp(line, "BOOTSEL") && fromUSB) {
    outLine("Rebooting to bootloader now!\r");
    delay(100);
    rp2040.rebootToBootloader();
    return;
  }
  // ==================== Reboot ====================
  if (!strcmp(line, "REBOOT") && fromUSB) {
    outLine("Rebooting now!\r");
    delay(100);
    rp2040.reboot();
    return;
  }
  // ==================== Factory reset ====================
  if (!strcmp(line, "FACTORYRESETNOW") && fromUSB) {
    outLine("Rebooting to bootloader now!");
    delay(100);
    LittleFS.remove("/vfo.cfg");
    delay(100);
    rp2040.reboot();
    delay(100);
    return;
  }
  // ==================== Cycle to next step size preset ====================
  if (!strcmp(line, "SETSTEP") && fromUSB) {
    time_now = (millis() + 300);
    setstep();
    cfgMarkDirty();  // persist stp/fstep
    delay(300);
    termPrompt();
    tuiMarkDirty();
    return;
  }
  // ==================== Cycle to next band preset ====================
  if (!strcmp(line, "INCBAND") && fromUSB) {
    time_now = (millis() + 300);
    inc_preset();
    cfgMarkDirty();  // persist stp/fstep
    delay(300);
    termPrompt();
    tuiMarkDirty();
    return;
  }
  // ==================== Return the current frequency ====================
  if (!strcmp(line, "FREQ?") && fromUSB) {
    char buf[32];
    formatFreqSmart(freq, buf, sizeof(buf));
    outPrintf("RP2040Zero: tuned to: %lu Hz (%s)\n", (unsigned long)freq, buf);
    time_now = millis();
    termPrompt();
    tuiMarkDirty();
    return;
  }
  // ==================== I2C device scanner ====================
  if (!strcmp(line, "I2C?") && fromUSB) {
    Serial.print("RP2040Zero (BOOT): Detected I2C device(s): ");
    int i2cDevsFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        if (i2cDevsFound >= 1) {
          Serial.print(", 0x");
          Serial.print(addr, HEX);
        } else {
          Serial.print("0x");
          Serial.print(addr, HEX);
        }
        display.print("0x");
        display.print(addr, HEX);
        display.print(" ");
        display.display();
        i2cDevsFound++;
      }
    }
    Serial.println();
    time_now = millis();
    termPrompt();
    tuiMarkDirty();
    return;
  }

  // ==================== TTY commands (Terminal UI) ====================
  if (fromUSB) {
    if (!strcmp(line, "TTY ON")) {
      tuiSetEnabled(true);
      termPrompt();
      return;
    }
    if (!strcmp(line, "TTY OFF")) {
      tuiSetEnabled(false);
      sPrintf(Serial, "\nTUI disabled.\n");
      termPrompt();
      return;
    }
    if (!strcmp(line, "TTY REDRAW")) {
      tuiDirtyFull = true;
      termPrompt();
      return;
    }
    if (!strcmp(line, "TTY?")) {
      outPrintf("TTY: %s, ANSI: %s, size=%ux%u\n",
                tuiEnabled ? "ON" : "OFF",
                tuiAnsi ? "ON" : "OFF",
                (unsigned)tuiRows, (unsigned)tuiCols);
      termPrompt();
      return;
    }
    if (!strcmp(line, "TTY ANSI ON")) {
      tuiAnsi = true;
      cfgMarkDirty();  // persist stp/fstep
      if (tuiEnabled) {
        tuiExit();
        tuiEnter();
      }
      termPrompt();
      return;
    }
    if (!strcmp(line, "TTY ANSI OFF")) {
      if (tuiEnabled) tuiSetEnabled(false);
      tuiAnsi = false;
      cfgMarkDirty();  // persist stp/fstep
      outLine("ANSI disabled. Use TTY ANSI ON then TTY ON to re-enable UI.");
      termPrompt();
      return;
    }
    // TTY SIZE <rows> <cols>
    {
      unsigned r = 0, c = 0;
      if (sscanf(line, "TTY SIZE %u %u", &r, &c) == 2) {
        if (r >= 12 && r <= 80 && c >= 40 && c <= 200) {
          tuiRows = r;
          tuiCols = c;
          cfgMarkDirty();  // persist stp/fstep
          tuiComputeRows();
          tuiDirtyFull = true;
          outPrintf("TTY size set to %ux%u\n", tuiRows, tuiCols);
        } else {
          outLine("TTY SIZE out of range (rows 12..80, cols 40..200)");
        }
        termPrompt();
        return;
      }
    }
  }

  // ==================== Scanner commands ====================
  if (fromUSB) {
    if (!strcmp(line, "SCAN?")) {
      // Write to TUI log or Serial depending on mode
      if (tuiEnabled) {
        // Build status into a buffer and log it
        char tmp[256];
        char fbuf[32];
        formatFreqSmart(uiScanCurrFreqHz, fbuf, sizeof(fbuf));
        size_t len = uiCurrentListLen();
        snprintf(tmp, sizeof(tmp),
                 "Scan: %s\n  Source: %s\n  Delay: %lu ms\n  Length: %lu\n  Index: %lu\n  Current: %lu Hz (%s)\n",
                 uiScanOn ? "ON" : "OFF",
                 uiScanSrc ? "CUSTOM" : "HARD",
                 (unsigned long)uiScanDelayMs,
                 (unsigned long)len,
                 (unsigned long)uiScanIdx,
                 (unsigned long)uiScanCurrFreqHz, fbuf);
        tuiLogf("%s", tmp);
      } else {
        sendScanStatus(Serial);
      }
      termPrompt();
      tuiMarkDirty();
      return;
    }
    if (!strcmp(line, "SCAN START")) {
      if (uiCurrentListLen() == 0) {
        outLine("Scan: no entries in current list");
      } else {
        uiScanStart();
        outLine("Scan: started");
      }
      termPrompt();
      tuiMarkDirty();
      return;
    }
    if (!strcmp(line, "SCAN STOP")) {
      uiScanStop();
      outLine("Scan: stopped");
      termPrompt();
      tuiMarkDirty();
      return;
    }
    if (!strcmp(line, "SCAN SRC HARD")) {
      uiScanUse(0);
      cfgMarkDirty();  // persist stp/fstep
      outLine("Scan: source = HARD");
      termPrompt();
      tuiMarkDirty();
      return;
    }
    if (!strcmp(line, "SCAN SRC CUSTOM")) {
      if (uiCustomScanLen == 0) {
        outLine("Scan: CUSTOM list empty (use: SCAN ADD <Hz>)");
      } else {
        uiScanUse(1);
        cfgMarkDirty();  // persist stp/fstep
        outLine("Scan: source = CUSTOM");
      }
      termPrompt();
      tuiMarkDirty();
      return;
    }
    {
      unsigned long ms = 0;
      if (sscanf(line, "SCAN DELAY %lu", &ms) == 1) {
        if (ms < 1) ms = 1;
        if (ms > 10000UL) ms = 10000UL;
        uiScanDelayMs = (uint32_t)ms;
        cfgMarkDirty();  // persist stp/fstep
        outPrintf("Scan: delay = %lu ms\n", (unsigned long)uiScanDelayMs);
        termPrompt();
        tuiMarkDirty();
        return;
      }
    }
    {
      unsigned long hz = 0;
      if (sscanf(line, "SCAN ADD %lu", &hz) == 1) {
        if (hz < 10000UL) hz = 10000UL;
        if (hz > 225000000UL) hz = 225000000UL;
        if (uiCustomScanLen >= UI_MAX_CUSTOM_SCAN) {
          outPrintf("Scan: custom list full (%lu max)\n", (unsigned long)UI_MAX_CUSTOM_SCAN);
        } else {
          uiCustomScanList[uiCustomScanLen++] = (uint32_t)hz;
          cfgMarkDirty();  // persist stp/fstep
          char buf[32];
          formatFreqSmart((uint32_t)hz, buf, sizeof(buf));
          outPrintf("Scan: added %lu Hz (%s), custom len=%lu\n",
                    (unsigned long)hz, buf, (unsigned long)uiCustomScanLen);
        }
        termPrompt();
        tuiMarkDirty();
        return;
      }
    }
    if (!strcmp(line, "SCAN CLEAR")) {
      uiCustomScanLen = 0;
      if (uiScanSrc == 1) {
        uiScanStop();
        uiScanUse(0);
      }
      cfgMarkDirty();  // persist stp/fstep
      outLine("Scan: custom list cleared");
      termPrompt();
      tuiMarkDirty();
      return;
    }
    if (!strcmp(line, "SCAN LIST?")) {
      const uint32_t* list = uiCurrentListPtr();
      const size_t len = uiCurrentListLen();
      outPrintf("Scan list (%s), len=%lu\n", uiScanSrc ? "CUSTOM" : "HARD", (unsigned long)len);
      for (size_t i = 0; i < len; ++i) {
        char buf[32];
        formatFreqSmart(list[i], buf, sizeof(buf));
        outPrintf("  [%lu] %lu Hz (%s)\n", (unsigned long)i, (unsigned long)list[i], buf);
      }
      termPrompt();
      tuiMarkDirty();
      return;
    }
  }

  // ==================== Help ====================
  if ((!strcmp(line, "HELP") || !strcmp(line, "?") || !strcmp(line, "H")) && fromUSB) {
    if (tuiEnabled) {
      // write help into log
      // simple adapter to Stream -> TUI
      struct LogStream : public Stream {
        size_t write(uint8_t c) override {
          char b[2] = { (char)c, 0 };
          tuiLogf("%s", b);
          return 1;
        }
        size_t write(const uint8_t* b, size_t n) override {
          char tmp[256];
          while (n) {
            size_t take = n;
            if (take > sizeof(tmp) - 1) take = sizeof(tmp) - 1;
            memcpy(tmp, b, take);
            tmp[take] = 0;
            tuiLogf("%s", tmp);
            b += take;
            n -= take;
          }
          return 1;
        }
        int available() override {
          return 0;
        }
        int read() override {
          return -1;
        }
        int peek() override {
          return -1;
        }
      } ls;
      sendHelpTo(ls);
    } else {
      sendHelpTo(Serial);
    }
    //Serial.println();
    if (!tuiEnabled) Serial.println();
    termPrompt();
    tuiMarkDirty();
    return;
  }

  // ==================== KEY [VALUE] numeric ====================
  char key[8] = { 0 };
  long val = 0;
  int matched = sscanf(line, "%7s %ld", key, &val);
  if (matched >= 1 && fromUSB) {
    if (!strcmp(key, "FREQ") && matched == 2) {
      if (val < 10000) val = 10000;
      if ((unsigned long)val > 225000000UL) val = 225000000UL;
      freq = (unsigned long)val;
      cfgMarkDirty();  // persist stp/fstep
      time_now = millis();
      termPrompt();
      tuiMarkDirty();
      return;
    } else if (!strcmp(key, "IF") && matched == 2) {
      if (val < 0) val = 0;
      if (val > 200000) val = 200000;
      interfreq = val;
      cfgMarkDirty();  // persist stp/fstep
      time_now = millis();
      termPrompt();
      tuiMarkDirty();
      return;
    } else if (!strcmp(key, "SIGMETER") && matched == 2) {
      if (val < 1) val = 1;
      if (val > 14) val = 14;
      sigmeter = (byte)val;
      cfgMarkDirty();  // persist stp/fstep
      time_now = millis();
      termPrompt();
      tuiMarkDirty();
      return;
    }
  }

  // ==================== Unknown ====================
  if (tuiEnabled) {
    tuiLogf("RP2040Zero: Unknown cmd: %s\n", line);
  } else {
    sPrintf(Serial, "RP2040Zero: Unknown cmd: %s\n", line);
    termPrompt();
  }
  tuiMarkDirty();
}

// ==================== Watchdog handlers ====================
static WDTResetReason classify_wdt_reason(uint32_t raw_reason_bits) {
  bool timeout = (raw_reason_bits & WATCHDOG_REASON_TIMER_BITS) != 0;
  bool force = (raw_reason_bits & WATCHDOG_REASON_FORCE_BITS) != 0;

  if (!timeout && !force) return WDT_REASON_NONE;
  if (timeout && !force) return WDT_REASON_TIMEOUT;
  if (!timeout && force) return WDT_REASON_FORCE;
  return WDT_REASON_MULTIPLE;
}
static void handle_wdt_reason_switch(WDTResetReason reason) {
  switch (reason) {
    case WDT_REASON_TIMEOUT:
      Serial.println("RP2040Zero (WDT):\tPrevious reset was caused by Watchdog TIMEOUT.");
      // Count consecutive TIMEOUT boots and escalate if threshold reached
      wdt_timeout_boot_bump_and_maybe_escalate();
      break;

    case WDT_REASON_FORCE:
      Serial.println("RP2040Zero (WDT):\tPrevious reset was caused by software-forced Watchdog reboot.");
      // Count consecutive FORCE boots and optionally escalate
      wdt_force_boot_bump_and_maybe_escalate();
      break;

    case WDT_REASON_MULTIPLE:
      Serial.println("RP2040Zero (WDT):\tPrevious reset had multiple Watchdog reason flags set.");
      // Count consecutive MULTIPLE-flag boots and optionally escalate
      wdt_multiple_boot_bump_and_maybe_escalate();
      break;

    case WDT_REASON_NONE:
      Serial.println("RP2040Zero (WDT):\tPrevious reset was NOT caused by the Watchdog.");
      break;

    default:
      Serial.println("RP2040Zero (WDT):\tPrevious reset reason UNKNOWN.");
      rp2040.rebootToBootloader();
      break;
  }
}
static void force_watchdog_reboot(uint32_t delay_ms = 1) {
  // Optional: call this wherever you want to force a clean software reboot via watchdog
  // This will set the FORCE reason bit and reset after the given delay.
  Serial.println("RP2040Zero (WDT):\tForcing watchdog reboot...");
  Serial.flush();
  watchdog_reboot(0, 0, delay_ms);
  while (true) { /* wait for reset */
  }
}
static inline uint32_t wdt_consecutive_timeouts_get() {
  return watchdog_hw->scratch[0];
}
static inline void wdt_consecutive_timeouts_set(uint32_t v) {
  watchdog_hw->scratch[0] = v;
}
static inline uint32_t wdt_consecutive_forces_get() {
  // Track other consecutive WDT reasons using additional scratch registers
  return watchdog_hw->scratch[1];
}
static inline void wdt_consecutive_forces_set(uint32_t v) {
  watchdog_hw->scratch[1] = v;
}
static inline uint32_t wdt_consecutive_multiple_get() {
  return watchdog_hw->scratch[2];
}
static inline void wdt_consecutive_multiple_set(uint32_t v) {
  watchdog_hw->scratch[2] = v;
}
static void wdt_timeout_boot_bump_and_maybe_escalate() {
  // Call on a boot that was caused by WDT TIMEOUT to bump and, if needed, escalate.
  uint32_t cnt = wdt_consecutive_timeouts_get() + 1;
  wdt_consecutive_timeouts_set(cnt);
  Serial.printf("\rRP2040Zero (WDT):\tConsecutive TIMEOUT boots = %lu\n\r", (unsigned long)cnt);
  if (cnt >= WDT_TIMEOUT_ESCALATE) {
    Serial.println("RP2040Zero (WDT):\tThreshold reached -> entering BOOTSEL for recovery.");
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();  // or enter your own "safe mode"
    while (true) {                /* wait for reset */
    }
  }
}
static void wdt_mark_boot_ok() {
  // Declare a clean boot; reset all consecutive counters
  wdt_consecutive_timeouts_set(0);
  wdt_consecutive_forces_set(0);
  wdt_consecutive_multiple_set(0);
}
static void wdt_force_boot_bump_and_maybe_escalate() {
  // Call on a boot that was caused by WDT FORCE to bump and, if needed, escalate.
  uint32_t cnt = wdt_consecutive_forces_get() + 1;
  wdt_consecutive_forces_set(cnt);
  Serial.printf("\rRP2040Zero (WDT):\tConsecutive FORCE boots = %lu\n\r", (unsigned long)cnt);
  if (cnt >= WDT_TIMEOUT_ESCALATE) {
    Serial.println("RP2040Zero (WDT):\tFORCE threshold reached -> entering BOOTSEL for recovery.");
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    while (true) { /* wait for reset */
    }
  }
}
static void wdt_multiple_boot_bump_and_maybe_escalate() {
  // Call on a boot that had MULTIPLE WDT flags to bump and, if needed, escalate.
  uint32_t cnt = wdt_consecutive_multiple_get() + 1;
  wdt_consecutive_multiple_set(cnt);
  Serial.printf("\rRP2040Zero (WDT):\tConsecutive MULTIPLE-flag boots = %lu\n\r", (unsigned long)cnt);
  if (cnt >= WDT_TIMEOUT_ESCALATE) {
    Serial.println("RP2040Zero (WDT):\tMULTIPLE threshold reached -> entering BOOTSEL for recovery.");
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    while (true) { /* wait for reset */
    }
  }
}
