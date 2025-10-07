#include <Adafruit_SSD1306.h>   // Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_NeoPixel.h>  // WS2812 LED
#include <si5351.h>             // Etherkit https://github.com/etherkit/Si5351Arduino
#include <Wire.h>               // I2C
#include <math.h>               // Standard math library

// ==================== Defines ====================
#define IF 10700           // Inter-Frequency in kHz
#define BAND_INIT 18       // Initial band preset
#define STEP_INIT 6        // Initial step size preset
#define XT_CAL_F 0         // XTAL ppm drift calibration amount
#define S_GAIN 505         //
#define BAUD 9600          // USB serial baud rate
#define rotLeft 6          // Rotary-left pin
#define rotRight 7         // Rotary-right pin
#define LED_PIN 16         // WS2812 LED
#define LED_COUNT 1        // The WaveShare RP2040 Zero only has the one LED
#define WAIT_USB_SERIAL 0  // Wait for a client to connect to USB serial connsole

// ==================== Tuner variables ====================
unsigned long freq, freqold, fstep;
long interfreq = IF, interfreqold = 0;
static long lastSentIF = -1;
long cal = XT_CAL_F;
unsigned int smval;
byte encoder = 1;
byte stp, n = 1;
byte count, x, xo;
bool sts = 0;
unsigned int period = 100;
unsigned long time_now = 0;

// ==================== Encoder state ====================
static uint8_t encPrev = 0;
static int8_t encAccum = 0;
static const int8_t encTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

// ==================== Line Reader ====================-
static const size_t CMD_BUF_SZ = 255;  // USB serial buffer length
static char sUsbBuf[CMD_BUF_SZ];       // USB serial buffer
static size_t sUsbLen = 0;             // USB serial pending buffer input length

// ==================== UI scanner ====================
static const size_t UI_MAX_CUSTOM_SCAN = 255;  // Max custom scan list length
static bool uiScanOn = false;                  // Scanner off initially
static uint8_t uiScanSrc = 0;                  // 0 = hard list, 1 = custom list
static uint32_t uiScanDelayMs = 5000;          // per-step delay (ms)
static uint32_t uiScanLastMs = 0;              // Last scan cycle in milliseconds
static size_t uiScanIdx = 0;                   // Current list index
static uint32_t uiScanCurrFreqHz = 0;          // Current scanner frequency in Hertz
static size_t uiCustomScanLen = 0;             // Length of custom scan list
static const uint32_t uiHardScanList[] PROGMEM = {
  96300000UL,   // 96.3 MHz -
  102500000UL,  // 102.5 MHz -
  103300000UL,  // 103.3 MHz -
  104100000UL,  // 104.1 MHz -
  106500000UL,  // 106.5 MHz -
  107700000UL,  // 107.7 MHz -
  162550000UL   // 162.550 MHz - NOAA WX Ch 7
};
static const size_t uiHardScanLen = sizeof(uiHardScanList) / sizeof(uiHardScanList[0]);  // Hard scan list length
static uint32_t uiCustomScanList[UI_MAX_CUSTOM_SCAN];                                    // Custom scanner list

// ==================== I2C devices ====================
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);  // SSD1306 OLED mono display (128x64 resolution)
Si5351 si5351(0x60);                                          // Si531 programmable clock generator

// ==================== WS2812 LED ====================
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);  // WS2812 LED on WaveShare RP2040 Zero PCB

// ==================== Rotary Encoder ====================
inline void pollEncoder() {
  uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
  uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (encPrev << 2) | curr;
  int8_t delta = encTable[idx];
  if (delta != 0) {
    encAccum += delta;
    if (encAccum >= 4) {
      set_frequency(1);
      encAccum = 0;
      delay(50);
    } else if (encAccum <= -4) {
      set_frequency(-1);
      encAccum = 0;
      delay(50);
    }
  }
  encPrev = curr;
}

// ==================== Improved terminal line reader ====================
static inline bool isPrintableAscii(char c) {
  return c >= 0x20 && c <= 0x7E;
}
static void termEraseChars(Stream& s, size_t n) {
  // Erase n characters on terminal by backspacing over them
  for (size_t i = 0; i < n; ++i) {
    s.write('\b');
    s.write(' ');
    s.write('\b');
  }
}
bool readLine(Stream& s, char* outBuf, size_t& inoutLen, size_t outSz) {
  // - Echo-back typed characters
  // - Backspace/Delete support (erase from screen)
  // - CR, LF, or CRLF line endings
  // - Ctrl-U: clear entire line
  // - Ctrl-W: delete previous word
  // - Ctrl-C: cancel current line (returns empty line to caller)
  while (s.available()) {
    char c = (char)s.read();

    // Newline: accept CR or LF
    if (c == '\n' || c == '\r') {
      // Swallow a possible CRLF second half without generating an empty line next time
      if (c == '\r' && s.peek() == '\n') (void)s.read();

      // Null-terminate
      size_t useLen = inoutLen;
      if (useLen >= outSz) useLen = outSz - 1;
      outBuf[useLen] = '\0';

      // Echo canonical newline for terminal
      s.write('\r');
      s.write('\n');

      // Reset for next line and signal a complete line to caller
      inoutLen = 0;
      return true;
    }

    // Handle backspace/delete
    if (c == 0x08 || c == 0x7F) {
      if (inoutLen > 0) {
        inoutLen--;
        termEraseChars(s, 1);
      } else {
        // nothing to delete, optional bell
        s.write('\a');
      }
      continue;
    }

    // Ctrl-U: clear entire line
    if (c == 0x15) {
      if (inoutLen > 0) {
        termEraseChars(s, inoutLen);
        inoutLen = 0;
      }
      continue;
    }

    // Ctrl-W: delete previous word
    if (c == 0x17) {
      if (inoutLen > 0) {
        // find word start (skip any trailing spaces)
        size_t i = inoutLen;
        while (i > 0 && outBuf[i - 1] == ' ') i--;
        while (i > 0 && outBuf[i - 1] != ' ') i--;
        size_t toErase = inoutLen - i;
        if (toErase > 0) {
          inoutLen = i;
          termEraseChars(s, toErase);
        }
      }
      continue;
    }

    // Ctrl-C: cancel current line (like a shell)
    if (c == 0x03) {
      // Echo ^C and newline
      s.write('^');
      s.write('C');
      s.write('\r');
      s.write('\n');
      // Deliver empty line to caller (who ignores it)
      outBuf[0] = '\0';
      inoutLen = 0;
      return true;
    }

    // Convert TAB to single space (minimal handling)
    if (c == '\t') c = ' ';

    // Printable ASCII: append if room
    if (isPrintableAscii(c)) {
      if (inoutLen < (outSz - 1)) {
        outBuf[inoutLen++] = c;
        s.write((uint8_t)c);  // echo
      } else {
        // Buffer full: optional bell to indicate overflow
        s.write('\a');
      }
    }
    // All other control chars ignored
  }
  return false;
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
  }

  const float g = 0.039087f;
  const float drag = 0.9999996f;
  const int frames = 116;
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
  }
}

// ==================== Command handlers ====================
void sendHelpRP() {
  Serial.println("RP2040Zero Help:");
  Serial.println("General:");
  Serial.println("  HELP | ? | H          - show this help");
  Serial.println();
  Serial.println("HW:");
  Serial.println("  BOOTSEL               - reboot to UF2 bootloader");
  Serial.println("  I2C?                  - scan I2C bus and print devices");
  Serial.println();
  Serial.println("Unimplemented HW buttons:");
  Serial.println("  SETSTEP               - set rotary knob step count");
  Serial.println("  INCBAND               - increment band preset");
  Serial.println();
  Serial.println("Tuning/state:");
  Serial.println("  FREQ?                 - query current frequency");
  Serial.println("  FREQ <Hz>             - set frequency (10 kHz .. 225 MHz)");
  Serial.println("  IF <kHz>              - set IF in kHz (0 .. 200000)");
  Serial.println("  SIGMETER <1..14>      - set S-meter bucket");
  Serial.println();
  Serial.println("Console editing:");
  Serial.println("  Enter (CR/LF)         - submit line");
  Serial.println("  Backspace/Delete      - erase last character");
  Serial.println("  Ctrl-U                - clear entire line");
  Serial.println("  Ctrl-W                - delete previous word");
  Serial.println("  Ctrl-C                - cancel current line");
}

void handleCommand(const char* line, Stream& io) {
  if (line[0] == '\0') return;  // Return if termination is first-in char

  const bool fromUSB = (&io == &Serial);  // Is our IO stream is from the USB serial

  // ==================== Boot to bootloader ====================
  if (!strcmp(line, "BOOTSEL") && fromUSB) {
    Serial.println("Rebooting to bootloader now!");
    Serial.printf("\r> ");
    delay(100);
    rp2040.rebootToBootloader();
    return;
  }

  // ==================== Cycle to next step size preset ====================
  if (!strcmp(line, "SETSTEP") && fromUSB) {
    time_now = (millis() + 300);
    setstep();
    delay(300);
    Serial.printf("\r> ");
    return;
  }

  // ==================== Cycle to next band preset ====================
  if (!strcmp(line, "INCBAND") && fromUSB) {
    time_now = (millis() + 300);
    inc_preset();
    delay(300);
    Serial.printf("\r> ");
    return;
  }

  // ==================== Return the current frequency ====================
  if (!strcmp(line, "FREQ?") && fromUSB) {
    char buf[32];
    formatFreqSmart(freq, buf, sizeof(buf));
    Serial.printf("RP2040Zero: tuned to: %u Hz (%s)\n", freq, buf);
    time_now = millis();
    Serial.printf("\r> ");
    return;
  }

  // ==================== I2C device scanner ====================
  if (!strcmp(line, "I2C?") && fromUSB) {
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("RP2040Zero (boot): Detected I2C device on bus: 0x");
        Serial.println(addr, HEX);
      }
    }
    time_now = millis();
    Serial.printf("\r> ");
    return;
  }

  // ==================== Return help dialog ====================
  if (!strcmp(line, "HELP") || !strcmp(line, "?") || !strcmp(line, "H") && fromUSB) {
    sendHelpRP();
    Serial.println();
    Serial.printf("\r> ");
    return;
  }

  // ==================== Handler for KEY [VALUE] numeric ====================
  char key[8] = { 0 };
  long val = 0;
  int matched = sscanf(line, "%7s %ld", key, &val);
  if (matched >= 1 && fromUSB) {
    if (!strcmp(key, "FREQ") && matched == 2 && fromUSB) {
      // ==================== Set a frequency in Hertz ====================
      if (val < 10000) val = 10000;
      if ((unsigned long)val > 225000000UL) val = 225000000UL;
      freq = (unsigned long)val;
      time_now = millis();
      Serial.printf("\r> ");
      return;
    } else if (!strcmp(key, "IF") && matched == 2 && fromUSB) {
      // ==================== Set an inter-frequency in kHz ====================
      if (val < 0) val = 0;
      if (val > 200000) val = 200000;
      interfreq = val;
      time_now = millis();
      Serial.printf("\r> ");
      return;
    } else if (!strcmp(key, "SIGMETER") && matched == 2 && fromUSB) {
      // ==================== Set the signal meter value ====================
      if (val < 1) val = 1;
      if (val > 14) val = 14;
      x = (byte)val;
      time_now = millis();
      Serial.printf("\r> ");
      return;
    }
  }

  // ==================== Unknown command ====================
  Serial.printf("RP2040Zero: Unknown cmd: ");
  Serial.println(line);
  Serial.printf("\r> ");
}

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
  Serial.println("\n\r\nRP2040Zero (boot): FW VER: 1.0.2");
  Serial.println("RP2040Zero (boot): Starting now..");

  // ==================== Rotary init ====================
  pinMode(rotLeft, INPUT_PULLUP);
  pinMode(rotRight, INPUT_PULLUP);
  {
    uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
    uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
    encPrev = (a << 1) | b;
    encAccum = 0;
  }

  // ==================== LED init ====================
  strip.begin();
  strip.show();
  strip.setBrightness(64);
  strip.setPixelColor(0, 64, 64, 64);
  strip.show();

  // ==================== I2C init ====================
  Wire.setClock(100000);
  Wire.begin();

  // ==================== Tuner init ====================
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);

  // ==================== Tuner variables init ====================
  count = BAND_INIT;
  bandpresets();
  stp = STEP_INIT;
  setstep();

  // ==================== Display init ====================
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();
  display.setTextSize(1);

  // ==================== Boot Animations ====================
  display.setCursor(0, 0);
  animateTextToCenterAndCompress(display, "VFO Generator", 25, 0, 150, 150, 0.3f, 0.025f);
  bootExplosion(display, 400);

  // ==================== I2C detection ====================
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("RP2040 I2C device(s):");
  display.setCursor(0, 12);
  Serial.println("RP2040Zero (boot): Detected I2C device(s):");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("RP2040Zero (boot): Detected I2C device on bus: 0x");
      Serial.println(addr, HEX);
      display.print("0x");
      display.print(addr, HEX);
      display.print(" ");
      display.display();
    }
  }

  // ==================== Boot process finished ====================
  delay(750);
  strip.setBrightness(32);
  strip.setPixelColor(0, 64, 0, 32);
  strip.show();
  Serial.print("RP2040Zero (boot): Running loop now..\n\n\rRP2040Zero (boot): HELP | ? | H for help dialog\n\r> ");
}

// ==================== Loop ====================
void loop() {
  // ==================== Pass USB serial data into our command handler ====================
  if (readLine(Serial, sUsbBuf, sUsbLen, CMD_BUF_SZ)) {
    handleCommand(sUsbBuf, Serial);
  }

  pollEncoder();  // ==================== Rotary encoder polling ====================
  uiScanTick();   // ==================== Scanner tick ====================

  // ==================== Frequency changed, retune now ====================
  if (freqold != freq) {
    time_now = millis();
    tunegen();
    freqold = freq;
  }

  // ==================== Inter-Frequency changed, retune now ====================
  if (interfreqold != interfreq) {
    time_now = millis();
    if (!uiScanOn && interfreq != lastSentIF) {  // don't push IF in scan
      lastSentIF = interfreq;
    }
    tunegen();  // early-returns when uiScanOn
    interfreqold = interfreq;
  }

  // ==================== ??? changed, ??? now ====================
  if (xo != x) {
    time_now = millis();
    xo = x;
  }

  // ==================== Limit display ticks ====================
  static unsigned long lastDraw = 0;     // last time since redraw in milliseconds
  const unsigned long drawPeriod = 100;  // redraw time in milliseconds
  if (millis() - lastDraw >= drawPeriod) {
    lastDraw = millis();
    displayfreq();
    layout();
  }
}

// ==================== UI Scanner ====================
static inline size_t uiCurrentListLen() {
  return uiScanSrc ? uiCustomScanLen : uiHardScanLen;
}
static inline const uint32_t* uiCurrentListPtr() {
  return uiScanSrc ? uiCustomScanList : uiHardScanList;
}
static inline void uiScanUse(uint8_t src) {
  uiScanSrc = src ? 1 : 0;
}
static void uiScanStart() {
  if (uiCurrentListLen() == 0) return;
  uiScanOn = true;
  uiScanIdx = 0;
  uiScanLastMs = millis() - uiScanDelayMs;
}
static void uiScanStop() {
  uiScanOn = false;
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
  if (uiScanOn) return;  // Mirror mode: don’t drive the Nano
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
  display.setTextSize(2);

  char buffer[15] = "";
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
      fstep = 1000;
      break;
    case 4:
      stp = 5;
      fstep = 5000;
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
  display.drawLine(0, 20, 127, 20, WHITE);
  display.drawLine(0, 43, 127, 43, WHITE);
  display.drawLine(105, 24, 105, 39, WHITE);
  display.drawLine(87, 24, 87, 39, WHITE);
  display.drawLine(87, 48, 87, 63, WHITE);
  display.drawLine(15, 55, 82, 55, WHITE);
  display.setTextSize(1);
  display.setCursor(59, 23);
  display.print("STEP");
  display.setCursor(51, 33);
  if (stp == 2) display.print("  1Hz");
  if (stp == 3) display.print(" 10Hz");
  if (stp == 4) display.print(" 1kHz");
  if (stp == 5) display.print(" 5kHz");
  if (stp == 6) display.print("10kHz");
  if (stp == 7) display.print("100kHz");
  if (stp == 8) display.print(" 1MHz");
  if (stp == 1) display.print(" 10MHz");

  display.setTextSize(1);
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
  if (interfreq != 0) display.print("L O");

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
  display.setTextSize(2);
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
  if (count == 21) display.print("1m");
  if (count == 1) interfreq = 0;
  else if (!sts) interfreq = IF;
}
void drawbargraph() {
  byte y = map(n, 1, 42, 1, 14);

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("TU");

  switch (y) {
    case 1: display.fillRect(15, 48, 2, 6, WHITE); break;
    case 2: display.fillRect(20, 48, 2, 6, WHITE); break;
    case 3: display.fillRect(25, 48, 2, 6, WHITE); break;
    case 4: display.fillRect(30, 48, 2, 6, WHITE); break;
    case 5: display.fillRect(35, 48, 2, 6, WHITE); break;
    case 6: display.fillRect(40, 48, 2, 6, WHITE); break;
    case 7: display.fillRect(45, 48, 2, 6, WHITE); break;
    case 8: display.fillRect(50, 48, 2, 6, WHITE); break;
    case 9: display.fillRect(55, 48, 2, 6, WHITE); break;
    case 10: display.fillRect(60, 48, 2, 6, WHITE); break;
    case 11: display.fillRect(65, 48, 2, 6, WHITE); break;
    case 12: display.fillRect(70, 48, 2, 6, WHITE); break;
    case 13: display.fillRect(75, 48, 2, 6, WHITE); break;
    case 14: display.fillRect(80, 48, 2, 6, WHITE); break;
  }

  display.setCursor(0, 57);
  display.print("SM");
  switch (x) {
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
  }
}
