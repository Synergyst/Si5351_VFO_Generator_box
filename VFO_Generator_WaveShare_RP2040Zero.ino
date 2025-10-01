#include <Adafruit_SSD1306.h>   // Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_NeoPixel.h>  // WS2812 LED
#include <Wire.h>               // I2C
#include <math.h>               // Standard math library
#define LED_PIN 16
#define LED_COUNT 1
#define IS_RGBW true
#if IS_RGBW
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
#else
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif
//------------------------------------------------------------------------------------------------------------
#define IF 10700  // IF in kHz
#define BAND_INIT 18
#define XT_CAL_F 0
#define S_GAIN 505
#define BAUD 9600
#define rotLeft 7   // The pin used by rotary-left input.
#define rotRight 6  // The pin used by rotary-right input.
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
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
Adafruit_SSD1306 displayAlt = Adafruit_SSD1306(128, 64, &Wire);
// Encoder state
static uint8_t encPrev = 0;
static int8_t encAccum = 0;
static const int8_t encTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};
// --------- Line Reader (newline-terminated) ----------
static const size_t CMD_BUF_SZ = 128;

// Serial1 (Nano link)
static char s1Buf[CMD_BUF_SZ];
static size_t s1Len = 0;

// USB Serial (console)
static char sUsbBuf[CMD_BUF_SZ];
static size_t sUsbLen = 0;
// --------- SYNC helpers ----------
static inline uint32_t fnv1aMix32(uint32_t h, uint32_t v) {
  h ^= v;
  h *= 16777619UL;
  return h;
}
static uint32_t computeSyncHashRP() {
  uint32_t h = 2166136261UL;
  h = fnv1aMix32(h, (uint32_t)freq);
  h = fnv1aMix32(h, (uint32_t)interfreq);
  h = fnv1aMix32(h, (uint32_t)BAND_INIT);
  h = fnv1aMix32(h, (uint32_t)S_GAIN);
  h = fnv1aMix32(h, (uint32_t)freqold);
  h = fnv1aMix32(h, (uint32_t)fstep);
  h = fnv1aMix32(h, (uint32_t)x);
  return h;
}

// UI scanner gate
static bool uiScanOn = false;
static uint8_t uiScanSrc = 0;          // 0 = hard list, 1 = custom list
static uint32_t uiScanDelayMs = 5000;  // per-step delay (ms)
static uint32_t uiScanLastMs = 0;
static size_t uiScanIdx = 0;
static uint32_t uiScanCurrFreqHz = 0;
static bool uiScanPending = false;

// Same hard-coded list as on Nano (Hz)
static const uint32_t uiHardScanList[] PROGMEM = {
  /*  // Time signals (WWV/WWVH/WWVB/CHU)
  60000UL,     // 0.060 MHz - WWVB (US time standard, LF)
  2500000UL,   // 2.5 MHz  - WWV/WWVH
  3330000UL,   // 3.33 MHz - CHU (Canada)
  5000000UL,   // 5 MHz    - WWV/WWVH
  7850000UL,   // 7.85 MHz - CHU
  10000000UL,  // 10 MHz   - WWV/WWVH
  14670000UL,  // 14.67 MHz- CHU
  15000000UL,  // 15 MHz   - WWV/WWVH
  20000000UL,  // 20 MHz   - WWV

  // Maritime MF distress/calling (SSB/DSC)
  2182000UL,  // 2.182 MHz - International MF distress/calling (voice/SSB)
  2187500UL,  // 2.1875 MHz- MF DSC (digital selective calling)

  // Amateur HF digital/calling/nets
  3573000UL,   // 3.573 MHz - 80m FT8
  7074000UL,   // 7.074 MHz - 40m FT8
  10136000UL,  // 10.136 MHz- 30m FT8
  14074000UL,  // 14.074 MHz- 20m FT8
  14100000UL,  // 14.100 MHz- IBP beacon segment
  14300000UL,  // 14.300 MHz- Maritime Mobile Net / common 20m net freq
  18100000UL,  // 18.100 MHz- 17m FT8
  18110000UL,  // 18.110 MHz- IBP beacons (17m)
  21074000UL,  // 21.074 MHz- 15m FT8
  21150000UL,  // 21.150 MHz- IBP beacons (15m)
  24915000UL,  // 24.915 MHz- 12m FT8
  24930000UL,  // 24.930 MHz- IBP beacons (12m)
  28074000UL,  // 28.074 MHz- 10m FT8
  28200000UL,  // 28.200 MHz- 10m beacons (IBP + others)

  // CB radio (AM; some SSB activity)
  26965000UL,  // 26.965 MHz - CB Ch 1
  27025000UL,  // 27.025 MHz - CB Ch 6 (AM "Super Bowl")
  27065000UL,  // 27.065 MHz - CB Ch 9 (Emergency)
  27185000UL,  // 27.185 MHz - CB Ch 19 (Highway)
  27385000UL,  // 27.385 MHz - CB Ch 38 LSB (SSB calling/hangout)
  27405000UL,  // 27.405 MHz - CB Ch 40

  // 6 meters (50 MHz amateur)
  50125000UL,  // 50.125 MHz - 6m SSB calling
  50313000UL,  // 50.313 MHz - 6m FT8
  52525000UL,  // 52.525 MHz - 6m FM simplex calling

  // Aviation (AM)
  121500000UL,  // 121.500 MHz - International airband emergency (VHF Guard)
  122200000UL,  // 122.200 MHz - Common Flight Service / GA use
  122800000UL,  // 122.800 MHz - Common CTAF/UNICOM (GA)
  123450000UL,  // 123.450 MHz - Air-to-air (informal, often used)

  // NOAA weather satellites (APT) - if your receiver supports them
  137100000UL,  // 137.100 MHz - NOAA/Met APT downlink (NOAA-19)
  137620000UL,  // 137.620 MHz - NOAA-15 APT
  137912500UL,  // 137.9125 MHz- NOAA-18 APT

  // 2 meters (144–148 MHz amateur)
  144200000UL,  // 144.200 MHz - 2m SSB calling
  144390000UL,  // 144.390 MHz - APRS (USA)
  145800000UL,  // 145.800 MHz - ISS voice downlink (worldwide)
  145825000UL,  // 145.825 MHz - ISS packet/APRS digipeater
  146520000UL,  // 146.520 MHz - 2m FM national simplex calling (USA)

  // VHF public safety interop (NFM; USA nationwide)
  151137500UL,  // 151.1375 MHz - VTAC11
  154265000UL,  // 154.2650 MHz - Fireground White
  154280000UL,  // 154.2800 MHz - Fire Mutual Aid (VFIRE21)
  154295000UL,  // 154.2950 MHz - Fireground Red
  154452500UL,  // 154.4525 MHz - VTAC12
  155160000UL,  // 155.1600 MHz - National SAR/EMS (often used)
  155340000UL,  // 155.3400 MHz - EMS-to-hospital (HEAR, legacy)
  155370000UL,  // 155.3700 MHz - Intercity/Mutual Aid (legacy)
  155475000UL,  // 155.4750 MHz - National Law (NLEEF/NLEEC)
  155752500UL,  // 155.7525 MHz - VCALL10

  // Marine VHF (FM)
  156450000UL,  // 156.450 MHz - Marine Ch 9 (Hailing)
  156650000UL,  // 156.650 MHz - Marine Ch 13 (Bridge-to-bridge)
  156800000UL,  // 156.800 MHz - Marine Ch 16 (Distress/Calling)
  157100000UL,  // 157.100 MHz - Marine Ch 22A (USCG liaison)

  // Railroad (N. America AAR channels; NFM)
  160215000UL,  // 160.215 MHz - AAR Ch 01 (road)
  160230000UL,  // 160.230 MHz - AAR Ch 08 (road)
  160800000UL,  // 160.800 MHz - AAR Ch 24 (road)
  161550000UL,  // 161.550 MHz - AAR Ch 96 (road)

  // NOAA Weather Radio (FM)
  162400000UL,  // 162.400 MHz - WX Ch 1
  162425000UL,  // 162.425 MHz - WX Ch 2
  162450000UL,  // 162.450 MHz - WX Ch 3
  162475000UL,  // 162.475 MHz - WX Ch 4
  162500000UL,  // 162.500 MHz - WX Ch 5
  162525000UL,  // 162.525 MHz - WX Ch 6
  162550000UL   // 162.550 MHz - WX Ch 7
  */


  96300000UL,   // 96.3 MHz -
  102500000UL,  // 102.5 MHz -
  103300000UL,  // 103.3 MHz -
  106500000UL,  // 106.5 MHz -
  107700000UL,  // 107.7 MHz -
  //162550000UL   // 162.550 MHz - NOAA WX Ch 7
};
static const size_t uiHardScanLen = sizeof(uiHardScanList) / sizeof(uiHardScanList[0]);
static const size_t UI_MAX_CUSTOM_SCAN = 128;
static uint32_t uiCustomScanList[UI_MAX_CUSTOM_SCAN];
static size_t uiCustomScanLen = 0;

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
    } else if (encAccum <= -4) {
      set_frequency(-1);
      encAccum = 0;
    }
  }
  encPrev = curr;
}
bool readLine(Stream& s, char* outBuf, size_t& inoutLen, size_t outSz) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\r') {
      continue;
    } else if (c == '\n') {
      if (inoutLen < outSz) outBuf[inoutLen] = '\0';
      else outBuf[outSz - 1] = '\0';
      inoutLen = 0;
      return true;
    } else {
      if (inoutLen < (outSz - 1)) {
        outBuf[inoutLen++] = c;
      } else {
        inoutLen = 0;  // overflow -> reset
      }
    }
  }
  return false;
}
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
inline void bootExplosion(Adafruit_SSD1306& d) {
  bootExplosion(d, 350);
}
void bootExplosion(Adafruit_SSD1306& d, uint16_t vibrateMs) {
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
// --------- Command handlers ----------
void sendHelpRP() {
  Serial.println("RP2040Zero Help:");
  Serial.println("  HELP | ? | H         - show this help");
  Serial.println("  _ <text>             - print <text> to console (non-command)");
  Serial.println("  Q                    - send full state to Nano");
  Serial.println("  R / L                - step frequency right/left (one detent)");
  Serial.println("  T                    - cycle tuning step");
  Serial.println("  B                    - cycle band preset");
  Serial.println("  K                    - (from Nano) retuned notification");
  Serial.println("  P                    - heartbeat from Nano (ignored here)");
  Serial.println("  TX / RX              - force TX (IF=0) / RX (IF=IF_kHz)");
  Serial.println("  READY?               - ask Nano to reply READY");
  Serial.println("  PING                 - ask Nano to reply PONG");
  Serial.println("  F <Hz>               - set frequency (Hz)");
  Serial.println("  IF <kHz>             - set IF (kHz)");
  Serial.println("  SM <1..14>           - set signal meter bucket");
  Serial.println("Notes:");
  Serial.println("  - Lines starting with '_' are printed to console only.");
  Serial.println("  - SCAN-related commands run on Nano; UI mirrors only after Nano ack.");
}
inline void s1SendKV(const char* key, long val) {
  Serial1.print(key);
  Serial1.print(' ');
  Serial1.println(val);
}
void sendStateToNano() {
  s1SendKV("F", (long)freq);             // Hz
  s1SendKV("IF", (long)interfreq);       // kHz
  s1SendKV("STP", (long)stp);            // 1..6 step index
  s1SendKV("BAND", (long)count);         // 1..21
  s1SendKV("STS", (long)(sts ? 1 : 0));  // 0=RX, 1=TX
  s1SendKV("SM", (long)x);               // 1..14
  s1SendKV("N", (long)n);                // 1..42
  // Extra fields for sync
  s1SendKV("BAND_INIT", (long)BAND_INIT);
  s1SendKV("S_GAIN", (long)S_GAIN);
  s1SendKV("FOLD", (long)freqold);
  s1SendKV("FSTEP", (long)fstep);
  s1SendKV("SMVAL", (long)x);
  Serial1.println("OK");
  // Send sync hash last
  uint32_t h = computeSyncHashRP();
  Serial1.print("SYNC RP ");
  Serial1.println(h);
}

/* ====================== UI Scanner Mirror (display-only) ====================== */
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
  uiScanPending = false;
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
/* ==================== End UI Scanner Mirror (display-only) ==================== */

void handleCommand(const char* line, Stream& io) {
  if (line[0] == '\0') return;
  const bool fromUSB = (&io == &Serial);

  if (line[0] == '_' && !fromUSB) {
    const char* msg = line + 1;
    if (*msg == ' ') ++msg;
    Serial.println(msg);
    return;
  }

  // SYNC request from Nano
  if (!strcmp(line, "SYNC?")) {
    sendStateToNano();  // includes SYNC RP <hash>
    time_now = millis();
    return;
  }
  // Nano scan acks control the UI mirror start/stop
  if (!strncmp(line, "_ SCAN ", 7)) {
    long v = atol(line + 7);
    if (v == 1) {
      uiScanPending = false;
      uiScanStart();
    } else {
      uiScanStop();
    }
    return;
  }

  if (!strcmp(line, "Q") && !fromUSB) {
    sendStateToNano();
    return;
  }
  if (!strcmp(line, "R") && !fromUSB) {
    set_frequency(1);
    return;
  }
  if (!strcmp(line, "L") && !fromUSB) {
    set_frequency(-1);
    return;
  }
  if (!strcmp(line, "T") && !fromUSB) {
    time_now = (millis() + 300);
    setstep();
    delay(300);
    return;
  }
  if (!strcmp(line, "B") && !fromUSB) {
    time_now = (millis() + 300);
    inc_preset();
    delay(300);
    return;
  }
  if (!strcmp(line, "K") && !fromUSB) {
    // When we successfully retune
    Serial.print("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.printf(": Nano MCU retuned to: %u\n", freq);
    time_now = millis();
    return;
  }
  if (!strcmp(line, "F?") && fromUSB) {
    Serial.print("RP2040Zero ");
    if (fromUSB) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.printf(": tuned to: %u\n", freq);
    Serial1.print("F?\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "P")) {
    time_now = millis();
    return;
  }
  if (!strcmp(line, "TX") && !fromUSB) {
    sts = 1;
    s1SendKV("IF", 0);
    time_now = millis();
    return;
  }
  if (!strcmp(line, "RX") && !fromUSB) {
    sts = 0;
    s1SendKV("IF", interfreq);
    time_now = millis();
    return;
  }
  if (!strcmp(line, "OK") && !fromUSB) {
    Serial.print("RP2040Zero ");
    if (fromUSB) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.print(": got OK from Nano MCU\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "SHK") && !fromUSB) {
    /*Serial.print("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.print(": SYNCHASH OK\n");
    display.setTextSize(1);
    display.setCursor(0, 9);
    display.print(" ");
    display.display();*/
    time_now = millis();
    return;
  }
  if (!strcmp(line, "SHB") && !fromUSB) {
    Serial.print("RP2040Zero ");
    if (fromUSB) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.print(": SYNCHASH BAD\n");
    display.setTextSize(1);
    display.setCursor(0, 9);
    display.print("!");
    display.display();
    time_now = millis();
    return;
  }
  if (!strcmp(line, "PONG")) {
    Serial.print("RP2040Zero ");
    if (fromUSB) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.print(": got PONG from Nano MCU\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "READY")) {
    Serial.print("RP2040Zero ");
    if (fromUSB) Serial.print("(from USB)");
    else Serial.print("(from Nano)");
    Serial.print(": got READY from Nano MCU\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "READY?")) {
    Serial1.print("READY?\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "PING")) {
    Serial1.print("PING\n");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "HELP") || !strcmp(line, "?") || !strcmp(line, "H")) {
    sendHelpRP();
    Serial.println();
    Serial1.print("?\n");
    return;
  }

  // Scanner UI mirror + forward
  if (!strcmp(line, "SCAN?")) {
    Serial.println("_ UI Scan values (RP2040 mirror):");
    Serial.print("_ SCAN ");
    Serial.println(uiScanOn ? 1 : 0);
    Serial.print("_ SRC ");
    Serial.println(uiScanSrc ? 1 : 0);
    Serial.print("_ DELAY ");
    Serial.println((long)uiScanDelayMs);
    Serial.print("_ LEN ");
    Serial.println((long)uiCurrentListLen());
    Serial1.println("SCAN?");
    time_now = millis();
    return;
  }
  if (!strncmp(line, "SLCLR", 5)) {
    uiCustomScanLen = 0;
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  if (!strncmp(line, "SLADD ", 6)) {
    long v = atol(line + 6);
    if (v > 0 && uiCustomScanLen < UI_MAX_CUSTOM_SCAN) uiCustomScanList[uiCustomScanLen++] = (uint32_t)v;
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  if (!strncmp(line, "SDELAY ", 7)) {
    long v = atol(line + 7);
    if (v < 10) v = 10;
    uiScanDelayMs = (uint32_t)v;
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  if (!strncmp(line, "SCAN USE ", 9)) {
    const char* p = line + 9;
    if (!strcasecmp(p, "H") || !strcasecmp(p, "HARD")) uiScanUse(0);
    else if (!strcasecmp(p, "C") || !strcasecmp(p, "CUSTOM")) uiScanUse(1);
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  if (!strncmp(line, "SCAN ", 5)) {
    char buf[CMD_BUF_SZ];
    strncpy(buf, line, CMD_BUF_SZ - 1);
    buf[CMD_BUF_SZ - 1] = '\0';
    char* toks[6] = { 0 };
    int nt = 0;
    char* savep = nullptr;
    char* tk = strtok_r(buf, " ", &savep);
    while (tk && nt < 6) {
      toks[nt++] = tk;
      tk = strtok_r(nullptr, " ", &savep);
    }
    if (nt >= 2 && !strcasecmp(toks[1], "STOP")) {
      uiScanStop();  // stop mirror immediately
      if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
      return;
    }
    if (nt >= 2 && !strcasecmp(toks[1], "START")) {
      uint8_t src = uiScanSrc;
      uint32_t delayMs = uiScanDelayMs;
      for (int i = 2; i < nt; ++i) {
        if (!toks[i]) continue;
        if (!strcasecmp(toks[i], "H") || !strcasecmp(toks[i], "HARD")) src = 0;
        else if (!strcasecmp(toks[i], "C") || !strcasecmp(toks[i], "CUSTOM")) src = 1;
        else if (!strcasecmp(toks[i], "D")) {
          if (i + 1 < nt) {
            long v = atol(toks[i + 1]);
            if (v >= 10) delayMs = (uint32_t)v;
            i++;
          }
        } else if (toks[i][0] >= '0' && toks[i][0] <= '9') {
          long v = atol(toks[i]);
          if (v >= 10) delayMs = (uint32_t)v;
        }
      }
      uiScanUse(src);
      uiScanDelayMs = delayMs;
      uiScanPending = true;  // wait for Nano ack
      if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
      return;
    }
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }

  // KEY [VALUE] numeric
  char key[8] = { 0 };
  long val = 0;
  int matched = sscanf(line, "%7s %ld", key, &val);
  if (matched >= 1) {
    if (!strcmp(key, "F") && matched == 2) {
      if (val < 10000) val = 10000;
      if ((unsigned long)val > 225000000UL) val = 225000000UL;
      freq = (unsigned long)val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "IF") && matched == 2) {
      if (val < 0) val = 0;
      if (val > 200000) val = 200000;
      interfreq = val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "SM") && matched == 2) {
      if (val < 1) val = 1;
      if (val > 14) val = 14;
      x = (byte)val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "OK-F") && matched == 2 && !fromUSB) {
      Serial.print("RP2040Zero ");
      if (fromUSB) Serial.print("(from USB)");
      else Serial.print("(from Nano)");
      Serial.printf(": OK-F %u\n", (long)val);
      time_now = millis();
      return;
    }
  }
  // Unknown command -> optionally report over USB and forward if from USB
  Serial.print("RP2040Zero ");
  if (fromUSB) Serial.print("(from USB)");
  else Serial.print("(from Nano)");
  Serial.print(": Unknown cmd: ");
  Serial.println(line);
  if (fromUSB) {
    Serial.println("Passing to Nano since command came from USB console..");
    Serial1.println(line);
  }
}

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

void setup() {
  Serial.begin(BAUD);
  Serial1.begin(BAUD);
  Serial.println("RP2040Zero (boot): Starting now..");
  pinMode(rotLeft, INPUT_PULLUP);
  pinMode(rotRight, INPUT_PULLUP);
  {
    uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
    uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
    encPrev = (a << 1) | b;
    encAccum = 0;
  }
  strip.begin();
  strip.show();
  strip.setBrightness(255);
  strip.setPixelColor(0, 255, 255, 255);
  strip.show();
  Wire.setClock(100000);
  Wire.begin();
  count = BAND_INIT;
  bandpresets();
  stp = 4;
  setstep();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();
  display.setTextSize(1);

  displayAlt.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  displayAlt.clearDisplay();
  displayAlt.setTextColor(WHITE);
  displayAlt.display();
  displayAlt.setTextSize(1);

  displayAlt.setCursor(10, 32);
  displayAlt.println("Ensure power switch");
  displayAlt.setCursor(48, 42);
  displayAlt.print("is on!");
  displayAlt.display();

  display.setCursor(0, 0);
  animateTextToCenterAndCompress(display, "VFO Generator", 25, 0, 750, 250, 0.3f, 0.025f);
  bootExplosion(display, 400);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("RP2040 I2C device(s):");
  display.setCursor(0, 12);
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
  delay(750);
  bool nanoReady = false;
  while (!nanoReady) {
    delay(500);
    if (readLine(Serial1, s1Buf, s1Len, CMD_BUF_SZ)) {
      if (!strcmp(s1Buf, "P")) {
        Serial.println("RP2040Zero (boot): Nano is alive..");
        time_now = millis();
        nanoReady = true;
        display.setCursor(0, 0);
        display.clearDisplay();
        display.print("Nano is awake");
        display.display();
        delay(500);
        // Immediately send full state and sync hash
        sendStateToNano();
        break;
      }
      Serial.print("RP2040Zero (boot): Unknown cmd: ");
      Serial.println(s1Buf);
    } else {
      display.setCursor(0, 0);
      display.clearDisplay();
      display.print("Waiting for Nano..");
      display.setCursor(0, 12);
      display.print(millis());
      display.setCursor(0, 24);
      display.print("ENSURE CORRECT BOOT:");
      display.setCursor(0, 36);
      display.print("PWR SW ON, AND THEN");
      display.setCursor(0, 48);
      display.print("PLUG USB CABLE(S) IN!");
      display.display();
      delay(500);
      Serial1.print("P");
      delay(500);
      Serial1.print("\n");
      delay(500);
    }
  }
  displayAlt.clearDisplay();
  displayAlt.display();
  Serial.println("RP2040Zero (boot): Running loop now..\n");
}

void loop() {
  if (readLine(Serial1, s1Buf, s1Len, CMD_BUF_SZ)) {
    handleCommand(s1Buf, Serial1);
  }
  if (readLine(Serial, sUsbBuf, sUsbLen, CMD_BUF_SZ)) {
    handleCommand(sUsbBuf, Serial);
  }
  pollEncoder();
  uiScanTick();
  if (freqold != freq) {
    time_now = millis();
    tunegen();
    freqold = freq;
  }
  if (interfreqold != interfreq) {
    time_now = millis();
    // Make sure IF kHz is sent to Nano when it changes
    if (interfreq != lastSentIF) {
      s1SendKV("IF", interfreq);
      lastSentIF = interfreq;
    }
    tunegen();  // still okay to send F (or you can skip if not needed)
    interfreqold = interfreq;
  }
  if (xo != x) {
    time_now = millis();
    xo = x;
  }
  static unsigned long lastDraw = 0;
  const unsigned long drawPeriod = 100;
  if (millis() - lastDraw >= drawPeriod) {
    lastDraw = millis();
    displayfreq();
    layout();
  }
}

void tunegen() {
  if (uiScanOn) return;  // Mirror mode: don’t drive the Nano
  Serial1.printf("F %u\n", (unsigned long)freq);
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
    case 15: freq = 27015000; break;
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
