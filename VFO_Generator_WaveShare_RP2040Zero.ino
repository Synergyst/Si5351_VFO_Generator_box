#include <Adafruit_SSD1306.h>   // Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_NeoPixel.h>  // WS2812 LED
#include <Rotary.h>             // Ben Buxton https://github.com/brianlow/Rotary
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
long cal = XT_CAL_F;
unsigned int smval;
byte encoder = 1;
byte stp, n = 1;
byte count, x, xo;
bool sts = 0;
unsigned int period = 100;
unsigned long time_now = 0;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
Rotary r = Rotary(rotRight, rotLeft);

// Encoder state
static uint8_t encPrev = 0;  // previous 2-bit state AB
static int8_t encAccum = 0;  // accumulate transitions to 1 detent
static const int8_t encTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

// --------- Line Reader (newline-terminated) ----------
static const size_t CMD_BUF_SZ = 255;
static char s1Buf[CMD_BUF_SZ];
static size_t s1Len = 0;

inline void pollEncoder() {
  // --------- Rotary handling ----------
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
  // Easing and scaled blit helper
  return 1.0f - powf(1.0f - t, 3.0f);
}
static void drawBitmapScaled1(Adafruit_SSD1306& d,
                              const uint8_t* buf, int bw, int bh,
                              int cx, int cy, float sx, float sy) {
  // Draw a 1bpp bitmap scaled around its center (sx/sy can be <1 for compression)
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
  // Move text from (startX,startY) top-left to the center, then compress.
  // minScaleY: final vertical scale (e.g., 0.35f); maxStretchX: optional horizontal stretch during squash.
  // moveMs/compressMs: durations in ms.
  // Uses the current GFX font & size to rasterize text into a 1bpp canvas, then blits without printing.
  if (!text || !*text) return;

  // Measure text bounds with current font
  int16_t x1, y1;
  uint16_t w, h;
  d.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  if (w == 0 || h == 0) return;

  // Render text into offscreen 1bpp canvas (no direct printing to display)
  GFXcanvas1 canvas(w, h);
  canvas.fillScreen(0);
  canvas.setTextColor(1);
  canvas.setTextSize(1);
  // Align so that text's bounding box sits at (0,0)
  canvas.setCursor(-x1, -y1);
  canvas.print(text);
  const uint8_t* buf = canvas.getBuffer();

  const int W = d.width();
  const int H = d.height();
  const int targetX = (W - (int)w) / 2;  // top-left at center
  const int targetY = (H - (int)h) / 2;

  // 1) Move: translate from start to center with easing
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

  // 2) Compress at center: vertical squash with slight horizontal stretch
  {
    const int cx = targetX + (int)w / 2;
    const int cy = targetY + (int)h / 2;
    const uint8_t dt = 16;
    for (uint16_t t = 0; t <= compressMs; t += dt) {
      float u = easeOutCubic(fminf(1.0f, (float)t / (float)compressMs));
      float sy = 1.0f + (minScaleY - 1.0f) * u;    // 1 -> minScaleY
      float sx = 1.0f + (maxStretchX - 1.0f) * u;  // 1 -> maxStretchX

      d.clearDisplay();
      drawBitmapScaled1(d, buf, w, h, cx, cy, sx, sy);
      d.display();
      delay(dt);
      yield();
    }
  }
}

inline void bootExplosion(Adafruit_SSD1306& d) {
  // Convenience overload: keeps existing calls working (defaults to ~350 ms vibrate)
  bootExplosion(d, 350);
}
void bootExplosion(Adafruit_SSD1306& d, uint16_t vibrateMs) {
  const int W = d.width();
  const int H = d.height();
  const float cx = W / 2.0f;
  const float cy = H / 2.0f;

  // 1) Grow a filled circle in the center
  d.clearDisplay();
  const int finalR = 10;
  for (int r = 2; r <= finalR; r += 2) {
    d.clearDisplay();
    d.fillCircle((int)cx, (int)cy, r, WHITE);
    d.display();
    delay(35);
  }

  // 1.5) Vibrate in-place for vibrateMs
  // Small sub-pixel jitter with a bit of randomness to feel organic.
  {
    const uint8_t frameDt = 16;  // ~60 FPS
    const float ampX = 1.2f;     // max ~±1.2 px
    const float ampY = 1.0f;     // max ~±1.0 px
    for (uint16_t t = 0; t < vibrateMs; t += frameDt) {
      float phase = (float)t * 0.06f;
      float dx = sinf(phase * 3.1f) * ampX + ((float)random(-10, 11)) * 0.03f;  // ±0.3 px noise
      float dy = cosf(phase * 2.7f) * ampY + ((float)random(-10, 11)) * 0.03f;

      d.clearDisplay();
      d.fillCircle((int)roundf(cx + dx), (int)roundf(cy + dy),
                   finalR + ((t >> 4) & 1), WHITE);  // tiny radius wobble
      d.display();
      delay(frameDt);
      yield();
    }
  }

  // 2) Explode into particles (your tuned values preserved)
  const int N = 1600;  // number of particles
  struct Particle {
    float x, y, vx, vy;
    uint16_t life;
  };
  Particle p[N];

  // Seed PRNG
  randomSeed((uint32_t)micros());

  const float startR = 8.0f;  // radius of initial ring where particles spawn
  for (int i = 0; i < N; ++i) {
    float ang = (float)random(0, 6283) * 0.001f;           // ~0..2π
    float r = startR + (float)random(-200, 201) * 0.005f;  // ±1 px variation
    float speed = 0.6f + (float)random(0, 140) * 0.01f;    // 0.6..1.99 px/frame
    float ca = cosf(ang);
    float sa = sinf(ang);
    p[i].x = cx + r * ca;
    p[i].y = cy + r * sa;
    p[i].vx = ca * speed;
    p[i].vy = sa * speed;
    p[i].life = 110 + (uint16_t)random(0, 30);  // frames alive
  }

  const float g = 0.039087f;      // gravity (px/frame^2)
  const float drag = 0.9999996f;  // mild air drag
  const int frames = 116;         // total frames to render

  for (int frame = 0; frame < frames; ++frame) {
    d.clearDisplay();

    for (int i = 0; i < N; ++i) {
      if (p[i].life == 0) continue;

      // Draw
      int xi = (int)(p[i].x + 0.5f);
      int yi = (int)(p[i].y + 0.5f);
      if ((uint32_t)xi < (uint32_t)W && (uint32_t)yi < (uint32_t)H) {
        d.drawPixel(xi, yi, WHITE);
      }

      // Integrate motion
      p[i].vx *= drag;
      p[i].vy = p[i].vy * drag + g;
      p[i].x += p[i].vx;
      p[i].y += p[i].vy;

      // Retire once well off-screen or life out
      if (p[i].y > (H + 10) || p[i].x < -10 || p[i].x > (W + 10)) {
        p[i].life = 0;
      } else {
        --p[i].life;
      }
    }

    d.display();
    delay(16);  // ~60 FPS
    yield();
  }
}

// --------- Command handlers ----------
void sendHelpRP() {
  // --------- Console Help (RP2040) ----------
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
  Serial.println("  - SCAN-related commands are implemented on the Nano side.");
}

inline void s1SendKV(const char* key, long val) {
  Serial1.print(key);
  Serial1.print(' ');
  Serial1.println(val);
}

void sendStateToNano() {
  s1SendKV("F", (long)freq);             // Hz
  s1SendKV("IF", (long)interfreq);       // kHz
  s1SendKV("STP", (long)stp);            // 1..6 (your step index)
  s1SendKV("BAND", (long)count);         // 1..21
  s1SendKV("STS", (long)(sts ? 1 : 0));  // 0=RX, 1=TX
  s1SendKV("SM", (long)x);               // 1..14 signal meter bucket
  s1SendKV("N", (long)n);                // 1..42 tune pointer
  Serial1.println("OK");                 // end of state dump
}

/* ====================== UI Scanner Mirror (display-only) ======================
   - Mirrors Nano scanner locally for OLED only (no retuning here).
   - Starts/stops when SCAN commands are issued to RP2040 USB console; the code
     forwards those to the Nano so both start together.
   - Pauses automatically in TX (sts==1), matching Nano behavior.
*/
static bool uiScanOn = false;
static uint8_t uiScanSrc = 0;          // 0 = hard list, 1 = custom list
static uint32_t uiScanDelayMs = 1000;  // per-step delay (ms)
static uint32_t uiScanLastMs = 0;
static size_t uiScanIdx = 0;
static uint32_t uiScanCurrFreqHz = 0;

// Same hard-coded list as on Nano (Hz)
static const uint32_t uiHardScanList[] = {
  1000000UL,   // 1.000 MHz
  1600000UL,   // 1.600 MHz
  3500000UL,   // 3.500 MHz
  7000000UL,   // 7.000 MHz
  10000000UL,  // 10.000 MHz
  14000000UL,  // 14.000 MHz
  18068000UL,  // 18.068 MHz
  21000000UL,  // 21.000 MHz
  24890000UL,  // 24.890 MHz
  28000000UL   // 28.000 MHz
};
static const size_t uiHardScanLen = sizeof(uiHardScanList) / sizeof(uiHardScanList[0]);

static const size_t UI_MAX_CUSTOM_SCAN = 16;
static uint32_t uiCustomScanList[UI_MAX_CUSTOM_SCAN];
static size_t uiCustomScanLen = 0;

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
  if (uiCurrentListLen() == 0) {
    // No local list -> nothing to show (Nano will still run)
    return;
  }
  uiScanOn = true;
  uiScanIdx = 0;
  // Make first visual step happen immediately:
  uiScanLastMs = millis() - uiScanDelayMs;
}
static void uiScanStop() {
  uiScanOn = false;
}
static void uiScanTick() {
  if (!uiScanOn) return;
  // Pause UI scanning during TX (matches Nano behavior)
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
    freq = uiScanCurrFreqHz;  // Store current scanner frequency to freq variable!
    uiScanIdx = (uiScanIdx + 1) % len;
  }
}
static inline unsigned long uiDisplayFreqHz() {
  // If UI scanning is on, show mirror freq; otherwise show RP’s current freq
  if (uiScanOn) {
    if (uiScanCurrFreqHz != 0) return uiScanCurrFreqHz;
    // If we just started and haven't ticked yet, show first item so UI isn't blank
    if (uiCurrentListLen() > 0) return uiCurrentListPtr()[0];
  }
  return freq;
}
static void drawScanUiOverlay() {
  if (!uiScanOn) return;
  // Small "S" tag and a tiny progress indicator at the very top so it doesn't
  // collide too much with the big frequency print (which starts at y=1).
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("S");
  size_t len = uiCurrentListLen();
  if (len > 0) {
    // Tiny 31px progress bar at top-right
    const int barX0 = 96;
    const int barW = 31;
    const int barY = 0;
    display.drawRect(barX0, barY, barW, 2, WHITE);
    int fillW = 0;
    // Use next index position as progress
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

  // Single-letter commands
  if (!strcmp(line, "Q")) {
    sendStateToNano();
    return;
  }
  if (!strcmp(line, "R")) {
    set_frequency(1);
    return;
  }
  if (!strcmp(line, "L")) {
    set_frequency(-1);
    return;
  }
  if (!strcmp(line, "T")) {
    time_now = (millis() + 300);
    setstep();
    delay(300);
    return;
  }
  if (!strcmp(line, "B")) {
    time_now = (millis() + 300);
    inc_preset();
    delay(300);
    return;
  }
  if (!strcmp(line, "K")) {
    Serial.print("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) {
      Serial.print("(from USB)");
    } else {
      Serial.print("(from Nano)");
    }
    Serial.printf(": Nano MCU retuned to: %u\n", freq);
    time_now = millis();
    return;
  }
  if (!strcmp(line, "P")) {
    time_now = millis();
    return;
  }
  if (!strcmp(line, "TX")) {
    sts = 1;
    s1SendKV("IF", 0);    // make Nano’s IF = 0
    time_now = millis();  // cause UI refresh soon
    return;
  }
  if (!strcmp(line, "RX")) {
    sts = 0;
    s1SendKV("IF", interfreq);  // restore IF
    time_now = millis();
    return;
  }
  if (!strcmp(line, "OK")) {
    Serial.println("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) {
      Serial.print("(from USB)");
    } else {
      Serial.print("(from Nano)");
    }
    Serial.print(": got OK from Nano MCU");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "PONG")) {
    Serial.println("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) {
      Serial.print("(from USB)");
    } else {
      Serial.print("(from Nano)");
    }
    Serial.print(": got PONG from Nano MCU");
    time_now = millis();
    return;
  }
  if (!strcmp(line, "READY")) {
    Serial.println("RP2040Zero ");
    if (&io == static_cast<Stream*>(&Serial)) {
      Serial.print("(from USB)");
    } else {
      Serial.print("(from Nano)");
    }
    Serial.print(": got READY from Nano MCU");
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

  // --- Console helper commands (RP2040-local) ---
  if (!strcmp(line, "HELP") || !strcmp(line, "?") || !strcmp(line, "H")) {
    sendHelpRP();
    Serial.println();
    Serial1.print("?\n");
    return;
  }
  if (line[0] == '_') {
    // Print message to USB console as a non-command
    const char* msg = line + 1;
    if (*msg == ' ') ++msg;  // trim one leading space after underscore
    Serial.println(msg);
    return;
  }

  // ----------------- Scanner (display-only mirror + forward) -----------------
  // Local UI status print and forward
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
    // Also ask Nano for its status
    Serial1.println("SCAN?");
    time_now = millis();
    return;
  }
  // Clear custom list (mirror) + forward to Nano
  if (!strncmp(line, "SLCLR", 5)) {
    uiCustomScanLen = 0;
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  // Add to custom list (mirror) + forward to Nano
  if (!strncmp(line, "SLADD ", 6)) {
    long v = atol(line + 6);
    if (v > 0 && uiCustomScanLen < UI_MAX_CUSTOM_SCAN) {
      uiCustomScanList[uiCustomScanLen++] = (uint32_t)v;
    }
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  // Set scan delay (mirror) + forward to Nano
  if (!strncmp(line, "SDELAY ", 7)) {
    long v = atol(line + 7);
    if (v < 10) v = 10;
    uiScanDelayMs = (uint32_t)v;
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  // Select source hard/custom (mirror) + forward to Nano
  if (!strncmp(line, "SCAN USE ", 9)) {
    const char* p = line + 9;
    if (!strcasecmp(p, "H") || !strcasecmp(p, "HARD")) {
      uiScanUse(0);
    } else if (!strcasecmp(p, "C") || !strcasecmp(p, "CUSTOM")) {
      uiScanUse(1);
    }
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  // START/STOP (mirror) + forward to Nano
  if (!strncmp(line, "SCAN ", 5)) {
    // tokenize a small buffer to check START/STOP and optional args
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
      uiScanStop();
      if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
      return;
    }
    if (nt >= 2 && !strcasecmp(toks[1], "START")) {
      // Optional args: H|C and/or <ms> or "D <ms>"
      uint8_t src = uiScanSrc;
      uint32_t delayMs = uiScanDelayMs;
      for (int i = 2; i < nt; ++i) {
        if (!toks[i]) continue;
        if (!strcasecmp(toks[i], "H") || !strcasecmp(toks[i], "HARD")) {
          src = 0;
        } else if (!strcasecmp(toks[i], "C") || !strcasecmp(toks[i], "CUSTOM")) {
          src = 1;
        } else if (!strcasecmp(toks[i], "D")) {
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
      uiScanStart();
      if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
      return;
    }
    // Unknown SCAN subcommand -> forward
    if (&io == static_cast<Stream*>(&Serial)) Serial1.println(line);
    return;
  }
  // --------------------------------------------------------------------------

  // KEY [VALUE] numeric
  char key[8] = { 0 };
  long val = 0;
  int matched = sscanf(line, "%7s %ld", key, &val);
  if (matched >= 1) {
    if (!strcmp(key, "F") && matched == 2) {
      // Frequency in Hz
      if (val < 10000) val = 10000;
      if ((unsigned long)val > 225000000UL) val = 225000000UL;
      freq = (unsigned long)val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "IF") && matched == 2) {
      // IF in kHz
      if (val < 0) val = 0;
      if (val > 200000) val = 200000;
      interfreq = val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "SM") && matched == 2) {
      // Signal meter bucket 1..14
      if (val < 1) val = 1;
      if (val > 14) val = 14;
      x = (byte)val;
      time_now = millis();
      return;
    } else if (!strcmp(key, "OK-F") && matched == 2) {
      Serial.print("RP2040Zero ");
      if (&io == static_cast<Stream*>(&Serial)) {
        Serial.print("(from USB)");
      } else {
        Serial.print("(from Nano)");
      }
      Serial.printf(": OK-F %u\n", (long)val);
      time_now = millis();
      return;
    }
  }

  // Unknown command -> optionally report over USB
  Serial.print("RP2040Zero ");
  if (&io == static_cast<Stream*>(&Serial)) {
    Serial.print("(from USB)");
  } else {
    Serial.print("(from Nano)");
  }
  Serial.print(": Unknown cmd: ");
  Serial.println(line);
  if (&io == static_cast<Stream*>(&Serial)) {
    Serial.println("Passing to Nano since command came from USB console..");
    Serial1.println(line);
  }
}

void set_frequency(short dir) {
  if (encoder == 1) {  // Up/Down frequency
    if (dir == 1) freq = freq + fstep;
    if (freq >= 225000000UL) freq = 225000000UL;
    if (dir == -1) freq = freq - fstep;
    if (fstep == 1000000 && freq <= 1000000) freq = 1000000;
    else if (freq < 10000) freq = 10000;
  }
  if (encoder == 1) {  // Up/Down graph tune pointer
    if (dir == 1) n = n + 1;
    if (n > 42) n = 1;
    if (dir == -1) n = n - 1;
    if (n < 1) n = 42;
  }
}

void setup() {
  Serial.begin(BAUD);   // USB CDC for debug
  Serial1.begin(BAUD);  // HW UART to Nano
  Serial.println("RP2040Zero (boot): Starting now..");

  pinMode(rotLeft, INPUT_PULLUP);
  pinMode(rotRight, INPUT_PULLUP);

  // Initialize encoder previous state
  {
    uint8_t a = (digitalRead(rotRight) == LOW) ? 1 : 0;
    uint8_t b = (digitalRead(rotLeft) == LOW) ? 1 : 0;
    encPrev = (a << 1) | b;
    encAccum = 0;
  }

  strip.begin();
  strip.show();
  strip.setBrightness(255);
  strip.setPixelColor(0, 255, 96, 128);
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
  display.setCursor(10, 32);
  display.println("Ensure power switch");
  display.setCursor(48, 42);
  display.print("is on!");
  display.display();
  display.setCursor(0, 0);
  animateTextToCenterAndCompress(display, "VFO Generator", 25, 12, 1250, 500, 0.0035f, 0.005f);
  // Then trigger your explosion animation
  bootExplosion(display, 400);  // or bootExplosion(display);

  // Prepare screen for I2C device list
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
        break;
      }
      // Unknown command -> optionally report over USB
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
  Serial.println("RP2040Zero (boot): Running loop now..\n");
}

void loop() {
  // Read complete commands from Nano on Serial1 (newline-terminated)
  if (readLine(Serial1, s1Buf, s1Len, CMD_BUF_SZ)) {
    handleCommand(s1Buf, Serial1);
  }
  // Read complete commands from RP2040Zero's USB on Serial (newline-terminated)
  if (readLine(Serial, s1Buf, s1Len, CMD_BUF_SZ)) {
    handleCommand(s1Buf, Serial);
  }

  pollEncoder();

  // Display-only UI scanner tick (does not retune hardware)
  uiScanTick();

  if (freqold != freq) {
    time_now = millis();
    tunegen();
    freqold = freq;
  }
  if (interfreqold != interfreq) {
    time_now = millis();
    tunegen();
    interfreqold = interfreq;
  }
  if (xo != x) {
    time_now = millis();
    xo = x;
  }

  static unsigned long lastDraw = 0;
  const unsigned long drawPeriod = 100;  // ms
  if (millis() - lastDraw >= drawPeriod) {
    lastDraw = millis();
    displayfreq();
    layout();
  }
}

void tunegen() {
  Serial1.printf("F %u\n", (unsigned long)freq);
  //Serial.printf("F %u\n", (unsigned long)freq);
}

void displayfreq() {
  // Use mirrored scanner freq for display when scanning; otherwise show local freq
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

  // Band name, bar graph, etc.
  bandlist();
  drawbargraph();

  // Draw scan UI overlay last so it sits on top
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
  // Pointer
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
  // Bargraph
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
  }
}
