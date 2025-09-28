#include <Adafruit_SSD1306.h>  // Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#define LED_PIN 16
#define LED_COUNT 1
#define IS_RGBW true
#if IS_RGBW
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
#else
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif
//------------------------------------------------------------------------------------------------------------
#define IF 10590  // IF in kHz
#define BAND_INIT 20
#define XT_CAL_F 0
#define S_GAIN 505
#define BAUD 9600
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
// --------- Line Reader (newline-terminated) ----------
static const size_t CMD_BUF_SZ = 64;
static char s1Buf[CMD_BUF_SZ];
static size_t s1Len = 0;
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
// --------- Command handlers ----------
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
void handleCommand(const char* line) {
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

  if (!strcmp(line, "TX")) {
    sts = 1;
    time_now = millis();  // cause UI refresh soon
    return;
  }
  if (!strcmp(line, "RX")) {
    sts = 0;
    time_now = millis();
    return;
  }
  if (!strcmp(line, "OK")) {
    Serial.println("RP2040Zero: got OK from Nano MCU");
    time_now = millis();
    return;
  }

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
      // ensure immediate update on next loop
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
      Serial.printf("RP2040Zero: OK-F %u\n", (long)val);
      // Frequency in Hz
      /*if (val < 10000) val = 10000;
      if ((unsigned long)val > 225000000UL) val = 225000000UL;
      freq = (unsigned long)val;*/
      // ensure immediate update on next loop
      time_now = millis();
      return;
    }
  }
  // Unknown command -> optionally report over USB
  Serial.print("Unknown cmd: ");
  Serial.println(line);
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
  delay(10000);
  Serial.println("RP2040Zero: Starting now..");
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
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("RP2040Zero: Detected I2C device on bus: 0x");
      Serial.println(addr, HEX);
      display.print(addr, HEX);
      display.print(" ");
      display.display();
    }
  }
  delay(750);
  Serial.println("RP2040Zero: Running loop now..\n");
}
void loop() {
  // Read complete commands from Nano on Serial1 (newline-terminated)
  if (readLine(Serial1, s1Buf, s1Len, CMD_BUF_SZ)) {
    handleCommand(s1Buf);
  }
  // Read complete commands from RP2040Zero's USB on Serial (newline-terminated)
  if (readLine(Serial, s1Buf, s1Len, CMD_BUF_SZ)) {
    handleCommand(s1Buf);
  }
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
  if ((time_now + period) > millis()) {
    displayfreq();
    layout();
  }
}
void tunegen() {
  Serial1.printf("F %u\n", (unsigned long)freq);
  Serial.printf("F %u\n", (unsigned long)freq);
}
void displayfreq() {
  unsigned int m = freq / 1000000;
  unsigned int k = (freq % 1000000) / 1000;
  unsigned int h = (freq % 1000) / 1;
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
    unsigned int h2 = (freq % 1000) / 10;
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
      stp = 1;
      fstep = 1000000;
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
    case 18: freq = 100000000; break;
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
  display.setCursor(54, 33);
  if (stp == 2) display.print("  1Hz");
  if (stp == 3) display.print(" 10Hz");
  if (stp == 4) display.print(" 1kHz");
  if (stp == 5) display.print(" 5kHz");
  if (stp == 6) display.print("10kHz");
  if (stp == 1) display.print(" 1MHz");
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
