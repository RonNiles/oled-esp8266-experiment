#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the DispcdIcon  

#ifndef STASSID
#define STASSID "ssid"
#define STAPSK  "pwd"
#endif

const char * ssid = STASSID; // your network SSID (name)
const char * pass = STAPSK;  // your network password

unsigned int localPort = 2390;      // local port to listen for UDP packets

static bool wifi_enabled = false;

static void DisableWifi() {
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
  wifi_enabled = false;
}

static void EnableWifi() {
  WiFi.forceSleepWake();
  delay(1);

  /* Don't save connection info to flash */
  WiFi.persistent( false );

  // Bring up the WiFi connection
  WiFi.mode( WIFI_STA );
  WiFi.begin(ssid, pass);
  for (int retries = 0;; ++retries) {
    if (WiFi.status() == WL_CONNECTED) {
      wifi_enabled = true;
      return;
    }
    if (retries >= 300) {
      DisableWifi();
      return;
    }
    delay(50);
  }
}

constexpr int time_zone = -7;
uint32_t last_time_t = 0;
uint32_t last_millis = 0;

uint32_t wait_until = 0;
bool time_valid = false;

uint32_t GuessCurrentTime() {
  uint32_t now = last_time_t;
  now += (millis() - last_millis) / 1000;
  return now;
}


IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "10.0.0.14"; //"time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void setup() {
  DisableWifi();
  pinMode(D8, INPUT_PULLUP);
  u8g2.begin();
  u8g2.setPowerSave(1);  /* OLED power off */
}

static bool inline ButtonPressed() {
 return digitalRead(D8) == HIGH;
}

static void inline WaitButtonReleased() {
  int countdown = 0;
  for (;;) {
    if (ButtonPressed()) {
      countdown = 0;
    } else if (++countdown > 4) {
      break;
    }
    delay(50);
  }
}

static void inline WaitButtonPressed() {
  int countdown = 0;
  for (;;) {
    if (!ButtonPressed()) {
      countdown = 0;
    } else if (++countdown > 4) {
      break;
    }
    delay(50);
  }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void GetNtpTime() {
  time_valid = false;
  unsigned long duration = millis();
  EnableWifi();
  duration = millis() - duration;
  char msg1[22];
  char msg2[22];
  snprintf(msg1, sizeof(msg1), "Wifi %s in %lu", wifi_enabled ? "OK" : "Fail", duration);
  u8g2.firstPage(); do { u8g2.drawStr(0, 0, msg1); } while( u8g2.nextPage());
  if (!wifi_enabled) return;
  udp.begin(localPort);

  int udp_len = 0;
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  uint32_t millis_now = millis();
  for (int retries = 0;; ++retries) {
    if (retries >= 40) break;
    if (udp.parsePacket() == 0) {
      delay(50);
      continue;
    }
    udp_len = udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    break;
  }
  snprintf(msg2, sizeof(msg2), "udp_len: %d", udp_len);
  u8g2.firstPage(); do { u8g2.drawStr(0, 0, msg1); u8g2.drawStr(0, 16, msg2); } while (u8g2.nextPage());

  if (udp_len >= 44) {
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    last_time_t = epoch + time_zone * 60 * 60;
    last_millis = millis_now;
    time_valid = true;
  }
  if (time_valid) {
    snprintf(msg2, sizeof(msg2), "time_t: %lu", last_time_t);
  u8g2.firstPage(); do { u8g2.drawStr(0, 0, msg1); u8g2.drawStr(0, 16, msg2); } while (u8g2.nextPage());
  }
  DisableWifi();
  WaitButtonPressed();
}

bool SelectOption(char *opt) {
  for (int i = 0; i < 4; ++i) {
    u8g2.firstPage();
    do {
      u8g2.drawStr(0, 0, opt);
      u8g2.drawBox(i*32,10,32,6);
    } while( u8g2.nextPage());
    for (int j = 0; j < 5; ++j) {
      delay(100);
      if (ButtonPressed())
        return true;
    }
  }
  return false;
}

void ActivatePump(long *level_reached_msec, long *pump_off_msec) {
  pinMode(D5, INPUT);

  uint32_t start_millis = millis();
  uint32_t end_millis = start_millis + 50 * 1000;
  digitalWrite(D7, LOW);
  pinMode(D7, OUTPUT);
  while (int32_t(millis() - end_millis) < 0) {
    delay(10);
    if (ButtonPressed())
      break;
    if (digitalRead(D5) == HIGH) {
      if (level_reached_msec)
        *level_reached_msec = long(millis() - start_millis);
      break;
    }
  }
  pinMode(D7, INPUT);
  if (pump_off_msec)
    *pump_off_msec = long(millis() - start_millis);
}

void ActivateFeeder(unsigned msec) {
  digitalWrite(D6, LOW);
  pinMode(D6, OUTPUT);
  unsigned long start = millis();
  do {
    delay(100);
    if (ButtonPressed())
      break;
  } while (millis() - start < msec);
  pinMode(D6, INPUT);
  delay(1000);
}

void RunPump() {
  WaitButtonReleased();
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, "Running Pump until full");
    u8g2.drawStr(0, 16, "Press Button to terminate early");
  } while(u8g2.nextPage());
  long pump_off_ms, level_reached_ms = -1;
  ActivatePump(&level_reached_ms, &pump_off_ms);
  u8g2.firstPage();
  char msg1[32];
  char msg2[32];
  sprintf(msg1, "Level reached %ld", level_reached_ms);
  sprintf(msg2, "Pump off %ld", pump_off_ms);
  do {
    u8g2.drawStr(0, 16, msg1);
    u8g2.drawStr(0, 26, msg2);
  } while(u8g2.nextPage());
  WaitButtonReleased();
  WaitButtonPressed();
}

void RunFeeder() {
  WaitButtonReleased();
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, "Running Feeder 2 Min.");
    u8g2.drawStr(0, 16, "Press Button to terminate early");
  } while( u8g2.nextPage());
  ActivateFeeder(10000);
}

void ShowInformation() {
  WaitButtonReleased();
  char buf[64];
  sprintf(buf, "Millis: %lu\n", millis());
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, buf);
  } while( u8g2.nextPage());
  WaitButtonPressed();
}

void Interactive() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  WaitButtonReleased();
  u8g2.setPowerSave(0);  /* OLED power on */
  if (SelectOption("Get NTP time")) {
    GetNtpTime();
  } else if (SelectOption("Pump Manual Control")) {
    RunPump();
  } else if (SelectOption("Feeder Manual Control")) {
    RunFeeder();
  } else if (SelectOption("Show Information")) {
    ShowInformation();
  }
  WaitButtonReleased();
  u8g2.setPowerSave(1);  /* OLED power off */
}

void loop() {
  if (ButtonPressed()) {
    delay(150); /* debounce */
    Interactive();
    return;
  }
}
