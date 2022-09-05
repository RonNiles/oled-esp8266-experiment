#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <set>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the DispcdIcon  

struct JobTime {
  unsigned hour;
  unsigned minute;
};

JobTime jobs[] = {{8, 40}, {13, 0}, {17, 30}};

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

static std::set<uint32_t> pending_jobs;

static char report[256];
static int report_ptr = 0;

void WriteReport(char *ptr) {
  if (report_ptr >= sizeof(report)) return;
  int len = snprintf(report + report_ptr, sizeof(report) - report_ptr,
    "%s", ptr);
  report_ptr += len;
}

void WriteReport(int32_t val) {
  if (report_ptr >= sizeof(report)) return;
  int len = snprintf(report + report_ptr, sizeof(report) - report_ptr,
    "%d", val);
  report_ptr += len;
}

void RepopulateJobs(uint32_t newer_than) {
  uint32_t now = GuessCurrentTime();
  uint32_t past = now % 86400;
  uint32_t last_day = now - past;
  for (const JobTime &job : jobs) {
    uint32_t next_job = last_day;
    next_job += uint32_t(job.minute) * 60;
    next_job += uint32_t(job.hour) * 60 * 60;
    if (next_job > newer_than)
      pending_jobs.insert(next_job);
    next_job += 86400;
    if (next_job > newer_than)
      pending_jobs.insert(next_job);
  }
  WriteReport("Jobs in");
  for (uint32_t job : pending_jobs) {
    WriteReport(" ");
    WriteReport(int32_t(job - now));
  }
  WriteReport("\n");
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

void UpdateTime() {
  unsigned long duration = millis();
  EnableWifi();
  duration = millis() - duration;
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
  DisableWifi();
}

/**
 * Convert an epoch time to UTC date and time.
 *
 * The function calculates the date numbers relative to a calendar that starts from
 * 03/01/0000 for convenience of calculation. Once the numbers are obtained, they can be
 * easily translated to calendar numbers.
 *
 * Some detailed ref to the algorithm: http://howardhinnant.github.io/date_algorithms.html
 */
static void EpochToUtc(uint32_t epoch, int *y, int *m, int *d, int *hh, int *mm, int *ss) {
  const uint32_t kEpochToRef = 719468;
  const uint32_t kSecsPerDay = 86400;
  const uint32_t kSecsPerHour = 3600;
  const uint32_t kSecsPerMinute = 60;
  const uint32_t kDaysPerEra = 146097;
  const uint32_t kDaysPerYear = 365;
  const uint32_t kYearsPerEra = 400;
  uint32_t days_from_epoch = epoch / kSecsPerDay;
  /* shift the reference from epoch to 03/01/0000 */
  uint32_t days_from_ref = days_from_epoch + kEpochToRef;
  /* the era (400 yrs) number that the date is in; each era has 146097 days */
  uint32_t era = days_from_ref / kDaysPerEra;
  /* the day number within the era */
  uint32_t doe = days_from_ref - era * kDaysPerEra;
  /* the year number within the era */
  uint32_t yoe = (doe
             - doe / 1460 /* leap day every 4 yrs (1460 days) */
             + doe / 36524 /* no leap day every 100 yrs (36524 days) */
             - (doe == 146096 ? 1 : 0)) /* leap day every 400 yrs */
            / kDaysPerYear;
  /* the day number within the year; days start counting from Mar 1st */
  uint32_t doy = doe - (yoe * kDaysPerYear + yoe / 4 - yoe / 100); 
  /* month number from day number within the year; starts counting from Mar 1st */
  uint32_t moy = (5 * doy + 2) / 153;
  /* day number within the month */
  uint32_t dom = doy - (153 * moy + 2) / 5;
  /* calculate the calendar dates */
  *d = dom + 1;
  *m = moy > 9 ? moy - 9 : moy + 3;
  *y = yoe + era * kYearsPerEra + (moy > 9 ? 1 : 0); /* moy 10 and 11 are Jan and Feb of
                                                       the next calendar year */
  /* calculate the time */
  epoch -= (days_from_epoch * kSecsPerDay);
  *hh = epoch / kSecsPerHour;
  epoch -= (*hh * kSecsPerHour);
  *mm = epoch / kSecsPerMinute;
  epoch -= (*mm * kSecsPerMinute);
  *ss = epoch;
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
  ActivateFeeder(120000);
}

void ShowInformation() {
  WaitButtonReleased();
  char l1[22];
  char l2[22];
  char l3[22];
  int y, m, d, hh, mm, ss;
  EpochToUtc(last_time_t, &y, &m, &d, &hh, &mm, &ss);
  snprintf(l1, sizeof(l1), "Millis: %lu\n", millis());
  snprintf(l2, sizeof(l2), "Date: %04d-%02d-%02d\n", y, m, d);
  snprintf(l3, sizeof(l3), "Time: %02d:%02d:%02d\n", hh, mm, ss);
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, l1);
    u8g2.drawStr(0, 16, l2);
    u8g2.drawStr(0, 32, l3);
    u8g2.drawStr(0, 48, "********-********");
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
  if (SelectOption("Pump Manual Control")) {
    RunPump();
  } else if (SelectOption("Feeder Manual Control")) {
    RunFeeder();
  } else if (SelectOption("Show Information")) {
    ShowInformation();
  }
  WaitButtonReleased();
  u8g2.setPowerSave(1);  /* OLED power off */
}

static void ExecuteJob() {
  ActivateFeeder(15000);
  long pump_off_ms, level_reached_ms = -1;
  ActivatePump(&level_reached_ms, &pump_off_ms);
}

constexpr unsigned kInterval = 10 * 60; /* 10 minutes */

void loop() {
  uint32_t current_millis = millis();
  int32_t remaining = int32_t(wait_until - current_millis);
  if (remaining > 0) {
    if (ButtonPressed()) {
      delay(150); /* debounce */
      Interactive();
    }
    return;
  }
  UpdateTime();
  if (!time_valid) {
    wait_until = millis() + kInterval;
    return;
  }

  uint32_t most_recent_job = 0;
  if (pending_jobs.empty()) {
    RepopulateJobs(GuessCurrentTime());
  }
  uint32_t now;
  while (!pending_jobs.empty()) {
    uint32_t next_job = *pending_jobs.begin();
    now = GuessCurrentTime();
    if (now < next_job) break;
    most_recent_job = next_job;
    pending_jobs.erase(pending_jobs.begin());
    ExecuteJob();
  }

  if (most_recent_job) {
    RepopulateJobs(most_recent_job);
  }

  now = GuessCurrentTime();
  uint32_t next = now + kInterval;
  uint32_t past = next % kInterval;
  next -= past;
  wait_until = millis() + (next - now) * 1000;
}
