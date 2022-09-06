#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <set>

static int time_zone = -7;

/***************************************************************************************
 * U8g2: Implements 4 lines of text and a progress box on top of the U8g2 interface
 ***************************************************************************************/
class U8g2 {
 public:
  U8g2() : u8g2_(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE) {
    u8g2_.begin();
    u8g2_.setFont(u8g2_font_6x10_tf);
    u8g2_.setFontRefHeightExtendedText();
    u8g2_.setDrawColor(1);
    u8g2_.setFontPosTop();
    u8g2_.setFontDirection(0);

    Off();  /* OLED power off */
    Reset();
  }

  void Reset() {
    for (int i = 0; i < 4; ++i) {
      lines_[i][0] = '\0';
      lbuf_[i].line_ = lines_[i];
      lbuf_[i].ofs_ = 0;
    }
    box_ = -1;
  }

  void Off() {
    u8g2_.setPowerSave(1);  /* OLED power off */
  }

  void On() {
    u8g2_.setPowerSave(0);  /* OLED power on */
  }

  class LineBuf {
   public:
    LineBuf &operator <<(int32_t var) {
      if (ofs_ < 22) {
        int len = snprintf(line_ + ofs_, 22 - ofs_, "%ld", var);
        ofs_ += len;
        if (ofs_ > 22) ofs_ = 22;
      }
      return *this;
    }

    LineBuf &operator <<(const char *ptr) {
      for (;;) {
        if (ofs_ >= 22) {
          line_[21] = '\0';
          break;
        }
        line_[ofs_] = *ptr;
        if (*ptr == '\0')
          break;
        ++ptr; ++ofs_;
      }
      return *this;
    }

   private:
    friend class U8g2;
    char *line_;
    int ofs_ = 0;
  };

  LineBuf &GetLineBuf(int i) { return lbuf_[i]; }

  void SetBox(int box) { box_ = box; }

  void Display() {
    u8g2_.firstPage();
    do {
      for (int i = 0; i < 4; ++i) {
        if (lines_[i][0] != '\0')
          u8g2_.drawStr(0, 16 * i, lines_[i]);
      }
      if (box_ >= 0)
        u8g2_.drawBox(box_ * 32, 10, 32, 6);
    } while( u8g2_.nextPage());
  }

 private:
  friend class LineBuf;
  U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2_;
  int box_ = -1;
  LineBuf lbuf_[4];
  char lines_[4][22];
};

static U8g2 *u8g2 = nullptr;

/***************************************************************************************
 * Connection class
 *   - turns Wifi on and off as needed
 *   - Does NTP to determine the UTC time
 *   - Implements a report log which flushes to external listener via TCP
 ***************************************************************************************/
class Connection {
 public:
  Connection() {
    WiFi.forceSleepWake();
    delay(1);

    /* Don't save connection info to flash */
    WiFi.persistent(false);

    // Bring up the WiFi connection
    WiFi.mode( WIFI_STA );
    WiFi.begin("ssid", "pwd");

    for (int retries = 0;; ++retries) {
      if (WiFi.status() == WL_CONNECTED) {
        enabled_ = true;
        return;
      }
      if (retries >= 300) {
        DisableWifi();
        return;
      }
      delay(50);
    }
  }
  ~Connection() {
    if (enabled_)
      DisableWifi();
  }

  static void DisableWifi() {
    WiFi.mode( WIFI_OFF );
    WiFi.forceSleepBegin();
    delay( 1 );
  }

  void CallNtpServer() {
    WiFiUDP udp;
    udp.begin(kLocalPort);
    IPAddress timeServerIP(10, 0, 0, 14);  /* local reference machine */

    /* NTP time stamp is in the first 48 bytes of the message */
    constexpr int kNtpPacketSize = 48;
    uint8_t packetBuffer[kNtpPacketSize];

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, sizeof(packetBuffer));
    // Initialize values needed to form NTP request
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;

    /* all NTP fields have been given values. Send a packet requesting a timestamp  */
    udp.beginPacket(timeServerIP, 123); //NTP requests are to port 123
    udp.write(packetBuffer, sizeof(packetBuffer));
    udp.endPacket();

    int udp_len = 0;
    uint32_t millis_now = millis();
    for (int retries = 0;; ++retries) {
      if (retries >= 40) break;
      if (udp.parsePacket() == 0) {
        delay(50);
        continue;
      }
      udp_len = udp.read(packetBuffer, sizeof(packetBuffer)); // read the packet into the buffer
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
      ntp_time_ = epoch + time_zone * 60 * 60;
      ntp_millis_ = millis_now;
      ntp_valid_ = true;
    }
  }

  bool GetNtpResult(uint32_t *millis, uint32_t *time) const {
    if (!ntp_valid_) return false;
    *millis = ntp_millis_;
    *time = ntp_time_;
    return true;
  }

  class Connection& operator<<(const char *ptr) {
    if (report_ptr_ >= sizeof(report_)) return *this;
    char *buf = (char *)report_;
    int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_,
      "%s", ptr);
    report_ptr_ += len;
    FlushIf();
    return *this;
  }

  class Connection& operator<<(int32_t val) {
    if (report_ptr_ >= sizeof(report_)) return *this;
    char *buf = (char *)report_;
    int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_,
      "%d", val);
    report_ptr_ += len;
    FlushIf();
    return *this;
  }

 private:
  void FlushIf() {
    if (report_ptr_ >= sizeof(report_)) {
      Flush();
      return;
    }
    if (report_ptr_ > 0 && report_[report_ptr_ - 1] == '\n') {
      Flush();
    }
  }
  void Flush() {
    report_[sizeof(report_) - 1] = '\0';
    report_[sizeof(report_) - 2] = '\n';
    static int ofs = 0;
    IPAddress receiver(10,0,0,14);
    WiFiClient client;

    int rc;
    rc = client.connect(receiver, 9876);
    if (!rc) {
      report_ptr_ = 0;
      return;
    }
    rc = client.write(report_, report_ptr_);
    report_ptr_ = 0;
    /* wait for server reply */
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        client.stop();
        break;
      }
    }
  }

  /* open local port to listen for UDP packets */
  constexpr static unsigned kLocalPort = 2390;
  bool enabled_ = false;
  bool ntp_valid_ = false;
  uint32_t ntp_millis_;
  uint32_t ntp_time_;
  int report_ptr_ = 0;
  byte report_[256];
};

static Connection *current_connection = nullptr;

/***************************************************************************************
 * JobInputOutput
 * Manages the pins for I/O
 ***************************************************************************************/
class JobInputOutput {
 public:
  static void Setup() {
   pinMode(D8, INPUT_PULLUP);
  }

  static void ActivatePump(long *level_reached_msec, long *pump_off_msec) {
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

  static void ActivateFeeder(unsigned msec) {
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

  static void ExecuteJob() {
    ActivateFeeder(15000);
    long pump_off_ms, level_reached_ms = -1;
    ActivatePump(&level_reached_ms, &pump_off_ms);
    if (current_connection) {
      (*current_connection) <<  "Level reached " << level_reached_ms;
      (*current_connection) <<  ", Pump off " <<  pump_off_ms << "\n";
    }
  }

  static bool ButtonPressed() {
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
};

/***************************************************************************************
 * Interactive: for button presses and screen displays
 ***************************************************************************************/
class Interactive : public JobInputOutput{
 public:
  static void Check() {
    if (ButtonPressed()) {
      delay(150); /* debounce */
      RunInteractive();
    }
  }
 private:
  static void RunInteractive() {
    WaitButtonReleased();
    u8g2->On(); /* OLED power on */
    if (SelectOption("Pump Manual Control")) {
      RunPump();
    } else if (SelectOption("Feeder Manual Control")) {
      RunFeeder();
    } else if (SelectOption("Show Information")) {
      ShowInformation();
    } else if (SelectOption("Time Zone")) {
      WaitButtonReleased();
      if (SelectOption("GMT - 8")) {
        time_zone = -8;
      } else if (SelectOption("GMT - 7")) {
        time_zone = -7;
      } else if (SelectOption("GMT - 6")) {
        time_zone = -6;
      } else if (SelectOption("GMT - 5")) {
        time_zone = -5;
      } else if (SelectOption("GMT - 4")) {
        time_zone = -4;
      }
      ShowInformation();
    }
    WaitButtonReleased();
    u8g2->Off();  /* OLED power off */
  }

  static bool SelectOption(char *opt) {
    for (int i = 0; i < 4; ++i) {
      u8g2->Reset();
      u8g2->GetLineBuf(0) << opt;
      u8g2->SetBox(i);
      u8g2->Display();
      for (int j = 0; j < 5; ++j) {
        delay(100);
        if (ButtonPressed())
          return true;
      }
    }
    return false;
  }

  static void RunPump() {
    WaitButtonReleased();
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Running Pump until full";
    u8g2->GetLineBuf(1) << "Press Button to terminate";
    u8g2->Display();
    long pump_off_ms, level_reached_ms = -1;
    ActivatePump(&level_reached_ms, &pump_off_ms);
    u8g2->Reset();
    u8g2->GetLineBuf(0) <<  "Level reached " << level_reached_ms;
    u8g2->GetLineBuf(1) <<  "Pump off " <<  pump_off_ms;
    u8g2->Display();
    WaitButtonReleased();
    WaitButtonPressed();
  }

  static void RunFeeder() {
    WaitButtonReleased();
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Running Feeder 2 Min.";
    u8g2->GetLineBuf(1) << "Press Button to terminate";
    u8g2->Display();
    ActivateFeeder(120000);
  }

  static void ShowInformation() {
    WaitButtonReleased();
    int y, m, d, hh, mm, ss;
//    EpochToUtc(last_time_t, &y, &m, &d, &hh, &mm, &ss);
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Millis: " << uint32_t(millis());
    u8g2->GetLineBuf(1) << "Time Zone: " << time_zone;
//    u8g2->GetLineBuf(1) << "Date: " << y << "-" << m << "-" << d;
//    u8g2->GetLineBuf(2) << "Time: " << hh << ":" << mm << ":" << ss;
    u8g2->GetLineBuf(3) <<  "********-********";
    u8g2->Display();
    WaitButtonPressed();
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
};

#define precision  8
#define log_alpha  6

struct EmaStat {
  int32_t ema = 0;
  uint32_t var = 0;
  int count = 0;
  int shift = -1;

  void Next(int64_t sample) {
    sample *= precision;
    /* each power of two, double EMA period, until max */
    if (shift < log_alpha) {
      ++count;
      if ((count & (count - 1)) == 0) {
        ema *= 2;
        var *= 2;
        ++shift;
      }
    }
    int32_t diff = (sample << shift) - ema;
    ema += (diff >> shift);
    var += diff * diff >> (shift * 2);
    var -= (var >> shift);
  }
  int32_t GetEma() { return (ema >> shift) / precision; }
  uint32_t GetVar() { return (var >> shift) / precision / precision; }
};

class TimeManager {
 public:
  void Update() {
    if (!current_connection) return;
    uint32_t m, t;
    if (current_connection->GetNtpResult(&m, &t)) {
      AddSample(m, t);
    }
  }

  bool ValidTime() const { return prev_time_t_ != 0; }

  /* TODO: EMA should use samples one day or more apart to get accurate PPM from one second resolution */
  void AddSample(uint32_t sample_millis, uint32_t sample_time_t) {
    if (prev_time_t_) {
      uint32_t diff_time_t = sample_time_t - prev_time_t_;
      uint32_t diff_millis = sample_millis - prev_millis_;
      if (diff_time_t >= 50 && diff_time_t <= 5000) {
        /* scale the millis to 1000 seconds for PPM */
        if (current_connection)
          (*current_connection) << "dtt: " << diff_time_t << " dmil: " << diff_millis << "\n";
        diff_millis *= 1000;
        diff_millis /= diff_time_t;
        int64_t adj = diff_millis - int64_t(1000) * 1000;
        emastat.Next(adj);
        if (current_connection) {
          (*current_connection) << "dmiln: " << diff_millis << " ema: " << emastat.GetEma() << " var: " << emastat.GetVar() << "\n";
        }
      }
    }
    prev_millis_ = sample_millis;
    prev_time_t_ = sample_time_t;
  }

  /* TODO use EMA, track millis() rollover */
  uint32_t GuessCurrentTime() {
    uint32_t now = prev_time_t_;
    now += (millis() - prev_millis_) / 1000;
    return now;
  }

 private:
  uint32_t prev_millis_ = 0;
  uint32_t prev_time_t_ = 0;
  EmaStat emastat;
} tmgr;

struct JobTime {
  unsigned hour;
  unsigned minute;
};

JobTime jobs[] = {{8, 40}, {13, 00}, {17, 30}};

uint32_t wait_until = 0;

static std::set<uint32_t> pending_jobs;

void RepopulateJobs(uint32_t newer_than) {
  uint32_t now = tmgr.GuessCurrentTime();
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
  if (current_connection) {
    (*current_connection) << "Jobs in";
    for (uint32_t job : pending_jobs) {
      (*current_connection) << " ";
      (*current_connection) << int32_t(job - now);
    }
    (*current_connection) << "\n";
  }
}

void setup() {
  Connection::DisableWifi();
  JobInputOutput::Setup();
  u8g2 = new U8g2;
}

constexpr unsigned kInterval = 10 * 60; /* 10 minutes */

void loop() {
  uint32_t current_millis = millis();
  int32_t remaining = int32_t(wait_until - current_millis);
  if (remaining > 0) {
    Interactive::Check();
    return;
  }
  Connection connection;
  current_connection = &connection;
  connection << "Wakeup " << millis() << "\n";
  connection.CallNtpServer();
  tmgr.Update();
  if (!tmgr.ValidTime()) {
    wait_until = millis() + kInterval * 1000;
    current_connection = nullptr;
    return;
  }

  uint32_t most_recent_job = 0;
  if (pending_jobs.empty()) {
    RepopulateJobs(tmgr.GuessCurrentTime());
  }
  uint32_t now;
  while (!pending_jobs.empty()) {
    uint32_t next_job = *pending_jobs.begin();
    now = tmgr.GuessCurrentTime();
    if (now < next_job) break;
    most_recent_job = next_job;
    pending_jobs.erase(pending_jobs.begin());
    JobInputOutput::ExecuteJob();
  }

  if (most_recent_job) {
    RepopulateJobs(most_recent_job);
  }

  now = tmgr.GuessCurrentTime();
  uint32_t next = now + kInterval;
  uint32_t past = next % kInterval;
  next -= past;
  wait_until = millis() + (next - now) * 1000;
  current_connection = nullptr;
}
