#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <WiFiUdp.h>

#include <cstring>
#include <map>
#include <set>

static int time_zone = -7;

/***************************************************************************************
 * U8g2: Implements 4 lines of text and a progress box on top of the U8g2 interface
 ***************************************************************************************/
class U8g2 {
 public:
  U8g2() : u8g2_(U8G2_R0, /* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE) {
    u8g2_.begin();
    u8g2_.setFont(u8g2_font_6x10_tf);
    u8g2_.setFontRefHeightExtendedText();
    u8g2_.setDrawColor(1);
    u8g2_.setFontPosTop();
    u8g2_.setFontDirection(0);

    Off(); /* OLED power off */
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

  void Off() { u8g2_.setPowerSave(1); /* OLED power off */ }

  void On() { u8g2_.setPowerSave(0); /* OLED power on */ }

  class LineBuf {
   public:
    LineBuf &operator<<(int32_t var) {
      if (ofs_ < 22) {
        int len = snprintf(line_ + ofs_, 22 - ofs_, "%ld", var);
        ofs_ += len;
        if (ofs_ > 22)
          ofs_ = 22;
      }
      return *this;
    }

    LineBuf &operator<<(const char *ptr) {
      for (;;) {
        if (ofs_ >= 22) {
          line_[21] = '\0';
          break;
        }
        line_[ofs_] = *ptr;
        if (*ptr == '\0')
          break;
        ++ptr;
        ++ofs_;
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
    } while (u8g2_.nextPage());
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
 * 32-bit millis can roll over in about 2 months, so track rollovers and implement
 * a 64-bit LongMillis() for when rollover might be a problem
 ***************************************************************************************/
class MillisRollover {
 public:
  uint64_t LongMillis() {
    uint32_t now = millis();
    if (now < prev_millis_)
      ++millis_rollovers_;
    prev_millis_ = now;
    return (int64_t{millis_rollovers_} << 32) | now;
  }

 private:
  int millis_rollovers_ = 0;
  uint32_t prev_millis_ = 0;
} mr;

/***************************************************************************************
 * InformationPage: inherit from this page and register for callback to display
 *information
 ***************************************************************************************/
class InformationPage {
 public:
  virtual void ShowInformationPage() = 0;

  template <class T>
  static void EnumInformationPages(const T &t) {
    for (InformationPage *page = pages_; page != nullptr; page = page->next_)
      t(page);
  }

 protected:
  void RegisterInformationPage() {
    InformationPage **page = &pages_;
    while (*page != nullptr)
      page = &(*page)->next_;
    *page = this;
  }

 private:
  InformationPage *next_ = nullptr;
  static InformationPage *pages_;
};
InformationPage *InformationPage::pages_ = nullptr;

/***************************************************************************************
 * HiResTime is precise to 1/256 second and is gotten from NTP packet by using the
 * first of the fractional bytes
 ***************************************************************************************/
class HiResTime {
 public:
  void SetFromNtp(uint8_t *ntp) {
    /* We'll take the 4 bytes of seconds plus an additional byte of binary fractional
     seconds. The result is 256 times the number of seconds */
    for (int i = 0; i < 5; ++i)
      time_x_256_ = (time_x_256_ << 8) | ntp[i];

    /* NTP time starts 1/1/1900 while unix time starts 1/1/1970, so subtract 25567 days */
    time_x_256_ -= uint64_t{25567} * uint64_t{86400} * 256;
  }
  HiResTime operator-(const HiResTime &op) {
    HiResTime res(*this);
    res.time_x_256_ -= op.time_x_256_;
    return res;
  }
  friend bool operator<(const HiResTime &a, const HiResTime &b);
  int64_t HiRes() const { return time_x_256_; }
  uint32_t Seconds() const { return time_x_256_ / 256; }

 private:
  int64_t time_x_256_ = 0;
};
bool operator<(const HiResTime &a, const HiResTime &b) {
  return a.time_x_256_ < b.time_x_256_;
}

/***************************************************************************************
 * Connection class
 *   - turns Wifi on and off as needed
 *   - Does NTP to determine the UTC time
 *   - Implements a report log which flushes to external listener via TCP
 ***************************************************************************************/
class Connection {
 public:
  Connection() {}
  ~Connection() {
    if (connected_)
      DisableWifi();
  }

  bool Connected() const { return connected_; }
  void Connect() {
    if (connected_)
      return;

    WiFi.forceSleepWake();
    delay(1);

    /* Don't save connection info to flash */
    WiFi.persistent(false);

    // Bring up the WiFi connection
    WiFi.mode(WIFI_STA);
    WiFi.begin("ssid", "pwd");

    for (int retries = 0;; ++retries) {
      if (WiFi.status() == WL_CONNECTED) {
        connected_ = true;
        return;
      }
      if (retries >= 300) {
        DisableWifi();
        return;
      }
      delay(50);
    }
  }

  static void DisableWifi() {
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);
  }

  void CallNtpServer() {
    if (!connected_)
      return;
    WiFiUDP udp;
    udp.begin(kLocalPort);
    IPAddress timeServerIP(10, 0, 0, 14); /* local reference machine */

    /* NTP time stamp is in the first 48 bytes of the message */
    constexpr int kNtpPacketSize = 48;
    uint8_t packetBuffer[kNtpPacketSize];

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, sizeof(packetBuffer));
    // Initialize values needed to form NTP request
    packetBuffer[0] = 0b11100011;  // LI, Version, Mode
    packetBuffer[1] = 0;           // Stratum, or type of clock
    packetBuffer[2] = 6;           // Polling Interval
    packetBuffer[3] = 0xEC;        // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    /* all NTP fields have been given values. Send the request to NTP port 123 */
    udp.beginPacket(timeServerIP, 123);
    udp.write(packetBuffer, sizeof(packetBuffer));
    udp.endPacket();

    int retry = 0;
    int udp_len = 0;

    for (;;) {
      if (udp.parsePacket() != 0) {
        ntp_millis_ = mr.LongMillis();
        udp_len = udp.read(packetBuffer, sizeof(packetBuffer));
        break;
      }
      if (++retry >= 40)
        break;
      delay(50);
    }

    if (udp_len >= 48) {
      ntp_time_.SetFromNtp(&packetBuffer[40]);
      ntp_valid_ = true;
    } else {
      (*this) << "NTP Invalid Length " << udp_len << "\n";
    }
  }

  bool GetNtpHires(uint64_t *millis, HiResTime *time) const {
    if (!ntp_valid_)
      return false;
    *millis = ntp_millis_;
    *time = ntp_time_;
    return true;
  }

  class Connection &operator<<(const char *ptr) {
    if (!connected_ || report_ptr_ >= sizeof(report_))
      return *this;
    char *buf = (char *)report_;
    int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_, "%s", ptr);
    report_ptr_ += len;
    FlushIf();
    return *this;
  }

  class Connection &operator<<(int32_t val) {
    if (!connected_ || report_ptr_ >= sizeof(report_))
      return *this;
    char *buf = (char *)report_;
    int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_, "%d", val);
    report_ptr_ += len;
    FlushIf();
    return *this;
  }

  class Connection &operator<<(uint32_t val) {
    if (!connected_ || report_ptr_ >= sizeof(report_))
      return *this;
    char *buf = (char *)report_;
    int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_, "%u", val);
    report_ptr_ += len;
    FlushIf();
    return *this;
  }

  class Connection &operator<<(uint64_t val) {
    if (!connected_ || report_ptr_ >= sizeof(report_))
      return *this;

    int groups[7];
    int gptr = 0;
    while (val) {
      groups[gptr++] = val % 1000;
      val /= 1000;
    }
    if (gptr == 0)
      groups[gptr++] = 0;
    bool first = true;
    while (gptr) {
      char *buf = (char *)report_;
      int len = snprintf(buf + report_ptr_, sizeof(report_) - report_ptr_,
                         (first ? "%d" : "%03d"), groups[--gptr]);
      first = false;
      report_ptr_ += len;
      if (gptr)
        report_[report_ptr_++] = ',';
      FlushIf();
    }
    return *this;
  }

  class Connection &operator<<(int64_t val) {
    if (!connected_ || report_ptr_ >= sizeof(report_))
      return *this;
    FlushIf();
    if (val < 0) {
      report_[report_ptr_++] = '-';
      val = -val;
    }
    return operator<<(uint64_t(val));
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
    IPAddress receiver(10, 0, 0, 14);
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
    uint32_t timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        client.stop();
        break;
      }
    }
  }

  /* open local port to listen for UDP packets */
  constexpr static unsigned kLocalPort = 2390;
  bool connected_ = false;
  bool ntp_valid_ = false;
  uint64_t ntp_millis_ = 0;
  HiResTime ntp_time_;
  int report_ptr_ = 0;
  uint8_t report_[256];
};

static Connection *current_connection = nullptr;

/***************************************************************************************
 * JobInputOutput
 * Manages the pins for I/O
 ***************************************************************************************/
class JobInputOutput {
 public:
  static void Setup() { pinMode(D8, INPUT); }

  static void ActivatePump(int32_t *level_reached_msec, int32_t *pump_off_msec) {
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
          *level_reached_msec = int32_t(millis() - start_millis);
        break;
      }
    }
    pinMode(D7, INPUT);
    if (pump_off_msec)
      *pump_off_msec = int32_t(millis() - start_millis);
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
    int32_t pump_off_ms, level_reached_ms = -1;
    ActivatePump(&level_reached_ms, &pump_off_ms);
    if (current_connection) {
      (*current_connection) << "Level reached " << level_reached_ms;
      (*current_connection) << ", Pump off " << pump_off_ms << "\n";
    }
  }

  static bool ButtonPressed() { return digitalRead(D8) == HIGH; }

  static void inline DebounceWait(bool (*fn)()) {
    uint32_t start_millis = millis();
    int countdown = 0;
    for (;;) {
      if (!(*fn)()) {
        countdown = 0;
      } else if (++countdown > 4) {
        break;
      }
      if (millis() - start_millis > 5000)
        break;
      delay(50);
    }
  }

  static void inline WaitButtonReleased() {
    DebounceWait([]() { return !ButtonPressed(); });
  }

  static void inline WaitButtonPressed() {
    DebounceWait([]() { return ButtonPressed(); });
  }
};

/***************************************************************************************
 * Interactive: for button presses and screen displays
 ***************************************************************************************/
class Interactive : public JobInputOutput {
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
      int prev_time_zone = time_zone;
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
      if (time_zone != prev_time_zone) {
        EEPROM.write(256, 'T');
        EEPROM.write(257, 'Z');
        EEPROM.write(258, time_zone);
        EEPROM.commit();
      }
      ShowInformation();
    }
    WaitButtonReleased();
    u8g2->Off(); /* OLED power off */
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
    int32_t pump_off_ms, level_reached_ms = -1;
    ActivatePump(&level_reached_ms, &pump_off_ms);
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Level reached " << level_reached_ms;
    u8g2->GetLineBuf(1) << "Pump off " << pump_off_ms;
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
    InformationPage::EnumInformationPages([&](InformationPage *info) {
      info->ShowInformationPage();
      WaitButtonPressed();
      WaitButtonReleased();
    });
  }
};

/**
 * Convert an epoch time to UTC date and time.
 *
 * The function calculates the date numbers relative to a calendar that starts from
 * 03/01/0000 for convenience of calculation. Once the numbers are obtained, they can be
 * easily translated to calendar numbers.
 *
 * Some detailed ref to the algorithm:
 * http://howardhinnant.github.io/date_algorithms.html
 */
static void EpochToUtc(uint32_t epoch, int *y, int *m, int *d, int *hh, int *mm,
                       int *ss) {
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
  uint32_t yoe = (doe - doe / 1460           /* leap day every 4 yrs (1460 days) */
                  + doe / 36524              /* no leap day every 100 yrs (36524 days) */
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

#define precision 1
#define log_alpha 6

struct EmaStat {
  int64_t ema = 0;
  uint64_t var = 0;
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
    int64_t diff = (sample << shift) - ema;
    ema += (diff >> shift);
    var += int64_t(diff) * diff >> (shift * 2);
    var -= (var >> shift);
  }
  int64_t GetEma() { return (ema >> shift) / precision; }
  uint64_t GetVar() { return (var >> shift) / precision / precision; }
};

class TimeManager : public InformationPage {
 public:
  TimeManager() { RegisterInformationPage(); }
  void Update() {
    if (!current_connection)
      return;
    uint64_t m;
    HiResTime t;
    if (current_connection->GetNtpHires(&m, &t)) {
      AddSample(m, t);
    }
  }

  bool ValidTime() const { return !recent_samples_.empty(); }

  constexpr static uint32_t kMinSeconds = 3600;
  constexpr static uint32_t kMaxSeconds = 2 * 86400 + 3600;
  void AddSample(uint64_t sample_millis, HiResTime sample_time_t) {
    if (!recent_samples_.empty()) {
      HiResTime diff_time_t = sample_time_t - recent_samples_.begin()->first;
      uint64_t diff_millis = sample_millis - recent_samples_.begin()->second;
      if (diff_time_t.Seconds() >= kMinSeconds) {
        /* scale the millis to 1000000 seconds for Parts Per Billion */
        int64_t diff_ms = 256 * diff_millis - 1000 * diff_time_t.HiRes();
        int64_t diff_ppb = diff_ms * 1000000 / diff_time_t.HiRes();
        if (current_connection)
          (*current_connection)
              << "dtt: " << diff_time_t.HiRes() * 1000 / 256 << " dmil: " << diff_millis
              << " diff_ms: " << diff_ms / 256 << " diff_ppb: " << diff_ppb << "\n";
        emastat.Next(diff_ppb);
        if (current_connection) {
          (*current_connection)
              << "ema: " << emastat.GetEma() << " var: " << emastat.GetVar()
              << " sd: " << uint32_t(sqrt(emastat.GetVar())) << "\n";
        }
      }
    }
    if (current_connection) {
      (*current_connection) << "Pre: ";
      for (const auto &item : recent_samples_)
        (*current_connection) << "(" << item.first.Seconds() << "," << item.second
                              << ") ";
      (*current_connection) << "\n";
    }
    if (recent_samples_.empty() ||
        (sample_time_t - std::prev(recent_samples_.end())->first).Seconds() >=
            kMinSeconds)
      recent_samples_.emplace(sample_time_t, sample_millis);
    while (recent_samples_.size() > 1 &&
           (sample_time_t - recent_samples_.begin()->first).Seconds() >= kMaxSeconds)
      recent_samples_.erase(recent_samples_.begin());
    if (current_connection) {
      (*current_connection) << "Post: ";
      for (const auto &item : recent_samples_)
        (*current_connection) << "(" << item.first.Seconds() << ", " << item.second
                              << ") ";
      (*current_connection) << "\n";
    }
  }

  /* TODO use EMA */
  uint32_t GuessCurrentTime() {
    if (!ValidTime())
      return 0;
    uint32_t now =
        std::prev(recent_samples_.end())->first.Seconds() + time_zone * 60 * 60;
    now += (mr.LongMillis() - std::prev(recent_samples_.end())->second) / 1000;
    return now;
  }

  void ShowInformationPage() override {
    int y, m, d, hh, mm, ss;
    char buf[20];
    EpochToUtc(GuessCurrentTime(), &y, &m, &d, &hh, &mm, &ss);
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Millis: " << mr.LongMillis();
    u8g2->GetLineBuf(1) << "Time Zone: " << time_zone;
    sprintf(buf, "%04d-%02d-%02d", y, m, d);
    u8g2->GetLineBuf(2) << "Date: " << buf;
    sprintf(buf, "%02d:%02d:%02d", hh, mm, ss);
    u8g2->GetLineBuf(3) << "Time: " << buf;
    u8g2->Display();
  }

 private:
  std::map<HiResTime, uint64_t> recent_samples_; /* <time_t, millis> */
  EmaStat emastat;
} tmgr;

/***************************************************************************************
 * JobManager: Keep track of the job schedule and execute when needed
 ***************************************************************************************/
class JobManager : public InformationPage {
 public:
  JobManager(uint32_t wait_until) : wait_until_(wait_until) { RegisterInformationPage(); }
  bool RunJobs() {
    uint32_t current_millis = millis();
    int32_t remaining = int32_t(wait_until_ - current_millis);
    if (remaining > 0)
      return false;
    Connection connection;
    bool tried_connection = false;

    auto do_connect = [&]() {
      if (tried_connection)
        return;
      connection.Connect();
      connection.CallNtpServer();
      connection << "Wakeup " << mr.LongMillis() << "\n";
      current_connection = &connection;
      tmgr.Update();
      tried_connection = true;
      if (connection.Connected())
        prev_connect_ = current_millis;
    };

    if (!tmgr.ValidTime()) {
      do_connect();
      connection << "Connected because of invalid time\n";
    }
#if 0
    if (current_millis - prev_connect_ > kInterval * 1000) {
      uint32_t prev_connect = prev_connect_;
      do_connect();
      connection << "Connected because current_millis: " << current_millis
                 << " prev_connect: " << prev_connect << "\n";
    }
#endif

    if (!tmgr.ValidTime()) {
      wait_until_ = millis() + kInterval * 1000;
      current_connection = nullptr;
      return false;
    }

    uint32_t most_recent_job = 0;
    if (pending_jobs_.empty()) {
      do_connect();
      connection << "Connected because jobs empty\n";
      RepopulateJobs(tmgr.GuessCurrentTime());
    }
    uint32_t now;
    while (!pending_jobs_.empty()) {
      uint32_t next_job = *pending_jobs_.begin();
      now = tmgr.GuessCurrentTime();
      if (now < next_job)
        break;
      do_connect();
      connection << "Connected because job will run\n";
      most_recent_job = next_job;
      pending_jobs_.erase(pending_jobs_.begin());
      JobInputOutput::ExecuteJob();
    }

    if (most_recent_job) {
      RepopulateJobs(most_recent_job);
    }

    now = tmgr.GuessCurrentTime();
    uint32_t next = now + kInterval;
    uint32_t past = next % kInterval;
    next -= past;
    wait_until_ = millis() + (next - now) * 1000;
    current_connection = nullptr;
    return true;
  }

  void ShowInformationPage() final {
    int y, m, d, hh, mm, ss;
    char buf[20];
    u8g2->Reset();
    u8g2->GetLineBuf(0) << "Next Jobs: ";
    auto it = pending_jobs_.begin();
    for (int i = 1; i < 4; ++i) {
      if (it == pending_jobs_.end())
        break;
      EpochToUtc(*it, &y, &m, &d, &hh, &mm, &ss);
      sprintf(buf, "%02d:%02d:%02d %04d-%02d-%02d", hh, mm, ss, y, m, d);
      u8g2->GetLineBuf(i) << buf;
      ++it;
    }
    u8g2->Display();
  }

 private:
  struct JobTime {
    unsigned hour;
    unsigned minute;
  };

  void RepopulateJobs(uint32_t newer_than) {
    uint32_t now = tmgr.GuessCurrentTime();
    uint32_t past = now % 86400;
    uint32_t last_day = now - past;
    for (const JobTime &job : jobs_) {
      uint32_t next_job = last_day;
      next_job += uint32_t(job.minute) * 60;
      next_job += uint32_t(job.hour) * 60 * 60;
      if (next_job > newer_than)
        pending_jobs_.insert(next_job);
      next_job += 86400;
      if (next_job > newer_than)
        pending_jobs_.insert(next_job);
    }
    if (current_connection) {
      (*current_connection) << "Jobs in";
      for (uint32_t job : pending_jobs_) {
        (*current_connection) << " ";
        (*current_connection) << int32_t(job - now);
      }
      (*current_connection) << "\n";
    }
  }

  constexpr static unsigned kInterval = 10 * 60; /* 10 minutes */
  constexpr static JobTime jobs_[] = {{8, 40}, {13, 00}, {17, 30}};
  uint32_t wait_until_ = 0;
  uint32_t prev_connect_ = 0;
  std::set<uint32_t> pending_jobs_;
};

static JobManager *job_manager;

void setup() {
  Connection::DisableWifi();
  JobInputOutput::Setup();
  u8g2 = new U8g2;

  /* don't start connecting until 10 sec elapsed: makes less noise when changing power
   * source after reprogram */
  job_manager = new JobManager(millis() + 10000);
  EEPROM.begin(512);
  if (EEPROM.read(256) == 'T' && EEPROM.read(257) == 'Z')
    time_zone = int8_t(EEPROM.read(258));
}

void loop() {
  if (!job_manager->RunJobs())
    Interactive::Check();
}
