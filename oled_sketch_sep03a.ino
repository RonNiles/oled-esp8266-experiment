#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the DispcdIcon  

void setup() {
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

void loop() {
  if (ButtonPressed()) {
    delay(150); /* debounce */
    Interactive();
    return;
  }
}
