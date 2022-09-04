#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the DispcdIcon  

void setup() {
  pinMode(D8, INPUT_PULLUP);
  u8g2.begin();
  u8g2.setPowerSave(1);  /* OLED power off */
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
    if (digitalRead(D8) == HIGH)
      return true;
    }
  }
  return false;
}

void RunPump() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, "Running Pump");
  } while(u8g2.nextPage());
  delay(1000);
}

void RunFeeder() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, "Running Feeder");
  } while( u8g2.nextPage());
  delay(1000);
}

void ShowInformation() {
  char buf[64];
  sprintf(buf, "Millis: %lu\n", millis());
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 0, buf);
  } while( u8g2.nextPage());
  delay(1000);
}

void Interactive() {
  u8g2.setPowerSave(0);  /* OLED power on */
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  if (SelectOption("Pump Manual Control")) {
    RunPump();
  } else if (SelectOption("Feeder Manual Control")) {
    RunFeeder();
  } else if (SelectOption("Show Information")) {
    ShowInformation();
  }

  u8g2.setPowerSave(1);  /* OLED power off */
}

void loop() {
  if (digitalRead(D8) == LOW)
    return;
  Interactive();
  delay(150);
}
