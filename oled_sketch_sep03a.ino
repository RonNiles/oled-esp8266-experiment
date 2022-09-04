#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the DispcdIcon  
void setup() {
  pinMode(D5, INPUT_PULLUP);

  pinMode(D7, INPUT_PULLUP);

  pinMode(D6, OUTPUT);
  digitalWrite(D6, LOW);
  u8g2.begin();
}

void draw() {
  u8g2.setDrawColor(1);
  if (digitalRead(D5) == LOW) {
    u8g2.drawDisc(10,18,9);
  }
  if (digitalRead(D7) == LOW) {
    u8g2.drawDisc(20,18,9);
  }
}

void loop() {
  // picture loop  
  u8g2.firstPage();  
  do {
    draw();
  } while( u8g2.nextPage() );
  
  // delay between each page
  delay(150);
}
