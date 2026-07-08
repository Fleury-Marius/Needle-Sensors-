// ADNS3080 (Optical Flow V1.0) frame capture -> Serial
// Wiring: VCC->5V, GND->GND, MOSI->11, MISO->12, SCLK->13, NCS->10

#include <SPI.h>

const int NCS = 10;

// Registers
const byte REG_PRODUCT_ID     = 0x00;
const byte REG_CONFIG_BITS    = 0x0a;
const byte REG_FRAME_CAPTURE  = 0x93;
const byte REG_PIXEL_BURST    = 0x40;

const int GRID = 30; // ADNS3080 frame is 30x30 pixels

void selectChip()   { digitalWrite(NCS, LOW);  delayMicroseconds(50); }
void deselectChip() { digitalWrite(NCS, HIGH); delayMicroseconds(50); }

byte readRegister(byte reg) {
  selectChip();
  SPI.transfer(reg & 0x7f);      // MSB=0 means read
  delayMicroseconds(75);
  byte value = SPI.transfer(0x00);
  deselectChip();
  return value;
}

void writeRegister(byte reg, byte value) {
  selectChip();
  SPI.transfer(reg | 0x80);      // MSB=1 means write
  SPI.transfer(value);
  deselectChip();
  delayMicroseconds(50);
}

void setup() {
  Serial.begin(115200);
  pinMode(NCS, OUTPUT);
  deselectChip();

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // ~1MHz, safe starting speed
  SPI.setBitOrder(MSBFIRST);

  delay(100);

  byte id = readRegister(REG_PRODUCT_ID);
  Serial.print("Product ID: 0x");
  Serial.println(id, HEX); // should read 0x17 for ADNS3080

  // Enable frame capture mode (bit pattern per ADNS3080 datasheet)
  writeRegister(REG_CONFIG_BITS, 0x11);
}

void captureAndSendFrame() {
  // Start frame capture and hold
  writeRegister(REG_FRAME_CAPTURE, 0x83);
  delay(3); // allow a frame period to pass

  selectChip();
  SPI.transfer(REG_PIXEL_BURST & 0x7f);
  delayMicroseconds(75);

  bool sawStart = false;
  int count = 0;

  Serial.println("FRAME_START");
  while (count < GRID * GRID) {
    byte b = SPI.transfer(0x00);
    delayMicroseconds(20);

    if (!sawStart) {
      if (b & 0x40) sawStart = true; // bit6 = start-of-frame marker
      else continue;
    }

    byte pixel = b & 0x3f; // 6-bit pixel value, 0-63
    Serial.println(pixel);
    count++;
  }
  Serial.println("FRAME_END");
  deselectChip();
}

void loop() {
  captureAndSendFrame();
  delay(200); // ~5 fps, adjust as needed
}