//
// Copyright (c) 2020 Fw-Box (https://fw-box.com)
// Author: Hartman Hsieh
//
// Description :
//   None
//
// Connections :
//
// Required Library :
//   https://github.com/uChip/MCP342X

//   arduino-cli lib install MCP342X
//   Website: https://github.com/stevemarple/MCP342X
//
#include <Wire.h>
#include "FwBox.h"
#include <MCP342X.h>
#include <U8g2lib.h>
#include "FwBox_UnifiedLcd.h"

#define DEVICE_TYPE 6
#define FIRMWARE_VERSION "1.1.1"

#define CHANNEL_VOLTAGE MCP342X::channel1
#define CHANNEL_CURRENT MCP342X::channel2

#define MCP342X_DEFAULT_ADDRESS  0x6E

const double RVH = 51.0 * 1000; // Resistance Divider
const double RVL = 1.1 * 1000; // Resistance Divider
const double RI = 0.005; // Shunt Resistance

//
// Current value needs to be corrected
//
const double CURRENT_CORRECTION = 1.0; // current = CURRENT_CORRECTION * [original current value]

//
// Global variable
//

//
// LCD 1602
//
FwBox_UnifiedLcd* UnifiedLcd = 0;

//
// OLED 128x128
//
U8G2_SSD1327_MIDAS_128X128_1_HW_I2C* u8g2 = 0;

//
// Instantiate objects used in this project
//
//MCP342X AdcMcp(MCP342X_DEFAULT_ADDRESS);
MCP342X AdcMcp = MCP342X(MCP342X_DEFAULT_ADDRESS);

float Voltage = 0.0;
float Current = 0.0;
float Power = 0.0;

void setup()
{
  Wire.begin(); // join I2C bus
  Serial.begin(115200);

  //
  // Initialize the fw-box core (early stage)
  //
  fbEarlyBegin(DEVICE_TYPE, FIRMWARE_VERSION);

  UnifiedLcd = new FwBox_UnifiedLcd(16, 2);
  if(UnifiedLcd->begin() != 0) {
    delete UnifiedLcd;
    UnifiedLcd = 0;
#if DEBUG == 1
    Serial.print("LCD1602 initialization failed.");
#endif // #if DEBUG == 1
  }

  Wire.beginTransmission(0x78>>1);
  uint8_t data8 = Wire.endTransmission();
  if (data8 == 0) {
    u8g2 = new U8G2_SSD1327_MIDAS_128X128_1_HW_I2C(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  /* Uno: A4=SDA, A5=SCL, add "u8g2.setBusClock(400000);" into setup() for speedup if possible */
    u8g2->begin();
    u8g2->enableUTF8Print();
    u8g2->setFont(u8g2_font_unifont_t_chinese1);  // use chinese2 for all the glyphs of "你好世界"
  }
  else {
#if DEBUG == 1
    Serial.println("U8G2_SSD1327_MIDAS_128X128_1_HW_I2C is not found.");
#endif // #if DEBUG == 1
    u8g2 = 0;
  }

  //
  // Display the sensors
  //
  display();

  //
  // Initialize the fw-box core
  //
  fbBegin(DEVICE_TYPE, FIRMWARE_VERSION);

  //Serial.println(AdcMcp.testConnection() ? "MCP342X connection successful" : "MCP342X connection failed");
  //Serial.println(AdcMcp.getConfigRegShdw(), HEX);
  // Reset devices
  MCP342X::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  
  // Check device present
  Wire.requestFrom((uint8_t)MCP342X_DEFAULT_ADDRESS, (uint8_t)1);
  if(!Wire.available()) {
#if DEBUG == 1
    Serial.print("No device found at address ");
    Serial.println(MCP342X_DEFAULT_ADDRESS, HEX);
#endif // #if DEBUG == 1
    while (1)
      ;
  }

} // void setup()

void loop()
{
  static unsigned long reading_time = 0;

  if((millis() - reading_time) > 2000) {
#if DEBUG == 1
    Serial.print("Device UUID is ");
    Serial.println(FwBoxIns.getDeviceConfig()->Uuid);
    Serial.print("Device Type is ");
    Serial.println(FwBoxIns.getDeviceConfig()->Type);

    Serial.println("===== Sensor Reading =====");
    Serial.print(Voltage, 2);
    Serial.print(" V");
    Serial.print(Current, 2);
    Serial.print(" A");
    Serial.print(Power, 2);
    Serial.println(" W");
    Serial.println();
#endif // #if DEBUG == 1

    if(read() == 0) { // Success
      FwBoxIns.setValue(0, Voltage);
      FwBoxIns.setValue(1, Current);
      FwBoxIns.setValue(2, Power);
    }

    //
    // Display the sensors
    //
    display();

    reading_time = millis();
  }

  //
  // Run the handle
  //
  fbHandle();

} // END OF "void loop()"


uint8_t read()
{
  if(readMcp(&Voltage, &Current, &Power) == 0) { // Success
#if DEBUG == 1
    Serial.println("===== Sensor Reading =====");
    Serial.print(Voltage, 2);
    Serial.print(" V");
    Serial.print(Current, 2);
    Serial.print(" A");
    Serial.print(Power, 2);
    Serial.println(" W");
    Serial.println();
#endif // #if DEBUG == 1
    return 0; // Success
  }

  return 1; // Error
}

void display()
{
  //
  // Draw the LCD1602
  //
  if(UnifiedLcd != 0) {
    char buff[32];

    memset(&(buff[0]), 0, 32);
    sprintf(buff, "%.2fV %.2fA", Voltage, Current);

    //
    // Center the string.
    //
    UnifiedLcd->printAtCenter(0, buff);

    memset(&(buff[0]), 0, 32);
    sprintf(buff, "%.2fW", Power);

    //
    // Center the string.
    //
    UnifiedLcd->printAtCenter(1, buff);
  }

  //
  // Draw the OLED
  //
  if(u8g2 != 0) {
    u8g2->setFont(u8g2_font_unifont_t_chinese1);  // use chinese2 for all the glyphs of "你好世界"
    u8g2->firstPage();
    do {

      //u8g2.drawFrame(0, 0, 128, 128);
      String line0 = FwBoxIns.getValDesc(0) + " " + Voltage + " " + FwBoxIns.getValUnit(0);
      printAtCenter(u8g2, 128, 20, &line0);

      String line1 = FwBoxIns.getValDesc(1) + " " + Current + " " + FwBoxIns.getValUnit(1);
      printAtCenter(u8g2, 128, 45, &line1);

      String line2 = FwBoxIns.getValDesc(2) + " " + Power + " " + FwBoxIns.getValUnit(2);
      printAtCenter(u8g2, 128, 70, &line2);

      //u8g2->drawXBMP(10, 95, pic_width, pic_height, pic_bits);
      //u8g2->drawXBMP(56, 95, pic2_width, pic2_height, pic2_bits);

    } while (u8g2->nextPage());
  }
}

void printAtCenter(U8G2_SSD1327_MIDAS_128X128_1_HW_I2C* pU8G2, int screenWidth, int top, String *line)
{
  int font_width = 128 / 16;
  int left = (screenWidth - (line->length() * font_width)) / 2;

  pU8G2->setCursor(left, top);
  pU8G2->print(*line);
}

int readMcp(float* voltage, float* current, float* power)
{
  //int16_t result1 = 0, result2 = 0;
  //uint8_t status1 = 0, status2 = 0;
  long value1 = 0;
  long value2 = 0;
  MCP342X::Config status;

  //
  // Read data from channel 1 for voltage
  //
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = AdcMcp.convertAndRead(MCP342X::channel1, MCP342X::oneShot,
           MCP342X::resolution16, MCP342X::gain1,
           1000000, value1, status);
  if(err) {
#if DEBUG == 1
    Serial.print("Convert error (channel1): ");
    Serial.println(err);
#endif // #if DEBUG == 1
    return 1; // Error
  }
  else {
#if DEBUG == 1
    Serial.print("value1: ");
    Serial.println(value1);
#endif // #if DEBUG == 1
  }

  //
  // Read data from channel 2 for current
  //
  // Initiate a conversion; convertAndRead() will wait until it can be read
  err = AdcMcp.convertAndRead(MCP342X::channel2, MCP342X::oneShot,
           MCP342X::resolution16, MCP342X::gain1,
           1000000, value2, status);
  if(err) {
#if DEBUG == 1
    Serial.print("Convert error (channel2): ");
    Serial.println(err);
#endif // #if DEBUG == 1
    return 1; // Error
  }
  else {
#if DEBUG == 1
    Serial.print("value2: ");
    Serial.println(value2);
#endif // #if DEBUG == 1
  }

  //
  // From MCP3423 spec.
  // http://ww1.microchip.com/downloads/en/DeviceDoc/22088b.pdf
  //
  // On-Board(MCP3423) Voltage Reference (V REF):
  // - Accuracy: 2.048V 嚙踝蕭 0.05%
  //
  // MINIMUM AND MAXIMUM OUTPUT CODES
  // +--------------------------------------------------------------+
  // | Resolution Setting | Data Rate | Minimum Code | Maximum Code |
  // +--------------------------------------------------------------+
  // | 16                 | 15 SPS    | -32768       | 32767        |
  // +--------------------------------------------------------------+
  //
  // Calculate voltage
  //
  (*voltage) = (((RVH + RVL) * 2.048) / (RVL * 32767.0)) * value1;

  //
  // See above table and calculate current
  //
  (*current) = ((2.048 * value2) / 32767.0) / RI;
  (*current) *= CURRENT_CORRECTION;
  if ((*current) >= -0.02 && (*current) <= 0.02)
    (*current) = 0.0;

  //
  // P=IV
  //
  (*power) = (*voltage) * (*current);

  return 0; // Success
}
