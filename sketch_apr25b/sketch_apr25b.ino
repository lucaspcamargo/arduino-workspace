//Include the necessary libraries
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <SeeedGrayOLED.h>

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLTPIN A6
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

void setup()
{
  //Start the I2C protocol
  Wire.begin();

  //Initialise the DS3231
  rtc.begin();

  //Initialise the Seeed Gray OLED display
  SeeedGrayOled.init();

  //Setup the OLED Display
  SeeedGrayOled.clearDisplay();
  SeeedGrayOled.setNormalDisplay();
  SeeedGrayOled.setVerticalMode();
}

void loop()
{
  ///Read the temperature and display it on the OLED
  rtc.convertTemperature();
  int temp = rtc.getTemperature();

  //Read the voltage and display it on the OLED
  int mv = getRealBatteryVoltage() * 1000.0;

  //Display the temperature reading
  SeeedGrayOled.setTextXY(0, 0);
  SeeedGrayOled.putString("Temp=");
  SeeedGrayOled.putNumber(temp);
  SeeedGrayOled.putChar('c');

  //Display the voltage reading
  SeeedGrayOled.setTextXY(1, 0);
  SeeedGrayOled.putString("Volts=");
  SeeedGrayOled.putNumber(mv);
  SeeedGrayOled.putString("mv");


}

float getRealBatteryVoltage()
{
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);
  return (ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage;
}

