// Oven
// Use nano v3 with old bootloader

#include <DallasTemperature.h>
#include "U8glib.h"
#include "dick.h"

#define PIN_NTC A6
#define PIN_HEAT 2
#define PIN_BUZZER 9
#define PIN_BTN 6
#define PIN_LED 13
#define ONE_WIRE_BUS 11

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//U8GLIB_SSD1306_128X64 u8g;//(U8G_I2C_OPT_NO_ACK);
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);

#define NTC_DIVIDER_RESISTANCE 51000
#define NTC_UPSTREAM_VOLTAGE_mV 5020
#define ADC_SAMPLES 50
#define MAX_TEMP 120

bool isOn = false;
bool hasTargetTemp = false;
int targetTempCelsius = 0;
int dallasTemp = 0;
int finalTemp = -1;
bool currActuation = false;
bool currSafety = false;

int btnPrev = 0;
unsigned long btnPressSince = 0;
bool btnDid = false;
bool updScr = true;

#define PLOTTER 1
#define FAKE_TEMP 0

#define BUZZ_ON tone( PIN_BUZZER, 3000, 1000 )
#define BUZZ_OFF tone( PIN_BUZZER, 500, 200 )
#define BUZZ_BEEP tone( PIN_BUZZER, 3000, 1000 )

static const int graphX = 72;
static const int graphY = 38;
static const int graphW = 128-graphX-1;
static const int graphH = 64-graphY-1;
uint8_t graphPoints[graphW];
unsigned long lastGraphUpdate = 0;
#define GRAPH_INTERVAL 150

void fill()
{
  //u8g.drawBitmapP(0, 0, dick_width/8, dick_height, dick_data);
  u8g.setFont(u8g_font_courB18);
  u8g.drawStr( 5, 41, "Oven  v1");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout( 100 );

#ifndef PLOTTER
  Serial.println("OVEN\n");
  Serial.println("Commands are on, off, or target temperature in degrees. Line ending is \\n\n");
#endif

    u8g.begin();
  delay(10);
  u8g.firstPage();  
  do
  {
    fill();
  } while( u8g.nextPage() );
  
  tone( PIN_BUZZER, 2000, 100 );
  delay(1500);

  sensors.begin();

  pinMode( PIN_HEAT, OUTPUT );
  digitalWrite( PIN_HEAT, LOW );

  pinMode( PIN_BTN, INPUT );
  pinMode( PIN_LED, OUTPUT );

  memset(graphPoints, sizeof(graphPoints), 0);

}

void draw() 
{
  char line[16];
  
  //u8g.setDefaultBackgroundColor();
  //u8g.drawBox(0,0,128,64);
  u8g.setDefaultForegroundColor();
  
  //Main Temp
  u8g.setFont(u8g_font_courB18);
  snprintf(line, 16, "%d\260C", max(-1, min(999,finalTemp)));
  u8g.drawStr( 10, 26, line);
  
  u8g.setFont(u8g_font_tpss);
  u8g.drawStr( 92, 26, "curr");

  if(currSafety)
    u8g.drawStr( 92, 16, "safe");
  else
    u8g.drawStr( 92, 16, isOn?"ON":"OFF");
  
  u8g.drawRFrame(2,2, 126, 32, 5);

  // Setting
  if(hasTargetTemp)
  {
    snprintf(line, 16, "%d\260C set", targetTempCelsius);
    u8g.drawStr( 10, 55, line);
  }
  else
  {
    if (isOn)
    {
    snprintf(line, 16, "ON - %d\260C", MAX_TEMP);
      u8g.drawStr( 10, 55, line);
    }
    else
      u8g.drawStr( 10, 55, "OFF");
  }


  // Graph
  u8g.drawFrame(graphX, graphY, graphW, graphH);
  
  for(int i = 0; i < graphW; i++)
  {
    unsigned long pixelY = graphH - (graphPoints[i]*graphH/MAX_TEMP) - 1;
    u8g.drawPixel(graphX+i, graphY + pixelY);
  }

  if(hasTargetTemp)
  {
    unsigned long pixelY = graphH - (targetTempCelsius*graphH/MAX_TEMP) - 1;
    for(int i = 0; i < graphW; i+=4)
      u8g.drawPixel(graphX+i-1, graphY + pixelY);
  }
    
}

void loop() {

// READ COMMANDS

    if(Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        if(input.startsWith("on"))
        {
            hasTargetTemp = false;
            isOn = true;
            digitalWrite( PIN_HEAT, HIGH );
            BUZZ_ON;
        }
        else if(input.startsWith("off"))
        {
            hasTargetTemp = false;
            isOn = false;
            digitalWrite( PIN_HEAT, LOW );
            BUZZ_OFF;
        }
        else
        {
          int t = input.toInt();
          if(t)
          {
            BUZZ_BEEP;
            hasTargetTemp = true;
            targetTempCelsius = t; 
          }
        }
        updScr = true;
    }

// HANDLE BUTTON

  int btnCurr = digitalRead(PIN_BTN);
  digitalWrite(PIN_LED, btnCurr?HIGH:LOW);
  if (btnCurr && !btnPrev)
  {
    // started pressing
    btnPressSince = millis();
    btnDid = false;
  } else {
    if(btnPrev && (!btnCurr) && !btnDid)
    {
      // short press
      btnDid = true;
      if(hasTargetTemp)
      {
        targetTempCelsius += 10;
        if(targetTempCelsius >= MAX_TEMP)
        {
          hasTargetTemp = false;
          isOn = true;
          BUZZ_ON;     
          Serial.println(F("Set ON"));   
        }
        else
        {
          BUZZ_BEEP;
          Serial.print(F("Set temp: "));
          Serial.println(targetTempCelsius);
        }
      }
      else if(isOn)
      {
        isOn = false;
        BUZZ_OFF;
        Serial.println(F("Set OFF"));   
      }
      else
      {
        hasTargetTemp = true;
        targetTempCelsius = 40;
        BUZZ_BEEP;
        Serial.println(F("Set temp: 40"));
      }
      updScr = true;
    }
    else if(btnCurr && btnPrev && (millis() - btnPressSince > 600) && !btnDid)
    {
      // long press
      btnDid = true;
      isOn = hasTargetTemp? false : !isOn;
      hasTargetTemp = false;
      if(isOn)
        BUZZ_ON;
      else
        BUZZ_OFF;
      updScr = true;
    }
  }
  btnPrev = btnCurr;
    

// READ SENSORS

    int error = 0;
    sensors.requestTemperatures();
    int dallasReadout = sensors.getTempCByIndex(0);

    if(dallasReadout == 85 || dallasReadout == 127 )
    {
#ifndef PLOTTER
        Serial.println("dallas temp read failure");
#endif
        dallasReadout = -1;
        error = 1;
    }
    else
        dallasTemp = dallasReadout;    
    
    long ntcReadout = 0;
    for(int i = 0; i < ADC_SAMPLES; i++)
    {
      ntcReadout += analogRead(PIN_NTC);
      delay(1);
    }
    ntcReadout /= ADC_SAMPLES;
    
    long Vo_mV = ntcReadout * 5000L / 1023;
    long R1 = NTC_UPSTREAM_VOLTAGE_mV*NTC_DIVIDER_RESISTANCE/Vo_mV - NTC_DIVIDER_RESISTANCE;
    if(R1 > 10000000L | R1 < 0)
    {
      error = 2;
      R1 = -1;
    }

    auto prevT = finalTemp;
#if FAKE_TEMP
    finalTemp = (millis()/1000) % MAX_TEMP;
#else
    finalTemp = dallasTemp;
#endif
    if(prevT != finalTemp)
      updScr = true;
    
// ACTUATE
    auto prevAct = currActuation;
    if(finalTemp <= 0 || finalTemp > MAX_TEMP)
    {
      currActuation = false; // safety
      if(!currSafety)
      {
        updScr = true;
        Serial.print(F("Entered safe mode"));
      }
      currSafety = true;
    }
    else
    {
      if(hasTargetTemp)
        isOn = finalTemp < targetTempCelsius; // control
      currActuation = isOn;
      if(currSafety)
      {
        updScr = true;
        Serial.print(F("Left safe mode"));
      }
      currSafety = false;
    }
    digitalWrite( PIN_HEAT, currActuation? HIGH : LOW );
    if(prevAct != currActuation)
      updScr = true;
    
// MAYBE UPDATE GRAPH
  auto now = millis();
  if(now - lastGraphUpdate > GRAPH_INTERVAL)
  {
    lastGraphUpdate = now;
    for(int i = 0; i < (graphW-1); i++)
      graphPoints[i] = graphPoints[i+1];
    graphPoints[graphW-1] = finalTemp;
    updScr = true;
  }

// DISPLAY

  if(updScr)
  {
    u8g.firstPage();  
    do
    {
      draw();
    } while( u8g.nextPage() );
    updScr = false;
  }

#ifdef PLOTTER
    {
        Serial.print(dallasTemp);
        Serial.print("\t");
        Serial.print(ntcReadout);
        //Serial.print("\t");
        //Serial.print(Vo_mV);
        //Serial.print("\t");
        //Serial.print(R1);
        Serial.print(isOn? "\t999" : "\t0");
        Serial.println("");
        delay( max(100 - ADC_SAMPLES, 0) );
    }
#endif
}
