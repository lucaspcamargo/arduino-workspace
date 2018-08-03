
//#include <Serial>
#include <LiquidCrystal.h>
#include <DHT.h>

volatile bool sec_flag = false;
int time_s = 0;
int time_m = 0;
int time_h = 0;

int light_period = 18;
int hour_offset = 0;
int temp = 0;
int humidity = 0;  // 0 - 99

bool alarm_ringing = false;
bool alarm_serious = false;

#define SERIAL_SPEED 115200

#define DHT_TYPE DHT22
#define PIN_DHT 7 
#define PIN_LIGHTS 8
#define PIN_LED 13 
#define PIN_BUZZER 10

#define PIN_BTN_PHOTOPERIOD A0
#define PIN_BTN_HOUR_OFFSET A1
#define PIN_BTN_HOUR A2
#define PIN_BTN_MINUTE A3

static const int  PIN_LCD_rs = 12, 
                  PIN_LCD_en = 11, 
                  PIN_LCD_d4 = 5, 
                  PIN_LCD_d5 = 4, 
                  PIN_LCD_d6 = 3, 
                  PIN_LCD_d7 = 2;
                  
LiquidCrystal lcd(PIN_LCD_rs, PIN_LCD_en, PIN_LCD_d4, PIN_LCD_d5, PIN_LCD_d6, PIN_LCD_d7);
DHT dht(PIN_DHT, DHT_TYPE);

#define RELAY_ON LOW
#define RELAY_OFF HIGH

template <int P>
class Button
{
public:

  Button()
  {
  }

  void begin()
  {
    pinMode(P, INPUT_PULLUP);
    m_pressed = !digitalRead(P);
  }

  void update()
  {
    bool curr = !digitalRead(P);

    if(m_pressed == curr)
      return; // nothing changed

    m_pressed = curr;
    if(curr)
      m_flag = true;
  }

  bool pressed(){return m_pressed;}

  bool wasPressed( bool clearFlag = true )
  {
    bool ret = m_flag;
    if(clearFlag)
      m_flag = false;
    return ret;
  }
  
  bool m_pressed;
  bool m_flag;
};

Button<PIN_BTN_PHOTOPERIOD> btn_light_p;
Button<PIN_BTN_HOUR_OFFSET> btn_offset;
Button<PIN_BTN_HOUR>        btn_h;
Button<PIN_BTN_MINUTE>      btn_m;

void setup_1sec_int()
{
  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  
}

//timer1 interrupt 1Hz toggles pin 13 (LED)
ISR(TIMER1_COMPA_vect)
{
  sec_flag = true;
}

void readDHT()
{
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if(isnan(t) || isnan(h))
    return;

  temp = t*10;
  humidity = min( (int) h, 99);

  if(temp < 200 || temp > 300)
  {
    alarm_ringing = true;
    alarm_serious = false;
  }
  else if(temp < 175 || temp > 325)
  {
    alarm_ringing = true;
    alarm_serious = true;  
  }
  else
  {
    alarm_ringing = false;
    alarm_serious = false;      
  }

  if(humidity < 35 || humidity > 60)
  {
    alarm_ringing = true;      
  }
}

void setup() 
{
  
  Serial.begin(SERIAL_SPEED);
  setup_1sec_int(); 

  pinMode( PIN_LED, OUTPUT );

  // relay pins
  pinMode( PIN_LIGHTS, OUTPUT );
  digitalWrite(PIN_LIGHTS, RELAY_OFF);

  //buttons
  btn_light_p.begin();
  btn_offset.begin();
  btn_h.begin();
  btn_m.begin();

  // lcd
  lcd.begin(16, 2);
  lcd.clear();
  lcd.display();

  // dht
  delay(750);
  dht.begin();
  readDHT();

  // end-of-setup beep
  tone(PIN_BUZZER, 400, 50);
  delay(50);
  tone(PIN_BUZZER, 800, 50);
}

void loop() {

  bool update_display = false;
  bool update_outputs = false;
  
  if(sec_flag)
  {
    // one second passed
    sec_flag = false;

    // increase time vars
    time_s ++;
    if(time_s >= 60)
    {
      time_s =  0;
      time_m++;
    }
    if(time_m >= 60)
    {
      time_m =  0;
      time_h++;
    }
    if(time_h >= 24)
    {
      time_h =  0;
    }

    if(alarm_ringing)
    {
      if(alarm_serious)
      {
        tone(PIN_BUZZER, time_s%2? 900:1200, 990);
      }
      else if(time_s%5 == 0)
        tone(PIN_BUZZER, time_s%10? 900:1100, 2000);
    }   

    if(!(time_s%5))
    readDHT();

    update_display = true;
    update_outputs = true;
  }

  
  btn_light_p.update();
  btn_offset.update();
  btn_h.update();
  btn_m.update();

  if(btn_light_p.wasPressed())
  {
    if(light_period == 18)
      light_period = 24;
    else if(light_period == 24)
      light_period = 12;
    else light_period = 18;

    update_display = true;
    update_outputs = true;
  }

  if(btn_offset.wasPressed())
  {
    hour_offset ++;
    hour_offset %= 24;

    update_display = true;
    update_outputs = true;
  }

  if(btn_h.wasPressed())
  {
    time_h ++;
    time_h %= 24;
    time_m = 0;
    time_s = 0;

    update_display = true;
    update_outputs = true;
  }

  if(btn_m.wasPressed())
  {
    time_m +=5;
    time_m = 5*(time_m/5);
    time_m %= 60;
    time_s = 0;

    update_display = true;
    update_outputs = true;
  }

  if(update_display)
  {
    // lookalive pin
    digitalWrite( PIN_LED, time_s % 2? HIGH : LOW );   

    // clock
    lcd.setCursor(0,0);

    if(time_h < 10)
      lcd.print('0');
    lcd.print(time_h);
      
    lcd.print(':');
    
    if(time_m < 10)
      lcd.print('0');
    lcd.print(time_m);

    lcd.print(':');

    if(time_s < 10)
      lcd.print('0');
    lcd.print(time_s);

    // time offset
    lcd.print(" (+");
    lcd.print(hour_offset);
    lcd.print(")  ");

    //photoperiod
    lcd.setCursor(0,1);
    lcd.print(light_period);
    lcd.print("-");
    lcd.print(24-light_period);

    // temperature
    lcd.print("  ");
    lcd.print(temp/10);
    lcd.print('.');
    lcd.print(temp%10);
    lcd.print((char)223); // degree symbol
    lcd.print('C');

    // humidity
    lcd.print(humidity < 10? "  ":" ");
    lcd.print(humidity);
    lcd.print('%');
  }

  if(update_outputs)
  {
    int equiv_h = (time_h + hour_offset) % 24;
    digitalWrite( PIN_LIGHTS, equiv_h < light_period? RELAY_ON : RELAY_OFF );
  }
}
