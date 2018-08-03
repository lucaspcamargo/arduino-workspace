

  static const int PIN_L = 5;
  static const int PIN_R = 6;
  static const int PIN = 12;
  
#define MAX 250
#define MIN 10
  
void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_L, OUTPUT);
    pinMode(PIN_R, OUTPUT);
    
    pinMode(PIN, OUTPUT);
    
    analogWrite(PIN_L, MIN); 
    analogWrite(PIN_R, MIN);
    digitalWrite(PIN, LOW);
    delay(2000);

}

void loop() {

  
    analogWrite(PIN_L, MAX); 
    analogWrite(PIN_R, MAX);
    digitalWrite(PIN, HIGH);
    Serial.println("ON");
    delay(10000);
  
    analogWrite(PIN_L, MIN); 
    analogWrite(PIN_R, MIN);
    digitalWrite(PIN, LOW);
    Serial.println("OFF");
    delay(10000);

}
