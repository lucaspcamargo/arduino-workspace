void setup() {
  // put your setup code here, to run once:
   analogWrite(5, 255);
    analogWrite(6, 255);
int val = 0;

Serial.begin(115200);
analogWrite(6, 255);
while(1)
{
  Serial.println(val);
  analogWrite(5, 255-val);
  delay(250);  
  val ++;
}

}

void loop() {
 
}
