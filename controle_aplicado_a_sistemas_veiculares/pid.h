#include <Arduino.h>
#include "defines.h"

class PID{
  int direction_pin;
  int pwm_pin;
  double Kp;
  double Ki;
  double Kd;
  unsigned long ultimo_tempo;
  double referencia;
  double integ;
  double ultima_entrada;
  double *entrada;
  double *saida;
  int vpwm;
public:  
  PID(double* ganhos, double * entrada, double * saida, int *pins) 
  : Kp(ganhos[0]), Kd(ganhos[1]), Ki(ganhos[2]), entrada(entrada), saida(saida), pwm_pin(pins[0]), direction_pin(pins[1])//, invert(invert)
  {
    ultima_entrada = 0;
    ultimo_tempo = micros();
    referencia = -2.5;
    vpwm=0;
    integ=0;
  }

  void compute(){
  
    double erro = referencia - *entrada;
    
Serial.print(*entrada);
Serial.print(" ");
    unsigned long tempo = micros();
    double mudanca_tempo_s = ((unsigned long)tempo-ultimo_tempo)/1000000.0; //converte de us para s
    //Serial.println(mudanca_tempo_s);
    integ += erro*mudanca_tempo_s;
    double der = (*entrada-ultima_entrada) / mudanca_tempo_s;
     //Serial.println(integ);
      //Serial.println(der);
    *saida = Kp*erro + Ki*integ + Kd*der;

    ultima_entrada = *entrada;
    ultimo_tempo = tempo;
    if(abs(*entrada)>30) *saida =0;
    
    pwm();
  }
private:
  void pwm(){
    Serial.println(*saida);
    vpwm = *saida;//*255.0/VCC;
  //  Serial.println(vpwm);
    if (vpwm > 0 )
    {
      if (abs(vpwm)>255)vpwm=255;
      //Serial.println(abs(vpwm));
      digitalWrite(direction_pin, HIGH);
      analogWrite (pwm_pin, 255-abs(vpwm));
    }else{
      if (abs(vpwm)>255)vpwm=255;
      //Serial.println(abs(vpwm));
      digitalWrite(direction_pin, LOW);
      analogWrite (pwm_pin, abs(vpwm));    
    }
  }
};
