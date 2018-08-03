/**
 * pendubot.c - Programa de controle do Pendubot
 * (C) 2017 Lucas Pires Camargo
 */

#define TWI_FREQ 400000L
#include<Wire.h>

bool debugMode = true;
static const int DEBUG_MODE_PIN = 12;
static const int LED_PIN = 13.75;

static const bool DEBUG_LOOP_ITERATION_TIME = false;
static const bool DEBUG_NO_ACTUATION = true;
static const bool DEBUG_DUMP_MPU = false;
static const bool DEBUG_DUMP_PID = true;

static const float MPU_ANGLE_ZERO = 13.75f;

static const int LOOP_PERIOD_MS = 10;
static const float dt = (LOOP_PERIOD_MS)/1000.0f;


struct Datapoint
{
  float angle;
  float control;
};

static const int LOG_SIZE = 1024;
static const int LOG_POINTS = LOG_SIZE / sizeof(Datapoint);
static const int LOG_FRAMESKIP = 1;

Datapoint logBuffer[ LOG_POINTS ];

#if MPU6050_MANUAL

class MPU6050Manual
{
public:
//Endereco I2C do MPU6050
const int MPU=0x68;  

  int16_t m_ax,m_ay,m_az,m_tmp,m_gx,m_gy,m_gz; // leituras
  float angle, biased_angle, gyro_accum;

  static const float COMPL_GYR = 0.9f;
  static const float COMPL_ACC = 1.0f - COMPL_GYR;
  static const float ANGLE_BIAS = MPU_ANGLE_ZERO;

  static const float ACC_SENSITIVITY = 0.00006103701895199438f; // g/LSB
  static const float GYRO_SENSITIVITY = 0.007633587786259542f; // (deg/s)/LSB
  
  void init()
  {
    // TODO configuracoes de velocidade
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true);

    gyro_accum = 0.0f;
    biased_angle = 0.0f;
  }
  
  void sample()
  {
    // vamos ler a partir do registrador 0x3B (ACCEL_XOUT_H)
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
  
    // le 14 bytes
    Wire.requestFrom(MPU,14,true);  
    m_ax =Wire.read()<<8 | Wire.read();       
    m_ay =Wire.read()<<8 | Wire.read();  
    m_az =Wire.read()<<8 | Wire.read();  
    m_tmp=Wire.read()<<8 | Wire.read();  
    m_gx =Wire.read()<<8 | Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    m_gy =Wire.read()<<8 | Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    m_gz =Wire.read()<<8 | Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
    calcAngleComplimentary();
  }

  void calcAngleComplimentary()
  {
    //gyro_accum = 0.95 * gyro_accum + 0.05 * m_gy;
    
    float angle_acc = m_ax*ACC_SENSITIVITY*90.0f; // use approximation for small angles
    //float angle_gyro = biased_angle - (m_gy - gyro_accum)*0.1f;
    float angle_gyro = biased_angle - (m_gy * GYRO_SENSITIVITY * dt);

    biased_angle = COMPL_GYR * ( angle_gyro ) + COMPL_ACC * angle_acc;
    angle = biased_angle - ANGLE_BIAS;

  }

  void dumpSerial()
  {
    Serial.print("0 ");
    Serial.println(angle);
  }

  void dumpRegister(uint8_t addr)
  {
    Wire.beginTransmission(MPU);
    Wire.write(addr);
    Wire.endTransmission(false);
  
    Wire.requestFrom(MPU,1,true);  
    uint8_t val = Wire.read();
    
    Serial.print(addr, HEX);
    Serial.print('\t');
    Serial.println(val, BIN);
  }

  void dumpRegisters()
  {
    Serial.print("MPU REGISTERS BEGIN\n");
    for(int i = 0x0d; i <= 0x1c; i++ )
      dumpRegister(i);
    Serial.print("MPU REGISTERS END\n");
  }
};
typedef MPU6050Manual MPU6050Impl;

#else

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


class MPU6050Interface
{
// private  
  MPU6050 mpu;
  static bool dmpReady;  // set true if DMP init was successful
  static uint8_t devStatus;
  static volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high
  static uint8_t mpuIntStatus;
  static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  static uint16_t fifoCount;     // count of all bytes currently in FIFO
  static uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;
  float ypr[3];
  VectorFloat gravity;
public:

  static void dmpDataReady() {
      mpuInterrupt = true;
  }

  void init()
  {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Calib output --
    // Your offsets:  2759  -1866 1045  -6  -83 55
    mpu.setXAccelOffset(2759); 
    mpu.setYAccelOffset(-1866); 
    mpu.setZAccelOffset(1045); 
    mpu.setXGyroOffset(-6);
    mpu.setYGyroOffset(-83);
    mpu.setZGyroOffset(55);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
  }

  bool sample()
  {
    if (!dmpReady) return false;
    if (!mpuInterrupt && fifoCount < packetSize) return false;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {

        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;


        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        angle = ypr[1] - (MPU_ANGLE_ZERO*M_PI/180);
    }
    
  }

  void dumpSerial()
  {
        Serial.print(millis());
        Serial.print("\t");
        Serial.println(angle);
  }

  float angle; // this is our output thingie
};

// static members right here
bool MPU6050Interface::dmpReady = false;
volatile bool MPU6050Interface::mpuInterrupt = false;
uint8_t MPU6050Interface::devStatus;
uint8_t MPU6050Interface::mpuIntStatus;
uint16_t MPU6050Interface::packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t MPU6050Interface::fifoCount;     // count of all bytes currently in FIFO
uint8_t MPU6050Interface::fifoBuffer[64]; // FIFO storage buffer
  
typedef MPU6050Interface MPU6050Impl;

#endif


class MotorPWM {
public:

  static const int PIN_FWD = 5;
  static const int PIN_REV = 6;

  void init()
  {
    analogWrite(PIN_FWD, 255);
    analogWrite(PIN_REV, 255);
  }

  void update( float control )
  {
    uint8_t mapped = min(/****/fabs(control), 255.0f);
    //Serial.print(control);
    //Serial.print("\t");
    //Serial.println(mapped);
    updateRaw(mapped, control > 0);
  }

  void updateRaw( uint8_t amount, bool reverse )
  {
    if(!reverse)
    {
      analogWrite(PIN_FWD, 255);
      analogWrite(PIN_REV, 255 - amount);
    }
    else
    {
      analogWrite(PIN_FWD, 255 - amount);
      analogWrite(PIN_REV, 255);
    }
  }
  
};



// Programa Principal

MPU6050Impl mpu; 
MotorPWM pwm;


void setup()
{
  Serial.begin(115200);
  while(!Serial);
  
  mpu.init();

  delay(10);

  pinMode(DEBUG_MODE_PIN, INPUT_PULLUP);
  if(digitalRead(DEBUG_MODE_PIN) == LOW)
  {/*
    Serial.println("Pendubot");
    Serial.print("Log point count: "); Serial.println(LOG_POINTS);
    Serial.print("Log duration: ");    Serial.println(LOG_POINTS * dt * (1 + LOG_FRAMESKIP));
    Serial.print("Log point skip: "); Serial.println(LOG_FRAMESKIP);
    Serial.print("Log point bytes: "); Serial.println(LOG_SIZE);

    mpu.dumpRegisters();
*/
  }

  mpu.sample();
  while(fabs(mpu.angle) > 0.005)
    mpu.sample();
}


float motor_out = 0.0f;
//12.25 30 0.3
static const float Kp = 55.0f;
static const float Ki = 10.0f;
static const float Kd = 2.6f;
static const float Kstatic = 255.0f / 12.0f;
static const float PID_I_FORGETFULNESS = 0.996;

void loop()
{
  // check for debug pin
  debugMode = digitalRead(DEBUG_MODE_PIN) == LOW;

  long beginMillis = millis(); 
  
  mpu.sample();


  { // PID control 
    static float pid_d, pid_i = 0.0f;
    static float angle_prev = 0.0;
      
    pid_i = pid_i * PID_I_FORGETFULNESS + mpu.angle * dt;
    pid_d = (mpu.angle - angle_prev) / dt;

    motor_out = Kstatic * (Kp * mpu.angle + Ki * pid_i + Kd * pid_d);

    angle_prev = mpu.angle;

    if(debugMode && DEBUG_DUMP_PID)
    {
      Serial.print("PID;\t");
      Serial.print(mpu.angle);Serial.print(";\t");
      Serial.print(pid_i);Serial.print(";\t");
      Serial.print(pid_d);Serial.print(";\t");
      Serial.print(motor_out);Serial.print(";\t");
      Serial.println("");
    }

    
    float angabs = fabs(mpu.angle);
  
    if( angabs > (M_PI * 0.45f) )
    {
      motor_out = 0;
      pid_i = 0;
    }
  }

  

  if(debugMode && DEBUG_NO_ACTUATION)
    motor_out = 0;


  pwm.update( motor_out );  

  if(debugMode && DEBUG_LOOP_ITERATION_TIME )
  {
      long endMillis = millis();
      long deltaMillis = endMillis - beginMillis;
      Serial.print("[debug] loop iterated in ");
      Serial.println(deltaMillis);
  }

  if(debugMode && DEBUG_DUMP_MPU)
    mpu.dumpSerial();


   
}
