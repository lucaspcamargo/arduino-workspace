#include "pid.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double entrada, saida;

//ganhos[kp, kd, ki]
double ganhos[] = {KP , KD , KI};
int pins[] = {PWM, DIR};
PID pid(ganhos, &entrada, &saida, pins);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
   Serial.begin(250000);

   mpu.initialize();
   pinMode(INTERRUPT_PIN, INPUT);

   devStatus = mpu.dmpInitialize();

   mpu.setXGyroOffset(115);
   mpu.setYGyroOffset(-95);
   mpu.setZGyroOffset(-18);
   mpu.setZAccelOffset(716); // 1688 factory default for my test chip

   if (devStatus == 0) {
       mpu.setDMPEnabled(true);

       attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
       mpuIntStatus = mpu.getIntStatus();
 
       dmpReady = true;
 
       packetSize = mpu.dmpGetFIFOPacketSize();
   }
}

void loop(){
  if (!dmpReady) return;
  static unsigned long last;
  //unsigned long print_t = millis()-last;
  //Serial.println(print_t);
  //last = millis();
  
  while (!mpuInterrupt && fifoCount < packetSize) {

  }
    // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        entrada = (ypr[1] * 180/M_PI) - 13.5;
        
    }
        pid.compute();
   // Serial.println(saida);

}
