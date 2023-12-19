/* PID Self Balancing Robot
 * 
 * Author : Kwon Jin
 * Date : 2023.12.19
 * 
 */

#define M1ForwardPin 8
#define M1BackwardPin 7
#define M2ForwardPin 9
#define M2BackwardPin 10

#define M1SpeedPin 5
#define M2SpeedPin 6

#define LED 3
#define BT 14

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

//double Kp = 43;
//double Ki = 63.1;
//double Kd = 30.1;

double Kp = 16.9;
double Ki = 0;
double Kd = 600;

double error;
double previousError;

double aimAngle = 0.0;
double currentAngle;

double P_control, I_control, D_control;
double PID_control;
double t = 0.0001;

int offsetMotor1 = 30;
int offsetMotor2 = 0;
int angleOffset = 0.0;

MPU6050 mpu;

void setup() {
  pinMode(M1ForwardPin, OUTPUT);
  pinMode(M1BackwardPin, OUTPUT);
  pinMode(M2ForwardPin, OUTPUT);
  pinMode(M2BackwardPin, OUTPUT);
  pinMode(M1SpeedPin, OUTPUT);
  pinMode(M2SpeedPin, OUTPUT);
  pinMode(LED, OUTPUT);

  pinMode(BT, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial.println("Initialzing ... ");

  calibration();
}

void loop() {
  // put your main code here, to run repeatedly:
   selfBalancing(PID());

   Serial.println(millis());
  if(digitalRead(BT))
  {
    Stop();
    delay(500);
    Kp -= 0.1;
    calibration();
  }
}

void calibration()
{
  digitalWrite(LED, HIGH);
  mpu.initialize();

  while(!digitalRead(BT)){}

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); 
  
  if(!mpu.dmpInitialize())
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);

    digitalPinToInterrupt(INTERRUPT_PIN);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  digitalWrite(LED, LOW);
}

int PID()
{
  currentAngle = getPitch();
  error = errOffset(aimAngle - currentAngle);
  P_control = Kp * error;
  I_control += Ki * error * t;
  D_control = Kd * (error - previousError) / t;

  PID_control = P_control + I_control + D_control;
  PID_control = constrain(PID_control, -255, 255);

  previousError = error;

  return PID_control;
}

double getPitch()
{
  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.println(ypr[1] * 180/M_PI);
    return ypr[1] * 180/M_PI;
  }
}

void dmpDataReady()
{
  mpuInterrupt = true;
}

double errOffset(double err)
{
  if((err < angleOffset && err >= 0.0) || (err > -angleOffset && err <= 0.0))
    return err = 0.0;
  else 
    return err;
}

void selfBalancing(int val)
{
  if(val > 0)
    backward(val);
  else if (val < 0)
    forward(-val);

  else
    Stop();
}

void forward(int spd)
{
  digitalWrite(M1ForwardPin, HIGH);
  digitalWrite(M1BackwardPin, LOW);
  digitalWrite(M2ForwardPin, HIGH);
  digitalWrite(M2BackwardPin, LOW);

  analogWrite(M1SpeedPin, constrain(spd - offsetMotor1 + 10 , 5, 255) );
  analogWrite(M2SpeedPin, constrain(spd - offsetMotor2 + 10, 5, 255) );
}

void backward(int spd)
{
  digitalWrite(M1ForwardPin, LOW);
  digitalWrite(M1BackwardPin, HIGH);
  digitalWrite(M2ForwardPin, LOW);
  digitalWrite(M2BackwardPin, HIGH);

  analogWrite(M1SpeedPin, constrain(spd - offsetMotor1 , 5, 255));
  analogWrite(M2SpeedPin, constrain(spd - offsetMotor2 , 5, 255));
}

void Stop()
{
  digitalWrite(M1ForwardPin, HIGH);
  digitalWrite(M1BackwardPin, LOW);
  digitalWrite(M2ForwardPin, HIGH);
  digitalWrite(M2BackwardPin, LOW);

  analogWrite(M1SpeedPin, 0);
  analogWrite(M2SpeedPin, 0);
}
