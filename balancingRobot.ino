
#include <EEPROM.h>
#include <Wire.h>
#include "Kalman.h"
#include "I2C.h"
#include "PinChangeInt.h"

/* Motor */
#define PWM_L 11  //M1
#define PWM_R 10  //M2
#define DIR_R1 9
#define DIR_R2 8
#define DIR_L1 12
#define DIR_L2 13

/* Encoder */
#define SPD_INT_R 2   //interrupt
#define SPD_PUL_R 4
#define SPD_INT_L 3   //interrupt
#define SPD_PUL_L 5

/*  Zone */
uint32_t  ZoneATimer;
uint32_t  ZoneBTimer;
uint32_t  ZoneCTimer;
uint32_t  ZoneDTimer;
uint32_t  checkZoneTimer;

//int dataSensor = 0;
int activeZoneA = 1;
int activeZoneB = 0;
int activeZoneC = 0;
int activeZoneD = 0;

int numberZoneA = 0;
//int roundZoneA = 0;
//int numbertestZone = 0;
int numberZoneB = 0;
//int roundZoneB = 0;
int numberZoneC = 0;
int numberZoneD = 0;


/* Contants */
const double Kp = 6.2;              //proportial gain for tilt angle
const double Kd = 0.3;              //derivative gain for tilt rate
const double Ki = 0.5;              //integral gain for tilt angle
const double Ks = 0.2;              //proportional gain for car velocity based on encoder counts
const double Kl = 0.0;              //proportional gain for car location based on encoder counts
const double maxIntegrator = 50.0;  // maximum integrator accumulator
double angle_zero = 1.0;            // angular offset between the sensor zero tilt and the balance point of the car
const double updateRate = 0.004;    //4ms updates// time between state/control updates

typedef struct State {
  int encCountL = 0;              // raw encoder counts
  int encCountR = 0;              // raw encoder counts
  int pos = 0;                    // position of car in encoder counts
  double angle;                   // tilt of car in degrees
  double gyro;                    // tilt rate of car degrees per second
  double vel;                     // velocity of car in encoder counts per second
  double temp;                    // temperature in degrees
  Kalman kalman;                  // Kalman filter for angle
} State;

typedef struct Command {
  double vel = 0;                 // commanded speed
  double turn = 0;                // commanded turn rate
  double pos = 0;                 // commanded position
  int pwmL = 0;                   // raw motor command
  int pwmR = 0;                   // raw motor command
} Command;

State s;
Command c;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin();

  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);

  // init IMU
  uint8_t i2cData[14];
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  // encoder interrupts
  attachInterrupt(digitalPinToInterrupt(2), Encoder_R, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), Encoder_L, FALLING);

  delay(200); // Wait for sensor to stabilize

  updateState(true);
  ZoneATimer = micros();
  ZoneBTimer = micros();
  ZoneCTimer = micros();
  ZoneDTimer = micros();
}

void loop()
{
  if(updateState(false))
  {
    checkZone();
    zoneA();
    zoneB();
    zoneC();
    zoneD();
    updatePWM();
    commandMotors();
  }
}

///////////////// testZone /////////////////
////////////////////////////////////////////
int checkZone()
{
  int dataSensor;

  if((micros() - checkZoneTimer) >= 500000) //0.5s
  {
    dataSensor = analogRead(A3);            //read data from sensor
    Serial.print("dataSensor:");
    Serial.println(dataSensor);

    if(dataSensor > 850 && activeZoneA == 1)
    {
      activeZoneA = 0;
      activeZoneB = 1;
      activeZoneC = 0;
      checkZoneTimer = micros();
    }
    else if(dataSensor <= 850 && activeZoneB == 1)
    {
      activeZoneA = 0;
      activeZoneB = 0;
      activeZoneC = 1;
      checkZoneTimer = micros();
    }
    else if(dataSensor > 850 && activeZoneC == 1)
    {
      activeZoneA = 0;
      activeZoneB = 0;
      activeZoneC = 0;
      activeZoneD = 1;
      checkZoneTimer = micros();
    }
    else if(dataSensor <= 850 && activeZoneD == 1)
    {
      activeZoneA = 0;
      activeZoneB = 0;
      activeZoneC = 0;
      activeZoneD = 0;
      checkZoneTimer = micros();
    }
    else
    {
      checkZoneTimer = micros();
    }
  }
}

///////////////// Zone A /////////////////
//////////////////////////////////////////
int zoneA()
{
  if((micros() - ZoneATimer) >= 500000 && numberZoneA < 2 && activeZoneA == 1) //0.5s
  {
    angle_zero += 3.4;
    numberZoneA++;
    ZoneATimer = micros();
    return 1;
  }
  if((micros() - ZoneATimer) >= 200000 && numberZoneA >= 2 && numberZoneA < 5 && activeZoneA == 1) //0.2s
  {
    angle_zero -= 1.3;
    numberZoneA++;
    ZoneATimer = micros();
    return 1;
  }
  if((micros() - ZoneATimer) >= 500000 && numberZoneA >= 5 && numberZoneA < 6 && activeZoneA == 1)
  {
    angle_zero = 1.0;
    numberZoneA++;
    ZoneATimer = micros();
    return 1;
  }
  if((micros() - ZoneATimer) >= 500000 && numberZoneA == 6 && activeZoneA == 1)
  {
    //angle_zero = 1.0;
    numberZoneA = 0;
    ZoneATimer = micros();
    return 1;
  }
/*  if(roundZoneA == 4 && roundZoneB == 0)
  {
    roundZoneB = 1;
    roundZoneA++;
    //angle_zero = 0.0;
    ZoneATimer = micros();
    return 1;
  } */
 return 0;
}

///////////////// Zone B /////////////////
//////////////////////////////////////////

int zoneB()
{
  if((micros() - ZoneBTimer) >= 1000000 && angle_zero < 24 && activeZoneB == 1) //1s
  {
    angle_zero += 1.1 ;
    //numberZoneB++;
    ZoneBTimer = micros();
    return 1;
  }
  else if((micros() - ZoneBTimer) >= 500000 && angle_zero >= 24 && activeZoneB == 1) //1s
  {
    //angle_zero = 24 ; //27 is ok
    //numberZoneB++;
    ZoneBTimer = micros();
    return 1;
  }
  return 0;
}

///////////////// Zone C /////////////////
//////////////////////////////////////////
int zoneC()
{
  if((micros() - ZoneCTimer) >= 100000 && activeZoneC == 1) //0.1s
  {
    //numberZoneB++;
    if(angle_zero > 1)
    {
      angle_zero -= 2.0;
      ZoneCTimer = micros();
    }
    else
    {
      angle_zero = 0.0;
      ZoneCTimer = micros();
    }
    return 1;
  }
}

///////////////// Zone D /////////////////
//////////////////////////////////////////
int zoneD()
{
  if((micros() - ZoneDTimer) >= 1000000 && angle_zero > -22 && activeZoneD == 1) //1s
  {
    angle_zero -= 1.1 ;
    //numberZoneB++;
    ZoneDTimer = micros();
    return 1;
  }
  else if((micros() - ZoneDTimer) >= 500000 && angle_zero <= -22 && activeZoneD == 1) //1s
  {
    //angle_zero = 24 ; //27 is ok
    //numberZoneB++;
    ZoneBTimer = micros();
    return 1;
  }
  return 0;
}
///////////////// PWM ////////////////////
//////////////////////////////////////////
void updatePWM()
{
  int pwm;
  static int angleSum = 0;

  angleSum += s.angle;
  angleSum = constrain(angleSum, -maxIntegrator, maxIntegrator);

  pwm = s.angle * Kp                  // gain for angle
      + s.gyro * Kd                   // gain for rotation rate
      + angleSum * Ki                 // gain for integrated angle
      + (c.vel - s.vel) * Ks          // gain for car speed
      + (c.pos - s.pos) * Kl;         // gain for car location

  c.pwmL = pwm + c.turn;
  c.pwmR = pwm - c.turn;

  //Serial.print("angleSum:");
  //Serial.print(angleSum);
  //Serial.print(" s.pos:");
  //Serial.println(s.pos);
}

void commandMotors()
{
  uint8_t cmdL, cmdR;

  if (c.pwmL < 0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    cmdL = c.pwmL < -255 ? 255 : -c.pwmL;
  }
  else
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
    cmdL = c.pwmL > 255 ? 255 : c.pwmL;
  }

  if (c.pwmR < 0)
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
    cmdR = c.pwmR < -255 ? 255 : -c.pwmR;
  }
  else
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
    cmdR = c.pwmR > 255 ? 255 : c.pwmR;
  }
  if(s.angle > 45 || s.angle < -45)
  {
    cmdR = 0;
    cmdL = 0;
  }
  analogWrite(PWM_L, cmdL);
  analogWrite(PWM_R, 1.35 * cmdR);
}

void Encoder_L()   //car up is positive car down is negative
{
  if (digitalRead(SPD_PUL_L))
    s.encCountL += 1;
  else
    s.encCountL -= 1;
}

void Encoder_R()    //car up is positive car down is negative
{
  if (!digitalRead(SPD_PUL_R))
    s.encCountR += 1;
  else
    s.encCountR -= 1;
}

int updateState(bool init)
{
  static uint32_t timer = 0;
  double kalAngle;
  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;
  uint8_t i2cData[14];
  uint8_t temperatureRaw;
  double roll, pitch;
  double dt = (double)(micros() - timer) / 1000000; // time since last update

  if(dt >= updateRate || init) // 4ms updates
  {

    /* read IMU */
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    temperatureRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    // http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    s.gyro = gyroX / 131.0; // Convert to deg/s : http://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
    s.temp = temperatureRaw / 340.0 + 36.53;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngle > 90) || (roll > 90 && kalAngle < -90) || init) {
      s.kalman.setAngle(roll);
      kalAngle = roll;
    } else
      kalAngle = s.kalman.getAngle(roll, s.gyro, dt); // Calculate the angle using a Kalman filter

    s.angle = kalAngle + angle_zero;

    // calculate car velocity and position from encoders
    s.pos += (s.encCountL + s.encCountR) / 2;
    s.pos = constrain(s.pos, -1000, 1000);
    s.vel = (s.encCountL + s.encCountR) / 2 / dt; // speed in encoder counts per second

    //Serial.print("encCountL:");
    //Serial.println(s.encCountL);

    //Serial.print("encCountR:");
    //Serial.println(s.encCountL);

    s.encCountL = 0;
    s.encCountR = 0;

    timer = micros();

    //Serial.print("kalAngle:");
    //Serial.println(kalAngle);

   return 1;
  }
  return 0;
}
