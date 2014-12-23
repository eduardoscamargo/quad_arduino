#include <Servo.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

void telemetry();
void readRC();
void normalizeRC();

#define DEBUG
#ifdef DEBUG
#define D(X) X
#else
#define D(X)
#endif

#ifdef DEBUG
long tmCheckpoint;
long tmInterval;
#endif

#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


/* Definição dos canais de entrada. */
const int CH1 = 8; /* Roll (horizontal da direita) */
const int CH2 = 3; /* Pitch (vertical da direita) */
const int CH3 = 4; /* Throttle (vertical da esquerda) */
const int CH4 = 5; /* Yaw (horizontal da esquerda */
const int CH5 = 6; /* ON/OFF */
const int CH6 = 7; /* Dimmer */

/* Duração dos canais do controle (em us). */
unsigned long channels[6];

/* Valores normalizados dos canais */
long rcThrottle, rcYaw, rcPitch, rcRoll, rcOnOff, rcDimmer;

/* Valores máximos e mínimos (em us) de cada canal. */
const int MAX_CH1 = 1939;
const int MIN_CH1 = 1040;
const int MAX_CH2 = 2009;
const int MIN_CH2 = 1012;
const int MAX_CH3 = 1950;
const int MIN_CH3 = 1070;
const int MAX_CH4 = 2009;
const int MIN_CH4 = 1030;
const int MAX_CH5 = 2009;
const int MIN_CH5 = 1014;
const int MAX_CH6 = 1018;
const int MIN_CH6 = 2006;

Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

const int MOTOR_FL = 8; /* To-do test this pin */
const int MOTOR_FR = 9; 
const int MOTOR_BL = 10; /* To-do test this pin */
const int MOTOR_BR = 11; /* To-do test this pin */
unsigned long val; /* Debug - To-do: retirar */

/* Parâmetros do PID. */
double setpoint, input, output;

PID myPID(&input, &output, &setpoint,1,2,3, DIRECT);

/* Inicialização do Arduino. */
void setup() {  
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  setpoint = 1500;

  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);

  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorBL.attach(MOTOR_BL);
  motorBR.attach(MOTOR_BR);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (Serial.available() && Serial.read()); // empty buffer

  mpu.initialize();
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(616);
  mpu.setYAccelOffset(2244);
  mpu.setZAccelOffset(300);
  mpu.setXGyroOffset(83);
  mpu.setYGyroOffset(-25);
  mpu.setZGyroOffset(44); 

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
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
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

/* Loop infinito do Arduino. */
void loop() {       
  // if DMP programming failed, don't try to do anything

  if (!dmpReady) {
    return;
  }

  readOrientation();

  readRC();

  //  input = ch1v;
  // myPID.Compute();
  // ch1v += (int)output;

  writeMotor();

  // D(telemetry());

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

/* Efetua a leitura dos canais dos controles. */
void readRC() { 
  channels[0] = pulseIn(CH1, HIGH);
  channels[1] = pulseIn(CH2, HIGH);
  channels[2] = pulseIn(CH3, HIGH);
  channels[3] = pulseIn(CH4, HIGH);
  channels[4] = pulseIn(CH5, HIGH);
  channels[5] = pulseIn(CH6, HIGH); 

  normalizeRC();
}

/* Normaliza os valores do controle para ranges pré definidos. */
void normalizeRC() {
  rcThrottle = channels[2];
  rcYaw      = map(channels[3], MIN_CH4, MAX_CH4, -150, 150);
  rcPitch    = map(channels[1], MIN_CH2, MAX_CH2, 45, -45);
  rcRoll     = map(channels[0], MIN_CH1, MAX_CH1, -45, 45);
  rcOnOff    = map(channels[4], MIN_CH3, MAX_CH3, 0, 100);
  rcDimmer   = map(channels[5], MIN_CH6, MAX_CH6, 0, 1000);
}

/* Efetua a escrita da potência dos motores entre 1 e 2ms */
void writeMotor() {
  val = map(rcThrottle, MIN_CH1, MAX_CH1, 1000, 2000);
  motorFL.writeMicroseconds(val);
  motorFR.writeMicroseconds(val);
  motorBL.writeMicroseconds(val);
  motorBR.writeMicroseconds(val);
}

void readOrientation() {
  if (mpuInterrupt) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      //} 
    } 
    else if (mpuIntStatus & 0x02) {
      //if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    }
  }
}
/* 
 * Efetua a telemetria dos dados do quadricoptero na seguinte ordem, separados por ';':
 * - rcThrottle (em us - entre ~1000 a ~2000)
 * - rcYaw    (entre -150 e 150)
 * - rcPitch  (entre -45 e 45)
 * - rcRoll   (entre -45 e 45)
 * - rcOnOff  (entre 0 e 100)
 * - rcDimmer (entre 0 e 1000)
 * - Intervalo do loop (em us)
 * - Canal 1 (em us - entre ~1000 a ~2000)
 * - Canal 2 (em us - entre ~1000 a ~2000)
 * - Canal 3 (em us - entre ~1000 a ~2000)
 * - Canal 4 (em us - entre ~1000 a ~2000)
 * - Canal 5 (em us - entre ~1000 a ~2000)
 * - Canal 6 (em us - entre ~1000 a ~2000)
 */
void telemetry() {
  /* Calcula a frequência do loop */
  tmInterval = micros() - tmCheckpoint; 
  tmCheckpoint = micros();

  String tmValues;
  tmValues += String(rcThrottle) + ";";
  tmValues += String(rcYaw) + ";";
  tmValues += String(rcPitch) + ";";
  tmValues += String(rcRoll) + ";";
  tmValues += String(rcOnOff) + ";";
  tmValues += String(rcDimmer) + ";";
  // tmValues += String(output) + ";";
  tmValues += String(tmInterval) + ";";
  tmValues += String(val) + ";";
  //for (int i=0; i < 6; i++) {
  //  tmValues += String(channels[i]) + ";";
  //}
  tmValues += "\n";

  Serial.print(tmValues);
}








