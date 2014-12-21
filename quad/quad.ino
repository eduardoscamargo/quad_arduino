#include <Servo.h>
#include <PID_v1.h>

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

/* Definição dos canais de entrada. */
const int CH1 = 2; /* Roll (horizontal da direita) */
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
void setup()
{  
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

  Serial.begin(57600);
}

/* Loop infinito do Arduino. */
void loop()
{       
  readRC();

  //  input = ch1v;
  // myPID.Compute();
  // ch1v += (int)output;
  
  writeMotor();
  
  D(telemetry());
}

/* Efetua a leitura dos canais dos controles. */
void readRC()
{ 
  channels[0] = pulseIn(CH1, HIGH);
  channels[1] = pulseIn(CH2, HIGH);
  channels[2] = pulseIn(CH3, HIGH);
  channels[3] = pulseIn(CH4, HIGH);
  channels[4] = pulseIn(CH5, HIGH);
  channels[5] = pulseIn(CH6, HIGH); 

  normalizeRC();
}

/* Normaliza os valores do controle para ranges pré definidos. */
void normalizeRC()
{
  rcThrottle = channels[2];
  rcYaw      = map(channels[3], MIN_CH4, MAX_CH4, -150, 150);
  rcPitch    = map(channels[1], MIN_CH2, MAX_CH2, 45, -45);
  rcRoll     = map(channels[0], MIN_CH1, MAX_CH1, -45, 45);
  rcOnOff    = map(channels[4], MIN_CH3, MAX_CH3, 0, 100);
  rcDimmer   = map(channels[5], MIN_CH6, MAX_CH6, 0, 1000);
}

void writeMotor()
{
  val = map(rcThrottle, MIN_CH1, MAX_CH1, 1000, 2000);
  motorFL.writeMicroseconds(val);
  motorFR.writeMicroseconds(val);
  motorBL.writeMicroseconds(val);
  motorBR.writeMicroseconds(val);
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
void telemetry() 
{
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




