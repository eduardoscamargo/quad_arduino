#include <PID_v1.h>

void telemetry();
void readRC();

#define DEBUG
#ifdef DEBUG
#define D(X) X
#else
#define D(X)
#endif

#ifdef DEBUG
long tmBegin;
long tmEnd;
#endif

/* Definição dos canais de entrada */
int CH1 = 2; /* Roll (horizontal da direita) */
int CH2 = 3; /* Pitch (vertical da direita) */
int CH3 = 4; /* Throttle (vertical da esquerda) */
int CH4 = 5; /* Yaw (horizontal da esquerda */
int CH5 = 6; /* ON/OFF */
int CH6 = 7; /* Dimmer */

/* Voltagem de leitura dos canais do controle */
unsigned long ch1v;
unsigned long ch2v;
unsigned long ch3v;
unsigned long ch4v;
unsigned long ch5v;
unsigned long ch6v;

double setpoint, input, output;

PID myPID(&input, &output, &setpoint,1,2,3, DIRECT);

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

  Serial.begin(57600);
}

void loop()
{ 
  D(tmBegin = micros());
   
  readRC();
  
  input = ch1v;
  myPID.Compute();
  ch1v += (int)output;
  
  D(telemetry());
}

void readRC()
{  
  ch1v = pulseIn(CH1, HIGH);
  ch2v = pulseIn(CH2, HIGH);
  ch3v = pulseIn(CH3, HIGH);
  ch4v = pulseIn(CH4, HIGH);
  ch5v = pulseIn(CH5, HIGH);
  ch6v = pulseIn(CH6, HIGH); 
}

/* 
 * Efetua a telemetria dos dados do quadricoptero na seguinte ordem, separados por ';':
 *   - Canal 1
 *   - Canal 2
 *   - Canal 3
 *   - Canal 4
 *   - Canal 5
 *   - Canal 6
 */  
void telemetry() 
{
  tmEnd = micros();
  long tmInterval = tmEnd - tmBegin;
  
  String RCValues = String(ch1v) + ";";
  RCValues += String(ch2v) + ";";
  RCValues += String(ch3v) + ";";
  RCValues += String(ch4v) + ";";
  RCValues += String(ch5v) + ";";
  RCValues += String(ch6v) + ";";
  RCValues += String(output) + ";";
  RCValues += String(tmInterval) + ";";
  RCValues += "\n";
  
  Serial.print(RCValues);
}



