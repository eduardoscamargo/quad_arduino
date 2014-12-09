#include <PID_v1.h>

int ch1 = 2;
int ch2 = 3;
int ch3 = 4;
int ch4 = 5;
int ch5 = 6;
int ch6 = 7;

int motor1 = 8;

unsigned long ch1v;
unsigned long ch2v;
unsigned long ch3v;
unsigned long ch4v;
unsigned long ch5v;
unsigned long ch6v;

double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint,1,2,3, DIRECT);

void setup()
{
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
  
  Input = analogRead(0);
  Setpoint = 1500;
  
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(57600);      // open the serial port at 9600 bps:
}

void loop()
{
 
  ch1v = pulseIn(ch1, HIGH);
  ch2v = pulseIn(ch2, HIGH);
  ch3v = pulseIn(ch3, HIGH);
  ch4v = pulseIn(ch4, HIGH);
  ch5v = pulseIn(ch5, HIGH);
  ch6v = pulseIn(ch6, HIGH);

  Input = ch1v;
  myPID.Compute();
  ch1v += (int)Output;
  
  String RCValues = String(ch1v) + ";" + String(ch2v) + ";" + String(ch3v) + ";" + String(ch4v) + ";" + String(ch5v) + ";" + String(ch6v) + ";" + String(Output) + "\n";
  Serial.print(RCValues);
}
