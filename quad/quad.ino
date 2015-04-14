#include <Servo.h>
#include <EEPROM.h>

/* MPU6050 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* PID */
#include <PID_v1.h>

#include "controller.h"
#include "mpu6050.h"
#include "pid.h"
#include "motors.h"

/*********
 * DEBUG *
 *********/
#define DEBUG // Habilita a telemetria
#ifdef DEBUG
  #define D(X) X
#else
  #define D(X)
#endif

#ifdef DEBUG
  unsigned long tmCheckpoint; // Marca o tempo no checkpoint
  unsigned long tmInterval;   // Intervalo entre última medição e o checkpoint
  int countToSend = 0; // Controla o intervalo de envio das informações
#endif

/* Indicação de atividade do processador */
const int LED_PIN = 13;
bool blinkState = false;

/******************************
 * Entrada do controle remoto *
 *****************************/
/* Armazena a duração dos canais do controle (em us). */
volatile unsigned int chvalue[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0 };

/* Indica quantas leituras houve no canal CH3 */
volatile unsigned long RCreadCount = 0;

/* Valores médios para roll, pitch e yaw */
unsigned long rcRollZero, rcPitchZero, rcYawZero = 0;

/* Valores normalizados dos canais */
double rcThrottle, rcYaw, rcPitch, rcRoll, rcOnOff, rcDimmer;

/**************************************
 * Giroscópio e Acelerômetro: MPU6050 *
 **************************************/
/* Variáveis de controle do MPU6050 (só MPU para facilitar) para a interface
 * com o DMP - Processador que entrega valor prontos do giroscópio. */
MPU6050 mpu;            // Entidade que representa o MPU
bool dmpReady = false;  // Definida para true se o DMP foi inicializado com sucesso
double ypr_degree[3]; // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade em graus/segundo
double yawRate;       // Velocidade de rotação do eixo yaw

/****************
 * Interrupções *
 ****************/
/* Rotina de interrupção da MPU */
volatile bool mpuInterrupt = false; // Indica que houve interrupção da MPU, ou seja, há dados a serem lidos
void dmpDataReady() {
  mpuInterrupt = true;
}

/*******
 * PID *
 *******/
pidParams pidParams;

/* Variáveis de saída do PID para a taxa de rotação */
// double pitch_rate_output;
// double roll_rate_output;
// double yaw_rate_output;

/* Variáveis de saída do PID para a estabilização */
double pitch_stab_output;
double roll_stab_output;
double yaw_stab_output;

/* PIDs para a taxa de rotação. Entrada: taxa de rotação do giroscópio. Objetivo: Taxa de giro proporcional ao erro. Saída: Potência para o motor. */
/*                         ENTRADA                         SAÍDA                  OBJETIVO        P   D  I  DIREÇÃO */
//PID PIDPitchRate (/* taxa de giro */, &pitch_rate_output, &pitch_stab_output, 1, 2, 3, DIRECT);
//PID PIDRollRate  (/* taxa de giro */, &roll_rate_output, &roll_stab_output, 1, 2, 3, DIRECT);
//PID PIDYawRate   (/* taxa de giro */, &yaw_rate_output, &yaw_stab_output, 1, 2, 3, DIRECT);

/* PIDs para a taxa de estabilização. Entrada: posição do quadricóptero através do MPU. Objetivo: Posição indicada através do controle. Saída: Taxa de giro proporcional ao erro. */
/*                        ENTRADA                 SAÍDA          OBJETIVO  P  I  D  DIREÇÃO */
PID PIDPitchStab (&ypr_degree[DMP_PITCH], &pitch_stab_output,    &rcPitch, 4.7, 6, 1.1, REVERSE);
PID PIDRollStab  (&ypr_degree[DMP_ROLL],  &roll_stab_output,     &rcRoll,  10, 0, 0, DIRECT);
PID PIDYawStab   (       &yawRate,        &yaw_stab_output,      &rcYaw,   5, 0, 0, DIRECT); // No yaw, utilizará a taxa de rotação, não a posição como referência.

/* Variável que indica se houve calibração. Se houve e finalizou, grava a EEPROM. */
bool pidChanged = false;
/* Fim do PID */

/*************************
 * Saída para os motores *
 *************************/
/* Motores (parte lógica) */
int motors[4];

int teeeeemp = 0;
/* Inicialização do Arduino. */
void setup() {
  /* Inicializa a porta serial */
  D(Serial.begin(115200));
  D(while (Serial.available() && Serial.read())); // Limpa o buffer

  D(Serial.println("Iniciando controle remoto"));
  setupRadio();
  getZeroValuesFromRC();

  D(Serial.println("Iniciando motores"));
  setupMotors();

  D(Serial.println("Iniciando PID"));
  setupPID();

  D(Serial.println("Iniciando MPU"));
  setupMPU();

  /* Configura a led para output de atividade */
  pinMode(LED_PIN, OUTPUT);

  D(Serial.println("Fim do setup"));
}

/* Loop infinito do Arduino. */
void loop() {
  static float yaw_target = 0;

  /* Se a programação do DMP falhou, não faz nada */
  if (!dmpReady) {
    return;
  }

  detectLostSignal();

  readOrientation();

  normalizeRC();

  PIDCalibration();

  /* Voe Forest, Voe! */
  if(rcThrottle > MIN_RC_THROTTLE + 100) {  // Throttle raised, turn on stablisation.
    PIDPitchStab.Compute();
    PIDRollStab.Compute();
    PIDYawStab.Compute();

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcYaw ) > 5) {
      yaw_stab_output = rcYaw;
      yaw_target = ypr_degree[DMP_YAW];   // remember this yaw for when pilot stops
    }

    /* Debug */
    // pitch_stab_output = 0;
    yaw_stab_output = 0;
    roll_stab_output = 0;

    motors[MOTOR_FL] = rcThrottle + roll_stab_output + pitch_stab_output - yaw_stab_output;
    motors[MOTOR_BL] = rcThrottle + roll_stab_output - pitch_stab_output + yaw_stab_output;
    motors[MOTOR_FR] = rcThrottle - roll_stab_output + pitch_stab_output + yaw_stab_output;
    motors[MOTOR_BR] = rcThrottle - roll_stab_output - pitch_stab_output - yaw_stab_output;

  } else {
    /* Motores desligados */
    motors[MOTOR_FL] = motors[MOTOR_FR] = motors[MOTOR_BL] = motors[MOTOR_BR] = 1000;

    // reset yaw target so we maintain this on takeoff
    yaw_target = ypr_degree[DMP_YAW];
  }

  writeMotor();

  D(telemetry());

  /* Pisca a led para indicar atividade */
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}