#include <PID_v1.h>
#include <Servo.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "controller.h"

void setupMotors();
void setupPID();
void setupMPU();
void readOrientation();
void writeMotor();
void telemetry();
void getZeroValuesFromRC();
void getPIDParametersAndSetTunnings();
void setPIDParameters(char param);
void writePIDParameters();
void PIDCalibration();
String padding(String text, int size);

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
uint8_t mpuIntStatus;   // Armazena o valor atual do byte de status de interrupção do MPU
uint8_t dmpStatus;      // Status a cada operação do DMP (0 = sucesso, !0 = erro)
uint16_t packetSize;    // Tamanho esperado do pacote do DMP (o padrão é 42 bytes)
uint16_t fifoCount;     // Conta quantos bytes há no FIFO
uint8_t fifoBuffer[64]; // FIFO do DMP

/* Variáveis de orientação que vem do DMP */
Quaternion q;         // [w, x, y, z]         Container do quaternion
VectorFloat gravity;  // [x, y, z]            Vetor de gravidade
float ypr[3];         // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade
double ypr_degree[3]; // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade em graus/segundo
double yawRate;       // Velocidade de rotação do eixo yaw

const int DMP_YAW = 0;
const int DMP_PITCH = 1;
const int DMP_ROLL = 2;

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

/* Definição da estrutura de um PID */
struct kpkikd {
  float kp;
  float ki;
  float kd;
};

/* Estrutura com os parâmetros dos PIDs */
struct pidParams {
  kpkikd PIDPitchRate;
  kpkikd PIDRollRate;
  kpkikd PIDYawRate;
  kpkikd PIDPitchStab;
  kpkikd PIDRollStab;
  kpkikd PIDYawStab;
} pidParams;

/* Variáveis de saída do PID para a taxa de rotação */
double pitch_rate_output;
double roll_rate_output;
double yaw_rate_output;

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

/* Configuração dos endereços da EEPROM para armazenar os PIDs */
const int EEPROM_PITCH_P = 0;
const int EEPROM_PITCH_I = 1;
const int EEPROM_PITCH_D = 2;
const int EEPROM_ROLL_P = 3;
const int EEPROM_ROLL_I = 4;
const int EEPROM_ROLL_D = 5;
const int EEPROM_YAW_P = 6;
const int EEPROM_YAW_I = 7;
const int EEPROM_YAW_D = 8;

/* Variável que indica se houve calibração. Se houve e finalizou, grava a EEPROM. */
bool pidChanged = false;

/* Fim do PID */

/*************************
 * Saída para os motores *
 *************************/

/* Canais de cada motor */
const int MOTOR_PIN_FL = 9;
const int MOTOR_PIN_FR = 10;
const int MOTOR_PIN_BL = 11;
const int MOTOR_PIN_BR = 12;

/* Entidades que representam os motores.
 * Responsáveis por acionar os ESCs realizando um PPM entre 1ms e 2ms. */
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

/* Motores (parte lógica) */
int motors[4];
const int MOTOR_FL = 0;
const int MOTOR_FR = 1;
const int MOTOR_BL = 2;
const int MOTOR_BR = 3;

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
  unsigned long start = millis();

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

  unsigned long now = millis();
  if (now - start < 60) {
    delay(60 - (now - start));
  }
}


/* Obtem os valores dos PIDs da EEPROM */
void getPIDParametersAndSetTunnings() {
  D(char tempString[10]);

  pidParams.PIDPitchStab.kp = (float)EEPROM.read(EEPROM_PITCH_P) / 10;
  pidParams.PIDPitchStab.ki = (float)EEPROM.read(EEPROM_PITCH_I) / 10;
  pidParams.PIDPitchStab.kd = (float)EEPROM.read(EEPROM_PITCH_D) / 10;
  PIDPitchStab.SetTunings(pidParams.PIDPitchStab.kp, pidParams.PIDPitchStab.ki, pidParams.PIDPitchStab.kd);

  pidParams.PIDRollStab.kp = (float)EEPROM.read(EEPROM_ROLL_P) / 10;
  pidParams.PIDRollStab.ki = (float)EEPROM.read(EEPROM_ROLL_I) / 10;
  pidParams.PIDRollStab.kd = (float)EEPROM.read(EEPROM_ROLL_D) / 10;
  PIDRollStab.SetTunings(pidParams.PIDRollStab.kp, pidParams.PIDRollStab.ki, pidParams.PIDRollStab.kd);


  pidParams.PIDYawStab.kp = (float)EEPROM.read(EEPROM_YAW_P) / 10;
  pidParams.PIDYawStab.ki = (float)EEPROM.read(EEPROM_YAW_I) / 10;
  pidParams.PIDYawStab.kd = (float)EEPROM.read(EEPROM_YAW_D) / 10;
  PIDYawStab.SetTunings(pidParams.PIDYawStab.kp, pidParams.PIDYawStab.ki, pidParams.PIDYawStab.kd);

  D(Serial.print("Valores do PID: ("));
  D(dtostrf(pidParams.PIDPitchStab.kp, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDPitchStab.ki, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDPitchStab.kd, 2, 1, tempString));
  D(Serial.print(String(tempString) + "), ("));
  D(dtostrf(pidParams.PIDRollStab.kp, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDRollStab.ki, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDRollStab.kd, 2, 1, tempString));
  D(Serial.print(String(tempString) + "), ("));
  D(dtostrf(pidParams.PIDYawStab.kp, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDYawStab.ki, 2, 1, tempString));
  D(Serial.print(String(tempString) + ", "));
  D(dtostrf(pidParams.PIDYawStab.kd, 2, 1, tempString));
  D(Serial.println(String(tempString) + ")"));
}

/* Efetua o setup dos motores */
void setupMotors() {
  /* Bind dos canais de saída dos motores (ESCs) */
  motorFL.attach(MOTOR_PIN_FL);
  motorFR.attach(MOTOR_PIN_FR);
  motorBL.attach(MOTOR_PIN_BL);
  motorBR.attach(MOTOR_PIN_BR);

  /* Desliga os motores */
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
}

/* Efetua o setup do PID */
void setupPID() {
  /* Inicializa o PID */
  /* TODO: REMOVER COMENTÁRIO */
  /* getPIDParametersAndSetTunnings(); */
  /* END TODO */

  PIDPitchStab.SetMode(AUTOMATIC);
  PIDRollStab.SetMode(AUTOMATIC);
  PIDYawStab.SetMode(AUTOMATIC);

  /* Limita o quanto será enviado ao motor: range máximo de 1000ms. */
  PIDPitchStab.SetOutputLimits(-500, 500);
  PIDRollStab.SetOutputLimits(-500, 500);
  PIDYawStab.SetOutputLimits(-500, 500);
}

/* Efetua o setup do MPU6050 */
void setupMPU() {
  /* Inicializa o barramento I2C (a biblioteca I2Cdev não faz isso automaticamente) */
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  /* Inicializa a MPU e seu respectivo DMP */
  mpu.initialize();
  dmpStatus = mpu.dmpInitialize();

  /* Define os offsets do MPU. Utilizado o programa MPU6050_calibration para chegar a esses números. */
  mpu.setXAccelOffset(616);
  mpu.setYAccelOffset(2244);
  mpu.setZAccelOffset(300);
  mpu.setXGyroOffset(83);
  mpu.setYGyroOffset(-25);
  mpu.setZGyroOffset(44);

  /* Só prossegue somente se o houve a inicialização correta do DMP */
  if (dmpStatus == 0) {
    /* Liga o DMP. Agora o MPU está pronto! */
    mpu.setDMPEnabled(true);

    /* Habilita a detecção de interrupção do Arduino */
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    /* Inicializa a flag de controle do DMP para o loop() saber que está tudo certo */
    dmpReady = true;

    /* Obtem o tamanho do pacote do DMP */
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERRO!
    // 1 = Carregamento inicial da memória falhou
    // 2 = Atualização da configuração do DMP falhou
    D(Serial.print(F("Inicialização do DMP falhou (código ")));
    D(Serial.print(dmpStatus));
    D(Serial.println(F(")")));
  }
}

/* Define os parâmetros dos PIDs com base no dimmer do controle. O range possível é de 0 a 25.5.
 * Atualmente, os valores do PID são compartilhados entre o roll, pitch e yaw.
 * O parâmetro sendo calibrado é definido de forma hard-coded. */
void setPIDParameters(char param) {
  if (param == 'P') {
    pidParams.PIDPitchStab.kp = pidParams.PIDRollStab.kp = pidParams.PIDYawStab.kp = rcDimmer/10;
  } else if (param == 'I') {
    pidParams.PIDPitchStab.ki = pidParams.PIDRollStab.ki = pidParams.PIDYawStab.ki = rcDimmer/10;
  } else {
    pidParams.PIDPitchStab.kd = pidParams.PIDRollStab.kd = pidParams.PIDYawStab.kd = rcDimmer/10;
  }

  PIDPitchStab.SetTunings(pidParams.PIDPitchStab.kp, pidParams.PIDPitchStab.ki, pidParams.PIDPitchStab.kd);
  PIDRollStab.SetTunings(pidParams.PIDRollStab.kp, pidParams.PIDRollStab.ki, pidParams.PIDRollStab.kd);
  PIDYawStab.SetTunings(pidParams.PIDYawStab.kp, pidParams.PIDYawStab.ki, pidParams.PIDYawStab.kd);

  pidChanged = true;
}

/* Escreve os parâmetros do PID na EEPROM */
void writePIDParameters() {
  EEPROM.write(EEPROM_PITCH_P, (int)(pidParams.PIDPitchStab.kp * 10));
  EEPROM.write(EEPROM_PITCH_I, (int)(pidParams.PIDPitchStab.ki * 10));
  EEPROM.write(EEPROM_PITCH_D, (int)(pidParams.PIDPitchStab.kd * 10));

  EEPROM.write(EEPROM_ROLL_P, (int)(pidParams.PIDRollStab.kp * 10));
  EEPROM.write(EEPROM_ROLL_I, (int)(pidParams.PIDRollStab.ki * 10));
  EEPROM.write(EEPROM_ROLL_D, (int)(pidParams.PIDRollStab.kd * 10));

  EEPROM.write(EEPROM_YAW_P, (int)(pidParams.PIDYawStab.kp * 10));
  EEPROM.write(EEPROM_YAW_I, (int)(pidParams.PIDYawStab.ki * 10));
  EEPROM.write(EEPROM_YAW_D, (int)(pidParams.PIDYawStab.kd * 10));
}

/* Efetua a calibração se a calibração estiver ativa (controle ON/OFF ligado) e grava a EEPROM ao final da calibração. Escolha do parâmetro de forma hard-coded. */
void PIDCalibration() {
  if (rcOnOff > 10) {
    setPIDParameters('I');
  } else if (pidChanged) {
    writePIDParameters();
    pidChanged = false;
  }
}

/* Define os valores referentes ao zero do controle para Roll, Yaw e Pitch com base em 10 leituras do controle. */
void getZeroValuesFromRC() {
  unsigned long currentRCreadCount = 0;
  int passes = 0;

  currentRCreadCount = RCreadCount;

  while (passes < 100) {
    /* Aguarda acontecer a leitura do controle. */
    while ((currentRCreadCount == RCreadCount) || !chvalue[CH_ROLL] || !chvalue[CH_PITCH] || !chvalue[CH_YAW]);

    currentRCreadCount = RCreadCount;

    rcRollZero += chvalue[CH_ROLL];
    rcPitchZero += chvalue[CH_PITCH];
    rcYawZero += chvalue[CH_YAW];

    passes++;

    /* TODO: REMOVER */
    // rcRollZero = rcYawZero = 1500;
  }

  rcRollZero /= passes;
  rcPitchZero /= passes;
  rcYawZero /= passes;
}

/* Efetua a escrita da potência dos motores entre 1ms e 2ms */
void writeMotor() {
  motorFL.writeMicroseconds(constrain(motors[MOTOR_FL], 1000, 2000));
  motorFR.writeMicroseconds(constrain(motors[MOTOR_FR], 1000, 2000));
  motorBL.writeMicroseconds(constrain(motors[MOTOR_BL], 1000, 2000));
  motorBR.writeMicroseconds(constrain(motors[MOTOR_BR], 1000, 2000));
}

/* Efetua a leitura da orientação a ser armazenada na variável "ypr" */
void readOrientation() {
  bool fifoOverflow = false; // Marca se houve overflow da FIFO durante a execução
  // D(Serial.println("init RO"));

  /* Le somente se houve interrupção */
  if (mpuInterrupt) {
    /* Reseta a flag de interrupção */
    mpuInterrupt = false;

    /* Obtém o byte INT_STATUS do MPU */
    mpuIntStatus = mpu.getIntStatus();

    /* Obtém a contagem da FIFO */
    fifoCount = mpu.getFIFOCount();

    /* Verifica se houve overflow da FIFO. Quanto mais ineficiente o código, mais irá ocorrer. */
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // Reseta a FIFO
      mpu.resetFIFO();
      fifoOverflow = true;
      return;
    }

    /* Se houve overflow, aguarda o próximo valor estar pronto */
    // int DMPwatchdog = 0;
    // if (fifoOverflow) {
    //   D(Serial.println("Agu Int"));
    //   while(!mpuInterrupt);
    //   D(Serial.println("Int rec"));
    // }

    if (mpuIntStatus & 0x01) {
      /* Obtém a contagem da FIFO */
      fifoCount = mpu.getFIFOCount();

      /* Aguarda encher a FIFO caso não esteja com os todos dados completos (pacote completo) */
      int DMPwatchdog = 0;
      while (fifoCount < packetSize && DMPwatchdog < 5) {
        fifoCount = mpu.getFIFOCount();
        DMPwatchdog++;
      }

      if (DMPwatchdog < 5) {
        /* Dado está pronto para ser lido do MPU */
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        /* Obtém os valores (Yaw, Pitch e Roll) do DMP */
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yawRate = mpu.getRotationZ()/16.4; // Precision FS_SEL = 3 -> 16.4/LSB/deg/s

        /* Converte para graus/segundo */
        for (int i = 0; i < 3; i++) {
          ypr_degree[i] = ypr[i] * 180/M_PI;
        }
      }
    }
  }
}

/*
 * Efetua a telemetria dos dados do quadricoptero na seguinte ordem, separados por ';':
 * - RCT - rcThrottle (em us - entre ~1000 a ~2000)
 * - RCY - rcYaw    (entre -150 e 150)
 * - RCP - rcPitch  (entre -45 e 45)
 * - RCR - rcRoll   (entre -45 e 45)
 * - RCO - rcOnOff  (entre 0 e 100)
 * - RCD - rcDimmer (entre 0 e 1000)
 * - ZRP - Posição 0 do Pitch no controle
 * - ZRR - Posição 0 do Roll no controle
 * - ZRY - Posição 0 do Pitch no controle
 * - DMP - Posição: pitch (em graus)
 * - DMR - Posição: roll (em graus)
 * - DMY - Posição: yaw (em graus)
 * - YRT - Taxa: yaw (em graus/seg)
 * - PIP - Saída pitch do PID stab: pitch_stab_output (entre -500 a 500)
 * - PIR - Saída roll do PID stab: roll_stab_output (entre -500 a 500)
 * - PIY - Saída yaw do PID stab: yaw_stab_output (entre -500 a 500)
 * - MFL - Motor FL (em us - entre 1000 a 2000)
 * - MFR - Motor FR (em us - entre 1000 a 2000)
 * - MBL - Motor BL (em us - entre 1000 a 2000)
 * - MBR - Motor BR (em us - entre 1000 a 2000)
 * - Canal 1 (em us - entre ~1000 a ~2000)
 * - Canal 2 (em us - entre ~1000 a ~2000)
 * - Canal 3 (em us - entre ~1000 a ~2000)
 * - Canal 4 (em us - entre ~1000 a ~2000)
 * - Canal 5 (em us - entre ~1000 a ~2000)
 * - Canal 6 (em us - entre ~1000 a ~2000)
 * - Intervalo do loop (em us)
 */
D(void telemetry() {
  /* Calcula a frequência do loop */
  tmInterval = micros() - tmCheckpoint;
  tmCheckpoint = micros();

  String tmValues;
  tmValues += padding("RCT=" + String(int(rcThrottle)), 9) + ";";
  tmValues += padding("RCY=" + String(int(rcYaw)), 8) + ";";
  tmValues += padding("RCP=" + String(int(rcPitch)), 8) + ";";
  tmValues += padding("RCR=" + String(int(rcRoll)), 8) + ";";
  tmValues += padding("RCO=" + String(int(rcOnOff)), 8) + ";";
  tmValues += padding("RCD=" + String(int(rcDimmer)), 8) + ";";
  // tmValues += padding("ZRP=" + String(int(rcPitchZero)), 9) + ";";
  // tmValues += padding("ZRR=" + String(int(rcRollZero)), 9) + ";";
  // tmValues += padding("ZRY=" + String(int(rcYawZero)), 9) + ";";
  tmValues += padding("DMP=" + String(int(ypr_degree[DMP_PITCH])), 9) + ";";
  tmValues += padding("DMR=" + String(int(ypr_degree[DMP_ROLL])), 9) + ";";
  // tmValues += padding("DMY=" + String(int(ypr_degree[DMP_YAW])), 9) + ";";
  tmValues += padding("YRT=" + String(int(yawRate)), 9) + ";";
  tmValues += padding("PIP=" + String(int(pitch_stab_output)), 8) + ";";
  tmValues += padding("PIR=" + String(int(roll_stab_output)), 8) + ";";
  tmValues += padding("PIY=" + String(int(yaw_stab_output)), 8) + ";";
  tmValues += padding("MFL=" + String(int(motors[MOTOR_FL])), 9) + ";";
  tmValues += padding("MFR=" + String(int(motors[MOTOR_FR])), 9) + ";";
  tmValues += padding("MBL=" + String(int(motors[MOTOR_BL])), 9) + ";";
  tmValues += padding("MBR=" + String(int(motors[MOTOR_BR])), 9) + ";";
  // for (int i=0; i < 6; i++) {
  //  tmValues += padding(String(channels[i]), 8) + ";";
  // }
  tmValues += padding("INT=" + String(tmInterval), 11) + ";";

  countToSend++;
  if (countToSend > 4) {
    Serial.println(tmValues);
    countToSend = 0;
  }
})

/* Cria padding em uma string preenchendo-a com espaços à direita */
String padding(String text, int size){
  String newString = String(text);
  if (text.length() < size) {
    for (int i = 0; i < size - text.length(); i++) {
      newString += " ";
    }
  }

  return newString;
}
