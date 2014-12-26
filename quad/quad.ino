#include <PID_v1.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

void readRC();
void normalizeRC();
void readOrientation();
void writeMotor();
void telemetry();
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
  long tmCheckpoint;   // Marca o tempo no checkpoint
  long tmInterval;     // Intervalo entre última medição e o checkpoint
  int countToSend = 0; // Controla o intervalo de envio das informações
#endif

/* Indicação de atividade do processador */
const int LED_PIN = 13;
bool blinkState = false;

/******************************
 * Entrada do controle remoto *
 ******************************/

/* Definição dos canais de entrada. */
const int CH1 = 8; /* Roll (horizontal da direita) */
const int CH2 = 3; /* Pitch (vertical da direita) */
const int CH3 = 4; /* Throttle (vertical da esquerda) */
const int CH4 = 5; /* Yaw (horizontal da esquerda */
const int CH5 = 6; /* ON/OFF */
const int CH6 = 7; /* Dimmer */

/* Valores máximos e mínimos (em us) de cada canal. */
const int MAX_RC_ROLL = 1939;
const int MIN_RC_ROLL = 1040;
const int MAX_RC_PITCH = 2009;
const int MIN_RC_PITCH = 1012;
const int MAX_RC_THROTTLE = 1950;
const int MIN_RC_THROTTLE = 1070;
const int MIN_RC_YAW = 1030;
const int MAX_RC_YAW = 2009;
const int MAX_RC_ON_OFF = 2009;
const int MIN_RC_ON_OFF = 1014;
const int MAX_RC_DIMMER = 1018;
const int MIN_RC_DIMMER = 2006;

/* Armazena a duração dos canais do controle (em us). */
unsigned long channels[6];

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
Quaternion q;        // [w, x, y, z]         Container do quaternion
VectorFloat gravity; // [x, y, z]            Vetor de gravidade
float ypr[3]; // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade
double ypr_degree[3]; // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade em graus/segundo

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
struct {
  kpkikd pid_pitch_rate;
  kpkikd pid_roll_rate;
  kpkikd pid_yaw_rate;
  kpkikd pid_pitch_stab;
  kpkikd pid_roll_stab;
  kpkikd pid_yaw_stab;
} pid_params;

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
//PID pid_pitch_rate (/* taxa de giro */, &pitch_rate_output, &pitch_stab_output, 1, 2, 3, DIRECT);
//PID pid_roll_rate  (/* taxa de giro */, &roll_rate_output, &roll_stab_output, 1, 2, 3, DIRECT);
//PID pid_yaw_rate   (/* taxa de giro */, &yaw_rate_output, &yaw_stab_output, 1, 2, 3, DIRECT);

/* PIDs para a taxa de estabilização. Entrada: posição do quadricóptero através do MPU. Objetivo: Posição indicada através do controle. Saída: Taxa de giro proporcional ao erro. */
/*                        ENTRADA                 SAÍDA          OBJETIVO  P  I  D  DIREÇÃO */
PID pid_pitch_stab (&ypr_degree[DMP_PITCH], &pitch_stab_output, &rcPitch, 10, 4, 0, DIRECT);
PID pid_roll_stab  (&ypr_degree[DMP_ROLL],  &roll_stab_output,  &rcRoll,  10, 4, 0, DIRECT);
PID pid_yaw_stab   (&ypr_degree[DMP_YAW],   &yaw_stab_output,   &rcYaw,   10, 4, 0, DIRECT);
/* Fim do PID */

/*************************
 * Saída para os motores *
 *************************/

/* Canais de cada motor */
const int MOTOR_PIN_FL = 9; /* To-do test this pin */
const int MOTOR_PIN_FR = 10;
const int MOTOR_PIN_BL = 11; /* To-do test this pin */
const int MOTOR_PIN_BR = 12; /* To-do test this pin */

/* Entidades que representam os motores. Responsáveis por acionar os ESCs realizando um PPM entre 1ms e 2ms. */
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

  /* Bind dos canais de entrada do controle */
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH6, INPUT);

  /* Bind dos canais de saída dos motores (ESCs) */
  motorFL.attach(MOTOR_PIN_FL);
  motorFR.attach(MOTOR_PIN_FR);
  motorBL.attach(MOTOR_PIN_BL);
  motorBR.attach(MOTOR_PIN_BR);

  /* Inicializa o PID */
  pid_pitch_stab.SetMode(AUTOMATIC);
  pid_roll_stab.SetMode(AUTOMATIC);
  pid_yaw_stab.SetMode(AUTOMATIC);

  /* Limita o quanto será enviado ao motor: range máximo de 1000ms. */
  pid_pitch_stab.SetOutputLimits(-500, 500);
  pid_roll_stab.SetOutputLimits(-500, 500);
  pid_yaw_stab.SetOutputLimits(-500, 500);

  /* Inicializa o barramento I2C (a biblioteca I2Cdev não faz isso automaticamente) */
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  /* Inicializa a MPU e seu respectivo DMP */
  mpu.initialize();
  D(Serial.println(F("Inicializando DMP...")));
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
    D(Serial.println(F("Habilitando interrupcao 0 (pino 2)...")));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    /* Inicializa a flag de controle do DMP para o loop() saber que está tudo certo */
    D(Serial.println(F("DMP pronto! Iniciando programa...")));
    dmpReady = true;

    /* Obtem o tamanho do pacote do DMP */
    packetSize = mpu.dmpGetFIFOPacketSize();
    D(Serial.println(F("Tamanho do pacote do DMP obtido...")));
  }
  else {
    // ERRO!
    // 1 = Carregamento inicial da memória falhou
    // 2 = Atualização da configuração do DMP falhou
    D(Serial.print(F("Inicialização do DMP falhou (código ")));
    D(Serial.print(dmpStatus));
    D(Serial.println(F(")")));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

/* Loop infinito do Arduino. */
void loop() {
  static float yaw_target = 0;

  /* Se a programação do DMP falhou, não faz nada */
  Serial.println("!dmpReady");
  if (!dmpReady) {
    return;
  }

  Serial.println("readOrientation");
  readOrientation();

  Serial.println("readRC");
  readRC();

  /* Voe Forest, Voe! */
  if(rcThrottle > MIN_RC_THROTTLE + 100) {  // Throttle raised, turn on stablisation.
    Serial.println("PID");
    pid_pitch_stab.Compute();
    pid_roll_stab.Compute();
    pid_yaw_stab.Compute();

    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcYaw ) > 5) {
      yaw_stab_output = rcYaw;
      yaw_target = ypr_degree[DMP_YAW];   // remember this yaw for when pilot stops
    }

    // pid_pitch_rate.calculate()
    // pid_roll_rate.calculate()
    // pid_yaw_rate.calculate()

    //  // Stablise PIDS                                                     'setpoint' 'input'
    // float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250);
    // float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    // float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

    // // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    // if(abs(rcyaw ) > 5) {
    //   yaw_stab_output = rcyaw;
    //   yaw_target = yaw;   // remember this yaw for when pilot stops
    // }

    // // rate PIDS                                                       'setpoint'         'input'
    // long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);
    // long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
    // long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);

    // // mix pid outputs and send to the motors.
    // hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    // hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    // hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    // hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);

    motors[MOTOR_FL] = rcThrottle + roll_stab_output + pitch_stab_output - yaw_stab_output;
    motors[MOTOR_BL] = rcThrottle + roll_stab_output - pitch_stab_output + yaw_stab_output;
    motors[MOTOR_FR] = rcThrottle - roll_stab_output + pitch_stab_output + yaw_stab_output;
    motors[MOTOR_BR] = rcThrottle - roll_stab_output - pitch_stab_output - yaw_stab_output;

  } else {
    Serial.println("else");
    /* Motores desligados */
    motors[MOTOR_FL] = motors[MOTOR_FR] = motors[MOTOR_BL] = motors[MOTOR_BR] = 1000;

    // reset yaw target so we maintain this on takeoff
    yaw_target = ypr_degree[DMP_YAW];

    // reset PID integrals whilst on the ground
    // for(int i=0; i<6; i++)
    //   pids[i].reset_I();

    // }
  }

  Serial.println("writeMotor");
  writeMotor();

  Serial.println("telemetry");
  D(telemetry());

  Serial.println("blink");
  /* Pisca a led para indicar atividade */
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

/* Efetua a leitura dos canais dos controles. */
void readRC() {
  channels[0] = pulseIn(CH1, HIGH); /* Roll (horizontal da direita) */
  channels[1] = pulseIn(CH2, HIGH); /* Pitch (vertical da direita) */
  channels[2] = pulseIn(CH3, HIGH); /* Throttle (vertical da esquerda) */
  channels[3] = pulseIn(CH4, HIGH); /* Yaw (horizontal da esquerda */
  channels[4] = pulseIn(CH5, HIGH); /* ON/OFF */
  channels[5] = pulseIn(CH6, HIGH); /* Dimmer */

  normalizeRC();
}

/* Normaliza os valores do controle para ranges pré definidos. */
void normalizeRC() {
  rcRoll     = map(channels[0], MIN_RC_ROLL, MAX_RC_ROLL, -45, 45);
  rcPitch    = map(channels[1], MIN_RC_PITCH, MAX_RC_PITCH, 45, -45);
  rcThrottle = channels[2];
  rcYaw      = map(channels[3], MIN_RC_YAW, MAX_RC_YAW, -150, 150);
  rcOnOff    = map(channels[4], MIN_RC_ON_OFF, MAX_RC_ON_OFF, 0, 100);
  rcDimmer   = map(channels[5], MIN_RC_DIMMER, MAX_RC_DIMMER, 0, 1000);
}

/* Efetua a escrita da potência dos motores entre 1ms e 2ms */
void writeMotor() {
  // int val = map(rcThrottle, MIN_RC_THROTTLE, MAX_RC_THROTTLE, 1000, 2000); /* To-do: trocar por constrain */
  motorFL.writeMicroseconds(constrain(motors[MOTOR_FL], 1000, 2000));
  motorFR.writeMicroseconds(constrain(motors[MOTOR_FR], 1000, 2000));
  motorBL.writeMicroseconds(constrain(motors[MOTOR_BL], 1000, 2000));
  motorBR.writeMicroseconds(constrain(motors[MOTOR_BR], 1000, 2000));
}

/* Efetua a leitura da orientação a ser armazenada na variável "ypr" */
void readOrientation() {
  bool fifoOverflow = false; // Marca se houve overflow da FIFO durante a execução

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
    }

    /* Se houve overflow, aguarda o próximo valor estar pronto */
    if (fifoOverflow) {
      while(!mpuInterrupt);
    }

    if (mpuIntStatus & 0x02) {
      /* Obtém a contagem da FIFO */
      fifoCount = mpu.getFIFOCount();

      /* Aguarda encher a FIFO caso não esteja com os todos dados completos (pacote completo) */
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      /* Dado está pronto para ser lido do MPU */
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      /* Obtém os valores (Yaw, Pitch e Roll) do DMP */
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      /* Converte para graus/segundo */
      for (int i = 0; i < 3; i++) {
        ypr_degree[i] = ypr[i] * 180/M_PI;
      }
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
 * - Posição: pitch (em graus)
 * - Posição: roll (em graus)
 * - Posição: yaw (em graus)
 * - Saída pitch do PID stab: pitch_stab_output (entre -500 a 500)
 * - Saída roll do PID stab: roll_stab_output (entre -500 a 500)
 * - Saída yaw do PID stab: yaw_stab_output (entre -500 a 500)
 * - Motor FL (em us - entre 1000 a 2000)
 * - Motor FR (em us - entre 1000 a 2000)
 * - Motor BL (em us - entre 1000 a 2000)
 * - Motor BR (em us - entre 1000 a 2000)
 * - Canal 1 (em us - entre ~1000 a ~2000)
 * - Canal 2 (em us - entre ~1000 a ~2000)
 * - Canal 3 (em us - entre ~1000 a ~2000)
 * - Canal 4 (em us - entre ~1000 a ~2000)
 * - Canal 5 (em us - entre ~1000 a ~2000)
 * - Canal 6 (em us - entre ~1000 a ~2000)
 * - Intervalo do loop (em us)
 */
void telemetry() {
  /* Calcula a frequência do loop */
  tmInterval = micros() - tmCheckpoint;
  tmCheckpoint = micros();

  String tmValues;
  tmValues += padding("RCT=" + String(int(rcThrottle)), 8) + ";";
  tmValues += padding("RCY=" + String(int(rcYaw)), 8) + ";";
  tmValues += padding("RCP=" + String(int(rcPitch)), 8) + ";";
  tmValues += padding("RCR=" + String(int(rcRoll)), 8) + ";";
  tmValues += padding("RCO=" + String(int(rcOnOff)), 8) + ";";
  tmValues += padding("RCD=" + String(int(rcDimmer)), 8) + ";";
  tmValues += padding("DMP=" + String(int(ypr_degree[DMP_PITCH])), 10) + ";";
  tmValues += padding("DMR=" + String(int(ypr_degree[DMP_ROLL])), 10) + ";";
  tmValues += padding("DMY=" + String(int(ypr_degree[DMP_YAW])), 10) + ";";
  tmValues += padding("PIP=" + String(int(pitch_stab_output)), 8) + ";";
  tmValues += padding("PIR=" + String(int(roll_stab_output)), 8) + ";";
  tmValues += padding("PIY=" + String(int(yaw_stab_output)), 8) + ";";
  tmValues += padding("MFL=" + String(int(motors[MOTOR_FL])), 8) + ";";
  tmValues += padding("MFR=" + String(int(motors[MOTOR_FR])), 8) + ";";
  tmValues += padding("MBL=" + String(int(motors[MOTOR_BL])), 8) + ";";
  tmValues += padding("MBR=" + String(int(motors[MOTOR_BR])), 8) + ";";
  tmValues += padding("INT=" + String(int(tmInterval)), 10) + ";";
  for (int i=0; i < 6; i++) {
   tmValues += padding(String(channels[i]), 8) + ";";
  }

  countToSend++;
  if (countToSend > 4) {
    Serial.println(tmValues);
    countToSend = 0;
  }
}

String padding(String text, int size){
  String newString = String(text);
  if (text.length() < size) {
    for (int i = 0; i < size - text.length(); i++) {
      newString += " ";
    }
  }

  return newString;
}