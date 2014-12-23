#include <Servo.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

void telemetry();
void readRC();
void normalizeRC();

#define DEBUG // Habilita a telemetria
#ifdef DEBUG
  #define D(X) X
#else
  #define D(X)
#endif

#ifdef DEBUG
long tmCheckpoint; // Marca o tempo no checkpoint
long tmInterval;   // Intervalo entre última medição e o checkpoint
#endif

/* Indicação de atividade do processador */
#define LED_PIN 13
bool blinkState = false;

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
Quaternion q;           // [w, x, y, z]         Container do quaternion
VectorFloat gravity;    // [x, y, z]            Vetor de gravidade
float ypr[3];           // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade

/* Rotina de interrupção */ 
volatile bool mpuInterrupt = false; // Indica que houve interrupção da MPU, ou seja, há dados a serem lidos
void dmpDataReady() {
  mpuInterrupt = true;
}
/* Fim da rotina de interrupção */

/* Definição dos canais de entrada. */
const int CH1 = 8; /* Roll (horizontal da direita) */
const int CH2 = 3; /* Pitch (vertical da direita) */
const int CH3 = 4; /* Throttle (vertical da esquerda) */
const int CH4 = 5; /* Yaw (horizontal da esquerda */
const int CH5 = 6; /* ON/OFF */
const int CH6 = 7; /* Dimmer */

/* Armazena a duração dos canais do controle (em us). */
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

/* Entidades que representam os motores. Responsáveis por acionar os ESCs realizando um PPM entre 1ms e 2ms. */
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

/* Canais de cada motor */
const int MOTOR_FL = 8; /* To-do test this pin */
const int MOTOR_FR = 9; 
const int MOTOR_BL = 10; /* To-do test this pin */
const int MOTOR_BR = 11; /* To-do test this pin */

/* Parâmetros do PID. */
double setpoint, input, output;

PID myPID(&input, &output, &setpoint,1,2,3, DIRECT);

/* Inicialização do Arduino. */
void setup() {
  /* Bind dos canais de entrada do controle */
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  
  /* Bind dos canais de saída dos motores (ESCs) */
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorBL.attach(MOTOR_BL);
  motorBR.attach(MOTOR_BR);

  setpoint = 1500;

  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);

  /* Inicializa o barramento I2C (a biblioteca I2Cdev não faz isso automaticamente) */
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  /* Inicializa a porta serial */
  Serial.begin(115200);
  while (Serial.available() && Serial.read()); // Limpa o buffer

  /* Inicializa a MPU e seu respectivo DMP */
  mpu.initialize();
  Serial.println(F("Inicializando DMP..."));
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
    Serial.println(F("Habilitando interrupcao 0 (pino 2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    /* Inicializa a flag de controle do DMP para o loop() saber que está tudo certo */
    Serial.println(F("DMP pronto! Iniciando programa..."));
    dmpReady = true;

    /* Obtem o tamanho do pacote do DMP */
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("Tamanho do pacote do DMP obtido..."));
  } 
  else {
    // ERRO!
    // 1 = Carregamento inicial da meméria falhou
    // 2 = Atualização da configuração do DMP falhou
    Serial.print(F("Inicialização do DMP falhou (código "));
    Serial.print(dmpStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

/* Loop infinito do Arduino. */
void loop() {       
  /* Se a programação do DMP falhou, não faz nada */
  if (!dmpReady) {
    return;
  }

  readOrientation();

  // readRC();
  
  //  input = ch1v;
  // myPID.Compute();
  // ch1v += (int)output;

  writeMotor();

  // D(telemetry());

  /* Pisca a led para indicar atividade */
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

/* Efetua a escrita da potência dos motores entre 1ms e 2ms */
void writeMotor() {
  int val = map(rcThrottle, MIN_CH1, MAX_CH1, 1000, 2000);
  motorFL.writeMicroseconds(val);
  motorFR.writeMicroseconds(val);
  motorBL.writeMicroseconds(val);
  motorBR.writeMicroseconds(val);
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
      
      /* Debug */
      D(Serial.print(F("ypr\t")));
      D(Serial.print(ypr[0] * 180/M_PI));
      D(Serial.print("\t"));
      D(Serial.print(ypr[1] * 180/M_PI));
      D(Serial.print("\t"));
      D(Serial.println(ypr[2] * 180/M_PI));
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
  //for (int i=0; i < 6; i++) {
  //  tmValues += String(channels[i]) + ";";
  //}
  tmValues += "\n";

  Serial.print(tmValues);
}









