#include "mpu6050.h"

uint8_t mpuIntStatus;   // Armazena o valor atual do byte de status de interrupção do MPU
uint16_t packetSize;    // Tamanho esperado do pacote do DMP (o padrão é 42 bytes)

/* Efetua o setup do MPU6050 */
void setupMPU() {
  uint8_t dmpStatus;      // Status a cada operação do DMP (0 = sucesso, !0 = erro)

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

/* Efetua a leitura da orientação a ser armazenada na variável "ypr" */
void readOrientation() {
  uint16_t fifoCount;     // Conta quantos bytes há no FIFO
  uint8_t fifoBuffer[64]; // FIFO do DMP

  /* Variáveis de orientação que vem do DMP */
  Quaternion q;         // [w, x, y, z]         Container do quaternion
  VectorFloat gravity;  // [x, y, z]            Vetor de gravidade
  float ypr[3];         // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade

  bool fifoOverflow = false; // Marca se houve overflow da FIFO durante a execução

  /* Le somente se houve interrupção */
  if (mpuInterrupt) {
    /* Reseta a flag de interrupção */
    mpuInterrupt = false;

    /* Obtém a contagem da FIFO */
    fifoCount = mpu.getFIFOCount();

    /* Obtém o byte INT_STATUS do MPU */
    mpuIntStatus = mpu.getIntStatus();

    /* Verifica se houve overflow da FIFO. Quanto mais ineficiente o código, mais irá ocorrer. */
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      Serial.println("Overflow");
      // Reseta a FIFO
      mpu.resetFIFO();
      fifoOverflow = true;
      return;
    }

    if (mpuIntStatus & 0x01) {
      /* Aguarda encher a FIFO caso não esteja com os todos dados completos (pacote completo) */
      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
        Serial.println("Aguardando FIFO");
      }

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