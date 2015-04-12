// #ifndef mpu6050_header
// #define mpu6050_header

// /**************************************
//  * Giroscópio e Acelerômetro: MPU6050 *
//  **************************************/
// /* Variáveis de controle do MPU6050 (só MPU para facilitar) para a interface
//  * com o DMP - Processador que entrega valor prontos do giroscópio. */
// MPU6050 mpu;            // Entidade que representa o MPU
// bool dmpReady = false;  // Definida para true se o DMP foi inicializado com sucesso
// uint8_t mpuIntStatus;   // Armazena o valor atual do byte de status de interrupção do MPU
// uint8_t dmpStatus;      // Status a cada operação do DMP (0 = sucesso, !0 = erro)
// uint16_t packetSize;    // Tamanho esperado do pacote do DMP (o padrão é 42 bytes)
// uint16_t fifoCount;     // Conta quantos bytes há no FIFO
// uint8_t fifoBuffer[64]; // FIFO do DMP

// /* Variáveis de orientação que vem do DMP */
// Quaternion q;         // [w, x, y, z]         Container do quaternion
// VectorFloat gravity;  // [x, y, z]            Vetor de gravidade
// float ypr[3];         // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade
// double ypr_degree[3]; // [yaw, pitch, roll]   Container do yaw/pitch/roll e do vetor de gravidade em graus/segundo
// double yawRate;       // Velocidade de rotação do eixo yaw

// const int DMP_YAW = 0;
// const int DMP_PITCH = 1;
// const int DMP_ROLL = 2;

// #endif