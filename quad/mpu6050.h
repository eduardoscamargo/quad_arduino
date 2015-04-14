#ifndef mpu6050_header
#define mpu6050_header

/**************************************
 * Giroscópio e Acelerômetro: MPU6050 *
 **************************************/
#define DMP_YAW 0
#define DMP_PITCH 1
#define DMP_ROLL 2

void setupMPU();
void readOrientation();

#endif