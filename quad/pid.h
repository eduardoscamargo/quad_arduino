#ifndef pid_header
#define pid_header

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
};

/* Configuração dos endereços da EEPROM para armazenar os PIDs */
#define EEPROM_PITCH_P 0
#define EEPROM_PITCH_I 1
#define EEPROM_PITCH_D 2
#define EEPROM_ROLL_P 3
#define EEPROM_ROLL_I 4
#define EEPROM_ROLL_D 5
#define EEPROM_YAW_P 6
#define EEPROM_YAW_I 7
#define EEPROM_YAW_D 8

void setupPID();
void getPIDParametersAndSetTunnings();
void setPIDParameters(char param);
void writePIDParameters();

#endif