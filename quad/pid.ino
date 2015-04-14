#include "pid.h"

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


