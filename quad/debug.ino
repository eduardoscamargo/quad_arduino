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
  countToSend++;
  if (countToSend > 20) {

    /* Calcula a frequência do loop */
    tmInterval = micros() - tmCheckpoint;
    tmCheckpoint = micros();

    String tmValues;
    // tmValues += padding("RCT=" + String(int(rcThrottle)), 9) + ";";
    // tmValues += padding("RCY=" + String(int(rcYaw)), 8) + ";";
    // tmValues += padding("RCP=" + String(int(rcPitch)), 8) + ";";
    // tmValues += padding("RCR=" + String(int(rcRoll)), 8) + ";";
    // tmValues += padding("RCO=" + String(int(rcOnOff)), 8) + ";";
    // tmValues += padding("RCD=" + String(int(rcDimmer)), 8) + ";";
    // tmValues += padding("ZRP=" + String(int(rcPitchZero)), 9) + ";";
    // tmValues += padding("ZRR=" + String(int(rcRollZero)), 9) + ";";
    // tmValues += padding("ZRY=" + String(int(rcYawZero)), 9) + ";";
    tmValues += padding("DMP=" + String(int(ypr_degree[DMP_PITCH])), 9) + ";";
    tmValues += padding("DMR=" + String(int(ypr_degree[DMP_ROLL])), 9) + ";";
    // tmValues += padding("DMY=" + String(int(ypr_degree[DMP_YAW])), 9) + ";";
    tmValues += padding("YRT=" + String(int(yawRate)), 9) + ";";
    tmValues += padding("PIP=" + String(int(pitch_stab_output)), 8) + ";";
    // tmValues += padding("PIR=" + String(int(roll_stab_output)), 8) + ";";
    // tmValues += padding("PIY=" + String(int(yaw_stab_output)), 8) + ";";
    // tmValues += padding("MFL=" + String(int(motors[MOTOR_FL])), 9) + ";";
    // tmValues += padding("MFR=" + String(int(motors[MOTOR_FR])), 9) + ";";
    // tmValues += padding("MBL=" + String(int(motors[MOTOR_BL])), 9) + ";";
    // tmValues += padding("MBR=" + String(int(motors[MOTOR_BR])), 9) + ";";
    // for (int i=0; i < 6; i++) {
    //  tmValues += padding(String(channels[i]), 8) + ";";
    // }
    tmValues += padding("INT=" + String(tmInterval), 11) + ";";

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