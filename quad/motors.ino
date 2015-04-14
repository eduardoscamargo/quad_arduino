/* Entidades que representam os motores.
 * Responsáveis por acionar os ESCs realizando um PPM entre 1ms e 2ms. */
Servo motorFL;
Servo motorFR;
Servo motorBL;
Servo motorBR;

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

/* Efetua a escrita da potência dos motores entre 1ms e 2ms */
void writeMotor() {
  motorFL.writeMicroseconds(constrain(motors[MOTOR_FL], 1000, 2000));
  motorFR.writeMicroseconds(constrain(motors[MOTOR_FR], 1000, 2000));
  motorBL.writeMicroseconds(constrain(motors[MOTOR_BL], 1000, 2000));
  motorBR.writeMicroseconds(constrain(motors[MOTOR_BR], 1000, 2000));
}