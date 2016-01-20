#include "controller.h"
#include <PinChangeInt.h>
#include <ByteBuffer.h>
#include <MemoryFree.h>

/* Define os pinos do Arduino para entrada do controle. */
uint8_t pins[NUM_CHANNELS] = {CH1, CH2, CH3, CH4, CH5, CH6};

/* Marca o momento em que começou a contagem do tempo. */
volatile unsigned long rising[NUM_CHANNELS]={ 0, 0, 0, 0, 0, 0 };

/* Marca o último momento em que o controle retornou um sinal. */
unsigned long lastAlive = 0;

/* Inicializa os pinos de entrada do rádio, anexando o controle de interrupção de subida. */
void setupRadio() {
  for(int i=0; i<NUM_CHANNELS; i++) {
    pinMode(pins[i], INPUT);
    digitalWrite(pins[i], HIGH);
    PCintPort::attachInterrupt(pins[i], chrising, RISING);
  }
}

/* Trata a interrupção de subida de algum pino dos controles. */
void chrising() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;

  unsigned long now = micros();

  int pinIndex = indexFromPin(interrupted_pin);

  rising[pinIndex] = now;

  lastAlive = millis();
  if (pinIndex == CH_THROTTLE) {
    RCreadCount++;
  }

  PCintPort::detachInterrupt(interrupted_pin);
  PCintPort::attachInterrupt(interrupted_pin, chfalling, FALLING);
}

/* Trata a interrupção de descida de algum pino dos controles. */
void chfalling() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;

  unsigned long now = micros();

  int pinIndex = indexFromPin(interrupted_pin);

  int newVal = now - rising[pinIndex];

  if (rising[pinIndex] != 0) {
    chvalue[pinIndex] = newVal;
  }

  PCintPort::detachInterrupt(interrupted_pin);
  PCintPort::attachInterrupt(interrupted_pin, chrising, RISING);
}

/* Retorna qual o índice de um dado pino. */
int indexFromPin(int pin) {
  for(int i = 0; i < NUM_CHANNELS; i++) {
    if (pins[i] == pin)
      return i;
  }

  return -1;
}

/* Detecta perda de sinal, ou seja, quando não existe nenhum sinal de nenhum canal do controle (throttle) dentro de 500ms. */
bool detectLostSignal() {
  if ((lastAlive > 0) && (millis() - lastAlive > 500)) {
    chvalue[CH_THROTTLE] = 0;
    D(Serial.println("Perda de sinal do RC!"));

    return true;
  }

  return false;
}

/* Normaliza os valores do controle para ranges pré definidos. */
void normalizeRC() {

  /* Leva em consideração o zero do roll controle */
  if (rcRoll > rcRollZero) {
    rcRoll = map(chvalue[0], rcRollZero, MAX_RC_ROLL, 0, 15);
  } else {
    rcRoll = map(chvalue[0], MIN_RC_ROLL, rcRollZero, -15, 0);
  }

  /* Leva em consideração o zero do roll controle */
  if (rcPitch > rcRollZero) {
    rcPitch = map(chvalue[1], rcPitchZero, MAX_RC_PITCH, 0, 15);
  } else {
    rcPitch = map(chvalue[1], MIN_RC_PITCH, rcPitchZero, -15, 0);
  }

  /* Leva em consideração o zero do yaw controle */
  if (rcYaw > rcYawZero) {
    rcYaw = map(chvalue[3], rcYawZero, MAX_RC_YAW, 0, 30);
  } else {
    rcYaw = map(chvalue[3], MIN_RC_YAW, rcYawZero, -30, 0);
  }

  rcThrottle = chvalue[2];
  rcOnOff    = map(chvalue[4], MIN_RC_ON_OFF, MAX_RC_ON_OFF, 0, 100);
  rcDimmer   = map(chvalue[5], MIN_RC_DIMMER, MAX_RC_DIMMER, 0, 255);

  /* TODO: REMOVER */
  rcRoll = rcPitch = 0;
  /* END TODO */
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