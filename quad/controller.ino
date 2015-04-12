#include "controller.h"
#include <PinChangeInt.h>
#include <ByteBuffer.h>
#include <MemoryFree.h>

/* Marca o momento em que começou a contagem do tempo. */
volatile unsigned long rising[NUM_CHANNELS]={ 0, 0, 0, 0, 0, 0 };

/* Indica se houve perda de sinal */
int lostSignal;

unsigned long lastAlive = 0;

/* Efetua o setup do controle remoto */
void setupRC() {
  /* Bind dos canais de entrada do controle */
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH6, INPUT);

  /* Zera os valores do Pitch, Roll e Yaw */
  getZeroValuesFromRc();
}

/* Efetua a leitura dos canais dos controles. */
void readRC() {
  channels[0] = pulseIn(CH1, HIGH, 20000); /* Roll (horizontal da direita) */
  channels[1] = pulseIn(CH2, HIGH, 20000); /* Pitch (vertical da direita) */
  channels[2] = pulseIn(CH3, HIGH, 20000); /* Throttle (vertical da esquerda) */
  channels[3] = pulseIn(CH4, HIGH, 20000); /* Yaw (horizontal da esquerda */
  channels[4] = pulseIn(CH5, HIGH, 20000); /* ON/OFF */
  channels[5] = pulseIn(CH6, HIGH, 20000); /* Dimmer */

  // channels[0] = 1500;
  // channels[1] = 1500;
  // channels[2] = 1500;
  // channels[3] = 1500;
  // channels[4] = 1000;
  // channels[5] = 1000;

  normalizeRC();
}

/* Normaliza os valores do controle para ranges pré definidos. */
void normalizeRC() {

  /* Leva em consideração o zero do roll controle */
  if (rcRoll > rcRollZero) {
    rcRoll = map(channels[0], rcRollZero, MAX_RC_ROLL, 0, 15);
  } else {
    rcRoll = map(channels[0], MIN_RC_ROLL, rcRollZero, -15, 0);
  }

  /* Leva em consideração o zero do roll controle */
  if (rcPitch > rcRollZero) {
    rcPitch = map(channels[1], rcPitchZero, MAX_RC_PITCH, 0, 15);
  } else {
    rcPitch = map(channels[1], MIN_RC_PITCH, rcPitchZero, -15, 0);
  }

  /* Leva em consideração o zero do yaw controle */
  if (rcYaw > rcYawZero) {
    rcYaw = map(channels[3], rcYawZero, MAX_RC_YAW, 0, 250);
  } else {
    rcYaw = map(channels[3], MIN_RC_YAW, rcYawZero, -250, 0);
  }

  rcThrottle = channels[2];
  rcOnOff    = map(channels[4], MIN_RC_ON_OFF, MAX_RC_ON_OFF, 0, 100);
  rcDimmer   = map(channels[5], MIN_RC_DIMMER, MAX_RC_DIMMER, 0, 255);

  /* TODO: REMOVER */
  rcRoll = rcYaw = 0;
  /* END TODO */
}

/* ========= NEW RC ========= */
/* Inicializa os pinos de entrada do rádio, anexando o controle de interrupção de subida. */
void initRadio() {
  for(int i=0; i<NUM_CHANNELS; i++) {

    Serial.println("a " + String(pins[i]));
    pinMode(pins[i], INPUT);
    Serial.println("b");
    digitalWrite(pins[i], HIGH);
    Serial.println("c");
    PCintPort::attachInterrupt(pins[i], chrising, RISING);
    Serial.println("d");
  }
}

/* Trata a interrupção de subida de algum pino dos controles. */
void chrising() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;

  lastAlive = millis();

  unsigned long now = micros();

  int pinIndex = indexFromPin(interrupted_pin);

  rising[pinIndex] = now;

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

  // #ifdef CONSTANT_CALIBRATING
  //   if (discardValues < 100) {
  //     discardValues++;
  //   }
  //   else {
  //     if (newVal < chmin[pinIndex])
  //       chmin[pinIndex] = newVal;
  //     if (newVal > chmax[pinIndex])
  //       chmax[pinIndex] = newVal;
  //   }
  // #endif

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

/* Detecta perda de sinal, ou seja, quando não existe nenhum sinal de nenhum canal do controle dentro de 500ms */
bool detectLostSignal() {

  if ((lastAlive > 0) && (millis() - lastAlive > 500)) {
    return true;
  }

  return false;
}