#ifndef controller_header
#define controller_header

/******************************
 * Entrada do controle remoto *
 *****************************/
#define NUM_CHANNELS 6

/* Definição dos canais de entrada. */
#define CH1 A8  /* Roll (horizontal da direita) */
#define CH2 A13 /* Pitch (vertical da direita) */
#define CH3 A12 /* Throttle (vertical da esquerda) */
#define CH4 A11 /* Yaw (horizontal da esquerda */
#define CH5 A10 /* ON/OFF */
#define CH6 A9  /* Dimmer */

/* Posição dos canais no vetor 'chvalues' */
#define CH_ROLL     0
#define CH_PITCH    1
#define CH_THROTTLE 2
#define CH_YAW      3
#define CH_ON_OFF   4
#define CH_DIMMER   5

/* Valores máximos e mínimos (em us) de cada canal. */
#define MAX_RC_ROLL     1939
#define MIN_RC_ROLL     1040
#define MAX_RC_PITCH    2009
#define MIN_RC_PITCH    1012
#define MAX_RC_THROTTLE 1950
#define MIN_RC_THROTTLE 1070
#define MIN_RC_YAW      1030
#define MAX_RC_YAW      2009
#define MAX_RC_ON_OFF   2009
#define MIN_RC_ON_OFF   1014
#define MAX_RC_DIMMER   1018
#define MIN_RC_DIMMER   2006

void setupRadio();
void chrising();
void chfalling();
int indexFromPin(int pin);
void normalizeRC();

#endif