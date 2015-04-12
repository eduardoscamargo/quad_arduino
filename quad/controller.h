#ifndef controller_header
#define controller_header

/******************************
 * Entrada do controle remoto *
 *****************************/
#define NUM_CHANNELS 6

/* Definição dos canais de entrada. */
#define CH1 8 /* Roll (horizontal da direita) */
#define CH2 3 /* Pitch (vertical da direita) */
#define CH3 4 /* Throttle (vertical da esquerda) */
#define CH4 5 /* Yaw (horizontal da esquerda */
#define CH5 6 /* ON/OFF */
#define CH6 7 /* Dimmer */

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

void readRC();
void normalizeRC();
void setupRC();
void initRadio();
void chrising();
void chfalling();
int indexFromPin(int pin);

#endif