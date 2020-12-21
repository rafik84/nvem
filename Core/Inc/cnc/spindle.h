#ifndef spindle_control_h
#define spindle_control_h
#include "grbl.h"

#define SPINDLE_NO_SYNC       false
#define SPINDLE_FORCE_SYNC    true

#define SPINDLE_STATE_DISABLE  0  // Must be zero.
#define SPINDLE_STATE_CW       bit(0)
#define SPINDLE_STATE_CCW      bit(1)

void spindle_init();
void spindle_stop();
uint8_t spindle_get_state();
void spindle_set_speed(float rpm);
void spindle_set_state(uint8_t state, float rpm);
void spindle_sync(uint8_t state, float rpm);

#endif
