#include "cnc/grbl.h"


void spindle_init(){
	spindle_set_speed(12000);
    // Start with spindle off off
	spindle_stop();
}

void spindle_stop(){

}
//
uint8_t spindle_get_state() {
  return 0;//VFD.state;
}
//
void spindle_set_speed(float rpm){

}
//
void spindle_set_state(uint8_t state, float rpm){
  if (sys.abort) { return; } // Block during abort.
  // Halt or set spindle direction and rpm.
  if (state == SPINDLE_DISABLE) {
    //sys.spindle_speed = 0.0;
    spindle_stop();
  } else {
		//
		spindle_set_speed(rpm);

  }
}
//
void spindle_sync(uint8_t state, float rpm){
	if (sys.state == STATE_CHECK_MODE) { return; }
	protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
	spindle_set_state(state,rpm);
}
