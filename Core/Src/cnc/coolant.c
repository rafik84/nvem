#include "cnc/grbl.h"

// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state(){
  uint8_t cl_state = COOLANT_STATE_DISABLE;
  //
  if(output_get_action_state(OUTPUT_TYPE_COOLANT_FLOOD)){
    cl_state |= COOLANT_STATE_FLOOD;
  }
  //
  if(output_get_action_state(OUTPUT_TYPE_COOLANT_MIST)){
    cl_state |= COOLANT_STATE_MIST;
  }

  return(cl_state);

}

// Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
void coolant_stop(){

}

// Main program only. Immediately sets flood coolant running state and also mist coolant,
// if enabled. Also sets a flag to report an update to a coolant state.
// Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser coolant_sync().
void coolant_set_state(uint8_t mode){
  if (sys.abort) { return; } // Block during abort.

  if (mode == COOLANT_DISABLE) {

  } else {

  }
}
// G-code parser entry-point for setting coolant state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void coolant_sync(uint8_t mode){
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
  coolant_set_state(mode);
}
