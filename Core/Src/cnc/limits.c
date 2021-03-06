#include "cnc/grbl.h"
// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.1 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

#ifdef ENABLE_DUAL_AXIS
  // Flags for dual axis async limit trigger check.
  #define DUAL_AXIS_CHECK_DISABLE     0  // Must be zero
  #define DUAL_AXIS_CHECK_ENABLE      bit(0)
  #define DUAL_AXIS_CHECK_TRIGGER_1   bit(1)
  #define DUAL_AXIS_CHECK_TRIGGER_2   bit(2)
#endif

void limits_init(){
	if (bit_istrue(settings.flags, BITFLAG_HARD_LIMIT_ENABLE))
	{

	}else{

	}


}
// Disables hard limits.
void limits_disable(){

}
void HandleLimitIT(void){

}
// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
uint8_t limits_get_state(){
  uint8_t limit_state = 0;
/**
  if(bit_isfalse(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)){
    return(limit_state);
  }
  //
  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10) == GPIO_PIN_RESET){
	  limit_state |= (1 << X_AXIS);
  }
  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13) == GPIO_PIN_RESET){
	  limit_state |= (1 << X_AXIS);
  }
  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14) == GPIO_PIN_RESET){
	  limit_state |= (1 << Y_AXIS);
  }
  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15) == GPIO_PIN_RESET){
	  limit_state |= (1 << Y_AXIS);
  }
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)){

  }
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)){

  }
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){

  }
  if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == GPIO_PIN_RESET){
	  limit_state |= (1 << Z_AXIS);
  }
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == GPIO_PIN_RESET){
	  limit_state |= (1 << Z_AXIS);
  }**/
  //
  return(limit_state);
}
// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
void limits_go_home(uint8_t cycle_mask){
  if (sys.abort) { return; } // Block if system reset has been issued.

  // Initialize plan data struct for homing motion. Spindle and coolant are disabled.
  plan_line_data_t plan_data;
  plan_line_data_t *pl_data = &plan_data;
  memset(pl_data,0,sizeof(plan_line_data_t));
  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
  pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

  // Initialize variables used for homing computations.
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    // Initialize step pin masks
    step_pin[idx] = get_step_pin_mask(idx);

    if (bit_istrue(cycle_mask,bit(idx))) {
      // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
      // NOTE: settings.max_travel[] is stored as a negative value.
      max_travel = MAX(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
    }
  }

  // Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
  bool approach = true;
  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;
  do {

    system_convert_array_steps_to_mpos(target,sys_position);
    // Initialize and declare variables needed for homing routine.
    axislock = 0;
    n_active_axis = 0;
    for (idx=0; idx<N_AXIS; idx++) {
      // Set target location for active axes and setup computation for homing rate.
      if (bit_istrue(cycle_mask,bit(idx))) {
        n_active_axis++;
        sys_position[idx] = 0;
        // Set target direction based on cycle mask and homing cycle approach state.
        // NOTE: This happens to compile smaller than any other implementation tried.
        if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
          if (approach) { target[idx] = -max_travel; } else { target[idx] = max_travel; }
        } else {
          if (approach) { target[idx] = max_travel; } else { target[idx] = -max_travel; }
        }
        // Apply axislock to the step port pins active in this cycle.
        axislock |= step_pin[idx];
      }

    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
    sys.homing_axis_lock = axislock;
    // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
    pl_data->feed_rate = homing_rate; // Set current homing rate.
    plan_buffer_line(target, pl_data); // Bypass mc_line(). Directly plan homing motion.
    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // Set to execute homing motion and clear existing flags.
    st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
    st_wake_up(); // Initiate motion
    do {
      if (approach) {
        // Check limit state. Lock out cycle axes when they change.
        limit_state = limits_get_state();
        for (idx=0; idx<N_AXIS; idx++) {
          if (axislock & step_pin[idx]) {
            if (limit_state & (1 << idx)) {
                axislock &= ~(step_pin[idx]);
            }
          }
        }
        sys.homing_axis_lock = axislock;
      }
      st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.
      // Exit routines: No time to run protocol_execute_realtime() in this loop.
      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
        uint8_t rt_exec = sys_rt_exec_state;
        // Homing failure condition: Reset issued during cycle.
        if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
        // Homing failure condition: Safety door was opened.
        if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
        // Homing failure condition: Limit switch still engaged after pull-off motion
        if (!approach && (limits_get_state() & cycle_mask)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); }
        // Homing failure condition: Limit switch not found during approach.
        if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
        if (sys_rt_exec_alarm) {
          mc_reset(); // Stop motors, if they are running.
          protocol_execute_realtime();
          return;
        } else {
          // Pull-off motion complete. Disable CYCLE_STOP from executing.
          system_clear_exec_state_flag(EXEC_CYCLE_STOP);
          break;
        }
      }

    } while (STEP_MASK & axislock);

    st_reset(); // Immediately force kill steppers and reset step segment buffer.
    delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.
    // Reverse direction and reset homing rate for locate cycle(s).
    approach = !approach;
    // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
    if (approach) {
      max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
      homing_rate = settings.homing_feed_rate;
    } else {
      max_travel = settings.homing_pulloff;
      homing_rate = settings.homing_seek_rate;
    }

  } while (n_cycle-- > 0);

  // The active cycle axes should now be homed and machine limits have been located. By
  // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
  // set up pull-off maneuver from axes limit switches that have been homed. This provides
  // some initial clearance off the switches and should also help prevent them from falsely
  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
  int32_t set_axis_position;
  // Set machine positions for homed limit switches. Don't update non-homed axes.
  for (idx=0; idx<N_AXIS; idx++) {
    // NOTE: settings.max_travel[] is stored as a negative value.
    if (cycle_mask & bit(idx)) {
      #ifdef HOMING_FORCE_SET_ORIGIN
        set_axis_position = 0;
      #else
        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
        } else {
          set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
        }
      #endif

      sys_position[idx] = set_axis_position;
    }
  }
  sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
}
// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limits_soft_check(float *target){
  if (system_check_travel_limits(target)) {
    sys.soft_limit = true;
    // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
    // workspace volume so just come to a controlled stop so position is not lost. When complete
    // enter alarm mode.
    if (sys.state == STATE_CYCLE) {
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      do {
        protocol_execute_realtime();
        if (sys.abort) { return; }
      } while ( sys.state != STATE_IDLE );
    }
    mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); // Indicate soft limit critical event
    protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    return;
  }
}
