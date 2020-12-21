#include "cnc/grbl.h"

// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t system_execute_line(char *line){
  uint8_t char_counter = 1;
  float parameter, value;

  switch( line[char_counter] ) {
    case '$':
    case 'G':
    case 'X':
      if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ) {
        case '$' : // Prints Grbl settings
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ){
        	  return(STATUS_IDLE_ERROR);
          }else {
        	  report_grbl_settings();
        	  return 0 ;
          }
        break;
        // $A settings axis

        case 'X' : // Disable alarm lock [ALARM]
          if (sys.state == STATE_ALARM) {
            // Block if safety door is ajar.
            if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Don't run startup script. Prevents stored moves in startup from causing accidents.
          } // Otherwise, no effect.
         break;
         //
         case 'G' : report(REPORT_GCODE_PARSER,0);break ;
      }
      break;

      //------------------------------------------------------------------------
      // $T
      case 'T':
          if(line[2] == '0'){
            // Tool change finished. Continue execution
            //system_clear_exec_state_flag(EXEC_TOOL_CHANGE);
            sys.state = STATE_IDLE;
            // Check if machine is homed and tls enabled
            if(sys.is_homed && (settings.tool_change == TOOL_CHANGE_MANUAL_TLS)){
                toolProbeTLS();
            }else{
                return STATUS_MACHINE_NOT_HOMED;
            }
        }else if(line[2] == 'S'){ //$TS - status TLS
          report_tls_param();
        }else if(line[2] == 'P'){ // $TP - store position TLS
          if(sys.is_homed){
              storeTlsPosition();
              //grbl_send("(TLS STORE POSITION)\r\n");
          }else{
              return STATUS_MACHINE_NOT_HOMED;
          }
        }
      break;
      /*/------------------------------------------------------------------------
      case 'L' :  // $L
        switch(line[2]){
          // disable limits
          case '0' : settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; if(sys.state == STATE_ALARM) sys.state = STATE_IDLE; grbl_send(CLIENT_ALL, "(LIMITS OFF)\r\n"); break ;
          // enable limits
          case '1' : settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; grbl_send(CLIENT_ALL, "(LIMITS ON)\r\n");break ;
        }
        //
        //limits_init();
      break ;
      //------------------------------------------------------------------------
       *
       */
    default :
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[1] ) {
        // $P reset position machine
        case 'P' :
          switch(line[2]){
            #ifdef A_AXIS
            case 'A' : sys_position[A_AXIS] = 0 ; break ;
            #endif
            #ifdef B_AXIS
            case 'B' : sys_position[B_AXIS] = 0 ; break ;
            #endif
            #ifdef C_AXIS
            case 'C' : sys_position[C_AXIS] = 0 ; break ;
            #endif
            case 'X' : sys_position[X_AXIS] = 0 ; break ;
            case 'Y' : sys_position[Y_AXIS] = 0 ; break ;
            case 'Z' : sys_position[Z_AXIS] = 0 ; break ;
            default :
              for(uint8_t axis = 0 ;axis<N_AXIS;axis++){sys_position[axis] = 0 ;}

              //grbl_send("(RESET MACHINE POSITION)\r\n");
            break ;
          }

         break ;
        // Print Grbl NGC parameters
        //case '#' : if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); } else { report_ngc_parameters(); }break;
        // Perform homing cycle [IDLE/ALARM]
        case 'H' :
          if (bit_isfalse(settings.flags,BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }
          //if (IO == IO_DISABLE)                                  {return(STATUS_SETTING_IO_DISABLED);}
          if (system_check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // Block if safety door is ajar.
          sys.state = STATE_HOMING; // Set system state variable
          if (line[2] == 0) {
            mc_homing_cycle(HOMING_CYCLE_ALL);
            #ifdef HOMING_SINGLE_AXIS_COMMANDS
            } else if (line[3] == 0) {
              switch (line[2]) {
                //
                #ifdef A_AXIS
                case 'A': mc_homing_cycle(HOMING_CYCLE_A); break;
                #endif
                #ifdef B_AXIS
                case 'B': mc_homing_cycle(HOMING_CYCLE_B); break;
                #endif
                #ifdef C_AXIS
                case 'C': mc_homing_cycle(HOMING_CYCLE_C); break;
                #endif
                //-----------------------------------------------
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                //
                default: return(STATUS_INVALID_STATEMENT);
              }
            #endif
          } else {
             return(STATUS_INVALID_STATEMENT);
          }
          if (!sys.abort) {         // Execute startup scripts after successful homing.
            sys.state = STATE_IDLE; // Set to IDLE when complete.
            st_go_idle();           // Set steppers to the settings idle state before returning.
          }
          break;
        // Save settings
        case 'S' : saveSettings(); break ;
        // Restore defaults [IDLE/ALARM]
        case 'R' :
          if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
          switch (line[5]) {
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
              default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Force reset to ensure settings are initialized correctly.
        break;
        // V - 1
        // = - 2
        case 'V' :
          char_counter = 3 ;
          if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
          //
          if((value < settings.rpm_min) || (value > settings.rpm_max)){
              return (STATUS_VFD_INVALID_RPM_VALUE);
          }
          //
          //vfd_set_speed(value,sys.spindle_speed_ovr);
        break ;
        default :  // Storing setting methods [IDLE/ALARM]

          if(!read_float(line, &char_counter, &parameter))   { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=')                    { return(STATUS_INVALID_STATEMENT); }

          if(!read_float(line, &char_counter, &value))       { return(STATUS_BAD_NUMBER_FORMAT); }
          if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
          // Store global setting.
          return(settings_store_global_setting((uint8_t)parameter, value));
        }
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}
// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint8_t system_check_safety_door_ajar(){
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); // Input pin not enabled, so just return that it's closed.
  #endif
}

// Special handlers for setting and clearing Grbl's real-time execution flags.
void system_set_exec_state_flag(uint8_t mask) {
  sys_rt_exec_state |= (mask);
}

void system_clear_exec_state_flag(uint8_t mask) {
  sys_rt_exec_state &= ~(mask);
}

void system_set_exec_alarm(uint8_t code) {
  sys_rt_exec_alarm = code;
}

void system_clear_exec_alarm() {
  sys_rt_exec_alarm = 0;
}

void system_set_exec_motion_override_flag(uint8_t mask) {
  sys_rt_exec_motion_override |= (mask);
}

void system_set_exec_accessory_override_flag(uint8_t mask) {
  sys_rt_exec_accessory_override |= (mask);
}

void system_clear_exec_motion_overrides() {
  sys_rt_exec_motion_override = 0;
}

void system_clear_exec_accessory_overrides() {
  sys_rt_exec_accessory_override = 0;
}


void system_flag_wco_change(){
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
}
// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx){
  float pos;

  pos = steps[idx]/settings.steps_per_mm[idx];
  return(pos);
}

void system_convert_array_steps_to_mpos(float *position, int32_t *steps){
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}
//
void system_convert_array_steps_to_wpos(float *position, int32_t *steps){
  uint8_t idx;

  float wco[N_AXIS];
  for (idx=0; idx< N_AXIS; idx++) {
	position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
    // Apply work coordinate offsets and tool length offset to current position.
    wco[idx] = gc_state.coord_system[idx]+gc_state.coord_offset[idx];
    if (idx == TOOL_LENGTH_OFFSET_AXIS) { wco[idx] += gc_state.tool_length_offset; }
    position[idx] -= wco[idx];
  }
}
// Checks and reports if target array exceeds machine travel limits.
uint8_t system_check_travel_limits(float *target){
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef HOMING_FORCE_SET_ORIGIN
      // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      // NOTE: max_travel is stored as negative
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) {
          return(true);
        }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
    #else
      // NOTE: max_travel is stored as negative
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
    #endif
  }
  return(false);
}

// Returns control pin state as a uint8 bitfield. Each bit indicates the input pin state, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Bitfield organization is
// defined by the CONTROL_PIN_INDEX in the header file.
uint8_t system_control_get_state(){
  return 0;

  uint8_t control_state = 0;
  #ifdef CONTROL_SAFETY_DOOR_PIN
    if (digitalRead(CONTROL_SAFETY_DOOR_PIN)) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
  #endif
  #ifdef CONTROL_RESET_PIN
    if (digitalRead(CONTROL_RESET_PIN)) { control_state |= CONTROL_PIN_INDEX_RESET; }
  #endif
  #ifdef CONTROL_FEED_HOLD_PIN
    if (digitalRead(CONTROL_FEED_HOLD_PIN)) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
  #endif
  #ifdef CONTROL_CYCLE_START_PIN
    if (digitalRead(CONTROL_CYCLE_START_PIN)) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
  #endif
  #ifdef INVERT_CONTROL_PIN_MASK
    control_state ^= INVERT_CONTROL_PIN_MASK;
  #endif

  return(control_state);
}

// Returns limit pin mask according to Grbl internal axis indexing.
uint8_t get_limit_pin_mask(uint8_t axis_idx){
 // if ( axis_idx == X_AXIS ) { return((1<<X_LIMIT_BIT)); }
 // if ( axis_idx == Y_AXIS ) { return((1<<Y_LIMIT_BIT)); }
 // return((1<<Z_LIMIT_BIT));
	return 0 ;
}
