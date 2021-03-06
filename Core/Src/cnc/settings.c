#include "cnc/grbl.h"




settings_t settings;

void settings_init(){
	settings_restore(SETTINGS_RESTORE_ALL);
   //loadSettings();
}
//-----------------------------------------------------------------------------------------
void loadSettings(){

}

void saveSettings(){

}
//-----------------------------------------------------------------------------------------
// Method to restore EEPROM-saved Grbl global settings back to defaults.
void settings_restore(uint8_t restore_flag) {
  if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {

    settings.pulse_microseconds =     DEFAULT_STEP_PULSE_MICROSECONDS;
    settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
    settings.step_invert_mask =       DEFAULT_STEPPING_INVERT_MASK;
    settings.dir_invert_mask =        DEFAULT_DIRECTION_INVERT_MASK;
    settings.status_report_mask =     DEFAULT_STATUS_REPORT_MASK;
    settings.junction_deviation =     DEFAULT_JUNCTION_DEVIATION;
    settings.max_feed_rate      =     DEFAULT_MAX_FEED_RATE;
    settings.arc_tolerance      =     DEFAULT_ARC_TOLERANCE;
    settings.backlash[X_AXIS]   =     DEFAULT_X_BACKLASH;
    settings.backlash[Y_AXIS]   =     DEFAULT_Y_BACKLASH;
    settings.backlash[Z_AXIS]   =     DEFAULT_Z_BACKLASH;
    //--------------------------------------------------------------------------
    settings.tool_change = DEFAULT_TOOL_CHANGE_MODE;
	settings.tls_valid = 0;
	settings.tls_position[X_AXIS] = 0;
	settings.tls_position[Y_AXIS] = 0;
	settings.tls_position[Z_AXIS] = 0;
    //--------------------------------------------------------------------------
    settings.rpm_max = DEFAULT_SPINDLE_RPM_MAX;
    settings.rpm_min = DEFAULT_SPINDLE_RPM_MIN;
    //--------------------------------------------------------------------------
    settings.homing_dir_mask        = DEFAULT_HOMING_DIR_MASK;
    settings.homing_feed_rate       = DEFAULT_HOMING_FEED_RATE;
    settings.homing_seek_rate       = DEFAULT_HOMING_SEEK_RATE;
    settings.homing_debounce_delay  = DEFAULT_HOMING_DEBOUNCE_DELAY;
    settings.homing_pulloff         = DEFAULT_HOMING_PULLOFF;
    //--------------------------------------------------------------------------
    settings.flags = 0;
    if (DEFAULT_REPORT_INCHES)      { settings.flags |= BITFLAG_REPORT_INCHES; }
    if (DEFAULT_LASER_MODE)         { settings.flags |= BITFLAG_LASER_MODE; }
    if (DEFAULT_INVERT_ST_ENABLE)   { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
    if (DEFAULT_HARD_LIMIT_ENABLE)  { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
    if (DEFAULT_HOMING_ENABLE)      { settings.flags |= BITFLAG_HOMING_ENABLE; }
    if (DEFAULT_SOFT_LIMIT_ENABLE)  { settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE; }
    if (DEFAULT_INVERT_LIMIT_PINS)  { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
    if (DEFAULT_INVERT_PROBE_PIN)   { settings.flags |= BITFLAG_INVERT_PROBE_PIN; }

    settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
    settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
    settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
    settings.max_rate[X_AXIS]     = DEFAULT_X_MAX_RATE;
    settings.max_rate[Y_AXIS]     = DEFAULT_Y_MAX_RATE;
    settings.max_rate[Z_AXIS]     = DEFAULT_Z_MAX_RATE;
    settings.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
    settings.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
    settings.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
    settings.max_travel[X_AXIS]   = (-DEFAULT_X_MAX_TRAVEL);
    settings.max_travel[Y_AXIS]   = (-DEFAULT_Y_MAX_TRAVEL);
    settings.max_travel[Z_AXIS]   = (-DEFAULT_Z_MAX_TRAVEL);

    uint8_t idx;
    float coord_data[N_AXIS];
    memset(&coord_data, 0, sizeof(coord_data));
    for (idx=0; idx <= SETTING_INDEX_NCOORD; idx++) { settings_write_coord_data(idx, coord_data); }

    saveSettings();
  }
}

// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data){

  memcpy(coord_data, settings.COORD[coord_select],  sizeof(float)*N_AXIS);

  return(true);
}
// Method to store coord data parameters into EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data){
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize();
  #endif

  memcpy(settings.COORD[coord_select], coord_data, sizeof(float)*N_AXIS);
  //
  saveSettings();
}
// A helper method to set settings from command line
uint8_t settings_store_global_setting(uint8_t parameter, float value) {
  if (value < 0.0) { return(STATUS_NEGATIVE_VALUE); }
  if (parameter >= AXIS_SETTINGS_START_VAL) {
    // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
    // NOTE: Ensure the setting index corresponds to the report.c settings printout.
    parameter -= AXIS_SETTINGS_START_VAL;
    uint8_t set_idx = 0;
    while (set_idx < AXIS_N_SETTINGS) {
      if (parameter < N_AXIS) {
        // Valid axis setting found.
        switch (set_idx) {
          case 0:
            #ifdef MAX_STEP_RATE_HZ
              if (value*settings.max_rate[parameter] > (MAX_STEP_RATE_HZ*60.0)) { return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.steps_per_mm[parameter] = value;
            break;
          case 1:
            #ifdef MAX_STEP_RATE_HZ
              if (value*settings.steps_per_mm[parameter] > (MAX_STEP_RATE_HZ*60.0)) {  return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.max_rate[parameter] = value;
            break;
          case 2: settings.acceleration[parameter] = value*60*60; break;    // Convert to mm/min^2 for grbl internal use.
          case 3: settings.max_travel[parameter] = -value; break;           // Store as negative for grbl internal use.
          case 4: settings.backlash[parameter] = value; break;
        }
        break; // Exit while-loop after setting has been configured and proceed to the EEPROM write call.
      } else {
        set_idx++;
        // If axis index greater than N_AXIS or setting index greater than number of axis settings, error out.
        if ((parameter < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS)) {
          return(STATUS_INVALID_STATEMENT);
        }
        parameter -= AXIS_SETTINGS_INCREMENT;
      }
    }
  } else if(parameter >= IN_SETTINGS_VAL && parameter <= (IN_SETTINGS_VAL+15)){   // 40.. 55
	  settings.INPUTS[(parameter-IN_SETTINGS_VAL)].Action   = (uint16_t) value;
  } else if(parameter >= OUT_SETTINGS_VAL && parameter <= (OUT_SETTINGS_VAL+15)){  // 56..71
	  settings.OUTPUTS[(parameter-OUT_SETTINGS_VAL)].Action   = (uint16_t) value;
  } else {
    // Store non-axis Grbl settings
    uint8_t int_value = trunc(value);
    switch(parameter) {
      case 0: if (int_value < 1) { return(STATUS_SETTING_STEP_PULSE_MIN); } settings.pulse_microseconds = int_value; break;
      case 1: settings.stepper_idle_lock_time = int_value; break;
      case 2: settings.step_invert_mask = int_value;st_generate_step_dir_invert_masks();break;
      case 3: settings.dir_invert_mask = int_value; st_generate_step_dir_invert_masks();break;
      case 4: if (int_value) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; } else { settings.flags &= ~BITFLAG_INVERT_ST_ENABLE; } break;
      case 5: if (int_value) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS;} else { settings.flags &= ~BITFLAG_INVERT_LIMIT_PINS;} break;
      case 6: if (int_value) { settings.flags |= BITFLAG_INVERT_PROBE_PIN; } else { settings.flags &= ~BITFLAG_INVERT_PROBE_PIN; } probe_configure_invert_mask(false); break;
      case 10: settings.status_report_mask = int_value; break;
      case 11: settings.junction_deviation = value; break;
      case 12: settings.arc_tolerance = value; break;
      case 13: if (int_value) { settings.flags |= BITFLAG_REPORT_INCHES; }else { settings.flags &= ~BITFLAG_REPORT_INCHES; } system_flag_wco_change(); break;
      case 14: settings.max_feed_rate = value ; break ;
      case 15: settings.spindle_time_run = value ; break ;    // Time RUN  (s)
      case 16: settings.spindle_time_stop = value ; break ;   // Time STOP  (s)
      case 17: settings.tool_change = int_value; break;       // Check for range?
      case 20:
        if (int_value) {
          if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE)) { return(STATUS_SOFT_LIMIT_ERROR); }
          settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
        } else {
          settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE;
        }
        break;
      case 21:
        if (int_value) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; } else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
        limits_init(); // Re-init to immediately change. NOTE: Nice to have but could be problematic later.
        break;
      case 22:
        if (int_value) {
          settings.flags |= BITFLAG_HOMING_ENABLE;
        }else {
          settings.flags &= ~BITFLAG_HOMING_ENABLE;
          settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; // Force disable soft-limits.
        }
        break;
      case 23: settings.homing_dir_mask = int_value; break;
      case 24: settings.homing_feed_rate = value; break;
      case 25: settings.homing_seek_rate = value; break;
      case 26: settings.homing_debounce_delay = int_value; break;
      case 27: settings.homing_pulloff = value; break;
      case 30: settings.rpm_max = value; spindle_init(); break; // Re-initialize spindle rpm calibration
      case 31: settings.rpm_min = value; spindle_init(); break; // Re-initialize spindle rpm calibration
      case 32:
        #ifdef VARIABLE_SPINDLE
          if (int_value) { settings.flags |= BITFLAG_LASER_MODE; }else { settings.flags &= ~BITFLAG_LASER_MODE; }
        #else
          return(STATUS_SETTING_DISABLED_LASER);
        #endif
        break;
      case 34: settings.inputMask  = (uint16_t) value; break ; // input mask
      case 35: settings.outputMask = (uint16_t) value; break ; // output mask
      default: return(STATUS_INVALID_STATEMENT);
    }
  }
  //saveSettings();
  return(STATUS_OK);
}
void storeTlsPosition(void){
    memcpy(settings.tls_position, sys_position, sizeof(float)*N_AXIS);
    settings.tls_valid = TOOL_IS_VALID;

    saveSettings();
}
// Returns step pin mask according to Grbl internal axis indexing.
uint8_t get_step_pin_mask(uint8_t axis_idx){
  // todo clean this up further up stream
  return(1<<axis_idx);
}
// Returns direction pin mask according to Grbl internal axis indexing.
uint8_t get_direction_pin_mask(uint8_t axis_idx){
	return(1<<axis_idx);
}
