#ifndef defaults_h

  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_STEP_PULSE_MICROSECONDS         5
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME          200 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STEPPING_INVERT_MASK            7 // uint8_t
  #define DEFAULT_DIRECTION_INVERT_MASK           0 // uint8_t
  #define DEFAULT_INVERT_ST_ENABLE                0 // boolean
  #define DEFAULT_INVERT_LIMIT_PINS               1 // boolean
  #define DEFAULT_INVERT_PROBE_PIN                0 // boolean
  #define DEFAULT_STATUS_REPORT_MASK              2 // MPos enabled

  #define DEFAULT_JUNCTION_DEVIATION              0.01 // mm
  #define DEFAULT_ARC_TOLERANCE                   0.002 // mm
  #define DEFAULT_REPORT_INCHES                   0 // false

  #define DEFAULT_SOFT_LIMIT_ENABLE               0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE               1  // false

  #define DEFAULT_HOMING_ENABLE                   1  // false
  #define DEFAULT_HOMING_DIR_MASK                 3 // move positive dir Z X,Y
  #define DEFAULT_HOMING_FEED_RATE                500.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE                4000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY           250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF                  8 // mm
  #define DEFAULT_SPINDLE_RPM_MAX                 24000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN                 6000.0 // rpm
  #define DEFAULT_LASER_MODE                      0 // false

  #define DEFAULT_X_STEPS_PER_MM                  200.0
  #define DEFAULT_Y_STEPS_PER_MM                  400.0
  #define DEFAULT_Z_STEPS_PER_MM                  640.0

  #define DEFAULT_X_MAX_RATE                      5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE                      5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE                      5000.0 // mm/min
  //
  #define DEFAULT_MAX_FEED_RATE                   50000.0 // mm/min
  #define DEFAULT_MIN_FEED_RATE                   1000.0  // mm/min
  //
  #define DEFAULT_X_BACKLASH                      0.01     // mm
  #define DEFAULT_Y_BACKLASH                      0.01     // mm
  #define DEFAULT_Z_BACKLASH                      0.01     // mm
  //
  #define DEFAULT_X_ACCELERATION                  (350.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION                  (350.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION                  (300.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2

  #define DEFAULT_X_MAX_TRAVEL                    800.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL                    1265.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL                    220.0 // mm NOTE: Must be a positive value.

  #define DEFAULT_TOOL_CHANGE_MODE                0       // 0 = Ignore M6; 1 = Manual tool change; 2 = Manual tool change + TLS
  #define DEFAULT_TOOL_SENSOR_OFFSET              100.0 // mm
#endif
