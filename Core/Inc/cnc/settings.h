#ifndef settings_h
#define settings_h

#include "grbl.h"

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_LASER_MODE         bit(1)
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)
#define BITFLAG_INVERT_PROBE_PIN   bit(7)
//

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS           bit(0)
#define SETTINGS_RESTORE_PARAMETERS         bit(1)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL              0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_SIZE				         1024U
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM       6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD      N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28         N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30         N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92      N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings
//
#define IN_SETTINGS_VAL          40
#define OUT_SETTINGS_VAL         56
// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  float backlash[N_AXIS];
  // Tool change mode
  uint8_t tool_change;
  // Position of tool length sensor (only XYZ axis)
  int32_t tls_position[N_AXIS];
  uint8_t tls_valid;
  // coords
  float COORD[SETTING_INDEX_NCOORD+1][N_AXIS];
  float max_feed_rate ;               // max feed rate
  // Remaining Grbl settings
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;     // If max value 255, steppers do not disable.
  uint8_t status_report_mask;         // Mask to indicate desired report data.
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  float spindle_time_run ;
  float spindle_time_stop;
  uint8_t spindle_id ;
  uint8_t flags;                      // Contains default boolean settings
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
  //---------------------------------
  uint16_t inputMask ;
  uint16_t outputMask ;
  In  INPUTS[NUM_INPUTS];
  Out OUTPUTS[NUM_OUTPUTS];
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();
void loadSettings();
void saveSettings();
void storeTlsPosition(void);
void settings_restore(uint8_t restore_flag);
//
uint8_t settings_store_global_setting(uint8_t parameter, float value);
// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);
// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);
// Returns the step pin mask according to Grbl's internal axis numbering
uint8_t get_step_pin_mask(uint8_t i);
// Returns the direction pin mask according to Grbl's internal axis numbering
uint8_t get_direction_pin_mask(uint8_t i);

#endif