#ifndef stepper_h
#define stepper_h

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 32
#endif

#include "grbl.h"
// Some useful constants.
#define DT_SEGMENT              (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL              0
#define RAMP_CRUISE             1
#define RAMP_DECEL              2
#define RAMP_DECEL_OVERRIDE     3

#define PREP_FLAG_RECALCULATE         bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK  bit(1)
#define PREP_FLAG_PARKING             bit(2)
#define PREP_FLAG_DECEL_OVERRIDE      bit(3)

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
///#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  #define MAX_AMASS_LEVEL 3
  // AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
	// Note ESP32 use F_STEPPER_TIMER rather than the AVR F_CPU
  #define AMASS_LEVEL1 (TIM_CLOCK/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
  #define AMASS_LEVEL2 (TIM_CLOCK/4000) // Over-drives ISR (x4)
  #define AMASS_LEVEL3 (TIM_CLOCK/2000) // Over-drives ISR (x8)

  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
//#endif

//-----------------------------------------------------------------------
void stepper_init();
// Enable steppers, but cycle does not start unless called by motion control or realtime command.
void st_wake_up();
// Immediately disables steppers
void st_go_idle();
// Generate the step and direction port invert masks.
void st_generate_step_dir_invert_masks();
// Reset the stepper subsystem variables
void st_reset();
// Changes the run state of the step segment buffer to execute the special parking motion.
void st_parking_setup_buffer();
// Restores the step segment buffer to the normal run state after a parking motion.
void st_parking_restore_buffer();
// Reloads step segment buffer. Called continuously by realtime execution system.
void st_prep_buffer();
// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters();
// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
float st_get_realtime_rate();
// disable (or enable) steppers via STEPPERS_DISABLE_PIN
void set_stepper_disable(uint8_t disable);
void set_step_pin_on(uint8_t axis, uint8_t isOn);
void set_direction_pin_on(uint8_t axis, uint8_t isOn);
void set_stepper_pins_on(uint8_t onMask);
void set_direction_pins_on(uint8_t onMask);

void Stepper_Timer_WritePeriod(uint64_t alarm_val);
void Stepper_Timer_Start();
void Stepper_Timer_Stop();
void onStepperDriverTimer() ;
void onStepperResetTimer();
//--------------------------------

#define STEP_SET_TIMER    TIM2        //-- Set Timer : Step pulse START - typically rising
#define STEP_SET_IRQ      TIM2_IRQn
#define STEP_RESET_TIMER  TIM3        //-- Reset Timer : Step pulse END - typically falling
#define STEP_RESET_IRQ    TIM3_IRQn

#define Step_Set_EnableIRQ()        HAL_NVIC_EnableIRQ(STEP_SET_IRQ)
#define Step_Set_DisableIRQ()       HAL_NVIC_DisableIRQ(STEP_SET_IRQ)

#define Step_Reset_EnableIRQ()      HAL_NVIC_EnableIRQ(STEP_RESET_IRQ)
#define Step_Reset_DisableIRQ()     HAL_NVIC_DisableIRQ(STEP_RESET_IRQ)

extern const uint16_t step_pin_mask[N_AXIS];
extern const uint16_t direction_pin_mask[N_AXIS];
#endif
