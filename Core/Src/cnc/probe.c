#include "cnc/grbl.h"

// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;

// Probe pin initialization routine.
void probe_init(){
#ifdef PROBE_PIN
	GPIO_InitTypeDef pin_probe;
	pin_probe.Pin = PROBE_PIN; 	// konfigurujemy pin 13
	pin_probe.Mode = GPIO_MODE_INPUT; 	// jako wejœcie
	pin_probe.Pull = GPIO_PULLUP;		// w³¹czamy rezystor podci¹gaj¹cy
	HAL_GPIO_Init(PROBE_GPIO_Port, &pin_probe);	// port GPIOC
    probe_configure_invert_mask(false); // Initialize invert mask.

#endif
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
void probe_configure_invert_mask(uint8_t is_probe_away){
  probe_invert_mask = 0; // Initialize as zero.
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask  ^= PROBE_MASK; }
}

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state() {
#ifdef PROBE_PIN
	return !(HAL_GPIO_ReadPin(PROBE_GPIO_Port,PROBE_PIN) ^ probe_invert_mask);
#else
	return false;
#endif
}
// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor(){
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
