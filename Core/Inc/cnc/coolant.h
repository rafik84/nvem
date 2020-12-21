#ifndef coolant_control_h
#define coolant_control_h

#define COOLANT_NO_SYNC     false
#define COOLANT_FORCE_SYNC  true

#define COOLANT_STATE_DISABLE   0  // Must be zero
#define COOLANT_STATE_FLOOD     bit(0)
#define COOLANT_STATE_MIST      bit(1)
// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state();
// Immediately disables coolant pins.
void coolant_stop();
// Sets the coolant pins according to state specified.
void coolant_set_state(uint8_t mode);
// G-code parser entry-point for setting coolant states. Checks for and executes additional conditions.
void coolant_sync(uint8_t mode);

#endif
