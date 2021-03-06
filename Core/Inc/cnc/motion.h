#ifndef motion_control_h
#define motion_control_h

#include "grbl.h"

// System motion commands must have a line number of zero.
#define HOMING_CYCLE_LINE_NUMBER    0
#define PARKING_MOTION_LINE_NUMBER  0

#define HOMING_CYCLE_ALL  0  // Must be zero.
#define HOMING_CYCLE_X    bit(X_AXIS)
#define HOMING_CYCLE_Y    bit(Y_AXIS)
#define HOMING_CYCLE_Z    bit(Z_AXIS)
#define HOMING_CYCLE_A    bit(A_AXIS)
#define HOMING_CYCLE_B    bit(B_AXIS)
#define HOMING_CYCLE_C    bit(C_AXIS)


// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
void mc_line(float *target, plan_line_data_t *pl_data);
// Execute an arc in offset mode format. position == current xyz, target == target xyz,
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, is_clockwise_arc boolean. Used
// for vector transformation direction.
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);
// Dwell for a specific number of seconds
void mc_dwell(float seconds);
// Perform homing cycle to locate machine zero. Requires limit switches.
void mc_homing_cycle(uint8_t cycle_mask);
// Perform tool length probe cycle. Requires probe switch.
uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags);
// Plans and executes the single special motion case for parking. Independent of main planner buffer.
void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data);
// Performs system reset. If in motion state, kills all motion and sets system alarm.
void mc_reset();

#endif
