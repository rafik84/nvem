#include "cnc/grbl.h"
#include "stm32f2xx.h"

TIM_HandleTypeDef tim2 ;
TIM_HandleTypeDef tim3;


// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint16_t direction_bits;
  #ifdef ENABLE_DUAL_AXIS
    uint16_t direction_bits_dual;
  #endif
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;           // Number of step events to be executed for this segment
  uint32_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
  uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y,
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;          // Stores out_bits output to complete the step pulse delay
  #endif
  uint8_t execute_step;         // Flags step execution for each interrupt.
  uint16_t step_pulse_time;     // Step pulse reset time after step rise
  uint16_t step_outbits;        // The next stepping-bits to be output
  uint16_t dir_outbits;
  #ifdef ENABLE_DUAL_AXIS
    uint8_t step_outbits_dual;
  #endif
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif
  uint16_t step_count;        // Steps remaining in line segment motion
  uint8_t exec_block_index;   // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;     // Pointer to the block data for the segment being executed
  segment_t *exec_segment;    // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;
// Step and direction port invert masks.
static uint16_t step_port_invert_mask;
static uint16_t dir_port_invert_mask;

const uint16_t step_pin_mask[N_AXIS] = 		{
		STEP_X_Pin,
		STEP_Y_Pin,
		STEP_Z_Pin
};
//
const uint16_t direction_pin_mask[N_AXIS] = {
		DIR_X_Pin,
		DIR_Y_Pin,
		DIR_Z_Pin
};

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;
// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped
// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t recalculate_flag;
  float dt_remainder;
  float steps_remaining;
  float step_per_mm;
  float req_mm_increment;
  #ifdef PARKING_ENABLE
    uint8_t last_st_block_index;
    float last_steps_remaining;
    float last_step_per_mm;
    float last_dt_remainder;
  #endif

  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)

} st_prep_t;
//
static st_prep_t prep;
//
void onStepperResetTimer(){
	HAL_TIM_Base_Stop(&tim3);
	HAL_WRITE_OUTPUT_GPIO(STEPPERS_PORT_STEP,(HAL_READ_OUTPUT_GPIO(STEPPERS_PORT_STEP) & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
}
//

void onStepperDriverTimer()  {
  // The busy-flag is used to avoid reentering this interrupt
  if (busy) { return; }
  // Set the direction pins a couple of nanoseconds before we step the steppers

  HAL_WRITE_OUTPUT_GPIO(STEPPERS_PORT_DIR, ((HAL_READ_OUTPUT_GPIO(STEPPERS_PORT_DIR) & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK))) ;

  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
  #else  // Normal operation
    HAL_WRITE_OUTPUT_GPIO(STEPPERS_PORT_STEP,((HAL_READ_OUTPUT_GPIO(STEPPERS_PORT_STEP) & ~STEP_MASK) | st.step_outbits));
  #endif

  TIM3->CNT = 0;
  TIM3->ARR = (uint32_t)(st.step_pulse_time -1); //
  //__HAL_TIM_ENABLE_IT(&tim3, TIM_IT_UPDATE );
  HAL_TIM_Base_Start(&tim3);

  busy = true;
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];
      // Initialize step segment timing per step and load number of steps to execute.
      //__HAL_TIM_SET_AUTORELOAD(&tim2,st.exec_segment->cycles_per_tick - 1);
      TIM2->CNT = 0;
      TIM2->ARR = (uint32_t)st.exec_segment->cycles_per_tick - 1;
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        // Initialize Bresenham line and distance counters
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ settings.dir_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif

    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }
  }

  // Check probing state.
  if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }
  // Reset step out bits.
  st.step_outbits = 0;
  #ifdef ENABLE_DUAL_AXIS
     st.step_outbits_dual = 0;
  #endif
  // Execute step displacement profile by Bresenham line algorithm
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif
  // ------------------------------ X ------------------------------
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= STEP_X_Pin;
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & DIR_X_Pin) { sys_position[X_AXIS]--; }else { sys_position[X_AXIS]++; }
  }
  //
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif
  // ------------------------------ Y ------------------------------
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= STEP_Y_Pin;
    //
    #if defined(ENABLE_DUAL_AXIS) && (DUAL_AXIS_SELECT == Y_AXIS)
      st.step_outbits_dual = (1<<DUAL_STEP_BIT);
    #endif

    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & DIR_Y_Pin) { sys_position[Y_AXIS]--; }else { sys_position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif
  // ------------------------------ Z ------------------------------
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= STEP_Z_Pin;
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & DIR_Z_Pin) { sys_position[Z_AXIS]--; } else { sys_position[Z_AXIS]++; }
  }

  // During a homing cycle, lock out and prevent desired axes from moving.
  if (sys.state == STATE_HOMING) {
    st.step_outbits &= sys.homing_axis_lock;

    #ifdef ENABLE_DUAL_AXIS
      st.step_outbits_dual &= sys.homing_axis_lock_dual;
    #endif
  }

  st.step_count--; // Decrement step events count
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }
  st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask

  busy = false;
}

void TIM2_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim2);
}

void TIM3_IRQHandler(void){
  HAL_TIM_IRQHandler(&tim3);
}

//
void stepper_init(){

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	tim2.Instance = TIM2;

	tim2.Init.Period = 0;
	tim2.Init.Prescaler = 0;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&tim2);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&tim2, &sClockSourceConfig);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&tim2, &sMasterConfig);

	//----------------------------------------------------------
	tim3.Instance = TIM3;
	tim3.Init.Prescaler = 0;
	tim3.Init.Period = 0;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&tim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&tim3, &sClockSourceConfig) ;

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&tim3, &sMasterConfig);

	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

	HAL_TIM_Base_Start_IT(&tim2);
	HAL_TIM_Base_Start_IT(&tim3);

    // init GPIO // |DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PIN Steps
    GPIO_InitStruct.Pin = STEP_X_Pin|STEP_Y_Pin|STEP_Z_Pin|STEP_A_Pin|STEP_B_Pin|STEP_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(STEPPERS_PORT_STEP, &GPIO_InitStruct);
  	HAL_GPIO_WritePin(STEPPERS_PORT_STEP, STEP_X_Pin|STEP_Y_Pin|STEP_Z_Pin|STEP_A_Pin|STEP_B_Pin|STEP_C_Pin , GPIO_PIN_RESET);
  	// PIN Dir
    GPIO_InitStruct.Pin = DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin|DIR_B_Pin|DIR_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  	HAL_GPIO_Init(STEPPERS_PORT_DIR, &GPIO_InitStruct);
  	HAL_GPIO_WritePin(STEPPERS_PORT_DIR, DIR_X_Pin |DIR_Y_Pin|DIR_Z_Pin|DIR_A_Pin|DIR_B_Pin|DIR_C_Pin , GPIO_PIN_RESET);
}

// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up(){
  // Initialize stepper output bits to ensure first ISR call does not step.
  st.step_outbits = step_port_invert_mask;
  // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
  #ifdef STEP_PULSE_DELAY

  #else // Normal operation
    // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
    st.step_pulse_time = (settings.pulse_microseconds*TICKS_PER_MICROSECOND);
  #endif
    // Enable Stepper Driver Interrupt
    TIM2->ARR = (uint32_t)st.exec_segment->cycles_per_tick - 1;
    TIM3->ARR = (uint32_t)(st.step_pulse_time-1); //

    // Set the Autoreload value
  #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIM2->PSC = st.exec_segment->prescaler;
  #endif
    TIM2->EGR = TIM_EGR_UG;
    TIM3->EGR = TIM_EGR_UG; // ++

    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// Reset and clear stepper subsystem variables
void st_reset(){
  // Initialize stepper driver idle state.
  st_go_idle();
  // Initialize stepper algorithm variables.
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // Initialize direction bits to default.

  STEPPERS_PORT_STEP->ODR = ((STEPPERS_PORT_STEP->ODR & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK));
  STEPPERS_PORT_DIR->ODR = ((STEPPERS_PORT_DIR->ODR & ~DIRECTION_MASK) | (dir_port_invert_mask & DIRECTION_MASK));
}

//------------------------------------------------------------------------------------------------------------
// Stepper shutdown
void st_go_idle(){
 // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn); // ++

  busy = false;
}
 // Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters(){
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load and check active velocity profile.
  }
}

#ifdef PARKING_ENABLE
  // Changes the run state of the step segment buffer to execute the special parking motion.
  void st_parking_setup_buffer(){
    // Store step execution data of partially completed block, if necessary.
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      prep.last_st_block_index = prep.st_block_index;
      prep.last_steps_remaining = prep.steps_remaining;
      prep.last_dt_remainder = prep.dt_remainder;
      prep.last_step_per_mm = prep.step_per_mm;
    }
    // Set flags to execute a parking motion
    prep.recalculate_flag |= PREP_FLAG_PARKING;
    prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
    pl_block = NULL; // Always reset parking motion to reload new block.
  }


  // Restores the step segment buffer to the normal run state after a parking motion.
  void st_parking_restore_buffer(){
    // Restore step execution data and flags of partially completed block, if necessary.
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      st_prep_block = &st_block_buffer[prep.last_st_block_index];
      prep.st_block_index = prep.last_st_block_index;
      prep.steps_remaining = prep.last_steps_remaining;
      prep.dt_remainder = prep.last_dt_remainder;
      prep.step_per_mm = prep.last_step_per_mm;
      prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
      prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; // Recompute this value.
    } else {
      prep.recalculate_flag = false;
    }
    pl_block = NULL; // Set to reload next block.
  }
#endif

// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
void st_generate_step_dir_invert_masks(){

	uint8_t idx;
	step_port_invert_mask = 0;
	dir_port_invert_mask = 0;
	for (idx=0; idx<N_AXIS; idx++) {
		if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= step_pin_mask[idx]; }
		if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= direction_pin_mask[idx]; }
	}
}

// Increments the step segment buffer block data ring buffer.
static uint8_t st_next_block_index(uint8_t block_index){
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}

/* Prepares step segment buffer. Continuously called from main program.

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it.
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer(){
  // Block step prep buffer, while in a suspend state and there is no suspend motion to execute.
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }
  // Check if we need to fill the buffer.
  while (segment_buffer_tail != segment_next_head) {

    // Determine if we need to load a new planner block or if the block needs to be recomputed.
    if (pl_block == NULL) {
      // Query planner for a queued block
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
        pl_block = plan_get_system_motion_block();
      }else {
        pl_block = plan_get_current_block();
      }
      // No planner blocks. Exit.
      if (pl_block == NULL) { return; }
      // Check if we need to only recompute the velocity profile or load a new block.
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) {
            prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
          }else {
            prep.recalculate_flag = false;
          }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {
        // Load the Bresenham stepping data for the block.
        prep.st_block_index = st_next_block_index(prep.st_block_index);
        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
/**
        #ifdef ENABLE_DUAL_AXIS
          #if (DUAL_AXIS_SELECT == X_AXIS)
            if (st_prep_block->direction_bits & (1<<X_DIRECTION_BIT)) {
          #elif (DUAL_AXIS_SELECT == Y_AXIS)
            if (st_prep_block->direction_bits & (1<<Y_DIRECTION_BIT)) {
          #endif
            st_prep_block->direction_bits_dual = (1<<DUAL_DIRECTION_BIT);
          }  else {
            st_prep_block->direction_bits_dual = 0;
          }
        #endif
**/
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) {
            st_prep_block->steps[idx] = pl_block->steps[idx];
          }
          st_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          for (idx=0; idx<N_AXIS; idx++) {
            st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL;
          }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif
        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = (float)pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0; // Reset for new segment block

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // New block loaded mid-hold. Override planner block entry speed to enforce deceleration.
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr);
        }
      }

      /* ---------------------------------------------------------------------------------
			 Compute the velocity profile of a new planner block based on its entry and exit
			 speeds, or recompute the profile of a partially-completed planner block if the
			 planner has updated it. For a commanded forced-deceleration, such as from a feed
			 hold, override the planner velocities and decelerate to the target exit speed.
	 */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.

      float inv_2_accel = 0.5/pl_block->acceleration;
      // [Forced Deceleration to Zero Velocity]
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
    	  // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
    	  // the planner block profile, enforcing a deceleration to zero speed.
    	  prep.ramp_type = RAMP_DECEL;
    	  // Compute decelerate distance relative to end of block.
    	  float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
    	  if (decel_dist < 0.0) {
    		  // Deceleration through entire planner block. End of feed hold is not in this block.
    		  prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
    	  } else {
    		  prep.mm_complete = decel_dist; // End of feed hold.
    		  prep.exit_speed = 0.0;
    	  }
      } else {
    	  // [Normal Operation]
    	  // Compute or recompute velocity profile parameters of the prepped planner block.
    	  prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
    	  prep.accelerate_until = pl_block->millimeters;

    	  float exit_speed_sqr;
    	  float nominal_speed;
    	  if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
    		  prep.exit_speed = exit_speed_sqr = 0.0; // Enforce stop at end of system motion.
    	  } else {
    		  exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
    		  prep.exit_speed = sqrt(exit_speed_sqr);
    	  }

    	  nominal_speed = plan_compute_profile_nominal_speed(pl_block);
    	  float nominal_speed_sqr = nominal_speed*nominal_speed;
    	  float intersect_distance = 0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

    	  if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.
    		  prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
    		  if (prep.accelerate_until <= 0.0) { // Deceleration-only.
    			  prep.ramp_type = RAMP_DECEL;
    			  // prep.decelerate_after = pl_block->millimeters;
    			  // prep.maximum_speed = prep.current_speed;

    			  // Compute override block exit speed since it doesn't match the planner exit speed.
    			  prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
    			  prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // Flag to load next block as deceleration override.

    			  // TODO: Determine correct handling of parameters in deceleration-only.
    			  // Can be tricky since entry speed will be current speed, as in feed holds.
    			  // Also, look into near-zero speed handling issues with this.

    		  } else {
    			  // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
    			  prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
    			  prep.maximum_speed = nominal_speed;
    			  prep.ramp_type = RAMP_DECEL_OVERRIDE;
    		  }
    	  } else if (intersect_distance > 0.0) {
    		  if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
					// NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
					prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
					if (prep.decelerate_after < intersect_distance) { // Trapezoid type
						prep.maximum_speed = nominal_speed;
						if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
							// Cruise-deceleration or cruise-only type.
							prep.ramp_type = RAMP_CRUISE;
						} else {
							// Full-trapezoid or acceleration-cruise types
							prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
						}
					} else { // Triangle type
						prep.accelerate_until = intersect_distance;
						prep.decelerate_after = intersect_distance;
						prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
					}
				} else { // Deceleration-only type
					prep.ramp_type = RAMP_DECEL;
					// prep.decelerate_after = pl_block->millimeters;
					// prep.maximum_speed = prep.current_speed;
				}
    	  	  } else { // Acceleration-only type
				prep.accelerate_until = 0.0;
				// prep.decelerate_after = 0.0;
				prep.maximum_speed = prep.exit_speed;
    	  	  }
		}
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];
    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create
      a full segment based on the current ramp conditions. If the segment time is incomplete
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if
      an incomplete segment terminates at the end of the velocity profile, the segment is
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at
      the end of planner block (typical) or mid-block at the end of a forced deceleration,
      such as from a feed hold.
    */

    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0;            // Initialize segment time

    float time_var = dt_max; // Time worker variable
    float mm_var;            // mm-Distance worker variable
    float speed_var;         // Speed worker variable
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
          mm_var = time_var*(prep.current_speed - 0.5*speed_var);
          mm_remaining -= mm_var;
          if ((mm_remaining < prep.accelerate_until) || (mm_var <= 0)) {
            // Cruise or cruise-deceleration types only for deceleration override.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // Mid-deceleration override ramp.
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;

          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; } else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.
            prep.current_speed += speed_var;
          }
          //grbl_sendf(CLIENT_SERIAL,"'%4.8f' : '%4.8f : '%4.8f' , \r\n",mm_remaining , prep.accelerate_until ,dt);
          break;
        case RAMP_CRUISE:
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise.
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          }
          // Otherwise, at end of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // Add computed ramp time to total segment time.


      if (dt < dt_max) {
        time_var = dt_max - dt; // **Incomplete** At ramp junction.
      } else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }

      //grbl_sendf(CLIENT_SERIAL,"'%4.8f' : '%4.8f : '%4.8f' , \r\n",mm_remaining , prep.accelerate_until ,dt);
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions.
       However, since floats have only 7.2 significant digits, long moves with extremely
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    float step_dist_remaining    = prep.step_per_mm*mm_remaining;       // Convert mm_remaining to steps
    float n_steps_remaining      = ceil(step_dist_remaining);           // Round-up current steps remaining
    float last_n_steps_remaining = ceil(prep.steps_remaining);          // Round-up last steps remaining
    prep_segment->n_step         = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.

    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS
        // requires full steps to execute. So, just bail.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Segment not generated, but current step data still retained.
      }
    }
    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse
    // Compute CPU cycles per step for the prepped segment.
    //uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)
    uint32_t cycles = (uint32_t)ceil((TICKS_PER_MICROSECOND * 1000000) *inv_rate * 60); // (cycles/step)

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) {
    	  prep_segment->amass_level = 0;
      }else {
        if (cycles < AMASS_LEVEL2) {
          prep_segment->amass_level = 1;
        }else if (cycles < AMASS_LEVEL3) {
          prep_segment->amass_level = 2;
        }else {
          prep_segment->amass_level = 3;
        }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }

      if (cycles < (1UL << 16)) {
        prep_segment->cycles_per_tick = cycles; // < 65536 (4.1ms @ 16MHz)
      } else {
        prep_segment->cycles_per_tick = 0xffff;  // Just set the slowest speed possible.
      }
    #else
      prep_segment->cycles_per_tick = cycles;
    #endif
    // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }
    // Update the appropriate planner and segment data.
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;
    // Check for exit conditions and flag to load next planner block.
    if (mm_remaining == prep.mm_complete) {
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // Set pointer to indicate check and load next planner block.
        plan_discard_current_block();
      }
    }

  }
}
// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds.
float st_get_realtime_rate(){
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
    return prep.current_speed;
  }
  return 0.0f;
}


void set_stepper_disable(uint8_t isOn){
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { isOn = !isOn; } // Apply pin invert.
  if(settings.stepper_idle_lock_time == 255 && isOn == true){
    return ;
  }
}
