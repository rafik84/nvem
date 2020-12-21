#include "cnc/grbl.h"

// Tool
struct{
    uint8_t isFirst ;
    int32_t offset ;
    int32_t referense ;
    float  position[N_AXIS] ;
} Tool ;

void toolInit(){
  //
  Tool.isFirst    = 1 ;
  Tool.offset     = 0 ;
  Tool.referense  = 0 ;

  memset(Tool.position, 0, sizeof(float)*N_AXIS);
  gc_state.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL;
  gc_state.tool_length_offset = 0.0;
}
//
void toolCurrentChange(){
  plan_line_data_t pl_data = {0};
  float position[N_AXIS]   = {0.0};
  if(sys.state == STATE_CHECK_MODE){
    return;
	}
  // Wait until queue is processed
  protocol_buffer_synchronize();
  // Don't move XY. Go to Z 0
	system_convert_array_steps_to_mpos(position, sys_position);
	position[Z_AXIS] = 0.0;
	memcpy(Tool.position, position, sizeof(float)*N_AXIS);
  //System_SetExecStateFlag(EXEC_TOOL_CHANGE);
  pl_data.feed_rate = 0.0;
  pl_data.condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.
  //pl_data.backlash_motion = 0;
  pl_data.spindle_speed = 0;
  pl_data.line_number = gc_state.line_number;

	mc_line(position, &pl_data);
	delay_ms(20);

	spindle_stop();
	// Wait until queue is processed
  protocol_buffer_synchronize();
  // Wait until move is finished
  while(sys.state != STATE_IDLE);
  //sys.state = STATE_TOOL_CHANGE;
  gc_sync_position();
}
//
void toolProbeTLS(){
  plan_line_data_t pl_data = {0};
  float position[N_AXIS] = {0.0};
  uint8_t flags = 0;

  if(sys.state == STATE_CHECK_MODE || settings.tls_valid == TOOL_NO_VALID){
		return;
	}
	// Move to XY position of TLS
	system_convert_array_steps_to_mpos(position, settings.tls_position);
	position[Z_AXIS] = 0.0;
  // Set-up planer
  pl_data.feed_rate = 0.0;
	pl_data.condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.
  //pl_data.backlash_motion = 0;
  pl_data.spindle_speed = 0;
  pl_data.line_number = gc_state.line_number;
  // Move to X/Y position of TLS
	mc_line(position, &pl_data);
  // Move down with offset (for tool)
	position[Z_AXIS] = (settings.tls_position[Z_AXIS] / settings.steps_per_mm[Z_AXIS]) + 90.0;
	mc_line(position, &pl_data);
	// Wait until queue is processed
  protocol_buffer_synchronize();
  // Set up fast probing
  pl_data.feed_rate = 200.0;
	pl_data.condition = 0; // Reset rapid motion condition flag.
  // Probe TLS fast
  position[Z_AXIS] -= 200.0;
  uint8_t ret = mc_probe_cycle(position, &pl_data, flags);
  if(ret != GC_PROBE_FOUND){
    return;// Error
  }
  // Get current position
  system_convert_array_steps_to_mpos(position, sys_position);
  position[Z_AXIS] += 1.8;
  // Move up a little bit for slow probing
  pl_data.feed_rate = 200.0;
  mc_line(position, &pl_data);
  // Probe TLS slow
  pl_data.feed_rate = 15.0;
  position[Z_AXIS] -= 200;
  ret = mc_probe_cycle(position, &pl_data, flags);
  if(ret != GC_PROBE_FOUND){
    // Error
    return;
  }

  if(Tool.isFirst){
    // Save first tool as reference
    Tool.isFirst = 0;
    Tool.referense = sys_probe_position[Z_AXIS];
  }else{
    // Calculate tool offset
    Tool.offset = sys_probe_position[Z_AXIS] - Tool.referense;
    // Apply offset as dynamic tool length offset
    gc_state.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
    gc_state.tool_length_offset = Tool.offset / settings.steps_per_mm[Z_AXIS];
  }

  delay_ms(5);
  // Move Z up
  position[Z_AXIS] = 0.0;
  pl_data.condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.

  mc_line(position, &pl_data);
  // Move back to initial tc position
  mc_line(Tool.position, &pl_data);
  // Wait until queue is processed
  protocol_buffer_synchronize();
  gc_sync_position();
}
