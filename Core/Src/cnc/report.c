#include "cnc/grbl.h"
//
void report(uint8_t type, uint8_t code){
	uint16_t n =0;

	struct {
		uint8_t type ;
		uint8_t data[300];
	}send ;

	struct{
		uint8_t status ;
		float mco[N_AXIS];
		float wco[N_AXIS];
		uint16_t buffer ;
		uint32_t line ;
		uint8_t rapid ;
		float feed ;
		uint8_t ovr[3];
		//uint32_t upTime ;
		uint32_t IN ;
		uint16_t OUT ;
	}report;

	send.type = type ;
	send.data[0] = 0 ;
	// 269 bytes settings
	int32_t position[N_AXIS]; // Copy current state of the system position variable

	switch(type){
		case REPORT_TYPE_INIT : tcpWrite((char *) &send, n+1);  break ;
		case REPORT_TYPE_REAL :
			report.status = sys.state;
			memcpy(position, sys_position, sizeof(sys_position));
			system_convert_array_steps_to_mpos(report.mco,position);
			system_convert_array_steps_to_wpos(report.wco,position);
			report.buffer = plan_get_block_buffer_available();
			plan_block_t * cur_block = plan_get_current_block();
			report.line = 0 ;
			if (cur_block != NULL) {
				if (cur_block->line_number > 0) {
					report.line = cur_block->line_number;
			    }
			}
			report.rapid = (settings.max_feed_rate / 1000);
			report.feed  = st_get_realtime_rate();
			report.ovr[0] = sys.f_override;
			report.ovr[1] = sys.r_override;
			//report.upTime = sys.uptime ;
			report.IN  = INPUTS ;
			report.OUT = OUTPUTS;
			n = sizeof(report);

			memcpy(&send.data,&report,n);

			tcpWrite((char *) &send, n+1);
		break ;
		case REPORT_STATUS_OK 		:
		case REPORT_STATUS_ERROR	: //send.data[n++] = code ;tcpWrite((char *) &send, n+1); break ;
		case REPORT_STATUS_MSG		: //send.data[n++] = code ;tcpWrite((char *) &send, n+1); break ;
		case REPORT_STATUS_ALARM 	:
		case REPORT_FEEDBACK		: send.data[n++] = code ;tcpWrite((char *) &send, n+2); break ;
		case REPORT_TLS				: break ;
		case REPORT_PROBE			: break ;
		case REPORT_SETTINGS		: n = sizeof(settings);memcpy(&send.data,&settings,n);tcpWrite((char *) &send, n+1);break ;
		//case REPORT_NGC				: break ;
		// gc_state
		case REPORT_GCODE_PARSER	: n = sizeof(gc_state);memcpy(&send.data,&gc_state,n);tcpWrite((char *) &send, n+1);break ;
		case REPORT_UPTIME			: n = sizeof(sys.uptime);memcpy(&send.data,&sys.uptime,n);tcpWrite((char *) &send, n+1);break ;
		//
		default : break ;
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// this is a generic send function that everything should use, so interfaces could be added (Bluetooth, etc)
void grbl_send(char *text){
	tcpWrite(text, sizeof(text));
}
//
void report_tls_param(){

	float print_position[N_AXIS];
	system_convert_array_steps_to_mpos(print_position, settings.tls_position);
	//printInteger(settings.tls_valid);

}

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
void report_status_message(uint8_t status_code){

  switch(status_code) {
    case STATUS_OK	:  	report(REPORT_STATUS_OK,0); break;
    default			:	report(REPORT_STATUS_ERROR,status_code); break;
  }
}
// Prints alarm messages.
void report_alarm_message(uint8_t alarm_code){
	report(REPORT_STATUS_ALARM,alarm_code);
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.4
// OK to send to all clients
void report_feedback_message(uint8_t message_code)  {
	report(REPORT_FEEDBACK,message_code);
}
// Welcome message
void report_init_message(){
	report(REPORT_TYPE_INIT,0);
}
// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void report_grbl_settings() {
	report(REPORT_SETTINGS,0);
}
// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported).
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters(){
	float coord_data[N_AXIS];	// get the machine position and put them into a string and append to the probe report
	system_convert_array_steps_to_mpos(coord_data,sys_probe_position);
	//report_util_axis_values(coord_data);
	//printInteger(sys.probe_succeeded);
	// add the success indicator and add closing characters
}
