#ifndef io_h
#define io_h
#include "grbl.h"

void ioCheckTask(void *pvParameters);
#define bytes_to_u16(MSB,LSB)   ((uint16_t) (MSB << 8) | LSB)
extern volatile uint32_t INPUTS  ;
extern volatile uint16_t OUTPUTS ;
extern volatile uint16_t ACTIONS;
//
#define NUM_INPUTS      32
#define NUM_OUTPUTS     16
//
#define IO_CHANGE       2
#define IO_ACTIVE       1
#define IO_DISABLE      0
//`
#define isActiveIo()    (IO == IO_CHANGE || IO == IO_ACTIVE)
#define isChangeIo()    (IO == IO_CHANGE)
// Inputs
typedef struct{
    uint8_t Action;
} In ;
// Outputs
typedef struct{
    uint8_t Action;
} Out ;
//--------------------------------------------------------------------------------------------
// boolean
#define TRUE          1
#define FALSE         0
// makros for operating of bits
#define _BIT(x)            (1ul << (x))        // makro bitu (max 32 bit)
#define SETBITS(x,y)      ((x) |= (y))        // ustaw bity
#define CLEARBITS(x,y)    ((x) &= (~(y)))     // wyczysc bity

//--------------------------------------------------------------------------------------------
#define INPUT_TYPE_LINE_NC          0
#define INPUT_TYPE_LINE_NO          1
//
#define INPUT_TYPE_NOT_USED         0
#define INPUT_TYPE_HOLD             1
#define INPUT_TYPE_HOME_X           2
#define INPUT_TYPE_LIMIT_X          3
#define INPUT_TYPE_HOME_Y           4
#define INPUT_TYPE_LIMIT_Y          5
//
#define INPUT_TYPE_HOME_Z           6
#define INPUT_TYPE_LIMIT_Z          7
#define INPUT_TYPE_PROBE            8
#define INPUT_TYPE_SPINDLE_START    9
#define INPUT_TYPE_SPINDLE_STOP     10
#define INPUT_TYPE_CYCLE_START      11
#define INPUT_TYPE_DOOR_SAFETY      12
//
#define INPUT_TYPE_HOME_Y_LEFT       13
#define INPUT_TYPE_LIMIT_Y_LEFT      14
#define INPUT_TYPE_HOME_Y_RIGHT      15
#define INPUT_TYPE_LIMIT_Y_RIGHT     16
//------------------------------------ OUTPUTS ----------------------------------------------
#define OUTPUT_TYPE_ACTIVE_HIGH       0
#define OUTPUT_TYPE_ACTIVE_LOW        1

#define OUTPUT_OFF                    0
#define OUTPUT_ON                     1

#define OUTPUT_TYPE_NOT_USED          0
#define OUTPUT_TYPE_SPINDLE           1
#define OUTPUT_TYPE_COOLANT_FLOOD     2
#define OUTPUT_TYPE_COOLANT_MIST      3
#define OUTPUT_TYPE_PROGRAM_START     4
#define OUTPUT_TYPE_PROGRAM_END       5
#define OUTPUT_TYPE_ERROR             6
#define OUTPUT_TYPE_LIMITS            7
#define OUTPUT_TYPE_HOLD              8
#define OUTPUT_TYPE_PROBE             9
#define OUTPUT_TYPE_EXTRACTION        10
#define OUTPUT_TYPE_COMPRESSOR        11
#define OUTPUT_TYPE_VACUUM_PUMP       12
#define OUTPUT_TYPE_VACUUM_SECTION_1  13
#define OUTPUT_TYPE_VACUUM_SECTION_2  14
#define OUTPUT_TYPE_VACUUM_SECTION_3  15
#define OUTPUT_TYPE_VACUUM_SECTION_4  16
#define OUTPUT_TYPE_VACUUM_SECTION_5  17
#define OUTPUT_TYPE_VACUUM_SECTION_6  18
#define OUTPUT_TYPE_VACUUM_SECTION_7  19
#define OUTPUT_TYPE_VACUUM_SECTION_8  20

void ioInit();
uint8_t input_get_state_by_action(uint8_t action);
void output_set_action(uint8_t action,uint8_t state);
uint8_t output_get_action_state(uint8_t action);
#endif
