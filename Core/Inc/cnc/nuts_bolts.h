#ifndef nuts_bolts_h
#define nuts_bolts_h

#define false   0
#define true    1

#define SOME_LARGE_VALUE      1.0E+38

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS    3 // Number of axes
#define X_AXIS    0 // Axis indexing value.
#define Y_AXIS    1
#define Z_AXIS    2

// Conversions
#define MM_PER_INCH           (25.40)
#define INCH_PER_MM           (0.0393701)
#define TIM_CLOCK			  (SystemCoreClock/2)
#define TICKS_PER_MICROSECOND (TIM_CLOCK/1000000) // Different from AVR version

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))
// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask)    (x) |= (mask)
#define bit_false(x,mask)   (x) &= ~(mask)
#define bit_istrue(x,mask)  ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);
// Non-blocking delay function used for general operation and suspend features.
void delay_sec(float seconds, uint8_t mode);
// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);
// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
float hypot_f(float x, float y);
float convert_delta_vector_to_unit_vector(float *vector);
float limit_value_by_axis_maximum(float *max_value, float *unit_vec);
float mapConstrain(float x, float in_min, float in_max, float out_min, float out_max);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);
float constrain_float(float in, float min, float max);
uint16_t crc16(const char* data_p, uint8_t length);
#endif
