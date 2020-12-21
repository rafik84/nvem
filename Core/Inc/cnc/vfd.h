#ifndef __VFD__
#define __VFD__

typedef struct {
    uint8_t  state ;
    uint16_t set_frequency ;
    uint16_t out_frequency ;
    uint16_t out_current ;
    uint16_t bus_voltage ;
    uint16_t speed ;
    uint16_t rpm ;
} t_VFD;

t_VFD VFD ;
void initVFD();
//-----------------------------------------------------------------------------
#define VFD_CALC_RPM(rpm)           (rpm /6)
//-----------------------------------------------------------------------------
#define VFD_CMD_SET_FREQUENCY       1
#define VFD_CMD_OUT_FREQUENCY       2
#define VFD_CMD_OUT_CURRENT         3
#define VFD_CMD_SPINDLE_SPEED       4
#define VFD_CMD_BUS_VOLTAGE         5
//
#define FUNCTION_READ               0x01
#define FUNCTION_WRITE              0x02
#define WRITE_CONTROL_DATA          0x03
#define READ_CONTROL_STATUS         0x04
#define WRITE_FREQ_DATA             0x05

#define READ_REGISTER               0x03
#define WRITE_REGISTER              0x06

#define HB(x)                       (x >> 8)
#define LB(x)                       (x & 0xFF)
//-----------------------------------------------------------------------------
#define VFD_ID                		0x01

#endif
