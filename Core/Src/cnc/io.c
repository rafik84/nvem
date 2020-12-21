#include "cnc/grbl.h"
#include "cmsis_os.h"
//
volatile uint32_t INPUTS;
volatile uint16_t OUTPUTS;
//
typedef struct {
	GPIO_TypeDef* gpio;
	uint16_t pin;
} pin_t;
/*
		IN16	PF15			IN17	PH15
		IN15	PG1				IN18	PH13
		IN14	PE12			IN19	PH14
		IN13	PE14			IN20	PG7
		IN12	PH6				IN21	PG5
		IN11	PH8				IN22	PG3
		IN10	PH10			IN23	PD13
		IN9		PH11			IN24	PD9
		IN8		PH9				IN25	PB15
		IN7		PH7				IN26	PH12
		IN6		PE15			IN27	PB14
		IN5		PE13			IN28	PD8
		IN4		PE11			IN29	PD10
		IN3		PE9----  		IN30	PG2
		IN2		PG0				IN31	PG4
		IN1		PF14			IN32	PG6
 */
const pin_t INPUTS_PINS[] = {
		{GPIOF,GPIO_PIN_14},	//IN 1
		{GPIOG,GPIO_PIN_0},
		{GPIOB,GPIO_PIN_2},	//---------------
		{GPIOE,GPIO_PIN_11},
		{GPIOE,GPIO_PIN_13},
		{GPIOE,GPIO_PIN_15},
		{GPIOH,GPIO_PIN_7},
		{GPIOH,GPIO_PIN_9},
		{GPIOH,GPIO_PIN_11},
		{GPIOH,GPIO_PIN_10},
		{GPIOH,GPIO_PIN_8},
		{GPIOH,GPIO_PIN_6},
		{GPIOE,GPIO_PIN_14},
		{GPIOE,GPIO_PIN_12},
		{GPIOG,GPIO_PIN_1},
		{GPIOF,GPIO_PIN_15},	// IN 16
		//-----------------
		{GPIOH,GPIO_PIN_15},	// IN17
		{GPIOH,GPIO_PIN_13},
		{GPIOH,GPIO_PIN_14},
		{GPIOG,GPIO_PIN_7},
		{GPIOG,GPIO_PIN_5},
		{GPIOG,GPIO_PIN_3},
		{GPIOD,GPIO_PIN_13},
		{GPIOD,GPIO_PIN_9},
		{GPIOB,GPIO_PIN_15},
		{GPIOH,GPIO_PIN_12},
		{GPIOB,GPIO_PIN_14},
		{GPIOD,GPIO_PIN_8},
		{GPIOD,GPIO_PIN_10},
		{GPIOG,GPIO_PIN_2},
		{GPIOG,GPIO_PIN_4},
		{GPIOG,GPIO_PIN_6}		// IN32
};

const pin_t OUTPUS_PINS[] = {
	{GPIOH,GPIO_PIN_5},		// out1
	{GPIOH,GPIO_PIN_2},
	{GPIOF,GPIO_PIN_10},
	{GPIOF,GPIO_PIN_8},
	{GPIOF,GPIO_PIN_9},
	{GPIOH,GPIO_PIN_3},
	{GPIOF,GPIO_PIN_11},
	{GPIOF,GPIO_PIN_13},	// out8
	{GPIOI,GPIO_PIN_1},		// out9
	{GPIOI,GPIO_PIN_3},
	{GPIOC,GPIO_PIN_11},
	{GPIOD,GPIO_PIN_3},
	{GPIOG,GPIO_PIN_9},
	{GPIOG,GPIO_PIN_10},
	{GPIOG,GPIO_PIN_12},
	{GPIOG,GPIO_PIN_15}		// out16
	//PF12 RE1
	//PI6  RE2
	// GPIOB DIR PORT
	//PF step port
};
// cs_pins[E0].port->BSRR = cs_pins[E0].pin‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍‍;
/*
// List element with pointer to GPIO object and pin number.
typedef struct {
  GPIO_PORT *port;
  uint8_t pin
} fan_pin_t;

// List of fan ports and pint
const fan_pin_t fan_pin_list[] = {
  {PORTA, 4},
  {PORTC, 7},
  ...
  {0,0}
};

// Function to set fans based on bitmask.
void set_fan(uint8_t command){
  fan_pin_t *fp = fan_pin;              // Load list pointer
  uint8_t mask = 1;                     // Load mask 00000001
  while(fp->port != 0){               // While not element {0,0}
    if(command & mask){
      fp->port->BSRR = (1<<fp->pin);    // Write to Bit/Set/Reset/Register from current element
    }else{
      fp->port->BRR = (1<<fp->pin);    // Write to Bit/Reset/Register from current element
    }
    mask <<= 1;                         // Left shift mask by 1
    fp++;                               // Next element in list
  }
}

// makros for operating of bits
#define BIT(x) 				(1ul << (x)) 				// makro bitu (max 32 bit)
#define SETBITS(x,y) 		((x) |= (y)) 				// ustaw bity
#define CLEARBITS(x,y) 		((x) &= (~(y))) 			// wyczysc bity

 */
osThreadId IoTaskHandle;

void ioTask(){

	while(1){

		for(uint8_t i=0;i<32;i++){
			if(HAL_GPIO_ReadPin(INPUTS_PINS[i].gpio,INPUTS_PINS[i].pin) == GPIO_PIN_SET){
				INPUTS |= 1ul << i;
			}else{
				INPUTS &= ~(1ul << i);
			}
		}
		osDelay(1);//sys.uptime++;
	}
}

void ioInit(){
	GPIO_InitTypeDef GPIO_Limits = {0};
	GPIO_Limits.Pull = GPIO_PULLUP;
	GPIO_Limits.Mode = GPIO_MODE_INPUT;
	GPIO_Limits.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Limits.Pin = GPIO_PIN_2|GPIO_PIN_15|GPIO_PIN_14;
	HAL_GPIO_Init(GPIOB, &GPIO_Limits);
	GPIO_Limits.Pin = GPIO_PIN_13|GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
	HAL_GPIO_Init(GPIOD, &GPIO_Limits);
	GPIO_Limits.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_11;
	HAL_GPIO_Init(GPIOE, &GPIO_Limits);
	GPIO_Limits.Pin = GPIO_PIN_15|GPIO_PIN_14;
	HAL_GPIO_Init(GPIOF, &GPIO_Limits);
	GPIO_Limits.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
	HAL_GPIO_Init(GPIOG, &GPIO_Limits);
	GPIO_Limits.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	HAL_GPIO_Init(GPIOH, &GPIO_Limits);
	//-----------------------------------------------------------------------------------------------------------------------------------

	GPIO_InitTypeDef GPIO_Outputs= {0};
    GPIO_Outputs.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Outputs.Pull = GPIO_NOPULL;
    GPIO_Outputs.Speed = GPIO_SPEED_FREQ_HIGH;
    //
    GPIO_Outputs.Pin = GPIO_PIN_11;
  	HAL_GPIO_Init(GPIOC, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11, GPIO_PIN_RESET);
    //
    GPIO_Outputs.Pin = GPIO_PIN_3;
  	HAL_GPIO_Init(GPIOD, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET);
    //
    GPIO_Outputs.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13;
  	HAL_GPIO_Init(GPIOF, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);

    GPIO_Outputs.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15;
  	HAL_GPIO_Init(GPIOG, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

    GPIO_Outputs.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
  	HAL_GPIO_Init(GPIOH, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);
    //
    GPIO_Outputs.Pin = GPIO_PIN_1|GPIO_PIN_3; // GPIO_PIN_8 = LED
  	HAL_GPIO_Init(GPIOI, &GPIO_Outputs);
  	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

    osThreadDef(IOsTask, ioTask, osPriorityNormal, 0, 64);
    IoTaskHandle = osThreadCreate(osThread(IOsTask), NULL);
}
//-------------------------------------------
