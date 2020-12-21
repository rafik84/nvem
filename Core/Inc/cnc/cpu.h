#ifndef cpu_map_h
#define cpu_map_h
#define F_CPU 	SystemCoreClock
//----------------------------------------------------------------------------------------------------------------------------------
#define STEP_X_Pin GPIO_PIN_15
#define STEP_Y_Pin GPIO_PIN_13
#define STEP_Z_Pin GPIO_PIN_11
#define STEP_A_Pin GPIO_PIN_9
#define STEP_B_Pin GPIO_PIN_7
#define STEP_C_Pin GPIO_PIN_5

#define DIR_X_Pin GPIO_PIN_14
#define DIR_Y_Pin GPIO_PIN_12
#define DIR_Z_Pin GPIO_PIN_10
#define DIR_A_Pin GPIO_PIN_8
#define DIR_B_Pin GPIO_PIN_6
#define DIR_C_Pin GPIO_PIN_4

#define STEPPERS_PORT_STEP GPIOE
#define STEPPERS_PORT_DIR  GPIOE

#define PROBE_PIN  		GPIO_PIN_9
#define PROBE_GPIO_Port GPIOD
//
#define LIMITS_PORT	GPIOD
// D
#define LIMIT_PIN_3		GPIO_PIN_10
#define LIMIT_PIN_4		GPIO_PIN_13
#define LIMIT_PIN_5		GPIO_PIN_14
#define LIMIT_PIN_6		GPIO_PIN_15
// C
#define LIMIT_PIN_7		GPIO_PIN_6
#define LIMIT_PIN_8		GPIO_PIN_7
#define LIMIT_PIN_9		GPIO_PIN_8
#define LIMIT_PIN_10	GPIO_PIN_9
// A
#define LIMIT_PIN_11	GPIO_PIN_11
#define LIMIT_PIN_12	GPIO_PIN_12

//#define LIM_GPIO_Port GPIOB
//#define LIM_MASK        (LIM_X_Pin | LIM_Y_Pin | LIM_Z_Pin) // All limit pins
//#define DIR_GPIO_Port GPIOA
#define DIRECTION_MASK  (DIR_X_Pin | DIR_Y_Pin | DIR_Z_Pin) // All direction pins
#define STEP_MASK       (STEP_X_Pin| STEP_Y_Pin| STEP_Z_Pin) // All step pins

#define PORTPINDEF uint16_t
#define PROBE_MASK       1 // don't change


#define HAL_READ_OUTPUT_GPIO(GPIOx)			((uint16_t)GPIOx->IDR)
#define HAL_WRITE_OUTPUT_GPIO(GPIOx,Data)	(GPIOx->ODR = Data)
//
#define HAL_WRITE_PORT(GPIOx,PIN)			(GPIOx->ODR = PIN)
#define HAL_READ_PORT(GPIOx)				(GPIOx->IDR)

/*
void GPIO_WritePort(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR = GPIO_Pin;
}

uint16_t GPIO_ReadPort(GPIO_TypeDef* GPIOx)
{
	return GPIOx->IDR;
}
 */
#endif
