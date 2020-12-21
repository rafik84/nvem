/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "cnc\grbl.h"
#include "cnc\tcp.h"
/* Private includes ----------------------------------------------------------*/
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.

/* Private variables ---------------------------------------------------------*/
USART_HandleTypeDef husart1;
struct netif gnetif; /* network interface structure */
osThreadId mainTaskHandle ;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue,VarDataTmp = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_Init(void);
void StartDefaultTask(void const * argument);
void mainTask(void const * argument);


void uartStr(char* s){
	HAL_USART_Transmit(&husart1, (uint8_t*)s, strlen(s), 1);
}
//
void ethMonitor(struct netif *netif){
	if (netif_is_up(netif)){
		uartStr("ETH UP\r\n");
	}else{
		uartStr("ETH DOWN\r\n");
	}
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_Init();
  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
 EE_Init();
  //-------------------------------------------------------------------
  // mainTaskHandle
  osThreadDef(mTask, mainTask, osPriorityNormal, 0, 1024);
  mainTaskHandle = osThreadCreate(osThread(mTask), NULL);
  //__enable_irq();
  MX_LWIP_Init();
  ethMonitor(&gnetif);
  uartStr("Start\r\n");
  if(InitializeTelnetServer() == TELNET_OK){

  }
  settings_init();
  stepper_init();
  ioInit();
  memset(sys_position,0,sizeof(sys_position));      	// Clear machine position.
  /* Start scheduler */
  osKernelStart();
  //------------------------------------------------------------------------------------------------------------------

  /* We should never get here as control is now taken by the scheduler */
  while (1){

  }
}
//

void mainTask(void const * argument){
	while(1){
	  // Reset system variables.
	  uint8_t prior_state = sys.state;
	  uint8_t is_homed    = sys.is_homed;
	  uint32_t prevUptime = sys.uptime ;
	  memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
	  //
	  sys.uptime            = prevUptime;
	  sys.state             = prior_state;
	  sys.is_homed          = is_homed;
	  sys.f_override        = DEFAULT_FEED_OVERRIDE;           // Set to 100%
	  sys.r_override        = DEFAULT_RAPID_OVERRIDE;          // Set to 100%
	  sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;  // Set to 100%
	  memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
	  sys_probe_state               = 0;
	  sys_rt_exec_state             = 0;
	  sys_rt_exec_alarm             = 0;
	  sys_rt_exec_motion_override   = 0;
	  sys_rt_exec_accessory_override = 0;

	  gc_init(); // Set g-code parser to default state
	  spindle_init();
	  limits_init();
	  probe_init();
	  plan_reset(); // Clear block buffer and planner variables
	  st_reset();   // Clear stepper subsystem variables
	  plan_sync_position();
	  gc_sync_position();
	  //
	  report_init_message();
	  protocol_main_loop();
	}
}
//
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK){
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
	  onStepperDriverTimer();
  }
  if (htim->Instance == TIM3) {
	  onStepperResetTimer();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
