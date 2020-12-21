#include "main.h"
#include "cmsis_os.h"
//#include "cnc\tcp.h"
#include "cnc\vfd.h"
//------------------------------------------------------------------------------
uint16_t getCRC16(uint8_t  *data, unsigned int length) {
  uint16_t crc_value = 0xffff;
  while (length--){
    crc_value ^= *data++;
    for (int i = 0; i < 8; ++i){
      if (crc_value & 0x0001){
        crc_value = (crc_value >> 1) ^ 0xa001u;
      }else{
        crc_value = crc_value >> 1;
      }
    }
  }

  return crc_value;
}
//------------------------------------------------------------------------------

osThreadId VFDTaskHandle;

void USART6_IRQHandler(void){


	//HAL_UART_IRQHandler(&hVFD);
}
//------------------------------------------------------------------------------
void vfd_info(){
  uint8_t msg[8];
  // :01 03 0001 0005 F6
  //info_cmd.tx_length = 8;          // length send
  //info_cmd.rx_length = 15;         // length to read

  msg[0] = VFD_ID;
  msg[1] = READ_REGISTER;   		// 0x03 adres odczytu
  msg[2] = HB(0x01);      			// start rejestru HI
  msg[3] = LB(0x01);      			// start rejestru LO
  msg[4] = HB(0x05);      			// ilosc danych HI
  msg[5] = LB(0x05);      			// ilosc danych LO

  uint16_t crc = getCRC16(msg, 6);
  msg[6] = (crc & 0x00FF);
  msg[7] = (crc >> 8);
  // check queue
}

void vfdTask(){

	while(1){
		/**
		HAL_UART_Transmit(&hVFD, (uint8_t*)buffer, sizeof(buffer),100);
		osDelay(250);
		**/
		osDelay(250);//sys.uptime++;
	}
}

//
void initVFD(){
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();


    /**USART6 GPIO Configuration
    PG8     ------> USART6_RTS
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */


}


void vfdStr(char* s){
	//HAL_UART_Transmit_IT(&huart1, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	//HAL_USART_Transmit(&hVFD, (uint8_t*)s, sizeof(s),1);
}
