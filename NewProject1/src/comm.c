/* found somewhere */

#include <stm32f4xx.h>
#include "comm.h"

#define USARTx USART2 /* nucleo is USART2 op USB */

int comm_test(void)
{
    if (RESET == USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)) {
        return 0;
    }
    else {
        return 1;
    }
}

unsigned char comm_get(void)
{
	while(RESET == USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)) { ; }
	return (unsigned char)USART_ReceiveData(USARTx);
}

void comm_put(unsigned char d)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET) { ; }
	USART_SendData(USARTx, (uint16_t)d);
}

#if 0
/* not used in this project */
void comm_init(void)
{
	USART_InitTypeDef USART_InitStructure;



	/* USART1 configuration ------------------------------------------------------*/
	/* USART configured as follow:
	 - BaudRate = 19200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	*/
	USART_StructInit(&USART_InitStructure); // init de struct eerst
	// dan parameters erin
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USARTx */
	USART_Init(USARTx, &USART_InitStructure);

	/* Enable the USART1 */
	USART_Cmd(USARTx, ENABLE);
}
#endif
