#include "uart.h"
#include <stdint.h>

/*! Define that selects the Usart used in example. */
#define USART USARTC0

/* Default baud rate for SimpleSerial */
#define BAUD 115200

#define TIMEOUT 0
#define BYTE_REC 1

void												init_uart0
	(
   void
   )
   {
/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

        /* Compute baud rate divider based on F_CPU so the UART stays
         * in sync regardless of the system clock frequency.  The
         * hardware only needs the baud select value when the scale is
         * zero, so we can calculate it directly here instead of using a
         * hard-coded constant that assumed a specific clock rate.
         */
        uint16_t bsel = (F_CPU / (16UL * BAUD)) - 1;
        USART_Baudrate_Set(&USART, bsel, 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);
   }

unsigned char									input_ch_w_timeout_0
	(
   char *					 	data,
   volatile unsigned int				timeout
   )
   {
   unsigned int				timeout_counter = 0;

   
   //check if a byte has been received or if the timeout has been exceeded
   while (timeout_counter != timeout)
		{	
		if (USART_IsRXComplete(&USART))
			{
			*data = USART_GetChar(&USART);
			return BYTE_REC;
			}
		timeout_counter++;
		}
		
	return TIMEOUT;
	}
		
char												input_ch_0
	(
   void
   )
   {
   //check if a byte has been received or if the timeout has been exceeded
   while (!USART_IsRXComplete(&USART))
		{
		continue;		
		}		
        return USART_GetChar(&USART);
	}
	
void												output_ch_0
	(
	char							data
	)
	{
        while(!USART_IsTXDataRegisterEmpty(&USART)) {
                ;
        }
        USART_PutChar(&USART, data);
        return;
        }