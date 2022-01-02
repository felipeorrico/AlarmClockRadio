#include "uart_ms.h"

#ifndef MASTER

//SLAVE MODE
void send_tmp(){
	
}


ISR(USART0_RX_vect){
	static  uint8_t  usart_idx;
	usart_rx_char = UDR0;              //get character
	usart_str_array[usart_idx++]=usart_rx_char;  //store in array 
	if(usart_rx_char == '\0'){
		if (usart_str_array[0] == 'T'){
			send_tmp();
		}
		usart_idx =0; 
	}
}

#endif
#ifdef MASTER

ISR(USART0_RX_vect){
	static  uint8_t  usart_idx;
	usart_rx_char = UDR0;              //get character
	usart_str_array[usart_idx++]=usart_rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(usart_rx_char == '\0'){
		usart_rcv_rdy=1; 
		usart_str_array[--usart_idx]  = (' ');     //clear the count field
		usart_str_array[usart_idx+1]  = (' ');
		usart_str_array[usart_idx+2]  = (' ');
		usart_idx =0;  
	}
}

#endif

void request_data(){
	uart_putc('T');
	uart_putc('\0');
}