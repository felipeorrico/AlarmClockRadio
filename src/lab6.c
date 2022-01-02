// lab6.c 
// F. Orrico Scognamiglio
// 12.03.2021

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base and enable of the bargraph.
//  PORTB bits 0-3 go to the bar graph and bit 1 also goes to the encoder clock
//	PORTE pin 6 goes to the Shift / Load of the encoder board
// 	Currently, all pins of PORTA and PORTB are in use.

//	PORTF bit 0 is Photocell ADC
//  PORTD bit 4 is Alarm PWM
//  PORTE bit 3 is Volume Control

// Counter 0 used to measure seconds
// Counter 1 used to Alarm PWM
// Counter 2 used to Dimming PWM
// Counter 3 used to Volume PWM

//Pressing SW0 toggles set_clock option to change time (minute based). This resets seconds to 0
//Pressing SW1 toggles set_alarm option to set the alarm time (does not enable alarm).
//set_clock and set_alarm are never enabled at the same time. if set_alarm is on, and set_clock is turned on, set_alarm is turned off
//and vice versa.
//Pressing SW2 arms or disarms the alarm
//Pressing SW3 enables 10s snooze

// PORTD bit 0 is SCL
// PORTD bit 1 is SDA

// PORTE bit 0 is RX
// PORTE bit 1 is TX

#define F_CPU 16000000UL // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define NO_LEAD_ZERO
//#define Trevors_Board
#define MASTER

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

//for port B, define the selector values and pwm
#define SEL_0 0b00010000 //Pin B4 active (A)
#define SEL_1 0b00100000 //Pin B5 active (B)
#define SEL_2 0b01000000 //Pin B6 active (C)
#define PWM_0 0b10000000 //Pin B7 PWM control pin (R30)

//For port A, define values for each display segments (one-hot encoded for simplicity)
#define SEG_A 0b00000001 //Pin A0 (A)
#define SEG_B 0b00000010 //Pin A1 (B)
#define SEG_C 0b00000100 //Pin A2 (C)
#define SEG_D 0b00001000 //Pin A3 (D)
#define SEG_E 0b00010000 //Pin A4 (E)
#define SEG_F 0b00100000 //Pin A5 (F)
#define SEG_G 0b01000000 //Pin A6 (G)
#define SEG_DP 0b10000000 //Pin A7 (DP)

//now define segment values for each decimal ((Remember to ~ values before assigning)
#define _zero (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F)
#define _one (SEG_B | SEG_C)
#define _two (SEG_A | SEG_B | SEG_G | SEG_E | SEG_D)
#define _three (SEG_A | SEG_B | SEG_G | SEG_C | SEG_D)
#define _four (SEG_F | SEG_G | SEG_B | SEG_C)
#define _five (SEG_A | SEG_F | SEG_G | SEG_C | SEG_D)
#define _six (SEG_A | SEG_F | SEG_E | SEG_D | SEG_C | SEG_G)
#define _seven (SEG_A | SEG_B | SEG_C)
#define _eight (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G)
#define _nine (SEG_A | SEG_B | SEG_G | SEG_F | SEG_C)
#define _a 0x88
#define _b 0x83
#define _c 0xC6
#define _d 0xA1
#define _e 0x86
#define _f 0x8E
#define _colon (SEG_A | SEG_B) 

//Definitions for Lab4
#define alarm_bit 0x10
#define photocell_bit 0x01
#define volume_bit 0x80
#define colon_off 16
#define colon_on 99
#define alarm_freq 17500


//Global Variables

//holds current displayed count
uint16_t 			encoder_count;
uint8_t 			segsum_hex_en;

//holds what number each display will display
uint8_t 			segment_data[5]; 

//holds 8-bit encoder value
uint8_t 			encoder_data;

//holds the state of increase/decrease of the encoders
uint8_t 			inc[2] = {0,0};
#define inc_2 inc[0]
#define inc_4 inc[1]

//holds the output to the bar graph
uint8_t 			bargraph_o;


//time information
volatile typedef struct time {
	uint8_t volatile seconds;
	uint8_t volatile minutes;
	uint8_t volatile hours;
} Time;

Time volatile 		alarm_time;
Time volatile 		clock_time;
uint8_t volatile 	clock_counter = 0;
uint8_t volatile 	colon_en = colon_off; //(off at 16, on at 99)
uint8_t volatile 	alarm_armed = FALSE;
uint8_t volatile 	alarm_triggered = FALSE;
uint16_t volatile 	snooze_counter = 0;
uint8_t volatile 	snooze_en = FALSE;

//changing settings
uint8_t volatile 	setting_alarm = FALSE;
uint8_t volatile 	setting_time = FALSE;
uint8_t volatile	setting_radio = FALSE;

//adc
uint16_t volatile 	adc_voltage;

//music
uint16_t 			fight_song[] = {19600,18500,15550,11000,20750,16500,20750,13050};
uint8_t volatile 	music_idx = 0;
#define max_music_idx 8

//TWI LM73 TEMP
extern uint8_t 		lm73_wr_buf[2];
extern uint8_t 		lm73_rd_buf[2];
volatile uint16_t 	lm73_tmp_in = 0;
char				raw_tmp_val[16] = "99";
char    			lcd_string_array[16];  //holds a string to refresh the LCD

//USART 
uint8_t 		  	usart_idx = 0;
volatile uint8_t  	usart_rcv_rdy;
char              	usart_rx_char; 
char      			usart_str_array[16] = "-1";  //holds string to send to lcd

//RADIO
extern enum radio_band{FM, AM};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;
volatile uint8_t alarm_mode = 0; //Mode 0 = Alarm, 1 = Radio
uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t current_fm_freq = 10630;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

uint8_t volatile volume = 100;
uint8_t volatile radio_en = FALSE;
uint8_t volatile freq_change = FALSE;
uint8_t volatile req_update_ss = FALSE;


/*TODO*/
//
//	Radio Controls (Tune, EN/_EN) - OK
//	Radio Alarm Mode - OK
//  Volume change on radio enabled or alarm vs disabled - OK
//	Volume Knob - OK
//

void toggle_radio(){
	if (!radio_en){
		//back to default volume
		set_property(0x4000, 0x003F);
		radio_en = TRUE;
	} 
	else {
		//set volume to 0
		set_property(0x4000, 0x0000);
		radio_en = FALSE;
	}
}

//******************************************************************************
//                            set_PORTB
// Sets the derired selector to PORTB (1-4) or enables T-BUFFER (default)
// Input: 1-4 (from right to left, thousands are 4) select digit, default enable tristate buffer
//
void set_PORTB (uint8_t value) {
	switch (value) {
		case 5:
			PORTB |= SEL_2; //enable select 2
			PORTB &= ~(SEL_0 | SEL_1); //disable select 0 and 1
			break;
		case 4:
			PORTB |= SEL_0 | SEL_1;
			PORTB &= ~(SEL_2);
			break;
		case 3:
		  PORTB |= SEL_1;
		  PORTB &= ~(SEL_2 | SEL_0);
			break;
		case 2:
			PORTB |= SEL_0;
			PORTB &= ~(SEL_1 | SEL_2);
			break;
		case 1:
			PORTB &= ~(SEL_0 | SEL_1 | SEL_2); 
			break;
		default: // T-Buffer enable
			#ifndef Trevors_Board
			PORTB |= SEL_0 | SEL_1 | SEL_2;
			#endif
			#ifdef Trevors_Board
			PORTB |= SEL_0 | SEL_2;
			PORTB &= ~(SEL_1);
			#endif
			break;
	}
}

/***********************************************************************/
//                              update_lcd                             
//Updates the LCD display to show important information
void update_lcd(){
	//clear_display();
	cursor_home();
	string2lcd("A:");
	if (snooze_en){
		string2lcd("S");
	}
	else if (alarm_armed && !alarm_triggered){
		string2lcd("Y"); //armed
	}
	else if (alarm_armed && alarm_triggered){
		string2lcd("R"); //ring
	}
	else {
		string2lcd("N"); //not armed
	}
	string2lcd(" AM:");
	if (alarm_mode == 1){ //Radio Mode
		string2lcd("R");
	} 
	else { //Song Mode
		string2lcd("S");
	}
}

uint8_t to_F(uint8_t C){
	return (9*C)/5 + 32;
}

void update_tmp_lcd(){
	static volatile uint8_t far = FALSE;
	line2_col1();
	string2lcd(" ");
	if (far) {
		uint8_t num = to_F(atoi(lcd_string_array));
		itoa(num, lcd_string_array, 10);
		string2lcd(lcd_string_array);
		string2lcd("F");
		far = FALSE;
	} 
	else {string2lcd(lcd_string_array); string2lcd("C"); far = TRUE;}
	string2lcd(" ");
	if (usart_rcv_rdy){
		if (far){
			uint8_t num = to_F(atoi(usart_str_array));
			itoa(num, usart_str_array, 10);
			string2lcd(usart_str_array);
			string2lcd("F");
		}
		else {
			string2lcd(usart_str_array);
			string2lcd("C");
			usart_rcv_rdy = 0;
		}
	} else {
		string2lcd("NC ");
	}
	if (radio_en){
		//static last_freq = 0;
		req_update_ss = TRUE;
		//if (last_freq != current_fm_freq){
			string2lcd(" SS:");
			int2lcd((int)si4734_tune_status_buf[4]);
			//last_freq = current_fm_freq;
		//}
	} else {
		string2lcd("      ");
	}
}

//******************************************************************************
//                            set_PORTA_7seg
//	Sets the segments within a segment (number 0-9)
//	input: number to be displayed (0-9), if other number, segment is cleared
//  
void set_PORTA_7seg_rad(uint8_t value) {
	uint8_t dot = 0;
	if (setting_radio){
		dot = SEG_DP;
	}
	
	switch(value) {
		case 0:
			PORTA = ~(_zero | dot);
			break;
		case 1:
			PORTA = ~(_one | dot);
			break;
		case 2:
			PORTA = ~(_two | dot);
			break;
		case 3:
			PORTA = ~(_three | dot);
			break;	
		case 4:
			PORTA = ~(_four | dot);
			break;
		case 5:
			PORTA = ~(_five | dot);
			break;
		case 6:
			PORTA = ~(_six | dot);
			break;
		case 7:
			PORTA = ~(_seven | dot);
			break;
		case 8:
			PORTA = ~(_eight | dot);
			break;
		case 9:
			PORTA = ~(_nine | dot);
			break;
		case 10:
			PORTA = _a;
			break;
		case 11:
			PORTA = _b;
			break;
		case 12:
			PORTA = _c;
			break;
		case 13:
			PORTA = _d;
			break;
		case 14:
			PORTA = _e;
			break;
		case 15:
			PORTA = _f;
			break;
		case 99:
			PORTA = ~(_colon);
			break;
		default: //clear
			PORTA = 0xff;
			break;
	}
}

void set_PORTA_7seg(uint8_t value) {
	
	switch(value) {
		case 0:
			PORTA = ~(_zero);
			break;
		case 1:
			PORTA = ~(_one);
			break;
		case 2:
			PORTA = ~(_two);
			break;
		case 3:
			PORTA = ~(_three);
			break;	
		case 4:
			PORTA = ~(_four);
			break;
		case 5:
			PORTA = ~(_five);
			break;
		case 6:
			PORTA = ~(_six);
			break;
		case 7:
			PORTA = ~(_seven);
			break;
		case 8:
			PORTA = ~(_eight);
			break;
		case 9:
			PORTA = ~(_nine);
			break;
		case 10:
			PORTA = _a;
			break;
		case 11:
			PORTA = _b;
			break;
		case 12:
			PORTA = _c;
			break;
		case 13:
			PORTA = _d;
			break;
		case 14:
			PORTA = _e;
			break;
		case 15:
			PORTA = _f;
			break;
		case 99:
			PORTA = ~(_colon);
			break;
		default: //clear
			PORTA = 0xff;
			break;
	}
}

//***********************************************************************************
//                                   segsum_radio                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum_radio(uint16_t sum) {
	//determine how many digits there are 

	//break up decimal sum into 4 digit-segments
	uint16_t val = sum/10;
	uint8_t thousands	=	val/1000;
	uint8_t hundreds 	=	(val%1000)/100;
	uint8_t tens 		=	(val%100)/10;
	uint8_t ones		=	(val%10);
	//blank out leading zero digits 
	segment_data[0] = ones;
	segment_data[1] = tens;
	segment_data[2] = 16;
	segment_data[3] = hundreds;
	segment_data[4] = thousands;
	#ifdef NO_LEAD_ZERO
	if (val < 1000) segment_data[4] = 16;
	if (val < 100)	segment_data[3] = 16;
	if (val < 10)	segment_data[1] = 16;
	#endif
	
  //now move data to right place for misplaced colon position
}//segment_sum


//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
	//determine how many digits there are 

	//break up decimal sum into 4 digit-segments
	uint8_t thousands	=	sum/1000;
	uint8_t hundreds 	=	(sum%1000)/100;
	uint8_t tens 		=	(sum%100)/10;
	uint8_t ones		=	(sum%10);
	//blank out leading zero digits 
	segment_data[0] = ones;
	segment_data[1] = tens;
	segment_data[2] = 0;
	segment_data[3] = hundreds;
	segment_data[4] = thousands;
	#ifdef NO_LEAD_ZERO
	if (sum < 1000) segment_data[4] = 16;
	if (sum < 100)	segment_data[3] = 16;
	if (sum < 10)	segment_data[1] = 16;
	#endif
	
  //now move data to right place for misplaced colon position
}//segment_sum

//***********************************************************************************
//                                   segment_sum_hex                                
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit hex
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum_hex(uint16_t sum) {
	//determine how many digits there are 

	//break up decimal sum into 4 digit-segments
	uint8_t low_8 = sum & 0xff;
	uint8_t high_8 = (sum>>8);
	
	uint8_t low_4_1 = low_8 & 0x0f;
	uint8_t low_4_2 = (low_8>>4);
	uint8_t high_4_1 = high_8 & 0x0f;
	uint8_t high_4_2 = (high_8>>4);
	//blank out leading zero digits 
	segment_data[0] = low_4_1;
	segment_data[1] = low_4_2;
	segment_data[2] = 0;
	segment_data[3] = high_4_1;
	segment_data[4] = high_4_2;
		#ifdef NO_LEAD_ZERO
	segment_data[4] = 16;
	if (high_4_2 == 0)	segment_data[4] = 16;
	if (high_4_1 == 0)	segment_data[3] = 16;
	if (low_4_2 == 0)	segment_data[1] = 16;
	#endif
}//segment_sum

//***********************************************************************************
//                                   segment_sum_clock                                    
void segsum_clock() {
	if (!setting_alarm && !setting_radio){
		//determine how many digits there are 
		uint8_t sum = clock_time.minutes;
		//break up decimal sum into 4 digit-segments
		uint8_t tens 		=	sum/10;
		uint8_t ones		=	sum%10;
		//blank out leading zero digits 
		segment_data[0] = ones;
		segment_data[1] = tens;
		segment_data[2] = colon_en;
		sum = clock_time.hours;
		//break up decimal sum into 4 digit-segments
		tens 		=	sum/10;
		ones		=	sum%10;
		segment_data[3] = ones;
		segment_data[4] = tens;
	} 
	else if(setting_alarm) {
		//determine how many digits there are 
		uint8_t sum = alarm_time.minutes;
		//break up decimal sum into 4 digit-segments
		uint8_t tens 		=	sum/10;
		uint8_t ones		=	sum%10;
		//blank out leading zero digits 
		segment_data[0] = ones;
		segment_data[1] = tens;
		segment_data[2] = colon_en;
		sum = alarm_time.hours;
		//break up decimal sum into 4 digit-segments
		tens 		=	sum/10;
		ones		=	sum%10;
		segment_data[3] = ones;
		segment_data[4] = tens;
	} else if (setting_radio){
		segsum_radio(current_fm_freq);
	}
	
  //now move data to right place for misplaced colon position
}//segment_sum


//******************************************************************************
//                            debounce_switch                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t debounce_switch(uint8_t pressed_button_idx){
	static uint16_t state[8] = {0}; //holds present state
	state[pressed_button_idx] = ((state[pressed_button_idx] << 1) | (! bit_is_clear(PINA, pressed_button_idx)) | 0xE000);
	if (state[pressed_button_idx] == 0xF000) return 1;
	return 0;
}

/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
  DDRB  |=   0x07; //Turn on SS, MOSI, SCLK
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);  //SPI, MAster mode, low idle, sample on leading edge
  SPSR  |=   (1 << SPI2X); // double speed operation
 }//spi_init
 
//***********************************************************************************

/*************************************************************************/
//                           ch_enc_cnt
//	increases or decreases the count and checks if it rolls over or under
/*************************************************************************/
void ch_enc_cnt(uint8_t direction){ //1 up, 0 down
	switch(direction){
		case 1://up
			if (inc_2 == 1 && inc_4 == 1){}
			else if (inc_2 == 1){
				encoder_count += 2;
			} else if (inc_4){
				encoder_count += 4;
			}
			else{
				encoder_count += 1;
			}
			if (encoder_count > 1023)
				encoder_count -= 1024;
			break;
		case 0: //down
			if (inc_2 == 1 && inc_4 == 1){}
			else if (inc_2 == 1){
				if (encoder_count < 2){
					encoder_count = 1022 + encoder_count;
				}
				else 
					encoder_count -= 2;
			} else if (inc_4){
				if (encoder_count < 4){
					encoder_count = 1020 + encoder_count;
				}
				else 
					encoder_count -= 4;
			}
			else {
				if (encoder_count < 1){
					encoder_count = 1023 + encoder_count;
				}
				else 
					encoder_count -= 1;
			}
			break;
	}
}

/*************************************************************************/
//                           set_time_
//	increases (1) or decreases (0) clock time.
/*************************************************************************/
void set_time_(uint8_t inc_dec_n){
	if (setting_time){
		if (inc_dec_n == 1) {
			clock_time.seconds = 0;
			clock_time.minutes++;
			if (clock_time.minutes == 60){
				clock_time.minutes = 0;
				clock_time.hours++;
				if (clock_time.hours >= 24){
					clock_time.hours = 0;
				}
			}
		}	
		else {
			clock_time.seconds = 0;
			if (clock_time.minutes == 0){
				clock_time.minutes = 59;
				if (clock_time.hours == 0){
					clock_time.hours = 23;
				}
				else 
					clock_time.hours--;
			}
			else 
				clock_time.minutes--;
		}
	}
}

/*************************************************************************/
//                           set_alarm_
//	increases (1) or decreases (0) alarm time.
/*************************************************************************/
void set_alarm_(uint8_t inc_dec_n){
	if (setting_alarm){
		if (inc_dec_n == 1) {
			alarm_time.seconds = 0;
			alarm_time.minutes++;
			if (alarm_time.minutes == 60){
				alarm_time.minutes = 0;
				alarm_time.hours++;
				if (alarm_time.hours >= 24){
					alarm_time.hours = 0;
				}
			}
		}	
		else {
			alarm_time.seconds = 0;
			if (alarm_time.minutes == 0){
				alarm_time.minutes = 59;
				if (alarm_time.hours == 0){
					alarm_time.hours = 23;
				}
				else 
					alarm_time.hours--;
			}
			else 
				alarm_time.minutes--;
		}
	}
}

void set_radio(uint8_t inc_dec_n){
	if (setting_radio){
		if (inc_dec_n == 1) { //increase
			current_fm_freq +=20;
			if (current_fm_freq > 10790){
				current_fm_freq = 8790;
			}
			//if (current_fm_freq <= 10790)
			//	current_fm_freq +=20;
		} 
		else { //decrease
			current_fm_freq -= 20;
			if (current_fm_freq < 8790){
				current_fm_freq = 10790;
			}
			//if (current_am_freq >= 8830)
			//	current_fm_freq -= 20;
		}
		freq_change = TRUE;
	}
}

/*************************************************************************/
//                           chk_encoder
//	checks if the encoder increased or decreased and calls increase or decrease
/*************************************************************************/
void chk_encoder (void){
	static uint8_t prev_L = 0;
	static uint8_t prev_R = 0;
	static uint8_t curr_L = 0;
	static uint8_t curr_R = 0;
	
	//update previous values
	prev_L = curr_L;
	prev_R = curr_R;
	
	//update current values
	curr_L = (encoder_data & 0x03);
	curr_R = ((encoder_data & 0x0c) >> 2);
	
	if (curr_L != prev_L){ //check state left encoder
		//check state count up
		if (((prev_L == 0x01) && (curr_L == 0x03)) || ((prev_L == 0x03) && (curr_L == 0x02)) || ((prev_L == 0x02) && (curr_L == 0x00)) || ((prev_L == 0x00) && (curr_L == 0x01))){
			//ch_enc_cnt(1);
			set_time_(1);
			set_alarm_(1);
			set_radio(1);
		}
		//check state count down
		if (((prev_L == 0x02) && (curr_L == 0x03)) || ((prev_L == 0x00) && (curr_L == 0x01)) || ((prev_L == 0x01) && (curr_L == 0x03)) || ((prev_L == 0x03) && (curr_L == 0x02))){
			//rotate left, count down!
			//ch_enc_cnt(0);
			set_time_(0);
			set_alarm_(0);
			set_radio(0);
		}
	}
	
	if (curr_R != prev_R){ //check state right encoder
		if (((prev_R == 0x01) && (curr_R == 0x03)) || ((prev_R == 0x03) && (curr_R == 0x02)) || ((prev_R == 0x02) && (curr_R == 0x00)) || ((prev_R == 0x00) && (curr_R == 0x01))){
			//rotate right, count up!
			//ch_enc_cnt(1);
			if (volume <= 80) {
				volume += 10;
				/*if (!bargraph_o){
				bargraph_o = 0b1000000;
				}
				bargraph_o |= bargraph_o >> 1;*/
			}
		}
		if (((prev_R == 0x02) && (curr_R == 0x03)) || ((prev_R == 0x00) && (curr_R == 0x01)) || ((prev_R == 0x01) && (curr_R == 0x03)) || ((prev_R == 0x03) && (curr_R == 0x02))){
			//rotate left, count down!
			//ch_enc_cnt(0);
			if (volume >= 10) {
				volume -= 10;
				//bargraph_o = bargraph_o << 1;
			}
		}
		OCR3A = (volume * 4.6);
	}	
}

/*************************************************************************/
//                           rd_wr_SPI
//	reads and writes data to and from the SPI interface
/*************************************************************************/
void rd_wr_SPI(){
	//enable SH/LD_n
	PORTE |=   0x40;
	
	SPDR = bargraph_o;
	encoder_data = SPDR;
	
	while (bit_is_clear(SPSR, SPIF)){};
	
	//disable SH/LD_n
	PORTE &= ~(0x40);
	
	PORTB |= 0x01;     //HC595 output reg - rising edge...
	PORTB &= ~(0x01);    //and falling edge
}

/*************************************************************************/
//                           chk_sw
//	reads switch input
/*************************************************************************/
void chk_sw(){
	
	DDRA = 0x00;
	set_PORTA_7seg(16);

	//enable tristate buffer for pushbutton switches
	set_PORTB(0);
	
	//check sw0 or sw1 for mode change
	_delay_us(50);
	if(debounce_switch(0)){ //setting clock time
		if (setting_time){setting_time = FALSE;}
		else {
			setting_time = TRUE;
			if (setting_alarm)
				setting_alarm = FALSE;
			if (setting_radio)
				setting_radio = FALSE;
		}
	}
	if (debounce_switch(1)){ //setting alarm time
		if (setting_alarm == 1){setting_alarm = 0;}
		else {
			setting_alarm = 1;
			if (setting_time)
				setting_time = FALSE;
			if (alarm_armed)
				alarm_armed = FALSE;
			if (setting_radio)
				setting_radio = FALSE;
			//if alarm not set, set it to current time for simplicity
			if ((alarm_time.hours == 0 && alarm_time.minutes == 0) && !alarm_armed){
				alarm_time.hours = clock_time.hours;
				alarm_time.minutes = clock_time.minutes;
			}
		}
	}
	if (debounce_switch(2)){ //will toggle the alarm armed or not
		if (alarm_armed == 1){
			alarm_armed = 0; 
			update_lcd();
			snooze_en = FALSE;
			snooze_counter = 0;
			if (alarm_triggered){alarm_triggered = FALSE;}
			if (radio_en) {
				toggle_radio();
			}
		}
		else {
			alarm_armed = 1;
			if (setting_alarm){setting_alarm = FALSE;}
			if (alarm_triggered){alarm_triggered = FALSE;}
			//update lcd panel with latest information
			update_lcd();
		}
	}
	if (debounce_switch(3)){ //will hit snooze
		if (alarm_triggered && alarm_armed){
			snooze_en = TRUE;
		}
	}
	if (debounce_switch(4)){ //toggle alarm mode between radio and tune
		if (!alarm_triggered) {
			if (alarm_mode) alarm_mode = FALSE;
			else alarm_mode = TRUE;	
		}
		update_lcd();
	}
	if (debounce_switch(5)){ //toggle radio on/off
		if (!alarm_armed){ //do nothing if alarm is armed (i.e.) cannot turn radio on.
			toggle_radio();
		}
	}
	if (debounce_switch(6)){
		if (!setting_radio){
			setting_radio = TRUE;
			setting_alarm = FALSE;
			setting_time = FALSE;
		} else {
			setting_radio = FALSE;
		}
	}
	if (debounce_switch(7)){
		//hardware reset of Si4734
		PORTE &= ~(1<<PE7); 
		DDRE  |= 0x80;     
		PORTE |=  (1<<PE2); 
		_delay_us(200);        
		PORTE &= ~(1<<PE2);
		_delay_us(30);      
		DDRE  &= ~(0x80);  
		fm_pwr_up(); //powerup  fm radio
		fm_tune_freq();
	}
	
	static uint8_t prev_vol = 0;
	if (prev_vol != volume){
		bargraph_o = 0xFF;
		bargraph_o = bargraph_o >> (volume/10);
		bargraph_o ^= 0xFF;
		prev_vol = volume;
	}
	
	//disable tristate buffer for pushbutton switches
	set_PORTB(1);
	
	//make PORTA an output
	DDRA = 0xFF;
	
}

/*************************************************************************/
//                           setup_adc
//	sets up ADC and initializes Timer 2
/*************************************************************************/
void setup_adc(){
	ADMUX  = (1<<REFS0); //single-ended input, PORTF bit 1, right adjusted, 10 bits
                                         //reference is AVCC

	ADCSRA = (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //ADC enabled, don't start yet, single shot mode 
                                         //division factor is 128 (125khz)
					 
	TCCR2 = (1<<WGM21) | (1<<WGM20) /*Fast PWM*/ | (1<<COM21) /*clear on compare match*/ | (1<<CS21) | (1<<CS20) /*64 prescale*/;
	OCR2 = 255/5;
}

/*************************************************************************/
//                           update_brightness
//	reads ADC and updates display brightness
/*************************************************************************/
void update_brightness(){
	ADCSRA |=  (1<<ADSC);//poke the ADSC bit and start conversion

	while(bit_is_clear(ADCSRA, ADIF));//spin while interrupt flag not set

	ADCSRA |=  (1<<ADIF);     //its done, clear flag by writing a one 

	uint16_t adc_result = ADC;
	
	div_t voltage = div(adc_result, 205);
	//4.8 = max light, 0.16 = min light
	uint8_t voltage_v = voltage.quot;
	
	if (voltage_v < 1)
		voltage_v = 1;
	
	//100 = max brightness, 25 = min brightness
	uint8_t voltage_p = voltage_v*25;
	
	uint8_t brightness_val = (uint8_t) 255-(255*voltage_p/100);
	
	OCR2 = brightness_val;//brightness_val;
}

/***********************************************************************/
//                              tcnt0_init                             
//Initializes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void){
	ASSR   |=  (1 << AS0);  //ext osc TOSC
	TIMSK  |=  (1 << TOIE0);  //enable timer/counter0 overflow interrupt
	TCCR0 |= (1<<CS00); //normal mode no prescaler
}

/*************************************************************************/
//                           timer/counter0 ISR   
//	timer counter 0 overflow interrupt about 128hz, close to 128.5hz. 
//	used 128 for simplicity, should be close enough. 
/*************************************************************************/
ISR(TIMER0_OVF_vect){	
	clock_counter++;
	if (clock_counter == 128){ //about 1hz
		if (setting_time == FALSE){
			clock_time.seconds++;
			if (clock_time.seconds == 60){
				clock_time.seconds = 0;
				clock_time.minutes++;
				if (clock_time.minutes == 60){
					clock_time.minutes = 0;
					clock_time.hours++;
					if (clock_time.hours >= 24){
						clock_time.hours = 0;
					}
				}
			}	
			if (clock_time.seconds == alarm_time.seconds && clock_time.minutes == alarm_time.minutes && clock_time.hours == alarm_time.hours){
				if (alarm_armed){
					//music_idx = 0;
					alarm_triggered = TRUE;
					//if (alarm_mode == 1 && !radio_en)
						//toggle_radio();
					update_lcd();
				}
			}
			
		}
		//Blink colon every second
		if (colon_en == colon_off){
			colon_en = colon_on;
		}
		else {
			colon_en = colon_off;
		}
	}
	if (clock_counter%64 == 0) { //about 2hz
	
	}
	if (clock_counter%32 == 0) { //about 4hz
		if (alarm_triggered){
			if (music_idx > max_music_idx)
				music_idx = 0;
			OCR1A = fight_song[music_idx];
			music_idx++;
		} else {
			music_idx = 0;
		}
		
	}
	if (clock_counter%16 == 0) { //about 8hz
		twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);
		lm73_tmp_in = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
		lm73_tmp_in = (lm73_tmp_in << 8); //shift it into upper byte 
		lm73_tmp_in |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
		itoa(lm73_tmp_in >> 7, lcd_string_array, 10); //can make faster by using int3lcd?
		strcpy(raw_tmp_val, lcd_string_array);
	}
	if (clock_counter%8 == 0){ //about 16hz
		
	}
	if (clock_counter%4 == 0){ //about 32hz
	
	}
	if (clock_counter%2 == 0){ //about 64hz
		if (snooze_en){ //check for snooze;
			snooze_counter++;
			if (snooze_counter == 640){ //10 second snooze done
				snooze_counter = 0; //reset counter
				snooze_en = FALSE; //disable snooze
				alarm_triggered = TRUE; //trigger alarm again
				//if(!radio_en && alarm_mode == 1)
				//	toggle_radio();
				update_lcd();
			}
			else {
				if (snooze_counter == 1 || !snooze_counter)
					update_lcd();
				alarm_triggered = FALSE; //turn alarm off
			}
		}
	}
	if (clock_counter%1 == 0){ //about 128hz
		//read and write from SPI
		rd_wr_SPI();
		//check encoder status
		chk_encoder();
		
	}
	if (clock_counter == 128){
		clock_counter = 0;
		update_tmp_lcd();
		uart_putc('T');
	} //clear counter on 1hz
}

/*************************************************************************/
//                           init_alarm_ctrl
//	initializes Timer 1 and 3 for alarm and volume
/*************************************************************************/
void init_alarm_ctrl(){
	//TCNT3 (volume control)
	TCCR3A |= (1<<COM3A1) /*clear on compare match*/ | (1<<WGM31) /*Mode 6*/;
	TCCR3B |= (1<<WGM32) /*Mode 6*/ | (1<<CS31) /*Prescaler 8*/;
	TCCR3C |= 0x00;
	//set volume  (256 is 50% duty cycle)
	//set min volume to be 4.6 (1%) and max to be 460 (100%)
	//460 is about 90% of max volume and 4.6 about 0.9%
	OCR3A = (volume * 4.6);

	//TCNT1 (Alarm PWM)
	TCCR1A |= 0x00;
	TCCR1B |= (1<<WGM12) | (1<<CS10);
	TCCR1C |= 0x00;
	TIMSK  |= (1<<OCIE1A);
	//set frequency
	OCR1A = alarm_freq;
}

ISR(TIMER1_COMPA_vect){
	if (alarm_triggered && alarm_armed && alarm_mode == 0){
		PORTD ^= 0x10; //Toggle audio output bit
	}
	else {
		PORTD &= 0b11101111; //keep alarm off
		//alarm_triggered = FALSE;
	}
}

ISR(USART0_RX_vect){
	usart_rx_char = UDR0;
#ifndef MASTER
	//static  uint8_t  usart_idx2;
	//usart_str_array[usart_idx++]=usart_rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(usart_rx_char == 'T'){
		//usart_rcv_rdy=1; 
		//usart_str_array[--usart_idx]  = (' ');     //clear the count field
		//usart_idx =0;  
		uart_putc(raw_tmp_val[0]);
		uart_putc(raw_tmp_val[1]);
		uart_putc('\0');
		return;
	}
#endif
	if (usart_rx_char == 'T'){
		return;
	}
	static  uint8_t  usart_idx;
	//usart_rx_char = UDR0;              //get character
	usart_str_array[usart_idx++]=usart_rx_char;  //store in array 
	//if entire string has arrived, set flag, reset index
	if(usart_rx_char == '\0'){
		usart_rcv_rdy=1; 
		//usart_str_array[--usart_idx]  = (' ');     //clear the count field
		usart_idx =0;  
	}
}

ISR(INT7_vect){STC_interrupt = TRUE;}

void radio_init(){
	DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
	PORTE |= 0x04; //radio reset is on at powerup (active high)

	EICRB |= (1<<ISC71) | (1<ISC70);
	EIMSK |= (1<<INT7);
	
	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); 
	DDRE  |= 0x80;     
	PORTE |=  (1<<PE2); 
	_delay_us(200);        
	PORTE &= ~(1<<PE2);
	_delay_us(30);      
	DDRE  &= ~(0x80);   
}

/*************************************************************************/
//                           init_values
//	initializes necessary values
/*************************************************************************/
void init_values(){
	
	alarm_time.minutes = 0;
	alarm_time.seconds = 0;
	alarm_time.hours = 0;
	clock_time.seconds = 0;
	clock_time.hours = 0;
	clock_time.minutes = 0;
	clock_counter = 0;
	
	//set port bits 4-7 B as outputs
	DDRB |= 0b11110000;
	//port E as output
	DDRE |= 0xff;
	PORTE |= 0x04;
	
	//port F as input
	DDRF = 0x00;
	PORTF = 0x00;
	
	//audio output
	DDRD |= 0x10;
	
	//initialize globals
	encoder_count = 0;
	encoder_data = 0;
	bargraph_o = 0;
	inc_2 = 0;
	inc_4 = 0;
	segsum_hex_en = 0;

	//init counters, lcd, and volume/buzzer
	tcnt0_init();  //initalize counter timer zero
	uart_init();
	spi_init();    //initalize SPI port
	lcd_init();
	init_twi();
	
	string2lcd("Starting TMP");
	/*CHANGED HERE, CHECK IF TMP STILL WORKS - Seems to work just fine...*/
	lm73_wr_buf[0] = 0x00; //load lm73_wr_buf[0] with temperature pointer address
	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);  //start the TWI write process
	_delay_ms(2);    //wait for the xfer to finish
	
	clear_display();
	
	string2lcd("Starting Alarm");
	init_alarm_ctrl();
	setup_adc();
	
	clear_display();
	
	string2lcd("Starting Radio");
	radio_init();
	
	clear_display();
	
	rd_wr_SPI();
}

void main()
{
	init_values();
	sei();         //enable interrupts before entering loop
	
	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); 
	DDRE  |= 0x80;     
	PORTE |=  (1<<PE2); 
	_delay_us(200);        
	PORTE &= ~(1<<PE2);
	_delay_us(30);      
	DDRE  &= ~(0x80);   
	
	string2lcd("Powering Radio U");
	fm_pwr_up(); //powerup  fm radio
	clear_display();
	string2lcd("Tunning Radio");
	fm_tune_freq(); //tune radio to frequency
	while(STC_interrupt == FALSE){}
	fm_rsq_status();
	clear_display();
	string2lcd("Set Volume");
	set_property(0x4000, 0x0000);
	
	clear_display();
	update_lcd();
	
	while(1){
		//check switch press
		chk_sw();
		
		//update clock time
		segsum_clock();
		
		//update brightness
		update_brightness();
		
		//update 7-seg displays (takes about 1.5ms)
		for (uint8_t i = 0; i < 5; i++){
			//update digit to display
			if (setting_radio && i == 1)
				set_PORTA_7seg_rad(segment_data[i]);
			else 
				set_PORTA_7seg(segment_data[i]);
			set_PORTB(i+1); //send PORTB the digit to display
			_delay_us(300);
		}
		
		if (freq_change){
			STC_interrupt = FALSE;
			fm_tune_freq();
			freq_change = FALSE;
		}
		
		if (req_update_ss && STC_interrupt){
			fm_rsq_status();
			req_update_ss = FALSE;
		}
		
		if (!alarm_triggered && !radio_en && OCR3A != 0){
			OCR3A = 0;
		}
		else if ((radio_en || alarm_triggered) && OCR3A == 0){
			OCR3A = (volume * 4.6);
		}

		if (alarm_triggered && alarm_mode == 1 && alarm_armed){
			if (!radio_en)
				toggle_radio();
		}
		else if (alarm_armed && alarm_mode == 1 && !alarm_triggered){
			if (radio_en)
				toggle_radio();
		}
	}//while
}//main
