/*
 * testiranje.c
 *
 * Created: 16/11/15 14:36:10
 *  Author: marko
 */ 

#include <avr/io.h>
#include "Headers/testiranje.h"
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"
#include "math.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"

static char step1 = 0;
static char flag1 = 0;

volatile signed long
X_pos,
Y_pos,
X_cilj,
Y_cilj,
teta,
teta_cilj,
teta_cilj_final;



void nuliraj_poziciju_robota(void)
{
	X_pos = X_cilj = 0;
	Y_pos = Y_cilj = 0;
	teta = teta_cilj = teta_cilj_final = 0;
	smer_zadati = smer_trenutni = 1;
	TCD0.CNT = 0;			//Desni enkoder
	TCD1.CNT = 0;			//Levi enkoder
}

void zadaj_X_Y_teta(signed long x, signed long y, signed long teta_des, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void zadaj_X_Y(signed long x, signed long y, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	smer_zadati = dir;
}

void zadaj_teta(signed long teta_des, unsigned char dir)
{
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void inicijalizuj_bluetooth()
{
	//USARTE1, PE7 -> USARTE1_TX, PE6 -> USARTE1_RX
	PORTE.DIR |= (1 << 7);		//set pin PE7 as output
	PORTE.DIR &= ~(1 << 6);		//set pin PE6 as input
	
	USARTE1.CTRLA |= (1 << 4 | 1 << 2);		//enable receiver and transmitter interrupts at low level
	USARTE1.CTRLB |= (1 << 4 | 1 << 3);		//enable receiver and transmitter
	USARTE1.CTRLC |= (1 << 1 | 1 << 0);		//no parity, 1 stop bit, 8 bit data size
	
	USARTE1.BAUDCTRLA = 12;
	USARTE1.BAUDCTRLB |= (2 << 4);
	
	PMIC.CTRL |= (1 << 0);		//enable low level interrupts
	sei();						//global interrupts enabled
}

void posalji_poruku_bt(char *poruka)
{
	while(*poruka != '\0'){
		posalji_karakter_bt(*poruka);
		poruka++;
	}
}

void posalji_karakter_bt(char c)
{
	USARTE0.DATA = c;
	while(!(USARTE0.STATUS & (1 << 5)));
}

void inicijalizuj_servo_tajmer_20ms()
{
	PORTF.DIR |= (1 << 0);	//servo 1
	
	//Clock source = 32/4 MHz = 8 MHz
	TCF0.CTRLA |= (1 << 2 | 1 << 0);						//Set presclaer to 64, PER 2500 = 20 ms
	TCF0.CTRLB |= (0x0F << 4 | 0x03 << 0);					//Enable Capture/compare A,B,C,D and select single slope PWM
	TCF0.INTCTRLA |= (1 << 0);								//Enable low level overflow interrupt
	TCF0.INTCTRLB |= (1 << 0 | 1 << 2 | 1 << 4 | 1 << 6);	//Enable Capture/compare low level interrupts
	TCF0.PER = 2500;
}

void pomeri_servo_1(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
		res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
		res = 250;
	TCF0.CCA = res;
}

ISR(TCF0_CCA_vect)
{
	PORTF.OUT |= (1 << 0);
}

ISR(TCF0_OVF_vect)
{
	PORTF.OUT &= ~(1 << 0);
}

void demo_1(void)
{
	switch(step1){
		case 1: USART_TXBuffer_PutByte(&USART_E0_data, 132);		//H
			_delay_ms(2000);
			step1 = 2;
		break;
		case 2: USART_TXBuffer_PutByte(&USART_E1_data, 100);		//H
			_delay_ms(2000);
			step1 = 1;
		break;
	}
}

void demo_2(void)
{
	switch(step1){
		case 0:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y(500, 0, 1);
			} 
			else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		break;
		case 1:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_teta(180,1);
			} else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		//case 2:
		//if(flag1 == 0){
			//stigao_flag = 0;
			//flag1 = 1;
			//zadaj_X_Y(0, 0, 1);
			//} else if(stigao_flag == 1){
			//step1++;
			//flag1 = 0;
		//}
		//break;
		//case 2:
		//if(flag1 == 0){
			//stigao_flag = 0;
			//flag1 = 1;
			//zadaj_X_Y_teta(0, 300, 0, 1);
			//} else if(stigao_flag == 1){
			//step1++;
			//flag1 = 0;
		//}
		//break;
		//case 3:
		//if(flag1 == 0){
			//stigao_flag = 0;
			//flag1 = 1;
			//zadaj_X_Y_teta(0, 0, 0, 1);
			//} else if(stigao_flag == 1){
			//step1++;
			//flag1 = 0;
		//}
		break;
		default:
		//do nothing
		break;
	}
}

void demo_3(void)
{
	switch(step1){
		case 0:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y_teta(750, 750, 0, 1);
			} else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		break;
		case 1:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y_teta(0, 750, 0, 1);
			} else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		break;
		case 2:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y_teta(750, 0, 0, 1);
			} else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		break;
		case 3:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y_teta(0, 0, 0, 1);
			} else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
		}
		break;
		default:
		//do nothing
		break;
	}
}