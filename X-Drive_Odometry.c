/*
 * X_Drive_Odometry.c
 *
 * Created: 03/11/15 11:29:01
 *  Author: marko
 */ 

#include <avr/io.h>
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"
#include "math.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"
#include "Headers/testiranje.h"

volatile signed int
PID_brzina_L,
PID_brzina_R,
motor_sample_L16,
motor_sample_R16,
Pe_brzina_L,
Pe_brzina_R,
Ie_brzina_L,
Ie_brzina_R;

volatile signed long
rastojanje_cilj,
PID_teta;

volatile float
sharp1_value;

int main(void)
{
	int msg_counter = 0;
	int servo_counter = 0;
	//char servo_flag = 0;
	Podesi_Oscilator();					//podesavanje oscilatora
	Podesi_Parametre_Robota();			//podesavanje broja impulsa u krugu
	Podesi_PID_Pojacanja();				//podesavanje pojacanja PID regulatora
	PodesiADC();						//podesavanje AD konvertora
	Podesi_Tajmere();					//podesavanje tajmera
	Podesi_QDEC();						//podesavanje kvadraturnih dekodera
	Podesi_PWM();						//podesavanje PWM signala za motore i servoe
	Podesi_Interapt();					//podesavanje interapt prioriteta
	Podesi_Pinove();					//podesavanje I/O pinova
	Podesi_USART_Komunikaciju();		//podesavanje komunikacije
	inicijalizuj_bluetooth();
	//inicijalizuj_servo_tajmer_20ms();
	//pomeri_servo_1(0);
	
	_delay_ms(1000);					//cekanje da se stabilizuje sistem
	
	////"Hello" poruka - usart_e0
	//USART_TXBuffer_PutByte(&USART_E0_data, 'h');	//e
	//_delay_ms(8000);
	//USART_TXBuffer_PutByte(&USART_E0_data, 'e');	//e
	/*
	_delay_ms(2000);
	USART_TXBuffer_PutByte(&USART_E0_data, 'l');	//e
	_delay_ms(2000);
	USART_TXBuffer_PutByte(&USART_E0_data, 'i');	//e
	_delay_ms(2000);
	USART_TXBuffer_PutByte(&USART_E0_data, 'o');	//e
	_delay_ms(2000);
	*/
	
	//posalji_poruku_bt("hello");
	//posalji_karakter_bt('H');
	//_delay_ms(2000);
	////"Hello" poruka - usart_e1
	//USART_TXBuffer_PutByte(&USART_E1_data, 72);		//H
	////USART_TXBuffer_PutByte(&USART_E1_data, 101);	//e
	//USART_TXBuffer_PutByte(&USART_E1_data, 108);	//l
	//USART_TXBuffer_PutByte(&USART_E1_data, 108);	//l
	//USART_TXBuffer_PutByte(&USART_E1_data, 111);	//o
	//
	////"Hello" poruka - usart_c0
	//USART_TXBuffer_PutByte(&USART_C0_data, 72);		//H
	//USART_TXBuffer_PutByte(&USART_C0_data, 101);	//e
	//USART_TXBuffer_PutByte(&USART_C0_data, 108);	//l
	//USART_TXBuffer_PutByte(&USART_C0_data, 108);	//l
	//USART_TXBuffer_PutByte(&USART_C0_data, 111);	//o
	nuliraj_poziciju_robota();
	//demo_3()
	//zadaj_teta(180,1);
	while(1)
	{
		//ovo je neka izmena u kodu za KEFU!!!!!!
	
	
		//{
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'L');	//O
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'A');	//K
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'Z');	//!
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'A');	//O
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'R');	//K
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, '!');	//!
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'c');	//O
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'A');	//K
			//_delay_ms(200);
			//USART_TXBuffer_PutByte(&USART_E0_data, 'R');	//!
			//_delay_ms(200);
			//while(1);
		//}
		
		
		
		//demo_1();
		//demo_2();
		demo_3();
		//demo_4();
		//zadaj_X_Y(500, 500, 1);
		//if (stigao_flag){
		//zadaj_X_Y(1000,0,2zadaj_X_Y(1000,0,1););
		//}
		//else {
		//zadaj_X_Y(1000,0,1);
		//}
		//zadaj_teta(180,1);
		//zadaj_X_Y(500,200,1);
		//zadaj_teta(0,1);
		//zadaj_X_Y(100,0,2);
		
		//provera asinhronog stop signala za momentalno zaustavljanje robota
		//CheckInputMotorControl();
		
		//Racunanje trenutne pozicije
		if (Rac_tren_poz_sample_counter >= 3){		//9ms   3
			Rac_tren_poz_sample_counter = 0;
			Racunanje_trenutne_pozicije();
		}
		
		//Korekcija pravca i distance prema cilju
		if(Pracenje_Pravca_sample_counter >= 30){	//90ms   30
			msg_counter++;
			servo_counter++;
			Pracenje_Pravca_sample_counter = 0;
			Pracenje_pravca();
		}
		
		//PID regulacija
		if(PID_pozicioni_sample_counter >= 3){		//9ms    3
			PID_pozicioni_sample_counter = 0;
			PID_ugaoni();
			PID_pravolinijski();
			//PID_brzinski se poziva direktno u interaptu sistemskog tajmera TCE1!
		}
		
		//if(msg_counter > 20){
		//	msg_counter = 0;
			//posalji_poruku_bt("hello, world\n");
			//posalji_karakter_bt('a');,
		//}
		
		//if(servo_counter > 60){
			//servo_counter = 0;
			//if(servo_flag){
				//servo_flag = 0;
				//pomeri_servo_1(180);
				//posalji_poruku_bt("servo 180\n\r");
			//} else if(!servo_flag){
				//servo_flag = 1;
				//pomeri_servo_1(0);
				//posalji_poruku_bt("servo 0\n\r");
			//}
			//posalji_karakter_bt('a');
		//}
		
		
	}
}
