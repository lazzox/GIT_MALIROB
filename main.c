/*
 * main.c
 *
 * Poslednje_izmene: 28/03/2016 01:47:41
 * Autor: AXIS team 
 
 Izmene:
 -Namesten baudrate za LCD
 -Dodate funkcije za sendMsg/sendChar
 -Dodata PGM_Mode funkcija dodat taster
 -Uklonjeno checkmotors..() funkcija
 -Proveriti brzinskiPid koji se poziva u interaptu sta se tamo nalazi
 --tamo ne sme biti nikakvih komplikovanih operacija ili definisanja promenljivih
 -Izbacena inicijalizacija bluetooth-a
 -PID I i D dejstvo kao da nemaju uticaja na kretanje :)
 -PID apdejtovan
 -Izmenjen baudrate za displej sa 9600 na 125000
 
 
 Potrebne izmene:
 -Promeniti baudrate
 -Promeniti funkciju kada robot treba da stigne u tacku koja je iza njega, 
 --ali da se ne krece unazad nego prednjom stranom, trenutno pocne da pravi veliki
 --polukrug i na kraju stigne gde treba, ali bolje je da se prvo okrene za 180 
 --stepeni pa onda da ide pravolinijski.
 -Provaliti kako robot bira u kom smeru ce se okretati
 
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
#include "Headers/funkcije.h"

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

volatile unsigned int PRG_flag = 0;

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
	//inicijalizuj_servo_tajmer_20ms();
	//pomeri_servo_1(0);

	_delay_ms(1000);			//mora bar 300 delay zbog delaya u PGM_Mode kojie je 300ms 
	nuliraj_poziciju_robota();
	
	//Cekaj cinc ovde u while();
	 //zadaj_teta(-45,2);
	
	while(1)
	{
		//CHECK PGM MODE - Uvek mora biti ispred svega!
		while(PGM_Mode()){
			set_direct_out = 1;
			PID_brzina_L = 0;
			PID_brzina_R = 0;
			if (!PRG_flag){
				sendMsg("PGM_Mode");
				PRG_flag = 1;
			}
			_delay_ms(500);
		}
		set_direct_out = PRG_flag = 0;
		
		
		//TAKTIKA
		kocka();
	
		//REGULACIJA
		if (Rac_tren_poz_sample_counter >= 3){		 //3 x 1.5ms = 4.5ms
			Rac_tren_poz_sample_counter = 0;
			Racunanje_trenutne_pozicije();
		}
		
		//Korekcija pravca i distance prema cilju
		if(Pracenje_Pravca_sample_counter >= 30){	//30 x 1.5ms = 45ms
			msg_counter++;
			Pracenje_Pravca_sample_counter = 0;
			Pracenje_pravca();
		}
		
		//PID regulacija
		if(PID_pozicioni_sample_counter >= 3){		//3 x 1.5ms = 4.5ms
			PID_pozicioni_sample_counter = 0;
			PID_ugaoni();
			if (stigao_flag == 2)
			{
			  PID_pravolinijski();
			}
		}
	}//while
}//main
