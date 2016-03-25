/*
 * X_Drive_Odometry.c
 *
 * Poslednje_izmene: 24/03/2016 20:07:41
 * Autor: Kefa 
 
 Izmene:
 -Namesten baudrate za LCD
 -Dodate funkcije za sendMsg/sendChar
 -Dodata PGM_Mode funkcija dodat taster
 -Uklonjeno checkmotors..() funkcija
 -Proveriti brzinskiPid koji se poziva u interaptu sta se tamo nalazi
 --tamo ne sme biti nikakvih komplikovanih operacija ili definisanja promenljivih
 -Pidovanje
 
 Potrebne izmene:
 -Izbaciti inicijalizaciju bluetooth-a
 -Promeniti baudrate
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
	//inicijalizuj_bluetooth();
	//Trebalo bi da sve radi i bez ove funkcije iznad, treba je izbrisati!!!
	
	//inicijalizuj_servo_tajmer_20ms();
	//pomeri_servo_1(0);
	//sendChar('k');
	//_delay_ms(1000);					//cekanje da se stabilizuje sistem
	nuliraj_poziciju_robota();
	//CheckInputMotorControl();
	while(1)
	{
		demo_3();
		
		//Racunanje trenutne pozicije
		if (Rac_tren_poz_sample_counter >= 3){		 //3 x 1.5ms = 4.5ms
			Rac_tren_poz_sample_counter = 0;
			Racunanje_trenutne_pozicije();
		}
		
		//Korekcija pravca i distance prema cilju
		if(Pracenje_Pravca_sample_counter >= 30){	//30 x 1.5ms = 45ms
			msg_counter++;
			servo_counter++;
			Pracenje_Pravca_sample_counter = 0;
			Pracenje_pravca();
		}
		
		//PID regulacija
		if(PID_pozicioni_sample_counter >= 3){		//3 x 1.5ms = 4.5ms
			PID_pozicioni_sample_counter = 0;
			PID_ugaoni();
			PID_pravolinijski();
			//PID_brzinski se poziva direktno u interaptu sistemskog tajmera TCE1!
		}
		
		//PROGRAMMING MODE - kod koji ako je pritisnut CRVENI taster gasi motore
		while(PGM_Mode() == 1){
			set_direct_out = 1;
			PID_brzina_L = 0;
			PID_brzina_R = 0;
			sendMsg("PGM_MODE");
			_delay_ms(300);
		}
		set_direct_out = 0;
		
	}//while
}//main
