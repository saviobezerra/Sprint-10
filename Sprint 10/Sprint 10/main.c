/*
 * Sprint 10.c
 *
 * Created: 01/10/2021 10:19:17
 * Author : savio
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "PCD8544/nokia5110.h"

//NOVOS TIPOS
typedef enum enum_parametros {Sel_modo, Sel_tempo_verde, Sel_tempo_vermelho, Sel_tempo_amarelo, Size_enum_parametros} enum_parametros;
typedef struct stc_semaforo
{
	uint8_t modo;
	uint16_t tempo_verde_ms;
	uint16_t tempo_vermelho_ms;
	uint16_t tempo_amarelo_ms;
	uint16_t carros_por_min;
	uint16_t sensor_lux;
	uint16_t senstor_temp;
} stc_semaforo;

//VARIÁVEIS GLOBAIS
stc_semaforo semaforo ={.modo=0, .tempo_verde_ms=1000, .tempo_vermelho_ms=2000, .tempo_amarelo_ms=1000, .carros_por_min=0, .sensor_lux=0, .senstor_temp=0};
enum_parametros selecao_parametro = Sel_modo;
uint8_t flag_5000ms = 0, flag_500ms = 0, flag_1000ms=0, flag_10000ms=0;
uint32_t tempo_ms = 0;
uint16_t num_carros = 0;

//PROTÓTIPOS
void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms);
void anima_LCD(stc_semaforo Semaforo);
void estima_carros_por_min(uint8_t *flag_disparo);
void leituraADC_sensores(uint8_t *flag_disparo);
void USART_Transmit(unsigned char data);

//TRATAMENTO DE INTERRUPÇÕES
ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	if((tempo_ms % 5000)==0)
		flag_5000ms=1;
	if((tempo_ms % 500)==0)
		flag_500ms=1;

	
}

ISR(INT0_vect)
{
	num_carros++;
};

ISR(PCINT2_vect)
{
	if((PIND&0b00000010)==0 && semaforo.tempo_vermelho_ms<=3000 && semaforo.tempo_verde_ms >= 5000){
		semaforo.tempo_verde_ms=1000;
		semaforo.tempo_vermelho_ms=5000;
	}
		
			

	
	if((PIND&0b00010000)==0)
	{
		switch(selecao_parametro)
		{
			case Sel_modo:
				semaforo.modo =! semaforo.modo;
				break;
			case Sel_tempo_verde:
				if(semaforo.tempo_verde_ms <=8000)
					semaforo.tempo_verde_ms += 1000;
				break;
			case Sel_tempo_vermelho:
				if(semaforo.tempo_vermelho_ms <=8000)
					semaforo.tempo_vermelho_ms += 1000;
				break;
			case Sel_tempo_amarelo:
				if(semaforo.tempo_amarelo_ms <=8000)
					semaforo.tempo_amarelo_ms += 1000;
				break;
		}
	}
	if((PIND&0b00100000)==0)
	{
		switch(selecao_parametro)
		{
			case Sel_modo:
			semaforo.modo =! semaforo.modo;
			break;
			case Sel_tempo_verde:
			if(semaforo.tempo_verde_ms >= 2000)
			semaforo.tempo_verde_ms -= 1000;
			break;
			case Sel_tempo_vermelho:
			if(semaforo.tempo_vermelho_ms >= 2000)
			semaforo.tempo_vermelho_ms -= 1000;
			break;
			case Sel_tempo_amarelo:
			if(semaforo.tempo_amarelo_ms >= 2000)
			semaforo.tempo_amarelo_ms -= 1000;
			break;
		}
	}
	if((PIND&0b01000000)==0)
	{
		if(selecao_parametro < (Size_enum_parametros-1))
			selecao_parametro++;
		else
			selecao_parametro = Sel_modo;
	}

	anima_LCD(semaforo);
}

void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

void anima_semaforo(stc_semaforo Semaforo, uint32_t Tempo_ms)
{
	const uint16_t estados[9] = {0b000001111, 0b000000111, 0b000000011, 0b000000001, 0b100000000, 0b011110000,0b001110000, 0b000110000,0b000010000};
	static int8_t i_M=0, i_E=0;
	static uint32_t tempo_anterior_ms_M=0, tempo_anterior_ms_E=0;
	
	PORTB = estados[i_M] & 0b011111111;
		
		
	if(estados[i_M] & 0b100000000)
		PORTD |= 0b10000000;
	else
		PORTD &= 0b01111111;
	
	if(i_M <= 3)
	{
		
		if((Tempo_ms - tempo_anterior_ms_M) >= (Semaforo.tempo_verde_ms/4))
		{		
			i_M++;
			tempo_anterior_ms_M += (Semaforo.tempo_verde_ms/4);
		}
	}
	else
	{
		if(i_M <=4)
		{
			if((Tempo_ms - tempo_anterior_ms_M) >= (Semaforo.tempo_amarelo_ms))
			{
				i_M++;
				tempo_anterior_ms_M += (Semaforo.tempo_amarelo_ms);
			}
		}
		else
		{
			if (i_M<=8)
			{
				if((Tempo_ms - tempo_anterior_ms_M) >= (Semaforo.tempo_vermelho_ms/4))
				{
					i_M++;
					tempo_anterior_ms_M += (Semaforo.tempo_vermelho_ms/4);
				}
			}
			else
			{
				i_M=0;				
				tempo_anterior_ms_M = Tempo_ms;
				tempo_anterior_ms_E = Tempo_ms;
			}
		}
	}
	
	if(i_E <= 3)
	{
		if((Tempo_ms - tempo_anterior_ms_E) >= ((Semaforo.tempo_verde_ms + Semaforo.tempo_amarelo_ms)/4))
		{
			i_E++;
			tempo_anterior_ms_E += ((Semaforo.tempo_verde_ms + Semaforo.tempo_amarelo_ms)/4);
			USART_Transmit('0'+ i_E);
		}
	}
	else
	{
		if(i_E <=7)
		{
			if((Tempo_ms - tempo_anterior_ms_E) >= (Semaforo.tempo_vermelho_ms - Semaforo.tempo_amarelo_ms)/4)
			{
				i_E++;
				tempo_anterior_ms_E += (Semaforo.tempo_vermelho_ms - Semaforo.tempo_amarelo_ms)/4;
				USART_Transmit('0'+i_E);
			}
		}
		else
		{
			if (i_E<=8)
			{
				if((Tempo_ms - tempo_anterior_ms_E) >= (Semaforo.tempo_amarelo_ms))
				{
					i_E++;
					tempo_anterior_ms_E += (Semaforo.tempo_amarelo_ms);
					USART_Transmit('0'+ i_E);
				}
			}
			else
			{
				i_E=0;
				USART_Transmit('0' + i_E);							
			}
		}
	}	
}

void anima_LCD(stc_semaforo Semaforo)
{
	unsigned char modo_string[2];
	unsigned char tempo_verde_s_string[2];
	unsigned char tempo_vermelho_s_string[2];
	unsigned char tempo_amarelo_s_string[2];
	unsigned char carros_por_min_string[4];
	unsigned char sensor_lux_string[5] ;
	unsigned char sensor_temp_string[5]; 
	
	modo_string[0] = (Semaforo.modo) ?  'A'  :  'M'; modo_string[1] = '\0';
	
	sprintf(tempo_verde_s_string, "%u", Semaforo.tempo_verde_ms/1000);
	sprintf(tempo_vermelho_s_string, "%u", Semaforo.tempo_vermelho_ms/1000);
	sprintf(tempo_amarelo_s_string, "%u", Semaforo.tempo_amarelo_ms/1000);
	sprintf(carros_por_min_string, "%u", Semaforo.carros_por_min);
	sprintf(sensor_lux_string, "%u",  Semaforo.sensor_lux);
	sprintf(sensor_temp_string, "%u", Semaforo.senstor_temp);
	
	nokia_lcd_clear();
	nokia_lcd_set_cursor(0, 5);
	nokia_lcd_write_string("Modo", 1);
	
	nokia_lcd_set_cursor(30, 5);
	
	nokia_lcd_write_string(modo_string, 1);
	nokia_lcd_set_cursor(0, 15);
	nokia_lcd_write_string("T.Vd", 1);
	nokia_lcd_set_cursor(30, 15);
	nokia_lcd_write_string(tempo_verde_s_string, 1);
	nokia_lcd_set_cursor(0, 25);
	nokia_lcd_write_string("T.Vm", 1);
	nokia_lcd_set_cursor(30, 25);
	nokia_lcd_write_string(tempo_vermelho_s_string, 1);
	nokia_lcd_set_cursor(0, 35);
	nokia_lcd_write_string("T.Am", 1);
	nokia_lcd_set_cursor(30, 35);
	nokia_lcd_write_string(tempo_amarelo_s_string, 1);
	
	nokia_lcd_set_cursor(38, 5+selecao_parametro*10);
	nokia_lcd_write_string("<", 1);
	
	
	nokia_lcd_set_cursor(45, 25);
	nokia_lcd_write_string(carros_por_min_string, 1);
	nokia_lcd_set_cursor(66, 25);
	nokia_lcd_write_string("c/m", 1);
	
	nokia_lcd_set_cursor(45, 15);
	nokia_lcd_write_string(sensor_lux_string, 1);
	nokia_lcd_set_cursor(66, 15);
	nokia_lcd_write_string("lux", 1);
	
	nokia_lcd_set_cursor(45, 35);
	nokia_lcd_write_string(sensor_temp_string, 1);
	nokia_lcd_set_cursor(66, 35);
	nokia_lcd_write_string("°C", 1);
	
	
	
	nokia_lcd_render();
}

void estima_carros_por_min(uint8_t *flag_disparo)
{
	static uint16_t aux = 0;
	
	if (*flag_disparo)
	{
		*flag_disparo = 0;
		aux = num_carros;
		num_carros = 0;
		semaforo.carros_por_min = aux*12;
		
		
		if (semaforo.modo)
		{
			semaforo.tempo_verde_ms = 1000 + ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if (semaforo.tempo_verde_ms > 9000)
			semaforo.tempo_verde_ms = 9000;
			
			semaforo.tempo_vermelho_ms = 9000 - ((uint16_t)(semaforo.carros_por_min*16.7)/1000)*1000;
			if (semaforo.tempo_vermelho_ms > 32000)
			semaforo.tempo_vermelho_ms = 1000;
		}
	}
}

void leituraADC_sensores(uint8_t *flag_disparo)
{
	static uint16_t cont=0;	//Variável auxiliar
	float R1=9998,logR2,R2,T, c1=0.9187244108E-03, c2=2.643518478E-04,c3=0.006895693462E-07; //Variáveis utilizadas no cálculo da temperatura
	
	if (*flag_disparo)
	{
		switch(cont)
		{
		case 0:
		semaforo.sensor_lux = (1023000/ADC) - 1000;
		if (((semaforo.sensor_lux <= 300)&& (!(PIND & (1<<0)))) || (semaforo.sensor_lux <= 300)&&(semaforo.carros_por_min>0))
		{
			OCR2B = 250;
		}
		else if (((semaforo.sensor_lux <= 300)&& (!(!(PIND & (1<<0))))) || (semaforo.sensor_lux <= 300)&&(semaforo.carros_por_min>0))
		{
			OCR2B = 80;
		}
		else
		OCR2B = 1;
		ADMUX = 0b01000001;
		break;
		
		case 1:
		R2 = (1023 / (float)ADC - 1) * R1;
		logR2 = log(R2);
		T = (1 / (c1 + (c2*logR2) + (c3*logR2*logR2*logR2)));
		semaforo.senstor_temp = T - 177;
		ADMUX= 0b01000000;
		break;
		}
		
		if(cont <1)
		{
			cont++;
		}
		else
			cont=0;
		}
		
		*flag_disparo = 0;
		anima_LCD(semaforo);
	}


int main(void)
{
	//GPIO
	DDRB  = 0b11111111;
	DDRD  = 0b10001000;
	DDRC  = ~((1<<6)|(1<<0));
	PORTD = 0b01110111;
	PORTC |= (1<<6);
	
	//USART
	UBRR0H = (unsigned char)(MYUBRR>>8);	//Ajusta a taxa de transmissão, parte alta
	UBRR0L = (unsigned char)MYUBRR;		//Ajusta a taxa de transmissão, parte baixa
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita a interrup. do receptor, Habilita o transmissor e o receptor
	UCSR0C = (3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada, paridade none
	
	
	EICRA = 0b00001010;
	EIMSK = 0b00000011;
	PCICR = 0b00000100;
	PCMSK2 =0b01110010;
	
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000011;
	OCR0A = 249;
	TIMSK0 = 0b00000010;
	
	//Configuração PWM
	TCCR2A = 0b10100011;
	TCCR2B = 0b00000011;

	//Configuração do ADC
	ADMUX =  0b01000000;
	ADCSRA = 0b11100111;
	ADCSRB = 0b00000000;
	DIDR0 =  0b00000001;
	
	//Inicialização do LCD
	nokia_lcd_init();
	anima_LCD(semaforo);
	sei();
	
	
	
	
	while (1)
	{
		anima_semaforo(semaforo,tempo_ms);
		estima_carros_por_min(&flag_5000ms);
		leituraADC_sensores(&flag_500ms);
	}
}




