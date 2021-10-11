/*
 * Sprint10_Escravo.c
 *
 * Created: 06/10/2021 00:09:51
 * Author : savio
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


//Variáveis globais
uint8_t I=0;

//Protótipos
void anima_semaforo(uint8_t i);


ISR(USART_RX_vect)
{
	anima_semaforo(UDR0-'0');
}

void anima_semaforo(uint8_t i)
{
	const uint16_t estados[9] = {0b011110000, 0b001110000, 0b000110000, 0b000010000, 0b000001111, 0b000000111,0b000000011, 0b000000001,0b100000000};
	
	PORTB = estados[i] & 0b011111111;
	if(estados[i] & 0b100000000)
		PORTD |= 0b10000000;
	else
		PORTD &= 0b01111111;
}




int main(void)
{
    DDRB = 0b11111111;
	DDRD = 0b10000000;
	
	UBRR0H = (unsigned char)(MYUBRR>>8);	//Ajusta a taxa de transmissão, parte alta
	UBRR0L = (unsigned char)MYUBRR;		//Ajusta a taxa de transmissão, parte baixa
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita a interrup. do receptor, Habilita o transmissor e o receptor
	UCSR0C = (3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 1 de parada, paridade none
	
	sei();
    while(1)
	{
		
	}
}

