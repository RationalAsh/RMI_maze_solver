/*
	This will eventually grow into a program that allows me to control an
	avr mcu using a hacked radio controleld car's radio interface. Now I'm
	just testing if the basic avr-gcc toolchain works on Ubuntu.
*/

/*
	Connections to be made: This section will make a record of the physical
	connections necessary for the code to work. This includes the pin
	connections of the RC module, the motor drivers and the ADC.

	MOTOR:
	Left motor Black Wire goes to  - 
	Left motor Red Wire goes to    -
	Right motor Black wire goes to -
	Right motor Red Wire goes to   -

	MOTOR DRIVER (H-BRIDGE):
	INPA1 goes to    -
	INPA2 goes to    -
	INPB1 goes to    -
	INPB2 goes to    -
	ENABLE A goes to -
	ENABLE B goes to -

	ADC:
	Right metal strip is connected to -
	Left metal strip is connected to  -

	ARM:
	Motor Black Wire is connected to -
	Motor Red Wire is connected to   -
	

	Radio Receiver:
	*Wires should go to some port pins.

*/

#define F_CPU 1000000

#include <avr/io.h>
#include <util/delay.h>

void delay_ms(int amount)
{
	while(amount--) _delay_ms(1);
}

void forward()
{

}

void backwards()
{

}

void right()
{

}

void left()
{

}

void stop()
{

}

int main()
{
	DDRB |= 0b00000001;
	PORTB = 0x00;
	
	while(1)
	{
		PORTB |= (1<<PB0);
		delay_ms(50);
		PORTB &= ~(1<<PB0);
		delay_ms(50);
	}
}
