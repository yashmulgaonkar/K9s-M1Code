// -------------------------------
// MAEVARM microcontroller board
// custom header file
// version: 0.3.0
// date: November 9, 2009
// author: J. Fiene
// -------------------------------

#ifndef _MAEVARM_H_
#define _MAEVARM_H_

#include <avr/io.h>
#include <avr/interrupt.h>

// operations to set, clear, toggle, and check individual register bits
#define set(reg,bit)	  reg |= (1<<(bit))
#define clear(reg,bit)	  reg &= ~(1<<(bit))
#define toggle(reg,bit)	  reg ^= (1<<(bit))
#define check(reg,bit)	  (reg & (1<<(bit)))

//allow access to F4-F7 as normal port pins
void disableJTAG(){
	MCUCR |= (1 << JTD);// | (1 << IVCE) | (0 << PUD);
	MCUCR |= (1 << JTD);// | (0 << IVSEL) | (0 << IVCE) | (0 << PUD);
}

#endif
