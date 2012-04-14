/************************** K9 M1 Firmware ****************************/
//	Motor Driver Firmware for the K9
//  Chip: MaEvArM 1
//	Version 2.1.0				Tested : OK! 
//	Date : 03/29/2012  			Authors: Yash Mulgaonkar & J. Fiene 
//            yashm@seas.upenn.edu , jfiene@seas.upenn.edu
/**********************************************************************/


#define ROBOT 1

// communication string format from Matlab
// set speeds: 'SlLLLrRRR'
//		l=left dir (1=FWD, 0=REV)
//		LLL = left speed (000-999 -> 0=100% PWM)
//		r = right dir (1=FWD, 0=REV)
//		RRR = right speed (000-999 -> 0-100% PWM)
// set gains: 'GPPPIIIDDDFFF'
//		PPP = proportional gain (000-999)
//		III = integral gain (000-999)
//		DDD = derivative gain (000-999)			
//		FFF = velocity filter gain (000-999 -> 0.000 to 0.999)			
// get encoders: 'E'
//		will return a string 'LLLLLLLLRRRRRRRRX' with the current encoder counts

#define MAEVARM_RF_RED
#define PACKET_SIZE 17
#define T3MAX	0xFFFF

#include <stdlib.h>
#include "maevarm.h"
#include "maevarm-usb.h"

#define V_FILTER 0.95
#define REPEATS  20

// function prototypes
void init(void);

// global variables for encoders
long L_encoder=0, R_encoder=0;
long L_encoder_temp=0; 
long R_encoder_temp=0;
long L_encoder_delta=0; 
long R_encoder_delta=0;
long L_encoder_last=0;
long R_encoder_last=0;
long L_delta=0;
long L_last=0;
unsigned int L_overflow=0;
long R_delta=0;
long R_last=0;
unsigned int R_overflow=0;

float L_velocity=0, R_velocity=0;
char A, B, C, D;

int main(void)
{		
	int i;
	char gains[PACKET_SIZE]   = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	char message[PACKET_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	char L_string[PACKET_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	char R_string[PACKET_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	float P_gain=3, I_gain=0, D_gain=10, F_gain=1; // default gains
	volatile int L_des=0, R_des=0;
	float L_error=0, R_error=0, L_error_last, R_error_last, L_integral=0, R_integral=0, L_derivative, R_derivative;
	int L_out=0, R_out=0;
	
	init();
	
	while(1){

        // update velocity calculations
        if(L_delta){
            L_velocity = L_velocity*(V_FILTER) + (125000/(float)L_delta) * (1-V_FILTER); // encoder counts per timer tick * N ticks/second
        }
        if(R_delta){
            R_velocity = R_velocity*(V_FILTER) + (125000/(float)R_delta) * (1-V_FILTER); // encoder counts per timer tick * N ticks/second
        }
        
        L_error_last = L_error;
        L_error = (float)L_des - L_velocity;
        L_integral += L_error/1000; // assume I_gain given in milliunits
        L_derivative = L_error - L_error_last;
        L_out = (int)(P_gain*L_error + I_gain*L_integral + D_gain*L_derivative + F_gain*L_des); // velocity is negative
        if(L_out>=0){
            OCR1B = L_out;
            set(PORTD,6);
        }else{
            OCR1B = -L_out;
            clear(PORTD,6);
        }

        R_error_last = R_error;
        R_error = (float)R_des - R_velocity;
        R_integral += R_error/1000; // assume I_gain given in milliunits
        R_derivative = R_error - R_error_last;
        R_out = (int)(P_gain*R_error + I_gain*R_integral + D_gain*R_derivative + F_gain*R_des); // velocity is negative
        if(R_out>=0){
            OCR1C = R_out;
            set(PORTD,7);
        }else{
            OCR1C = -R_out;
            clear(PORTD,7);
        }
        
        // parse any incoming messages
        if(usb_rx_available()){
            switch(usb_rx_char()){
				case 'S':							// set speeds
					message[0] = 'S';
					for(i=1; i<9; i++){
						while(!usb_rx_available());	// wait
						message[i] = (char)usb_rx_char();
					}
                    L_des = (unsigned int)(message[4] - '0') + (10 * (unsigned int)(message[3] - '0')) + (100 * (unsigned int)(message[2] - '0'));
                    if(!(message[1]-'0')){
                        L_des = -L_des;
                    }
                    R_des = (unsigned int)(message[8] - '0') + (10 * (unsigned int)(message[7] - '0')) + (100 * (unsigned int)(message[6] - '0'));
                    if(!(message[5]-'0')){
                        R_des = -R_des;
                    }
					break;
				case 'G':
					gains[0] = 'G';
					for(i=1; i<18; i++){
						while(!usb_rx_available());	// wait
						message[i] = (char)usb_rx_char();
					}
                    P_gain = ((float)(message[4] - '0')/10) + (float)(message[2] - '0') + (10 * (float)(message[1]-'0'));
                    I_gain = ((float)(message[8] - '0')/10) + (float)(message[6] - '0') + (10 * (float)(message[5]-'0'));
                    D_gain = ((float)(message[12] - '0')/10) + (float)(message[10] - '0') + (10 * (float)(message[9]-'0'));
                    F_gain = ((float)(message[16] - '0')/10) + (float)(message[14] - '0') + (10 * (float)(message[13]-'0'));
					break;
				case 'E': // request a sendback of the encoder delta
                    
                    for(i=0;i<PACKET_SIZE;i++){ // clear out the old string values
                        L_string[i] = 0;
                        R_string[i] = 0;
                    }
                    
                    // grab the current values, which could otherwise be changing as we go through the code;
                    L_encoder_temp = -L_encoder;
                    R_encoder_temp = -R_encoder;
                    
                    // send the L wheel value to the dongle						
                    //L_encoder_delta = L_encoder_temp - L_encoder_last;
                    //L_encoder_last = L_encoder_temp;
                    ltoa(L_encoder_temp,&L_string[0],10);
                    //L_string[0]='L';
                    
                    // send the R wheel value to the dongle						
                    //R_encoder_delta = R_encoder_temp - R_encoder_last;
                    //R_encoder_last = R_encoder_temp;
                    ltoa(R_encoder_temp,&R_string[0],10);
                    //R_string[0]='R';
                    
                    for(i=0;i<PACKET_SIZE;i++){
                        usb_tx_char(L_string[i]);							
                    }
					usb_tx_string("\n");
					for(i=0;i<PACKET_SIZE;i++){
						usb_tx_char(R_string[i]);							
					}
					usb_tx_string("\n"); 

					break;
				default:
					break;					
			}
        }
	}
}


ISR(TIMER3_COMPA_vect)
{
/*	
	if(L_overflow<2){
		L_velocity = L_velocity*(V_FILTER) + (125000/(float)L_delta) * (1-V_FILTER); // encoder counts per timer tick * N ticks/second
	}else{
		L_velocity = 0;
	}
	
	if(R_overflow<2){
		R_velocity = R_velocity*(V_FILTER) + (125000/(float)R_delta) * (1-V_FILTER); // encoder counts per timer tick * N ticks/second
	}else{
		R_velocity = 0;
	}
	*/
}

ISR(TIMER3_OVF_vect)
{
	L_overflow++;
	R_overflow++;	
	if(L_overflow>2){	// if we've overflown twice, we are very likely stopped
		L_velocity=0;	// so force velocity to zero
	}	
	if(R_overflow>2){	// if we've overflown twice, we are very likely stopped
		R_velocity=0;	// so force velocity to zero
	}
}


// L_cpt (timer counts per tick)

void L_up(void){
	L_encoder++;			// increment the encoder count

	int L_temp = TCNT3;		// pull the timer value
	if(L_overflow){			// if we've rolled over
		L_delta = (T3MAX*L_overflow) - L_last + L_temp;	// calculate the delta spanning the rollover
		L_overflow = 0;
	}else{
		L_delta = L_temp - L_last;	// calculate the delta
	}
	L_last = L_temp;		// store off the last
}

void L_down(void){
	L_encoder--;			// decrement the encoder count

	int L_temp = TCNT3;
	if(L_overflow){
		L_delta = (T3MAX*L_overflow) - L_last + L_temp;
		L_overflow = 0;
	}else{
		L_delta = L_temp - L_last;
	}
	L_last = L_temp;	
	L_delta = -L_delta;		// negate to show reverse
}

void R_up(void){
	R_encoder++;			// decrement the encoder count
	
	int R_temp = TCNT3;
	if(R_overflow){
		R_delta = (T3MAX*R_overflow) - R_last + R_temp;
		R_overflow = 0;
	}else{
		R_delta = R_temp - R_last;
	}
	R_last = R_temp;	
}

void R_down(void){
	R_encoder--;			// decrement the encoder count
	
	int R_temp = TCNT3;
	if(R_overflow){
		R_delta = (T3MAX*R_overflow) - R_last + R_temp;
		R_overflow = 0;
	}else{
		R_delta = R_temp - R_last;
	}
	R_last = R_temp;	
	R_delta = -R_delta;
}

ISR(INT0_vect){
	if(A){ // falling edge
		A=0;
		(B)?(L_up()):(L_down());
	}else{
		A=1;
		(B)?(L_down()):(L_up());
	}
}

ISR(INT1_vect){
	if(B){ // falling edge
		B=0;
		(A)?(L_down()):(L_up());
	}else{
		B=1;
		(A)?(L_up()):(L_down());
	}
}

ISR(INT3_vect){
	if(C){ // falling edge
		C=0;
		(D)?(R_up()):(R_down());
	}else{
		C=1;
		(D)?(R_down()):(R_up());
	}
}

ISR(INT2_vect){
	if(D){ // falling edge
		D=0;
		(C)?(R_down()):(R_up());
	}else{
		D=1;
		(C)?(R_up()):(R_down());
	}
}

void init(void)
{	
	
//	disableJTAG(); // allow access to F4-F7

// system clock prescaler
	
	CLKPR = (1<<CLKPCE);
	CLKPR = 0;            // set to /1 (8MHz)

// LED outputs
	
	set(DDRE,6);      // red LED enabled
	set(PORTE,6);     // red LED off
	set(DDRE,2);      // green LED enabled
	set(PORTE,2);     // green LED off

	
	if(ROBOT){

        // initialize USB
        usb_init();
		while(!usb_configured());

		// velocity calculation ISR timer		
//		OCR3A = T3MAX;		// match every 10 milliseconds (10 * 125 * 64 / 8M) to calc. velocity
		set(TIMSK3,TOIE3);	// interrupt on 16-bit overflow
		set(TCCR3B,CS31); // clock prescaler to /64, starts timer
		set(TCCR3B,CS30); // clock prescaler to /64, starts timer
		
		// Motor PWM outputs on OC1B (B6), OC1C (B7)
		
		set(DDRB,6); // PWM for L motor
		set(DDRB,7); // PWM for R motor
		
		set(TCCR1A,COM1B1); // clear on match with OCR1B
		set(TCCR1A,COM1C1); // clear on match with OCR1C
		
		set(TCCR1B,WGM13);
		set(TCCR1B,WGM12);
		set(TCCR1A,WGM11);
		set(TCCR1A,WGM10); // UP to OCR1A, PWM mode
		
		OCR1A = 400;	  // max is 65535
		OCR1B = 0;        // 0% by default
		OCR1C = 0;        // 0% by default
		
		set(TCCR1B,CS10); // clock prescaler to /1, starts timer
		
		// Motor direction lines (D6, D7)
		
		clear(PORTD,6); // forward by default
		clear(PORTD,7); // forward by default
		set(DDRD,6); // direction for Right motor
		set(DDRD,7); // direction for Left motor
		
		A = check(PIND,0); // get initial encoder values
		B = check(PIND,1);
		C = check(PIND,2);
		D = check(PIND,3);
		
		set(EICRA,ISC00); // INT0: interrupt on either edge
		set(EICRA,ISC10); // INT2: interrupt on either edge
		set(EICRA,ISC20); // INT3: interrupt on either edge
		set(EICRA,ISC30); // INT4: interrupt on either edge
		
		set(EIMSK,0);     // enable interrupt on INT0
		set(EIMSK,1);     // enable interrupt on INT1
		set(EIMSK,2);     // enable interrupt on INT2
		set(EIMSK,3);     // enable interrupt on INT3

		sei();            // enable global interrupts
	}
}