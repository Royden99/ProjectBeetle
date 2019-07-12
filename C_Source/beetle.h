/* 
 * File:   beetle.h
 * Author: Royden
 *
 * Created on 04 March 2019, 11:42
 */

#ifndef BEETLE_H
#define	BEETLE_H

#ifdef	__cplusplus
extern "C" {
#endif
#ifdef	__cplusplus
}
#endif  /* __cplusplus */
//*************** stepper motor system *****************************************

// output latches to stepper motor drivers (L6219)
#define L0    LA0      // }current limiting logic options
#define L1    LA1      // } 
#define m1ph1 LA3      //  Phase: motor 1 phase 1
#define m1ph2 LA4      //         motor 1 phase 2
#define m2ph1 LA2      //         motor 2 phase 1
#define m2ph2 LA6      //         motor 2 phase 2
// signal to turn move() inverter on or off
#define IEN   LA7
// PushButton sensors input
#define PB1   PORTBbits.RB1
#define PB2   PORTBbits.RB2
#define PB3   PORTBbits.RB0
#define PB4   PORTBbits.RB4

// [bx_y] refers to [variable x, motor y]
extern unsigned int b1_1, b1_2, b2_1, b2_2;
// symbols for motor1 & motor2
extern volatile bit M1, M2;
// these increment every motor half-step
extern volatile unsigned char aa, cc; 
extern volatile unsigned int  bb;
// this tells bb when to stop incrementing
extern unsigned int bb_stop;
// this tells mainloop whether or not it's waiting to engage motor interrupt
extern unsigned char waiting;
// a variable that remembers the last move() operation
extern unsigned char prev_mode;

//*************** light sensor system ******************************************
// LDRx says whether photosensor signal 'x' has been detected, and (modules 1-6)
//  gives signal strength if it has.
extern unsigned int LDR1, LDR2, LDR3, LDR4, LDR5, LDR6, LDR7, LDR8;

//  STATE is the result of any incoming signals
extern volatile unsigned int STATE @ (0xF36);
typedef union 
{   struct  //each labeled bit in 'STATE' is an event flag pointing to a signal
    {   unsigned l1     : 1;    // cntr-clkws from front right collision sensor:
        unsigned p1     : 1;    //Pushbutton1
        unsigned l2     : 1;    //Ldr2
        unsigned p2     : 1;    // etc...
        unsigned l3     : 1;
        unsigned l4     : 1;
        unsigned p3     : 1;
        unsigned l5     : 1;
        unsigned p4     : 1;
        unsigned l6     : 1;
        unsigned l7     : 1;    // M1 } these sensors watch to see if the wheels
        unsigned l8     : 1;    // M2 }  are turning
        unsigned done   : 1;    //go-ahead to move forward
        unsigned unused : 3;    //unused
    };
} STATEbits_t;
extern volatile STATEbits_t STATEbits @ (0xF36);

//***************** other ******************************************************
// needed for __delay_ms() & __delay_us() functions
#define _XTAL_FREQ 32000000

// pseudo-random bit sequence buffer
extern volatile unsigned int SHFTREG @ 0xF34;
typedef union
{   struct
    {   unsigned a  : 1;
        unsigned b  : 1;
        unsigned c  : 1;
        unsigned d  : 1;
        unsigned e  : 1;
        unsigned f  : 1;
        unsigned g  : 1;
        unsigned h  : 1;
        unsigned i  : 1;
        unsigned j  : 1;
        unsigned k  : 1;
        unsigned l  : 1;
        unsigned m  : 1;
        unsigned n  : 1;
        unsigned o  : 1;
        unsigned p  : 1;
    };
} SHFTREGbits_t;
extern volatile SHFTREGbits_t SHFTREGbits @ 0xF34;

// more than one piece of code uses Timer6: don't run two or more simultaneously
extern bit Dbounce_in_progress; 

// De-bouncing Initialization:
#define Dbounce_ms(_sfr_, _bit_)    Dbounce_in_progress = 1;    \
                                    sample_time = 5;            \
                                    SFR = _sfr_;  BIT = _bit_;  \
                                    TMR6IF = 0;                 \
                                    PR6 = 0xFF;                 \
                                    T6CON = 0xFF;               

// Reaction Initialization:
#define trigger_front(event)    reaction = event;               \
                                LATC0 = 0;                      \
                                move(0, "now");                 \
                                move(2, "wait");                \
                                bb_stop = 255;

#define trigger_rear(event)     reaction = event;               \
                                LATC0 = 0;                      \
                                move(0, "now");                 \
                                move(1, "wait");                \
                                bb_stop = 255;

// ADC Conversion Sequence:
#define convert_channel(ch)     ADCON0bits.CHS = ch;            \
                                ADON = 1;                       \
                                GO_nDONE = 1;                   \
                                while(GO_nDONE == 1){   ;}      \
                                ADIF = 0;                   

#endif	/* BEETLE_H */

