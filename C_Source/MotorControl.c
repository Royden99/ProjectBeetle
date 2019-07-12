
#include <xc.h>
#include "beetle.h"

void sing (const char song[])
/* Vibrate the motors just for fun, and leave them in a stable OFF condition. */
{
    unsigned char i = 0, ph_state = 0;
    unsigned int j = 0;
    
    // 'len_song' determines how many notes to play
    unsigned char len_song;
    // each element of 'song_pitch' is the number of microseconds in one period
    //  of a desired wavelength
    unsigned int song_pitch[7];
    
    // song options:
    if (song == "on")
    {
        len_song = 6;
        song_pitch[0] = 473; // C
        song_pitch[1] = 473;
        song_pitch[2] = 355; // F
        song_pitch[3] = 355;
        song_pitch[4] = 237; // C'
        song_pitch[5] = 237;
    }
    else if (song == "off")
    {
        len_song = 7;
        song_pitch[0] = 237; // C'
        song_pitch[1] = 237;
        song_pitch[2] = 355; // F
        song_pitch[3] = 355;
        song_pitch[4] = 473; // C
        song_pitch[5] = 473;
        song_pitch[6] = 473;
    }
    else if (song == "start")
    {  
        len_song = 5;
        song_pitch[0] = 355; // F
        song_pitch[1] = 355;
        song_pitch[2] = 237; // C'
        song_pitch[3] = 237;
        song_pitch[4] = 237;
        song_pitch[5] = 237;        
    }
    else if (song == "stop")
    {
        len_song = 5;
        song_pitch[0] = 237; // C'
        song_pitch[1] = 237;
        song_pitch[2] = 355; // F
        song_pitch[3] = 355;
        song_pitch[4] = 355;
        song_pitch[5] = 355;
    }
    
    // init Timer5
    TMR5IF = 0;
    TMR5H = 0;
    TMR5L = 0;
    T5CON = 0b00110001;
    
    // both motors at full voltage
    IEN = 0;
    L0 = 0;
    L1 = 0;
    
    // keep phase2 constant and toggle phase1
    m1ph2 = 0;
    m2ph2 = 0;
    
    for (i = 0; i < len_song; i++)
    {
        while(TMR5IF == 0) // each note lasts ~200 ms
        {
            for (j = 0; j < song_pitch[i]; j++)
            {   __delay_us(1);  }
            
            if (ph_state == 0)
            {
                m1ph1 = 0;
                m2ph1 = 0;
                ph_state = 1;
            }
            else
            {
                m1ph1 = 1;
                m2ph1 = 1;
                ph_state = 0;
            }
        }
        TMR5IF = 0;
    }
    //  clean up Timer5
    TMR5L = 0;
    TMR5H = 0;
    T5CON = 0;
    
    // leave motors in OFF state
    m1ph1 = 0;
    m2ph1 = 0;
    m1ph2 = 0;
    m2ph2 = 0;
    L0 = 1;
    L1 = 1;
}


void move(char mode, const char when[])
/* This function sets up the initial motor conditions for the desired movement;
 *  then the conditions are updated over time via Timer2 interrupt.
 * 
 *  mode "1" drives robot forward;
 *  mode "2" drives robot in reverse;
 *  mode "3" pivots robot clockwise;
 *  mode "4" pivots robot counter-clockwise;
 *  mode "5" turns robot forward to the right;
 *  mode "6" turns robot forward to the left;
 *  mode "7" turns robot backward to the right;
 *  mode "8" turns robot backward to the left;
 *  mode "0" brings robot to a stop
 * 
 *  when[] "wait" delays the start of movement
 *  when[] "now" starts movement right away
 */
{       
    /* Timer2 on & set time interval
     * 
     *  *equations:   (assuming FOSC == 32 MHz)
     *      motor phase period    = [(PR2)(presc.)(postsc.)]   us
     * 
     *      TMR2 interrupt period = [(0.125)(PR2)(presc.)(postsc.)] us
     *                            = [(motor phase period)/(8)]
     * 
     *  *physical motor capability:
     *       minimum interrupt period approx. (1150 us)
     *        i.e. minimum motor phase period approx. (9500us)
     *       this corresponds to maximum motor rpm
     */
    
    // TMR2 to PR2 Match interrupt settings
    TMR2IF = 0; // flag bit starts LOW
    TMR2IP = 0; // priority low
    
    if (when == "now")
    {   // interrupt period = 1920 us (i.e. 15360 cyc)
        T2CON = 0b00100111;        //[presc. = 1:16]; [postsc. = 1:5]
        PR2 = 0xC0;                // decimal '192'
        // interrupt enabled
        TMR2IE = 1;
        IEN = 1;    // enable motor logic inverter
    }
    else // "wait":
    {   // set motor conditions for a mode, but don't enable T2 interrupt:
        //  use T2 to wait ~500 ms (monitor TMR2IF & 'waiting' in mainloop);
        //  when done waiting, start interrupt
        T2CON = 0b01111111;
        PR2 = 0xFF;             // TMR2IF set every 8.16 ms
        waiting = 0;
        // motor current level = 0%  while waiting
        IEN = 0;    // disable motor logic inverter
    }
    
    //initial iterator values
    aa = 0;
    bb = 1;
    cc = 0;     // don't call signal(modules 6 & 7) while not in motion
    
    //initial output values 
    m1ph2   = 0;
    m2ph2   = 0;
    m1ph1   = 0;
    m2ph1   = 0;
    L0      = 1;
    L1      = 1;
    
    /*  *variables b1, b2 determine the orientation of phase2 sq. wave, and
     *      thus stepper motor direction  
     *      --[forward and reverse options]
     *  *option [Mx = 0] makes both phase inputs of stepper motor driver(x)
     *      a constant LOW voltage level, causing motor(x) to hold still;
     *      else if [Mx = 1], stepper motor rotates as planned
     *      --[stationary option]
     *  *combinations of forward, reverse, and stationary options for motors
     *      1 and 2 determine robot behavior in each mode.
     */
    
    if(mode == 1)          //forward
    {   M1 = 1;
        M2 = 1;
        b1_1 = 1;
        b1_2 = 1;
        b2_1 = 0;
        b2_2 = 0;
    }
    else if(mode == 2)     //reverse
    {   M1 = 1;
        M2 = 1;
        b1_1 = 0;
        b1_2 = 0;
        b2_1 = 1;
        b2_2 = 1;
    }
    else if(mode == 3)     //pivot clockwise (m1 forward; m2 reverse)
    {   M1 = 1;
        M2 = 1;
        b1_1 = 1;
        b1_2 = 0;
        b2_1 = 0;
        b2_2 = 1;
    }
    else if(mode == 4)     //pivot anti-clockwise (m1 reverse, m2 forward)
    {   M1 = 1;
        M2 = 1;
        b1_1 = 0;
        b1_2 = 1;
        b2_1 = 1;
        b2_2 = 0;
    }    
    else if(mode == 5)     //turn forward right (m1 forward, m2 stationary)
    {   M1 = 1;
        M2 = 0;
        b1_1 = 1;
        b1_2 = 0;
        b2_1 = 0;
        b2_2 = 0;
    }
    else if(mode == 6)     //turn forward left (m2 forward, m1 stationary)
    {   M1 = 0;
        M2 = 1;
        b1_1 = 0;
        b1_2 = 1;
        b2_1 = 0;
        b2_2 = 0;
    }
    else if(mode == 7)     //turn backward right (m1 backward, m2 stationary)
    {   M1 = 1;
        M2 = 0;
        b1_1 = 0;
        b1_2 = 0;
        b2_1 = 1;
        b2_2 = 0;
    }
    else if(mode == 8)     //turn backward left (m2 backward, m1 stationary)
    {   M1 = 0;
        M2 = 1;
        b1_1 = 0;
        b1_2 = 0;
        b2_1 = 0;
        b2_2 = 1;
    }
    else if(mode == 0)     //stop completely
    {   TMR2ON = 0; // Timer2 off
        TMR2IE = 0; // Timer2 interrupt disabled    
        // phase outputs all zero
        m1ph2 = 0;
        m2ph2 = 0;
        m1ph1 = 0;
        m2ph1 = 0;
        // current level: 0%     
        IEN = 0;    // disable inverter, enabling all L6219 logic inputs to go 
                    // high
        L0 = 1;
        L1 = 1;
    }
    // remember the previous movement
    prev_mode = mode;
}

/*  PRECISION PIVOT
 * 
 *  Given:
 *  distance between Beetle's wheels (center to center) == 145 mm
 *  wheel diameter == 144 mm
 *  1 tire revolution == ~618 motor half-steps (dependent on gear or belt ratio)
 * 
 *************************
 * The number of half-steps in a pivot of X degrees ==
 *  (145/144)*(turn ratio)*(618);
 * 
 * where:
 *  turn ratio == [(X degrees)/360] 
 *
 * e.g. For a 45 deg. pivot:
 *  (145/144)*(45/360)*(618) == ~78 half-steps
 * 
 **************************
 *
 * N.B. this formula doesn't work, possibly due to too much imprecision in the
 *  mechanical structure. Actual values are chosen by trial & error:
 * 
 * 45  degrees:     bb_stop ~ 120
 * 90  degrees:     bb_stop ~ 220
 * 135 degrees:     bb_stop ~ 315
 * 180 degrees:     bb_stop ~ 410
 */

void pivot(unsigned int direction, unsigned int degree)
/* initialize a pivot movement */
{   
    if (direction == 'R')       //clockwise
    {   move(3, "wait");
    }
    else if (direction == 'L')  //anti-clockwise
    {   move(4, "wait");
    }
    // continue pivoting until stopping point
    bb_stop = degree;
} 


unsigned int rand(const char type[])
/*  This function is a Pseudo-Random Bit Sequence generator, inspired by
 *      maximal-length feedback shift register electronics hardware.
 * 
 * HOW IT WORKS:
 *  The "shift" buffer is 15 bits; a maximal-length sequence is obtained by
 *      placing "feedback taps" on the 14th and 15th bits.  These are XOR-ed
 *      and the result 'D' overwrites the 1st bit.
 *     This results in 32,767 "unpredictable" 1s and 0s before it repeats.
 *     Instead of shifting all the elements every time, it is a circular buffer:
 *      'bitnum' points to the appropriate three bits, and an extra "spacer" bit
 *      exists which is overwritten by 'D'.
 *  Instead of constantly running on a clocked input, this generator only 
 *      outputs as many bits as needed per function call (specified by local
 *      variable 'j').
 *     Because of this, an initial 16-bit "seed" should be used to begin the
 *      sequence at an unpredictable value.
 *  When certain arguments are passed to the function, it returns an integer
 *      within a certain range.
 *     This is done by masking 'SHFTREG' first with a bitwise AND operation
 *      against a literal to set the maximum value, and then with a bitwise OR
 *      operation against a second literal to set the minimum value.
 *     The problem here is that some values within the desired range are made
 *      impossible since the bitwise OR requires that one or more bits is always
 *      '1'.
 * 
 * USAGE:
 *  If 'type' == <"direction">, 'L' or 'R' is returned depending on the value of
 *      bit 'D'.
 *  If 'type' == <"move">, one of the integers (3, 4, 5, or 6) is returned
 *      depending on two bits of SHFTREG generated for that purpose.
 *  If 'type' == <"degree">, an integer consisting of the 9 LSB's of 'SHFTREG' 
 *      is returned (the 6th and 7th bits are always set).
 *  If 'type' == <"time">, an integer consisting of the 13 LSB's of 'SHFTREG' is
 *      returned (the 10th and 9th bits are always set).
 * 
 * NOTES:
 *  I originally developed two versions of this function to compare their
 *      efficiencies:
 *      1) uses a bit-field struct as the shift register, and 
 *      2) uses an array of chars as the shift register.
 *     Ultimately, version "2" produced less assembly code; however, it took
 *      up more space in Data Memory.
 *     Version "1" proved more convenient to add further functionality to, as I 
 *      have done; bitwise masking couldn't be done with version "2".
 *     Both functions had similar cycle counts when tested in original bare-
 *      bones form.
 */
{    
/*  V.1 -- USING A BIT-FIELD STRUCT AS THE SHIFT REGISTER:
 *  (SHFTREG and SHFTREGbits are declared in 'beetle.h'; SHFTREG is initialized
 *   to '0' in 'main.c')
 */    
    unsigned char i, j;
    static unsigned char bitnum = 0;
    static char D;
    
    // SEED: initialize pseudo-random decision sequence with Timer1 counter
    //  (this timer drives the signal LEDs and its counter could be anywhere
    //   between 0x0 and 0xFFFF)
    if (SHFTREG == 0)   // prevent all zeros or we're "stuck" (0^0 = 0)
    {   if (TMR1L == 0 && TMR1H == 0)
        {   SHFTREG = 0xFF;}    // if all else fails, just bring everything HIGH
        else
        {   SHFTREG = (unsigned)(TMR1L|(TMR1H << 8));}
    }
    
    // Determine how many new bits of SHFTREG to generate
    if (type == "direction")
    {   j = 1;  }
    else if (type == "move")
    {   j = 3;  }
    else if (type == "degree" || type == "time")
    {   j = 15;  }  // refresh 'SHFTREG'
    
    // Generate 'j' bits
    for (i = 0; i < j; i++)
    {
        // increment
        bitnum++;
        if (bitnum >= 16)
        {   bitnum = 0;
        }        
        switch(bitnum)  // navigate the circular buffer
        {
            case 0: D = (unsigned)(SHFTREGbits.b ^ SHFTREGbits.c);
                SHFTREGbits.a = (unsigned)D;
                break;
            case 1: D = (unsigned)(SHFTREGbits.c ^ SHFTREGbits.d);
                SHFTREGbits.b = (unsigned)D;
                break;
            case 2: D = (unsigned)(SHFTREGbits.d ^ SHFTREGbits.e);
                SHFTREGbits.c = (unsigned)D;
                break;
            case 3: D = (unsigned)(SHFTREGbits.e ^ SHFTREGbits.f);
                SHFTREGbits.d = (unsigned)D;
                break;
            case 4: D = (unsigned)(SHFTREGbits.f ^ SHFTREGbits.g);
                SHFTREGbits.e = (unsigned)D;
                break;
            case 5: D = (unsigned)(SHFTREGbits.g ^ SHFTREGbits.h);
                SHFTREGbits.f = (unsigned)D;
                break;
            case 6: D = (unsigned)(SHFTREGbits.h ^ SHFTREGbits.i);
                SHFTREGbits.g = (unsigned)D;
                break;
            case 7: D = (unsigned)(SHFTREGbits.i ^ SHFTREGbits.j);
                SHFTREGbits.h = (unsigned)D;
                break;
            case 8: D = (unsigned)(SHFTREGbits.j ^ SHFTREGbits.k);
                SHFTREGbits.i = (unsigned)D;
                break;
            case 9: D = (unsigned)(SHFTREGbits.k ^ SHFTREGbits.l);
                SHFTREGbits.j = (unsigned)D;
                break;
            case 10: D = (unsigned)(SHFTREGbits.l ^ SHFTREGbits.m);
                SHFTREGbits.k = (unsigned)D;
                break;
            case 11: D = (unsigned)(SHFTREGbits.m ^ SHFTREGbits.n);
                SHFTREGbits.l = (unsigned)D;
                break;
            case 12: D = (unsigned)(SHFTREGbits.n ^ SHFTREGbits.o);
                SHFTREGbits.m = (unsigned)D;
                break;
            case 13: D = (unsigned)(SHFTREGbits.o ^ SHFTREGbits.p);
                SHFTREGbits.n = (unsigned)D;
                break;
            case 14: D = (unsigned)(SHFTREGbits.p ^ SHFTREGbits.a);
                SHFTREGbits.o = (unsigned)D;
                break;
            case 15: D = (unsigned)(SHFTREGbits.a ^ SHFTREGbits.b);
                SHFTREGbits.p = (unsigned)D;
                break;
            default:
                break;  // shouldn't happen
        }
    }
    // Return the desired random literal
    if (type == "direction")
        /* return 'L' or 'R' randomly */
    {
        // bit 0 means RIGHT TURN; bit 1 means LEFT TURN
        if (D == 0)
        {   return 'R'; }
        else
        {   return 'L'; }
    }
    else if (type == "degree")
        /* return a random integer between and including 192 and 511 */
    {
        if (SHFTREGbits.i == 0)
        {   return ((SHFTREG & 0x01FF) | 0x00C0);   }
        else
        {   return (SHFTREG & 0x01FF);  }
    }
    else if (type == "move")
        /* return ('move' parameters) 3, 4, 5, or 6 randomly */
    {
        unsigned int shftreg_temp = SHFTREG;
        unsigned char randnum;
        if (bitnum > 1)
        {   shftreg_temp >> (bitnum - 1);
        }
        else if (bitnum == 0)
        {   shftreg_temp >> 14;
        }
        // (if bitnum == 1, shftreg_temp can stay as is)
        // At this point, 'shftreg_temp' bits 0 & 1 have been newly generated
        
        randnum = (shftreg_temp & 0x0003);  // i.e. randnum = 0, 1, 2, or 3
        switch (randnum)
        {
            case 0:
                return 3;   // pivot clkws
            case 1:
                return 4;   // pivot cntrclkws
            case 2:
                return 5;   // turn right
            case 3:
                return 6;   // turn left
            default:
                return 0; // shouldn't happen
        }
    }
    else if (type == "time")
        /* return a random integer between and including 1536 and 8191 */
    {
        if (SHFTREGbits.l == 0)
        {   return ((SHFTREG & 0x1FFF) | 0x0600);   }
        else
        {   return (SHFTREG & 0x1FFF);  }
    }        
    else
        /* shouldn't happen */
    {   return 0;   }
    
    
 /* V.2 -- USING AN ARRAY AS THE SHIFT REGISTER:
  */  
//    unsigned char SHIFT[16] = {0};  //buffer array of 16 chars (put this
//                                      outside the function)
//    static char D;
//    
//    // INCREMENT (i.e. shift the register)
//    D++;
//    if (D >= 16)
//    {   D = 0;}    
//    
//    //**************************************************************************
//    // SEED: initialize pseudo-random decision sequence with Timer1 counter 
//    //  (this timer drives the signal LEDs and its counter could be anywhere
//    //   between 0 and 65535)
//    // (Ideally this segment of code only runs once)
//    
//        // prevent all zeros or we're "stuck" (0^0 = 0)
//    if (   SHIFT[0]  == 0 && SHIFT[1]  == 0 && SHIFT[2]  == 0 && SHIFT[3]  == 0
//        && SHIFT[4]  == 0 && SHIFT[5]  == 0 && SHIFT[6]  == 0 && SHIFT[7]  == 0
//        && SHIFT[8]  == 0 && SHIFT[9]  == 0 && SHIFT[10] == 0 && SHIFT[11] == 0
//        && SHIFT[12] == 0 && SHIFT[13] == 0 && SHIFT[14] == 0 && SHIFT[15] == 0
//       )    
//    {   unsigned char jj;
//        // if all else fails, just bring all 'SHIFT' elements HIGH:
//        if (TMR1L == 0 && TMR1H == 0)
//        {   for( jj = 0; jj <= 15; jj++) 
//            {   SHIFT[jj] = 1;
//            }
//        }
//        // 'SHIFT[] = {TMR1L(0), TMR1L(1), ... TMR1H(7)}'
//        else
//        {   unsigned int tmrcntr = (unsigned)(TMR1L|(TMR1H << 8));
//            // for all 'SHIFT' elements:
//            for (jj = 0; jj <= 15; jj++)    
//            {       // find out whether bit 'jj' of 'tmrcntr' is 1 or 0
//                if ((tmrcntr & ((unsigned int)1 << jj)) == 0)
//                    // and overwrite accordingly
//                {   SHIFT[jj] = 0;}
//                else
//                {   SHIFT[jj] = 1;}
//            }
//        }
//    }//*************************************************************************
//    
//    // XOR the feedback taps, and overwrite 'D' with the result
//    unsigned char m = SHIFT[(unsigned)(D + 1)];
//    unsigned char n = SHIFT[(unsigned)(D + 2)];    
//    SHIFT[D] = (unsigned)(m ^ n);    
//    
//    // bit 0 means RIGHT TURN; bit 1 means LEFT TURN
//    if (SHIFT[D] == 0)
//    {   return 'R';}
//    else
//    {   return 'L';}  
}