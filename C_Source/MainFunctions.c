#include <xc.h>
#include "beetle.h"


bit Dbounce_us (volatile unsigned char *SFR, char BIT)
// De-bounce a LOW->HIGH signal from voltage spikes <= a few microseconds wide,
/*      by taking a closer look at bit 'BIT' of special function register 'SFR'
 *      to see whether it has been purposely set HIGH or triggered by noise.
 *      Assumes that if 'BIT' was set by noise, 'BIT' will not stay HIGH.
 *  Returns 0 if 'BIT' goes LOW.
 *  Returns 1 if 'BIT' remains consistently HIGH until TMR6 == PR6
 */
{   // abort if a separate debounce procedure is already happening
    //  (it needs Timer6 to not be over-written)
    if (Dbounce_in_progress == 1)
    {   return 0;    }
    
    // set T6 interrupt (poll for flag)
    TMR6IF = 0;    // clear flag
    PR6 = 0xFF;    // no. of instruction cycles until interrupt after Timer6 ON;
                   //  no more than 15360 i.e. motor interrupt frequency
    GIEH = 0;      // disable interrupts to avoid distraction
    T6CON = 0x04;  // prescaler 1; postscaler 1:1; Timer6 ON
    
    char ref = (unsigned)(1 << BIT);  // 'ref' points to the bit in question
    while(1)    // 1 iteration == 13 instruction cycles (Microchip Xc8 compiler)
    {
        // if 'BIT' goes LOW:
        if ((*SFR & ref) == 0)
        {
            T6CON = 0x00;   // Timer6 OFF & clear
            TMR6 = 0x00;
            GIEH = 1;       // re-enable interrupts
            return 0;
        }
        // if 'BIT' remains HIGH for the duration of Timer6:
        if (TMR6IF == 1)
        {
            T6CON = 0x00;   // Timer6 OFF & clear
            TMR6 = 0x00;
            GIEH = 1;       // re-enable interrupts
            return 1;
        }
    }
    // should never happen
    return 0;
}


void interrupt T2 (void)
{
    if(TMR2IF = 1)  // if TMR2 interrupt:
    /* set up next half-step of motor square wave control */
    {       
        TMR2IF = 0; 
        switch(aa)
        {   case 0: L1 = 0;         
                    m1ph1 = (M1 == 1)? 1 : 0;   // Ternary operators set phase
                    m2ph1 = (M2 == 1)? 1 : 0;   // latches to appropriate values
                    break;                      // depending on whether motors 
            case 1: L0 = 0;                     // are supposed to be on or off
                    break;          
            case 2: L0 = 1;         
                    m1ph2 = (M1 == 1)? b1_1 : 0;   
                    m2ph2 = (M2 == 1)? b1_2 : 0;   
                    break;          
            case 3: L1 = 1;         
                    break;          
            case 4: L1 = 0;         
                    m1ph1 = 0;      
                    m2ph1 = 0;      
                    break;          
            case 5: L0 = 0;         
                    break;          
            case 6: L0 = 1;         
                    m1ph2 = (M1 == 1)? b2_1 : 0;   
                    m2ph2 = (M2 == 1)? b2_2 : 0;   
                    break;          
            case 7: L1 = 1;         
                    break;    
        }        
        ++aa;
        if(aa >= 8) // reset once every motor phase period  (i.e. 8 half-steps)
        {   aa = 0; }
                
        ++bb;
        if (bb == bb_stop) // end of a reaction
        {   // effectively call 'move(0)'
            TMR2IE = 0;
            m1ph2 = 0;
            m2ph2 = 0;
            m1ph1 = 0;
            m2ph1 = 0;
            IEN = 0;   
            L0 = 0;
            L1 = 1;
        }
        
        // call signal(mod. 6 & 7) from mainloop every 3 T2 interrupts
        if (cc >= 3)
        {   cc = 0; }
        ++cc;
    }   
    
}