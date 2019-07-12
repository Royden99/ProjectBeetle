
#include <xc.h>
#include "beetle.h"

/*  LIGHT DEPENDENT RESISTOR (LDR) SENSORS:
 * 
 * < Modules 1, 2, 3, 4, 5, 6 (collision detectors)>
 *  A single square wave is output to six LED's in various positions on the
 *      robot's front and rear bumpers.
 *  Next to each LED is a photosensitive resistor in a potential divider circuit
 *      such that the resulting voltage level corresponds to the ambient light 
 *      detected.
 *  Analog data is collected through channels AN14-19 from these circuits;
 *      for each module in turn, function 'signal()' converts the data to
 *      digital and analyzes it over time.
 *  Should the robot come into contact with a sufficiently reflective object,
 *      light from an LED will be reflected into its corresponding photo-
 *      resistor. Then a wave should be detected in the incoming data for that
 *      module, at the same frequency that drives the LED's.
 * < Modules 7, 8 (wheel rotation sensors) >
 *  These modules are exactly the same as the others physically, but differ in
 *      their purpose:
 *  Each wheel of the robot is fixed on the end of a bolt which serves as its 
 *      axle. There are two wheels, and their axles meet at the very center of
 *      the robot but are not fixed together. Here, the hexagonal bolt heads
 *      rotate as the wheels turn.
 *  A constant 5 volts is output to two LED's which illuminate the bolt heads;
 *      as a bolt head rotates, each of its six faces acts as a mirror which
 *      reflects light periodically into the module's photo-resistor.
 *  As long as the input data describes a constant wave, then, it can be 
 *      assumed that the wheel is rotating unhampered by outside objects.
 */

void start_signal(void)
{
/* startup sequence for CCP2 sq. wave output to pin RC1 (signal LED's)
 *  (duty cycle 50%; freq. 7.6295 Hz) 
 */     
    TRISC1 = 1;     //disable output pin temporarily
    CCPTMRS0 = 0x00;        //CCP2 capture/compare uses Timer1
    T1CON = 0b00110011;     //timer1 (fosc/4); (presc. 8); (16-bit); (ON).
    CCP2CON = 0b00000010;   //compare mode: toggle output on match
    CCPR2L = 0x00;    //}this number doesn't matter since TMR1 is not cleared
    CCPR2H = 0x00;    //}   upon TMR1-CCPR2 match
    while(TMR1IF == 0)      //wait one period
    {;}
    TRISC1 = 0;     //enable output pin 
    
/* preparation for signal detection */
    // CONFIGURE ADC*******************************
    // ADC setup
    ADCON2 = 0b10011010;    //right justified; ACQT = 6 Tad; clock = Fosc/32
                            //  (Tad = 1 us)
    ADCON1 = 0x00;          //Vref+ = Vdd;   Vref- = Vss
    
    // CONFIGURE Timer4****************************
    // interrupt flag set every (PR4 * presc. * postsc.) = 4160 instruction
    //  cycles  (i.e. 520 us  @ 32 MHz)
    PR4 = 0x68;             // PR4 = decimal '104'
    T4CON = 0b01001101;     // presc. 4, postsc. 10, timer4 on
    TMR4IF = 0;             // ensure flag bit is clear    
}

void stop_signal(void)
/* Clear all the timers & output latches used by 'signal()' */
{
    T1CON = 0x00;
    TMR1IF = 0;
    T4CON = 0x00;
    TMR4IF = 0;
    CCP2CON = 0x00;
    LATC1 = 0;    
}


//************** static variables for signal() -- initialized only once ********
/* For every photosensor module, there exists one of each following variable:
 * 
 * - 'count' is an indication of the length of time between data points
 * - 'MID' tracks the midpoint (and reference point) of circular queue 'LDRSIG'
 * - array 'LDRSIG' accumulates data points from Light Dependent Resistor
 *      potential divider SIGnal, via ADC converter
 * - array 'GRDNT' elements are 1, 0, or -1 depending on the gradient between
 *      'LDRSIG' elements
 */
static unsigned int
    count1 = 0, count2 = 0, count3 = 0, count4 = 0, count5 = 0, count6 = 0,
    count7 = 0, count8 = 0,
    LDRSIG1[21] = {0}, LDRSIG2[21] = {0}, LDRSIG3[21] = {0}, LDRSIG4[21] = {0},
    LDRSIG5[21] = {0}, LDRSIG6[21] = {0}, LDRSIG7[21] = {0}, LDRSIG8[21] = {0};
static signed char
    MID1 = 10,  MID2 = 10,  MID3 = 10,  MID4 = 10,  MID5 = 10,  MID6 = 10,
    MID7 = 10, MID8 = 10,
    GRDNT1[21] = {0}, GRDNT2[21] = {0}, GRDNT3[21] = {0}, GRDNT4[21] = {0},
    GRDNT5[21] = {0}, GRDNT6[21] = {0}, GRDNT7[21] = {0}, GRDNT8[21] = {0};

/* - array 'SPNTS' logs any Stationary PoiNTS (peaks and troughs)
 *      discovered in LDRSIG data
 */
static struct spnts
{	unsigned int v_level; // digital voltage value
    char         count;   // no. of counts since the previous stationary point
}   SPNTS1[2] = {0}, SPNTS2[2] = {0}, SPNTS3[2] = {0}, SPNTS4[2] = {0},
    SPNTS5[2] = {0}, SPNTS6[2] = {0}, SPNTS7[2] = {0}, SPNTS8[2] = {0};
//******************************************************************************

void signal(unsigned char module_no)
/*  'module_no' (1, 2, 3, 4, 5, 6, 7, or 8) specifies which photosensor
 *      module to analyze.  A meaningless value for 'module_no' does nothing.
 * <collision detectors> (mod. 1-6)
 *  [~ 3 milliseconds (or 6 TMR4IF events) per module]
 * This function, when called ~every 520 microseconds, evaluates the data from
 *  photosensor modules 1-6 via the ADC module (registers ADRESH:ADRESL) 
 *  searching for a frequency of 7.6295 Hz.  As long as it cannot be found,
 *  SIGNAL = 0; when the target frequency is consistently apparent, SIGNAL
 *  equals a value indicating its strength.
 * <wheel rotation sensors> (mod. 7, 8)
 *  [~ 5.8 milliseconds (or 3 Timer2 interrupts) per module]
 * Similar process to 'collision detectors.'  A consistent 'SIGNAL == 1' means
 *  the wheels are turning.
 */
{   // * auto pointers in place of their static counterparts:
    unsigned int *count, *SIGNAL, *LDRSIG;
    signed char  *GRDNT, *MID;
    struct spnts *SPNTS;   
    
    // * local vars, OK to be reset every function call:
	//  * 'L' and 'R' are slope detection variables
	//  * 'j' and 'k' are used in for loops
	signed int L, R, j, k;	
	//  * signal detection flag for collision detector modules
	char SIG_D = 0;
        
    // (1)process the analog channel; (2)assign local pointers to the addresses
    //  of the variables and arrays of the module being analyzed
    switch (module_no)
    {   case 1:
            convert_channel(0x0E);
            count  = &count1;
            MID    = &MID1;
            SIGNAL = &LDR1;
            LDRSIG = LDRSIG1;
            GRDNT  = GRDNT1;
            SPNTS  = SPNTS1;
            break;
        case 2:
            convert_channel(0x0F);
            count  = &count2;
            MID    = &MID2;
            SIGNAL = &LDR2;
            LDRSIG = LDRSIG2;
            GRDNT  = GRDNT2;
            SPNTS  = SPNTS2;
            break;
        case 3:
            convert_channel(0x10);
            count  = &count3;
            MID    = &MID3;
            SIGNAL = &LDR3;
            LDRSIG = LDRSIG3;
            GRDNT  = GRDNT3;
            SPNTS  = SPNTS3;
            break;
        case 4:
            convert_channel(0x11);
            count  = &count4;
            MID    = &MID4;
            SIGNAL = &LDR4;
            LDRSIG = LDRSIG4;
            GRDNT  = GRDNT4;
            SPNTS  = SPNTS4;
            break;
        case 5:
            convert_channel(0x12);
            count  = &count5;
            MID    = &MID5;
            SIGNAL = &LDR5;
            LDRSIG = LDRSIG5;
            GRDNT  = GRDNT5;
            SPNTS  = SPNTS5;
            break;
        case 6:
            convert_channel(0x13);
            count  = &count6;
            MID    = &MID6;
            SIGNAL = &LDR6;
            LDRSIG = LDRSIG6;
            GRDNT  = GRDNT6;
            SPNTS  = SPNTS6;
            break;
        case 7: // M1
            convert_channel(0x04);
            count  = &count7;
            MID    = &MID7;
            SIGNAL = &LDR7;
            LDRSIG = LDRSIG7;
            GRDNT  = GRDNT7;
            SPNTS  = SPNTS7;
            break;
        case 8: // M2
            convert_channel(0x0D);
            count  = &count8;
            MID    = &MID8;
            SIGNAL = &LDR8;
            LDRSIG = LDRSIG8;
            GRDNT  = GRDNT8;
            SPNTS  = SPNTS8;
            break;
        default:
            return;         
    }
    
// GATHER DATA
//  Update circular buffers 'LDRSIG[21]' and 'GRDNT[20]'
    //  Midpoint shifts one element to the right every function call
    if (*MID >= 20)  // at end of array:
        *MID = 0;    //  wrap back around to beginning of array
    else
        *MID += 1;     
    
    //  Overwrite oldest elements with new data
    if (*MID <= 10)
    /*oldest elements of LDRSIG and GRDNT are >MID but <20*/
    {   //LDRSIG:
        *(LDRSIG + ((*MID)+10)) = (((unsigned int)ADRESH << 8) | ADRESL);
        
        //GRDNT:
        // when positive slope, GRDNT value = 1
        if      (*(LDRSIG + ((*MID)+9)) < *(LDRSIG + ((*MID)+10)))
        {   *(GRDNT + ((*MID)+9)) = 1;
        }
        // when negative slope, GRDNT value = -1
        else if (*(LDRSIG + ((*MID)+9)) > *(LDRSIG + ((*MID)+10)))
        {   *(GRDNT + ((*MID)+9)) = -1;
        }
        // zero slope
        else
        {   *(GRDNT + ((*MID)+9)) = 0;
        }
    }       // Wrapping around the buffer:
    else if (*MID == 11)
    /*oldest element of GRDNT is 20; of LDRSIG is 0 */
    {   //LDRSIG:
        *(LDRSIG + 0) = (((unsigned int)ADRESH << 8) | ADRESL);
        
        //GRDNT:
        // when positive slope, GRDNT value = 1
        if      (*(LDRSIG + 20) < *(LDRSIG + 0))
        {   *(GRDNT + 20) = 1;
        }
        // when negative slope, GRDNT value = -1
        else if (*(LDRSIG + 20) > *(LDRSIG + 0))
        {   *(GRDNT + 20) = -1;
        }
        // zero slope
        else
        {   *(GRDNT + 20) = 0;    
        }
    }
    else if (*MID >= 12)
    /*oldest elements of LDRSIG and GRDNT are < MID*/
    {   //LDRSIG:
        *(LDRSIG + ((*MID)-11)) = (((unsigned int)ADRESH << 8) | ADRESL);
        
        //GRDNT:
        // when positive slope, GRDNT value = 1
        if      (*(LDRSIG + ((*MID)-12)) < *(LDRSIG + ((*MID)-11)))
        {   *(GRDNT + ((*MID)-12)) = 1;
        }
        // when negative slope, GRDNT value = -1
        else if (*(LDRSIG + ((*MID)-12)) > *(LDRSIG + ((*MID)-11)))
        {   *(GRDNT + ((*MID)-12)) = -1;
        }
        // zero slope
        else
        {   *(GRDNT + ((*MID)-12)) = 0;
        }
    }
        
// LOG ANY LDRSIG[] DATA POINTS THAT LOOK LIKE STATIONARY POINTS (SPNTS);
//  INCREMENT 'count' FOR EVERY POINT IN BETWEEN;
//  LOOK AT RECORDED SPNTS[] FOR EVIDENCE OF (7.6 HZ) FREQUENCY;
//  WHEN DISCOVERED, 'SIGNAL' = SIGNAL STRENGTH

    switch (*count)
    // only record a stationary point if the last one was > 12 counts ago
    // 'count' is reset when a stationary point is recorded
    {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6:
        case 7: case 8: case 9: case 10: case 11:
        {   ++(*count);
            break;
        }
        default:
        {
            // DETERMINE OVERALL SLOPE
            // * SLOPE LEFT OF MIDPOINT
            // add together the 10 GRDNT points older than 'MID'; result is L
            L = 0;
            j = (*MID <= 9)? ((*MID)+11) : ((*MID)-10);
            while (j != *MID)
            {	L += *(GRDNT+j);
                j++;
                j = (j > 20)? (j-21) : j;   //wrap around to start of buffer
            }
            // * SLOPE RIGHT OF MIDPOINT
            // add together the 10 GRDNT points equal to and newer than 'MID'; 
            //  result is R
            R = 0;
            j = *MID;
            k = (*MID >= 11)? ((*MID)-11) : ((*MID)+10);
            while (j != k)
            {   R += *(GRDNT+j);
                j++;
                j = (j > 20)? (j-21) : j;   //wrap around to start of buffer
            }
            
            // LOG STATIONARY POINTS
            // is midpoint a stationary point?
            //  (slope either side of midpoint is +ve if > +5, and -ve if < -5)
            if(((L > 5) && (R < -5)) || ((L < -5) && (R > 5)))
            {
                // if yes:
                // * update SPNTS[] with the new voltage level and 'count'
                //     * replace [0] with [1]
                *SPNTS = *(SPNTS + 1);
                //     * replace [2] with the new values
                (SPNTS + 1)->v_level = *(LDRSIG + 11);
                (SPNTS + 1)->count = *count;
                                
                // * SIGNAL DETECTION:
                //  - <wheel rotation sensor modules>
                if (module_no == 7 || module_no == 8)
                {   // if program execution reaches this far (SPNTS logged),
                    //  then we have success!
                    *SIGNAL = 1;
                }
                
                //  - <collision detector modules>
                else
                {   // These are more finicky:
                    //  check that both stationary points are the expected
                    //      distance from the previous one(~ 21 counts)
                    for(j = 0; j <= 1; j++)
                    {	// signal frequency not detected
                        if (((SPNTS + j)->count) < 18 || ((SPNTS + j)->count) > 25)
                        {   SIG_D = 0;
                        }                                        
                        // signal frequency detected 
                        else
                        {	++SIG_D;
                        }
                    }                
                    // when signal detected, assign SIGNAL a value indicating its
                    //  strength (i.e. SIGNAL = difference between the two SPNTS[]
                    //  voltage levels)
                    if (SIG_D == 2)
                    {   
                        if ((SPNTS + 0)->v_level > (SPNTS + 1)->v_level)
                        {	*SIGNAL = ((SPNTS + 0)->v_level - (SPNTS + 1)->v_level);
                        }
                        else
                        {	*SIGNAL = ((SPNTS + 1)->v_level - (SPNTS + 0)->v_level);
                        }
                    }
                }                
                // * reset count when a stationary point is logged
                *count = 0;
            }
            // if not a stationary point, increment 'count':
            else
            {	++(*count); }
            break;
        }
    }   /*end of switch(count)*/
    
    // if 'count' increments above 42 ('SIGNAL' hasn't been detected for one
    //  period), reset 'count' and all SPNTS[] elements; and SIGNAL = 0
    if (*count >= 42)
    {	for (j = 0; j < 2; j++)
        {	if ((SPNTS + j)->v_level != 0 || (SPNTS + j)->count != 0)
            {   (SPNTS + j)->v_level = 0;
                (SPNTS + j)->count   = 0;
            }
        }
        if (module_no == 7 || module_no == 8)
        {   // don't spring "wheel stuck" signal until after 1/3 revolution
            //  i.e. give the module time to rack up at least 2 'SPNTS'
            if (bb > 300)
            {   *SIGNAL = 0;  }
        }
        else
            // no such restriction for collision detector LDR's
        {   *SIGNAL = 0;    }
    
        *count = 0;
    }
}

