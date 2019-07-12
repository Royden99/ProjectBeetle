/*
 * File:   main.c
 * Author: Royden
 * 
 */

//******PIC18F26K22 Configuration Bit Settings**********************************

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTC6  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is mulitplexed with RC6)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//********************* include ************************************************
#include <pic18f26k22.h>
#include <xc.h>
#include "beetle.h"

//********************* extern functions ***************************************
// sensory
extern void         start_signal(void);
extern void         stop_signal(void);
extern void         signal(unsigned char);
// motor control
extern void         sing(const char[]);
extern void         move(char, const char[]);
extern void         pivot(unsigned int, unsigned int);
extern unsigned int rand(const char[]);
// main
extern bit          Dbounce_us(volatile unsigned char *, char);
extern void high_priority interrupt T6 (void);
extern void low_priority  interrupt T2 (void);

//********************* global vars definition *********************************
unsigned int b1_1, b1_2, b2_1, b2_2;
volatile bit M1, M2;
volatile unsigned int STATE = 0x00;    
unsigned int LDR1 = 0, LDR2 = 0, LDR3 = 0, LDR4 = 0, LDR5 = 0, LDR6 = 0,
    LDR7 = 1, LDR8 = 1;
volatile unsigned char aa = 0, cc = 0;
volatile unsigned int bb = 1;
unsigned int bb_stop = 0;
unsigned char waiting = 'n';
unsigned char prev_mode = 0;
volatile unsigned int SHFTREG = 0x00;
bit Dbounce_in_progress = 0;


int main(void) 
{  
//************************* RESET **********************************************
    // system clock frequency: (8MHz)(x4 PLL) = 32 MHz
    OSCCON  = 0b01101000;   // i.e. 1 instruction cycle = 125 ns
    PLLEN = 1;              // enable 4x PLL
    
    // global interrupt settings
    IPEN = 0;   // priority levels disabled
    GIEH = 1;   // all interrupts enabled
    GIEL = 1;   // all peripheral interrupts enabled
    
    // PORTS:
    // port A(0-4, 6, & 7) configured as dig. outputs (to stepper motors);
    //  RA5 configured as analog input (M1 "wheel stuck" photosensor)
    TRISA = 0x20;
    ANSELA = 0x20;
    // port B(0-4, 6, & 7) configured as dig. inputs (pushbuttons);
    //  RB5 configured as analog input (M2 "wheel stuck" photosensor)
    TRISB = 0xFF;
    ANSELB = 0x20;
    // led's & photosensor inputs: pins RC2-7 analog in; RC0-1 dig. out
    TRISC = 0xFC;
    ANSELC = 0xFC;

    // say Hello (and IMPORTANT: initialize stepper motors to OFF)
    sing("on");
    
    // flash LED's
    LATC1 = 1; LATC0 = 1;
    __delay_ms(200);
    LATC1 = 0; LATC0 = 0;
    __delay_ms(200);
    LATC1 = 1; LATC0 = 1;
    __delay_ms(200);
    LATC1 = 0; LATC0 = 0;
    
    // master pushbuttons setup
    IOCBbits.IOCB6 = 1; // interrupt on change pins RB6 & 7 enabled
    IOCBbits.IOCB7 = 1;
    RBIF = 0;           // flag clear
    
    /* Battery level--------------------------------
     *  An indicator voltage drives port RB3 (comparator input channel C12IN2-).
     *  Comparator module 1 monitors this voltage with respect to (DAC) 1.41v.
     *  SHUTDOWN is triggered when C1IF is set and it passes a filter test.
     */
    // Setup & enable DAC module:
    VREFCON1 = 0b10000000;  //enabled
    VREFCON2 = 0x0C;        //DACR '12'  i.e. Vref ~ 1.875v
    // Enable Comparator1:
    //  C1ON => enabled; C1OUT => 0; C1OE => disabled; C1POL => not inverted;
    //  C1SP => high-speed mode; C1R => C1Vref output; C1CH => C12IN2-(pin RB3)
    CM1CON0 = 0b10001110;
    // Rout DAC output to C1 voltage reference pin (+ve input)
    C1RSEL = 0;
    // Wait for comparator response time & bias circuitry settling (1 us)
    __delay_us(1);
    // Read CM1CON0  i.e. clear mismatch latches
    CM1CON0;
    // Interrupt flag (IOC disabled): 
    C1IF = 0;    //  clear flag
    C1IE = 0;
    //----------------------------------------------
        
    // mainloop local variables
    static bit    active      = 0; // toggled by master pushbutton 1
    unsigned char module      = 0; // track 'signal()' module(1-6)
    static bit    do_mod_8    = 0;
    unsigned int  reaction    = 0; 
    unsigned int  turntime    = 0;
    unsigned int  state       = 0; // holds the previous 'STATE'
    unsigned char mpb_state   = 0; // master_push_button "who-done-it"
    unsigned char mode        = 0; // for use in reaction 'fun' 
        /*  Dbouncing:    */
    volatile unsigned char *SFR;   // pointer to a special function register
    unsigned char           BIT;   // bit(0-7) of 'SFR'
    unsigned char sample_time = 0; // track when to sample 'BIT'
    
//***************************** MAINLOOP ***************************************    
    
    while(1) 
    {        
        // PROCESS LDR SENSOR INPUTS
        /* Process collision detectors anytime Timer4 is running
         *  i.e. if start_signal() has been called
         */
        if (TMR4IF == 1)
        {   TMR4IF = 0; 
            ++module;     
            if (module >= 7)
            {   module = 1;}
            signal(module);     // signal() 1-6           
        }
        /* Only process wheel rotation sensors:
         *  When Beetle is in active mode           (active == 1)
         *  When there is no reaction taking place  (reaction == 0)
         *  Every 3 Timer2 interrupts               (cc == 3)
         */
        if (do_mod_8 == 1) // process module 8 next time around mainloop
        {   signal(8);
            do_mod_8 = 0;
        }
        if (active == 1 && reaction == 0 && cc == 3)
        {   signal(7);     // process module 7 now
            cc = 0;
            do_mod_8 = 1;
        }
                
        // EVENT FLAGS  i.e. UPDATE 'STATE'  i.e. CHECK ALL SIGNALS
            // remember the state of STATE before updating
        state = STATE;
            // * FRONT RIGHT
        STATEbits.l1 = (LDR1 == 0)? (unsigned)0 : 1; 
        STATEbits.p1 = (PB1 == 0)?  (unsigned)0 : 1; 
        
            // * FRONT MIDDLE
        STATEbits.l2 = (LDR2 == 0)? (unsigned)0 : 1;
       
            // * FRONT LEFT
        STATEbits.p2 = (PB2 == 0)?  (unsigned)0 : 1;
        STATEbits.l3 = (LDR3 == 0)? (unsigned)0 : 1;
        
            // * BACK LEFT
        STATEbits.l4 = (LDR4 == 0)? (unsigned)0 : 1;
        STATEbits.p3 = (PB3 == 0)?  (unsigned)0 : 1;
        
            // * BACK MIDDLE
        STATEbits.l5 = (LDR5 == 0)? (unsigned)0 : 1;
        
            // * BACK RIGHT
        STATEbits.p4 = (PB4 == 0)?  (unsigned)0 : 1;
        STATEbits.l6 = (LDR6 == 0)? (unsigned)0 : 1;
        
            // * RIGHT WHEEL STUCK
        STATEbits.l7 = (LDR7 == 1)? (unsigned)0 : 1;
        
            // * LEFT WHEEL STUCK
        STATEbits.l8 = (LDR8 == 1)? (unsigned)0 : 1;
        
            // * SOFTWARE SIGNAL 'END OF REACTION'
        STATEbits.done = (bb == bb_stop)? (unsigned)1 : 0;
        
        
        // PERFORM REACTIONS BASED ON 'STATE'
        //      react only upon a change in STATE, if active
        if (STATE != state && active == 1)
        {   switch(STATE)
            /* Each 'case' is a Reaction to an Event or combination of events 
             *  which is deemed worthy of notice.
             * All other input ('distractions') are sent straight to 'default'
             *  and ignored.
             * Reactions are first initialized with a "trigger" macro, in which
             *  a non-0 value is given to 'bb_stop'; when 'bb' reaches this
             *  value ('bb' increments in Timer2 Interrupt), the reaction is
             *  continued or finished under 'case 4096'. 
             *  This sequence can last as many times as necessary.
             */
            {   // no signal detected:
                case 0:
                    // this happens when STATE changes from non-0 to 0
                    break;
                // LDR1
                case 1:
                    trigger_front(1);
                    break;
                // PB1 & similar
                case 2:   case 3:   case 5:   case 6:   case 7:   case 23: 
                    trigger_front(2);
                    break;
                // LDR2 & similar
                case 4:   case 21:
                    trigger_front(4);
                    break;
                // PB2 & similar
                case 8:   case 12:  case 20:  case 24:  case 28:  case 29:
                    trigger_front(8);
                    break;
                // LDR3
                case 16:
                    trigger_front(16);
                    break;                    
                // LDR4
                case 32:
                    trigger_rear(32);
                    break;                    
                // PB3 & similar
                case 64:  case 98:  case 160: case 192: case 224: case 736:
                    trigger_rear(64);
                    break;                    
                // LDR5 & similar
                case 128: case 672:
                    trigger_rear(128);
                    break;                
                // PB4 & similar
                case 256: case 384: case 640: case 768: case 896: case 928:
                    trigger_rear(256);
                    break;                    
                // LDR6
                case 512:
                    trigger_rear(512);
                    break;
                // LDR7 (left wheel stuck), LDR8 (right wheel stuck), or both
                case 1024:  case 2048:  case 3072:
                    trigger_front(1024);   
                    LDR7 = 1;
                    LDR8 = 1;                 
                    break;
                    
                // software signal "done"
                case 4096:
                    switch(reaction)
                    {   // finish a collision reaction, having reversed                        
                        case 2: case 4: case 8: case 64: case 128: case 256:
                        case 1024:
                            pivot(rand("direction"), rand("degree"));
                            break;                            
                        case 16: case 512:
                            pivot('R', rand("degree"));
                            break;                            
                        case 1: case 32:
                            pivot('L', rand("degree"));
                            break;
                        
                        // go forward
                        case 'g':
                            bb_stop = 0;    // stop variable is out of reach
                            LATC0 = 1;      // LED on
                            move(1, "now"); // proceed forward
                            reaction = 0;
                            turntime = rand("time");
                            break;
                            
                        // stop
                        case 's':
                            bb_stop = 0;
                            LATC0 = 0;
                            move(0, "now");
                            break;
                            
                        // just for fun
                        case 'f':
                            // perform all move() modes, then stop and sing()
                            move(mode, "wait");
                            bb_stop = 410;
                            if (mode == 0)  // time to stop
                            {   reaction = 0;
                                bb_stop = 0;
                                sing("stop");
                                active = 0;
                            }
                            mode++;
                            if (mode >= 9)
                            {   mode = 0;
                            }
                            break;
                            
                        default:    // shouldn't happen
                            break;
                    }
                    // Do this if not finished yet
                    if (reaction != 0 && reaction != 'f')
                    {   reaction = 'g';     // finish next time
                    }
                    break;
                    
                // "hanging state"  i.e. signal 'done' + a hardware signal
                case 4097:  case 4098:  case 4099:  case 4100:  case 4101:
                case 4102:  case 4103:  case 4104:  case 4119:  case 4117:
                case 4108:  case 4116:  case 4120:  case 4124:  case 4125:
                case 4112:  case 4128:  case 4160:  case 4194:  case 4256:
                case 4288:  case 4320:  case 4832:  case 4224:  case 4768:
                case 4352:  case 4480:  case 4736:  case 4864:  case 4992:
                case 5024:  case 4608:  case 5120:  case 6144:  case 7168:
                           
                    move(2, "wait");    // play it safe and reverse
                    LDR7 = 1;
                    LDR8 = 1;
                    break;
                    
                // anything else  i.e. 'distractions'
                default:
                    break;
            }
        }
        // A MECHANISM TO WAIT A MOMENT BEFORE MOVING
        if (TMR2IF == 1 && waiting != 'n')
        // while waiting: count Timer2 interrupt flags to keep track of time
        {   TMR2IF = 0;
            ++waiting; 
            if (waiting == 40) // condition 'proceed' (~500 ms wait time)
            // configure Timer2 interrupt now for move()
            {   waiting = 'n';
                // interrupt period = 1920 us (i.e. 15360 cyc)
                T2CON = 0b00100111;        //[presc. = 1:16]; [postsc. = 1:5]
                PR2 = 0xC0;                // decimal '192'
                // interrupt enabled, flag LOW
                TMR2IE = 1; 
                TMR2IF = 0;
                // enable motor logic inverter
                IEN    = 1;
            }
        }        
        // AFTER ~ 3 to 15 SECONDS OF SMOOTH DRIVING:
        if (bb == turntime && active == 1 && reaction == 0)
        // randomly turn or pivot
        {   move(rand("move"), "now");
            bb_stop = rand("degree");
            reaction = 'g';
        }
        
        // USER INTERFACE
        /*  <Detect master pushbutton events> */
        if (RBIF == 1)
        {   // only initialize Dbounce routine if one is not already running
            if (Dbounce_in_progress == 0)
            {
                // find out whether the culprit is PORTB6 or 7
                mpb_state = (unsigned)(PORTB & 0xC0);

                // PORTB6:
                if (mpb_state == 0x40)
                {   // Initialize de-bounce procedure for PORTB6
                    Dbounce_ms(&PORTB, 6)
                }
                // PORTB7:
                else if (mpb_state == 0x80)
                {   // Initialize de-bounce procedure for PORTB7
                    Dbounce_ms(&PORTB, 7)                        
                }
                // any other results  e.g. PORTB6 & 7 simultaneously:
                //  do nothing
                else {;}
            }
            // reset
            RBIF = 0;
        }        
        /*  <De-bouncing & command execution> */
        /*  'Dbounce_ms' works much like 'Dbounce_us' (located in 
         *       MainFunctions.c) except that it is split into several pieces
         *       which live in 'MAINLOOP()', instead of being in a single
         *       function.
         *      This is because it filters out any voltage spikes in the range
         *       of a few milliseconds instead of a few hundred microseconds,
         *       i.e. it needs more time to complete.
         *  1) Initialization:
         *      set '*SFR' and 'BIT' to point to the logic level bit in question
         *      set up Timer6 and poll for its interrupt flag
         *  2) Sampling:
         *      sample bit 'BIT' of register 'SFR' every 'sample_time'
         *       increments of TMR6
         *  3) Results:
         *      if 'BIT' always reads 'HIGH', then TMR6IF will be set and the
         *          'Success' routine executed
         *      if 'BIT' reads 'LOW' even once, then the 'FalseAlarm' routine
         *          is executed
         */ 
        if (Dbounce_in_progress == 1)
        {   
            // <Success:> ('BIT' stayed consistently HIGH)
            if (TMR6IF == 1)
            {   // clean up
                T6CON = 0x00;
                TMR6 = 0x00;
                TMR6IF = 0;
                Dbounce_in_progress = 0;
                
                // MasterPushButton Commands------------------------------------
                /*  PushButton1 (PORTB6)    */
                if (BIT == 6)
                {   // stop/start
                    if (active == 0)        // currently stopped:
                    {   start_signal();     //  start
                        LATC0 = 1;
                        sing("start");
                        move(1, "now");
                        active = 1;
                        turntime = rand("time");
                    }
                    else if (active == 1)   // currently active:
                    {   move(0, "now");     //  stop
                        stop_signal();
                        LATC0 = 0;
                        sing("stop");
                        bb_stop = 0;
                        reaction = 0;
                        active = 0;
                        turntime = 0;
                    }          
                }
                /*  PushButton2 (PORTB7)    */
                else if (BIT == 7)
                {   // stop if currently active
                    if (active == 1)
                    {   move(0, "now");
                        stop_signal();
                        LATC0 = 0;
                        sing("stop");
                        bb_stop = 0;
                        reaction = 0;
                        active = 0;
                        turntime = 0;
                    }
                    // otherwise showcase what beetle can do
                    else if (active == 0)
                    {   sing("start");
                        mode = 1;
                        reaction = 'f'; // set reaction 'fun' in motion
                        bb_stop = 1;    // trigger signal 'done'
                        active = 1;
                    }
                }
            }   //--------------------------------------------------------------
            else
            {   // ready for next sample?
                if(TMR6 >= sample_time)
                {   // sample 'BIT': if successful, set the next sample time 
                    if ((*SFR & (1 << BIT)) != 0)
                    {   // perform a sample ~ every 5 TMR6 increments
                        //  i.e. 51 times out of 255 (~10 ms / 51)
                        sample_time += 5;
                    }
                    // <False Alarm:> ('BIT' went LOW)
                    else
                    {   // clean up
                        T6CON = 0x00;
                        TMR6 = 0x00;
                        Dbounce_in_progress = 0;
                    }
                }
            }
        }        
        // MONITOR BATTERY LEVEL
        if (C1IF == 1)
        {   // attempt to rule out small voltage spikes
            if (Dbounce_us(&CM2CON1, 7) == 1)
            {   /* <Shutdown:>
                 * stop any running processes;
                 * make all pins digital outputs @ 0
                 *  except make sure motors aren't sinking current;
                 * SLEEP
                 */
                
                move(0, "now");
                sing("off");
                
                //Peripheral Module Disable: (stop the clock to all peripherals)
                PMD0 = 0xFF;
                PMD1 = 0xFF;
                PMD2 = 0xFF;

                // All pins LOW digital outputs
                TRISA = 0x00;
                TRISB = 0x00;
                TRISC = 0x00;
                ANSELA = 0x00;
                ANSELB = 0x00;
                ANSELC = 0x00;
                LATA = 0x03;    // except for LA0 & LA1: HIGH digital outputs
                LATB = 0x00;
                LATC = 0x00;

                SLEEP();
            }            
            C1IF = 0;
        }
        
    } /*end of mainloop*/ 
    return 0;   // shouldn't happen
}
