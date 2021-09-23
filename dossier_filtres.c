/*
 * FILTERS
 * 
 * Reads an audio file in Input, applies different filters to the signal and 
 * return the processed signal to the output.
 *  - moving average filter
 *  - low pass filter
 *  - high pass filter
 *  - echo effect
 * 
 * Microcontroller: PIC18F8722
 * Proteus File:    dossier_filtres.pdsprj
 * Author:          Samuel CIULLA
 *
 * Created on April 2021
 */

// PIC18F8722 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include "SPI_Alpha_LCD.h"


/*------------------------------------------------------------------------------
 * Global variables
 *----------------------------------------------------------------------------*/
double Fe=8000.0, Fs, K, W, A, B, Aamp, Bamp, Y0, Y1, Y2, POW7=7.0, cutoff=200.0;

signed int sy0=0, ay=0, by=0, cy=0, sy1=0, sy2=0, sy3=0, sy4=0, sy5=0, sy6=0;
signed int sx1=0, sx2=0, buf, X0=0, X1, buf_size=0, sy7=0, ra_out=0;

unsigned int first_done=0, read_sig=0, write_sig=0, delay=0, delay_ms=0;
unsigned int echo_weight=0, runavg_steps=2;

unsigned char echo_buf[3000];

char echo_amp[4] = "1/1";


/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
void PIC18_init(void);                      // PIC18 configuration
void GUI_settings(void);                    // initialise filters
void LCD_display(void);                     // handle LCD
void set_samplig_rate(void);                // set sampling rate
/// filters ///
void run_avg_filter_init(void);             // initialise running average filter
void run_avg_filter_exec(void);             // execute running average filter
void low_pass_filter_init(void);            // initialise low-pass filter
void low_pass_filter_exec(void);            // execute low-pass filter
void high_pass_filter_init(void);           // initialise high-pass filter
void high_pass_filter_exec(void);           // execute high-pass filter
void echo_filter_init(void);                // initialise echo effect
void echo_filter_exec(void);                // execute echo effect
void enable_filter(void);                   // execute selected filter


/*------------------------------------------------------------------------------
 * Interrupt CCP2 in compare special trigger event mode based on Timer1
 *----------------------------------------------------------------------------*/
void __interrupt() ccp1_interrupt() {
    
    INT_TICK = 1;
    
    ADCON0bits.NOT_DONE = 1;            // signal conversion
    while (ADCON0bits.NOT_DONE);
    X0 = ADRESH;
    
    enable_filter();
    
    INT_TICK = 0;
    PIR2bits.CCP2IF = 0;
}


/*------------------------------------------------------------------------------
 * Main program
 *----------------------------------------------------------------------------*/
void main(void) {
    
    PIC18_init();
    set_samplig_rate();
    Init_Alpha_LCD();
    GUI_settings();
    LCD_display();
    
    INTCONbits.GIE = 1;
    
    while (1);
}


/*------------------------------------------------------------------------------
 * Check PORTJ and set sampling rate accordingly
 * NOTE: 16KHz is ONLY available for the RUNNING AVERAGE FILTER
 *----------------------------------------------------------------------------*/
void set_samplig_rate(void) {
    
    if (PORTJbits.RJ0 == 1) {
        Fe = 16000.0;
        CCPR2 = 625;                    // 10000000/16000 = 625
    } else {
        Fe = 8000.0;
        CCPR2 = 1250;                   // 10000000/8000 = 1250
    }
}


/*------------------------------------------------------------------------------
 * Initialise µC PIC18F8722
 *----------------------------------------------------------------------------*/
void PIC18_init(void) {
    
    TRISD = 0x00;                       // DAC0808 output
    PORTD = 0x00;
    TRISJ = 0x01;                       // sample frequency
    TRISH = 0xFF;                       // parameters input
    PORTH = 0x00;
    TRISF = 0xFF;                       // echo weight
    PORTF = 0x00;
    
    CMCON = 0x07;                       // Comparators Off
    ADCON0 = 0x01;                      // ADC enabled
    ADCON1 = 0x0B;                      // AN0-AN2 analogic Input
    ADCON2 = 0x09;                      // justified left - 2Tad - Fosc/8
    TRISG = 0x00;
    TRISB = 0xF0;                       // switch input
    
    CCP2CON = 0x0B;                     // CCP2 ON - Special event trigger
    T3CON = 0x00;                       // Timer1 ON
    T1CON = 0x01;                       // Timer1 pre-scaler 1:1
    
    INTCONbits.PEIE = 1;                // enable peripheral interrupts
    PIR2bits.CCP2IF = 0;                // clear CCP2 interrupt flag
    PIE2bits.CCP2IE = 1;                // enable CCP1 interrupt
    
    TMR1 = 0;
}


/*------------------------------------------------------------------------------
 * Control Menu modes and parameters through different ports and buttons
 * depending on parameters sets onto the hardware
 *----------------------------------------------------------------------------*/
void GUI_settings(void) {

    switch (PORTB) {
        
        case 1:
            run_avg_filter_init();
            break;
        
        case 2:
            low_pass_filter_init();
            break;
            
        case 3:
            high_pass_filter_init();
            break;
            
        case 4:
            echo_filter_init();
            break;
    }
}


/*------------------------------------------------------------------------------
 * Set display data
 *----------------------------------------------------------------------------*/
void LCD_display(void) {
    
    char txt[21];
    
    switch (PORTB) {
        case 0:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("CONFIGURATION", 1);
            sprintf(txt, "Sample Rate: %5.0fHz", Fe);
            Send_Txt_LCD(txt, 2);
            break;
        case 1:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("RUNNING AVG FILTER", 1);
            sprintf(txt, "Steps: %d", runavg_steps);
            Send_Txt_LCD(txt, 2);
            break;
        case 2:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("LOW-PASS FILTER", 1);
            sprintf(txt, "Cutoff: %5.0fHz", cutoff);
            Send_Txt_LCD(txt, 2);
            break;
        case 3:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("HIGH-PASS FILTER", 1);
            sprintf(txt, "Cutoff: %5.0fHz", cutoff);
            Send_Txt_LCD(txt, 2);
            break;
        case 4:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("ECHO EFFECT", 1);
            sprintf(txt, "Delay    : %3dms", delay_ms);
            Send_Txt_LCD(txt, 2);
            sprintf(txt, "Amplitude:  %s", echo_amp);
            Send_Txt_LCD(txt, 3);
            break;
        default:
            Send_Cmd_LCD(LCD_CLEAR);
            Send_Cmd_LCD(LCD_CUR_OFF);
            Send_Txt_LCD("NOT SUPPORTED", 1);
            break;
    }
}


/*------------------------------------------------------------------------------
 * Enable filters/effects execution
 *----------------------------------------------------------------------------*/
void enable_filter(void) {

    switch (PORTB) {
        case 1:
            run_avg_filter_exec();
            break;
        case 2:
            low_pass_filter_exec();
            break;
        case 3:
            high_pass_filter_exec();
            break;
        case 4:
            echo_filter_exec();
            break;
    }
}


/*------------------------------------------------------------------------------
 * Echo filter initialisation
 * *** SAMPLING RATE ISSUE ***
 *----------------------------------------------------------------------------*/
void echo_filter_init(void) {
    
    switch (PORTH) {
        // echo delay
        case 0:
            delay=500;
            delay_ms = 62;                
            break;
        case 1:
            delay=1000;
            delay_ms = 125;
            break;
        case 2:
            delay=2000;
            delay_ms = 250;
            break;
        case 3:
            delay=3000;
            delay_ms = 375;
            break;
    }
    
    switch (PORTF) {
        // echo weight
        case 0:
            Aamp = 0.5;                 // source signal amplitude
            Bamp = 0.5;                 // echo signal amplitude
            sprintf(echo_amp, "1/1");
            break;
        case 1:
            Aamp = 0.67;
            Bamp = 0.33;
            sprintf(echo_amp, "2/1");
            break;
        case 2:
            Aamp = 0.75;
            Bamp = 0.25;
            sprintf(echo_amp, "3/1");
            break;
        default:
            Aamp = 0.67;
            Bamp = 0.33;
            sprintf(echo_amp, "2/1");
            break;
    }
            
    ay = (signed int)ceil(Aamp * pow(2.0, POW7));
    by = (signed int)ceil(Bamp * pow(2.0, POW7));
}


/*------------------------------------------------------------------------------
 * Echo filter execution
 * *** SAMPLING RATE ISSUE ***
 *----------------------------------------------------------------------------*/
void echo_filter_exec(void) {

    echo_buf[read_sig++] = X0;
    
    if (read_sig == delay) {
        read_sig = 0;
        first_done++;
    }
    if (first_done > 0) {
        X1 = echo_buf[write_sig++];
        if (write_sig == delay) write_sig = 0;
    }

    Y1 = (ay * X0 + by * X1) >> (int)POW7;
    PORTD = Y1;
}


/*------------------------------------------------------------------------------
 * High pass filter initialisation
 *----------------------------------------------------------------------------*/
void high_pass_filter_init(void) {
    
    switch (PORTH) {
        case 0:
            cutoff=200.0;
            break;
        case 1:
            cutoff=500.0;
            break;
        case 2:
            cutoff=1000.0;
            break;
        case 3:
            cutoff=2000.0;
            break;
        case 4:
            cutoff=2000.0;
            break;
    }
    
    Fs = cutoff;
    K = Fs / Fe;                        // numeric frequency
    W = 2.0 * M_PI * K;                 // numeric pulse
    A = 2.0 / (2.0 + W);                // coefficient A
    B = (2.0 - W) / (2.0 + W);          // coefficient B
    
    Y0 = 0.0;
    Y1 = 0.0;
    Y2 = 0.0;
    X1 = 0.0;
    
    ay = (signed int)ceil(A * pow(2.0, POW7));
    by = (signed int)ceil(B * pow(2.0, POW7));
}


/*------------------------------------------------------------------------------
 * High pass filter execution
 *----------------------------------------------------------------------------*/
void high_pass_filter_exec(void) {
                
    sy1 = sy0;                          // flip Y
    sx2 = sx1;                          // flip X
    sx1 = X0;
    
    sy0 = (ay * (sx1 - sx2) + by * sy1) >> (int)POW7;
    
    PORTD = sy0 + 128;
}


/*------------------------------------------------------------------------------
 * Low pass filter initialisation
 *----------------------------------------------------------------------------*/
void low_pass_filter_init(void) {
    
    switch (PORTH) {
        case 0:
            cutoff=200.0;
            break;
        case 1:
            cutoff=500.0;
            break;
        case 2:
            cutoff=1000.0;
            break;
        case 3:
            cutoff=2000.0;
            break;
    }
    
    Fs = cutoff;
    K = Fs / Fe;
    W = 2.0 * M_PI * K;
    A = W / (2.0 + W);                  // coefficient A
    B = (2.0 - W) / (2.0 + W);          // coefficient B
    
    Y0 = 0.0;
    Y1 = 0.0;
    Y2 = 0.0;
    X1 = 0.0;
    
    ay = (unsigned int)ceil(A * pow(2.0, POW7));
    by = (unsigned int)ceil(B * pow(2.0, POW7));
}


/*------------------------------------------------------------------------------
 * Low pass filter execution
 *----------------------------------------------------------------------------*/
void low_pass_filter_exec(void) {
    
    sy1 = sy0;                          // flip Y
    sx2 = sx1;                          // flip X
    sx1 = X0;
    
    sy0 = (ay * (sx2 + sx1) + by * sy1) >> (int)POW7;
    
    PORTD = sy0;
}


/*------------------------------------------------------------------------------
 * Running average filter initialisation
 *----------------------------------------------------------------------------*/
void run_avg_filter_init(void) {
    
    if      (PORTH == 0)     runavg_steps = 2;
    else if (PORTH == 1)     runavg_steps = 4;
    else if (PORTH == 2)     runavg_steps = 8;
    else                     runavg_steps = 2;
}


/*------------------------------------------------------------------------------
 * Filter the input signal using running average and output to PORTD (DAC0808)
 *----------------------------------------------------------------------------*/
void run_avg_filter_exec(void) {
    
    switch (runavg_steps) {
        case 2:                         // rot right by 1 = division by 2
            sy1 = sy0;
            sy0 = X0; 
            buf = (sy0 + sy1) >> 1;
            PORTD = buf;
            break;
        case 4:                         // rot right by 2 = division by 4
            sy3 = sy2;
            sy2 = sy1;
            sy1 = sy0;
            sy0 = X0;
            buf = (sy0 + sy1 + sy2 + sy3) >> 2;
            PORTD = buf;
            break;
        case 8:                         // rot right by 3 = division by 8
            sy7 = sy6;
            sy6 = sy5;
            sy5 = sy4;
            sy4 = sy3;
            sy3 = sy2;
            sy2 = sy1;
            sy1 = sy0;
            sy0 = X0;
            buf = (sy0 + sy1 + sy2 + sy3 + sy4 + sy5 + sy6 + sy7) >> 3;
            PORTD = buf;
            break;
        default:
            buf = 0;
            PORTD = buf;
            break;
    }
}
