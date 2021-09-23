/*------------------------------------------------------------------------------
 * Simple LCD library for 18F8722
 * LM044L alpha LCD via MCP23S17 SPI IO extension
 * Samuel CIULLA - Microcontroleurs 2021
 *----------------------------------------------------------------------------*/
#include <xc.h>
#include "SPI_Alpha_LCD.h"

/*------------------------------------------------------------------------------
 * Initialize SPI, MCP and LCD
 *----------------------------------------------------------------------------*/
void Init_Alpha_LCD(void)
{
    // Init SPI
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    
    SSPSTAT = 0x00;
    SSPSTATbits.CKE = 1;
    SSPCON1 = 0x00;
    SSPCON1bits.SSPEN = 1;
    
    SPI_CS1 = 1;
    SPI_CS2 = 1;
    
    // Init MCP via SPI
    Send_MCP(MCP_IODIRA, 0x00);
    Send_MCP(MCP_GPIOA, 0x00);
    Send_MCP(MCP_IODIRB, 0x00);
    Send_MCP(MCP_GPIOB, 0x00);
    
    // Init LCD via MCP
    Send_Cmd_LCD(0x33);
    __delay_ms(10);
    Send_Cmd_LCD(0x33);
    __delay_ms(10);
    Send_Cmd_LCD(0x38);
    __delay_ms(10);
    Send_Cmd_LCD(0x0C); // Cursor C,E,F
    Send_Cmd_LCD(0x06);
    
    Send_Cmd_LCD(LCD_CLEAR);
    Send_Cmd_LCD(LCD_LINE_1);
    
    return;
}

/*------------------------------------------------------------------------------
 * Send command and related data to MCP via SPI_CS1
 *----------------------------------------------------------------------------*/
void Send_MCP(char cmd, char dat)
{
    SPI_CS1 = 0;
    
    SSPIF = 0;
    SSPBUF = 0x40;
    while(!SSPIF);
    
    SSPIF = 0;
    SSPBUF = cmd;
    while(!SSPIF);
    
    SSPIF = 0;
    SSPBUF = dat;
    while(!SSPIF);
    
    SPI_CS1 = 1;
    
    return;
}

/*------------------------------------------------------------------------------
 * Send a command to LCD
 *----------------------------------------------------------------------------*/
void Send_Cmd_LCD(char cmd)
{
    Send_MCP(MCP_GPIOA, 0x00);  // RS to 0
    Send_MCP(MCP_GPIOB, cmd);   // Send cmd
    Send_MCP(MCP_GPIOA, 0x80);  // EN to 1
    Send_MCP(MCP_GPIOA, 0x00);  // EN to 0
    __delay_ms(4);

    return;
}

/*------------------------------------------------------------------------------
 * Send a character to LCD
 *----------------------------------------------------------------------------*/
void Send_Chr_LCD(char dat)
{
    Send_MCP(MCP_GPIOA, 0x40);  // RS to 1
    Send_MCP(MCP_GPIOB, dat);   // Send data
    Send_MCP(MCP_GPIOA, 0xC0);  // EN to 1
    Send_MCP(MCP_GPIOA, 0x40);  // EN to 0
    __delay_ms(4);
    
    return;
}

/*------------------------------------------------------------------------------
 * Display a string on line 1,2,3,4, at cursor otherwise
 *----------------------------------------------------------------------------*/
void Send_Txt_LCD(char * txt, char lnr)
{
    switch (lnr)
    {
        case 0: break;
        case 1: Send_Cmd_LCD(LCD_LINE_1); break;
        case 2: Send_Cmd_LCD(LCD_LINE_2); break;
        case 3: Send_Cmd_LCD(LCD_LINE_3); break;
        case 4: Send_Cmd_LCD(LCD_LINE_4); break;
        default: break;
    }
    
    while(*txt)
    {
        Send_Chr_LCD(*txt);
        txt++;
    }
        
    return;
}
