/* 
 * File:   myDefines.h
 * Author: 3clau
 *
 * Created on 15 de Outubro de 2022, 00:13
 */

#ifndef MYDEFINES_H
#define	MYDEFINES_H

#define	n_TMR0      28

#define vref        5
#define _XTAL_FREQ 16000000
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))

// set up the timing for the LCD delays
#define LCD_delay 5 // ~5mS
#define LCD_Startup 15 // ~15mS
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_CURSOR_BACK 0x10
#define LCD_CURSOR_FWD 0x14
#define LCD_PAN_LEFT 0x18
#define LCD_PAN_RIGHT 0x1C
#define LCD_CURSOR_OFF 0x0C
#define LCD_CURSOR_ON 0x0E
#define LCD_CURSOR_BLINK 0x0F
#define LCD_CURSOR_LINE2 0xC0
#define FUNCTION_SET 0x28 // 4 bit interface, 2 lines, 5x8 font
#define ENTRY_MODE 0x06 // increment mode
#define DISPLAY_SETUP 0x0C // display on, cursor off, blink offd
#define LCDLine1() LCDPutCmd(LCD_HOME) // legacy support
#define LCDLine2() LCDPutCmd(LCD_CURSOR_LINE2) // legacy support
#define shift_cursor() LCDPutCmd(LCD_CURSOR_FWD) // legacy support
#define cursor_on() LCDPutCmd(LCD_CURSOR_ON) // legacy support
#define DisplayClr() LCDPutCmd(LCD_CLEAR) // Legacy support
#define instr 0
#define data 1
#define LCD_PORT PORTA
//#define LCD_PWR  PORTBbits.RB6 // LCD power pin
#define LCD_EN   PORTBbits.RB4 // LCD enable
//#define LCD_RW   PORTBbits.RB7 // LCD read/write line
#define LCD_RS   PORTBbits.RB5 // LCD register select line
#define NB_LINES 2 // Number of display lines
#define NB_COL 16 // Number of characters per line

#define SCK                 PORTBbits.RB1
#define SCK_DIR             TRISB1
#define SDI                 PORTBbits.RB0
#define SDI_DIR             TRISB0
#define SDO                 PORTCbits.RC7
#define SDO_DIR             TRISC7
#define SS                  PORTAbits.RA5
#define SS_DIR              TRISA5
#define MASTER_OSC_DIV4     0b00000000
#define MASTER_OSC_DIV16    0b00000001
#define MASTER_OSC_DIV64    0b00000010
#define MASTER_TMR2_DIV2    0b00000011
#define SLAVE_SS_ENABLE     0b00000010
#define SLAVE_SS_DISABLE    0b00000101
#define DATA_END            0b10000000
#define DATA_MIDDLE         0b00000000
#define IDLE_HIGH           0b00110000
#define IDLE_LOW            0b00100000
#define ACT_IDLE            0b01000000
#define IDLE_ACT            0b00000000
#define CMD_NONE            0b00000000
#define CMD_WRITE           0b00010000
#define CMD_SDOWN           0b00100000
#define CMD_NONE2           0b00110000
#define POT_NONE            0b00000000
#define POT_P0              0b00000001
#define POT_P1              0b00000010
#define POT_BOTH            0b00000011
#define command             0x11


#define No          32
//#define	CHAN_0		0b00000000
//#define	CHAN_1		0b00000100
//#define	CHAN_2		0b00001000
#define CHAN_8      0b00100000
#define CHAN_9      0b00100100
#define CHAN_12     0b00110000
//#define	AN_0		0b00001110
//#define	AN_1		0b00001101
//#define	AN_2		0b00001100
#define AN_8        0b00000110
#define AN_9        0b00000101
#define AN_12       0b00000010
//#define DIV16_2TAD  0b10001101
//#define DIV16_4TAD  0b10010101
#define DIV16_12TAD 0b10101101
//#define	DIV32_12TAD	0b10101010
//#define	DIV32_20TAD	0b10111010

unsigned int a[No], b[No],v,e;
char k, canal,info[10],info2[10];
float aux_Voff,aux_D2,max1,max2;
float D2[20],voltage[20];
float Re_X[No], Im_X[No], Re_Xm1[No], Im_Xm1[No], Re_X1[No], Im_X1[No];
unsigned char passo, pot_value;

#endif	/* MYDEFINES_H */

