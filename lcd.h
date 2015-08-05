/* 
 * File:   lcd.h
 * Author: John Kinkennon
 *
 * Created on August 27, 2012, 3:58 PM
 */

#ifndef LCD_H
#define	LCD_H

#ifdef	__cplusplus
extern "C" {
#endif

#define HLCD    16      // LCD width=16 characters
#define VLCD    2       // LCD height=2 rows

#define LCDDATA 1       // address of data register
#define LCDCMD  0       // address of command register

void initLCD(void);
void writeLCD(int addr, char c);
char readLCD(int addr);

#define putLCD(d)   writeLCD(LCDDATA, (d))
#define cmdLCD(c)   writeLCD(LCDCMD, (c))

#define clrLCD()    writeLCD(LCDCMD, 1)
#define homeLCD()   writeLCD(LCDCMD, 2)

#define setLCDG(a)  writeLCD(LCDCMD, (a & 0x3f) | 0x40)
#define setLCDC(a)  writeLCD(LCDCMD, (a & 0x7f) | 0x80)

#define busyLCD()   (readLCD(LCDCMD) & 0x80)
#define addrLCD()   (readLCD(LCDCMD) & 0x7f)
#define getLCD()    readLCD(LCDDATA)

void putsLCD(char *s);

#ifdef	__cplusplus
}
#endif

#endif	/* LCD_H */

