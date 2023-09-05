#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

void lcdStrobe(void);
void lcdInitialize(void);
void lcdWriteData(uint8_t);
void lcdWriteCmd(uint8_t);
void lcdInit(void);
void lcdWriteString(char const *s);
void lcdClear(void);

#endif
