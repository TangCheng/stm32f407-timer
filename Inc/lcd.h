#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void LCD_Init();
void LCD_Clean(unsigned char data);
void LCD_Display_Graphic_32x32(unsigned char pageStart,
		unsigned char columnStart, unsigned char *dp);
void LCD_Display_Ascii_16x8(unsigned char pageStart, unsigned char columnStart,
		unsigned char *asciiString, int reverseColor);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */
