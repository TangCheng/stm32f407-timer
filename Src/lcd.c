#include "lcd.h"
#include "cmsis_os.h"
#include "ascii16.h"
#include "string.h"

#define LCD_WIDTH	192
#define LCD_HEIGHT	64

#define LCD_GPIO	GPIOE

#define LCD_CS		GPIO_PIN_7	// chip select pin, valid in low level
#define LCD_RS		GPIO_PIN_13	// reset pin, keep low level in 30ms for reset
#define LCD_CD		GPIO_PIN_15	// register select pin, high for data, low for command
#define LCD_SCLK	GPIO_PIN_11	// clock
#define LCD_SDAT	GPIO_PIN_9	// data

#define SET_HIGH(pin) HAL_GPIO_WritePin(LCD_GPIO, pin, GPIO_PIN_SET)
#define SET_LOW(pin) HAL_GPIO_WritePin(LCD_GPIO, pin, GPIO_PIN_RESET)

typedef enum
{
  COMMAND = 0,
  DATA = 1
} LCD_DataType;

void LCD_WriteRegister(unsigned char data, LCD_DataType type)
{
  unsigned char i;

  //osDelay(1);
  if (type == COMMAND) {
    SET_LOW(LCD_CD);
  } else {
    SET_HIGH(LCD_CD);
  }
  //osDelay(1);
  for (i = 0; i < sizeof(data) * 8; i++) {
	SET_LOW(LCD_SCLK);
	//DWT_Delay_us(10);
    if (data & 0x80) {
      SET_HIGH(LCD_SDAT);
    } else {
      SET_LOW(LCD_SDAT);
    }
    //DWT_Delay_us(10);
    SET_HIGH(LCD_SCLK);
    //DWT_Delay_us(10);
    data <<= 1;
  }
}

void LCD_WriteCommand(unsigned char command)
{
  LCD_WriteRegister(command, COMMAND);
}

void LCD_WriteData(unsigned char data)
{
  LCD_WriteRegister(data, DATA);
}

void LCD_Init()
{
  SET_LOW(LCD_SDAT);
  SET_LOW(LCD_SCLK);
  SET_LOW(LCD_CD);
  SET_LOW(LCD_RS);
  SET_LOW(LCD_CS);

  osDelay(10);
  SET_HIGH(LCD_RS);
  osDelay(10);

  LCD_WriteCommand(0xE2);
  osDelay(10);
  //LCD_WriteCommand(0x2C);
  //osDelay(10);
  LCD_WriteCommand(0x2E);
  osDelay(10);
  //LCD_WriteCommand(0x2F);
  //osDelay(10);
  /*
  LCD_WriteCommand(0x24);
  LCD_WriteCommand(0x81);
  LCD_WriteCommand(0x1f);
  LCD_WriteCommand(0xA2);
  LCD_WriteCommand(0xC8);
  LCD_WriteCommand(0xA0);
  LCD_WriteCommand(0x40);
  */
  //LCD_WriteCommand(0xEB);
  //LCD_WriteCommand(0xA1);
  //LCD_WriteCommand(0x89);
  LCD_WriteCommand(0x81);
  LCD_WriteCommand(0x78);
  LCD_WriteCommand(0xC2);
  LCD_WriteCommand(0xAF);

  osDelay(10);
  SET_HIGH(LCD_CS);
}

void LCD_Address(unsigned char page, unsigned char column)
{
  column -= 1;
  page -= 1;
  LCD_WriteCommand(0xB0 + page);
  LCD_WriteCommand(0x10 + ((column >> 4) & 0x0F));
  LCD_WriteCommand(column & 0x0F);
}

void LCD_Clean(unsigned char data)
{
  unsigned int page = 0;
  unsigned int column = 0;

  SET_LOW(LCD_CS);
  //osDelay(1);

  for (page = 0; page < LCD_HEIGHT / 8; page++)
  {
    LCD_Address(page + 1, 1);

    for (column = 0; column < LCD_WIDTH; column++)
    {
      LCD_WriteData(data);
    }
  }

  //osDelay(1);
  SET_HIGH(LCD_CS);
}

void LCD_Display_Graphic_32x32(unsigned char pageStart,
		unsigned char columnStart, unsigned char *dp)
{
  unsigned int page = 0;
  unsigned int column = 0;

  SET_LOW(LCD_CS);
  osDelay(1);

  for (page = 0; page < 4; page++)
  {
    LCD_Address(page + pageStart, columnStart);

    for (column = 0; column < 31; column++)
    {
      LCD_WriteData(*dp);
      dp++;
    }
  }

  osDelay(1);
  SET_HIGH(LCD_CS);
}

void LCD_Display_Ascii_16x8(unsigned char pageStart, unsigned char columnStart,
		unsigned char *asciiString, int reverseColor)
{
  unsigned int page = 0;
  unsigned int column = 0;
  unsigned int index = 0;
  unsigned int stringLength = strlen((char *)asciiString);
  unsigned char character;
  unsigned char data;

  SET_LOW(LCD_CS);
  //osDelay(1);

  for (page = 0; page < 2; page++)
  {
    LCD_Address(page + pageStart, columnStart);
    for (index = 0; index < stringLength; index++) {
      character = asciiString[index];
      for (column = 0; column < 8; column++)
      {
        data = ascii16[(character - 0x20) * 16 + page * 8 + column];
        if (reverseColor == 1) {
        	data = ~data;
        }
        LCD_WriteData(data);
      }
    }
  }

  //osDelay(1);
  SET_HIGH(LCD_CS);
}
