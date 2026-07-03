#include "OLED.h"
#include "OLED_Font.h"
#include "MyI2C.h"
#include "delay.h"

#define OLED_ADDR_7BIT (0x3CU)

static bool oled_fault = false;


void OLED_ClearFault(void)
{
    if (oled_fault) {
        MyI2C_Recover();
    }
    oled_fault = false;
}
static bool OLED_WriteByte(uint8_t control, uint8_t data)
{
    uint8_t tx[2];

    if (oled_fault) {
        return false;
    }

    tx[0] = control;
    tx[1] = data;
    if (!MyI2C_WriteBytes(OLED_ADDR_7BIT, tx, 2U)) {
        oled_fault = true;
        return false;
    }

    return true;
}

static bool OLED_WriteCommand(uint8_t command)
{
    return OLED_WriteByte(0x00U, command);
}

static bool OLED_WriteData(uint8_t data)
{
    return OLED_WriteByte(0x40U, data);
}

static void OLED_SetCursor(uint8_t y, uint8_t x)
{
    (void)OLED_WriteCommand((uint8_t)(0xB0U | y));
    (void)OLED_WriteCommand((uint8_t)(0x10U | ((x & 0xF0U) >> 4U)));
    (void)OLED_WriteCommand((uint8_t)(0x00U | (x & 0x0FU)));
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0U; j < 8U; j++) {
        OLED_SetCursor(j, 0U);
        for (i = 0U; i < 128U; i++) {
            (void)OLED_WriteData(0x00U);
        }
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    uint8_t index;

    if ((Line < 1U) || (Line > 4U) || (Column < 1U) || (Column > 16U)) {
        return;
    }
    if ((Char < ' ') || (Char > '~')) {
        Char = ' ';
    }

    index = (uint8_t)(Char - ' ');
    OLED_SetCursor((uint8_t)((Line - 1U) * 2U), (uint8_t)((Column - 1U) * 8U));
    for (i = 0U; i < 8U; i++) {
        (void)OLED_WriteData(OLED_F8x16[index][i]);
    }
    OLED_SetCursor((uint8_t)((Line - 1U) * 2U + 1U), (uint8_t)((Column - 1U) * 8U));
    for (i = 0U; i < 8U; i++) {
        (void)OLED_WriteData(OLED_F8x16[index][i + 8U]);
    }
}

void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
    uint8_t i;
    for (i = 0U; (String[i] != '\0') && ((Column + i) <= 16U); i++) {
        OLED_ShowChar(Line, (uint8_t)(Column + i), String[i]);
    }
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1U;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0U; i < Length; i++) {
        OLED_ShowChar(Line, (uint8_t)(Column + i),
            (char)(Number / OLED_Pow(10U, Length - i - 1U) % 10U + '0'));
    }
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;

    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        Number1 = (uint32_t)Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        Number1 = (uint32_t)(-Number);
    }

    for (i = 0U; i < Length; i++) {
        OLED_ShowChar(Line, (uint8_t)(Column + i + 1U),
            (char)(Number1 / OLED_Pow(10U, Length - i - 1U) % 10U + '0'));
    }
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0U; i < Length; i++) {
        SingleNumber = (uint8_t)(Number / OLED_Pow(16U, Length - i - 1U) % 16U);
        OLED_ShowChar(Line, (uint8_t)(Column + i),
            (char)((SingleNumber < 10U) ? (SingleNumber + '0') : (SingleNumber - 10U + 'A')));
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0U; i < Length; i++) {
        OLED_ShowChar(Line, (uint8_t)(Column + i),
            (char)(Number / OLED_Pow(2U, Length - i - 1U) % 2U + '0'));
    }
}

bool OLED_Init(void)
{
    oled_fault = false;
    Delay_ms(100U);

    if (!OLED_WriteCommand(0xAEU)) return false;
    (void)OLED_WriteCommand(0xD5U);
    (void)OLED_WriteCommand(0x80U);
    (void)OLED_WriteCommand(0xA8U);
    (void)OLED_WriteCommand(0x3FU);
    (void)OLED_WriteCommand(0xD3U);
    (void)OLED_WriteCommand(0x00U);
    (void)OLED_WriteCommand(0x40U);
    (void)OLED_WriteCommand(0xA1U);
    (void)OLED_WriteCommand(0xC8U);
    (void)OLED_WriteCommand(0xDAU);
    (void)OLED_WriteCommand(0x12U);
    (void)OLED_WriteCommand(0x81U);
    (void)OLED_WriteCommand(0xCFU);
    (void)OLED_WriteCommand(0xD9U);
    (void)OLED_WriteCommand(0xF1U);
    (void)OLED_WriteCommand(0xDBU);
    (void)OLED_WriteCommand(0x30U);
    (void)OLED_WriteCommand(0xA4U);
    (void)OLED_WriteCommand(0xA6U);
    (void)OLED_WriteCommand(0x8DU);
    (void)OLED_WriteCommand(0x14U);
    (void)OLED_WriteCommand(0xAFU);

    OLED_Clear();
    return true;
}
