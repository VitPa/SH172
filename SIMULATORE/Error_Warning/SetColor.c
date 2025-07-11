#include <windows.h>
#include "ErrorWarning.h"

/*
| Codice | Colore           |
|--------|------------------|
| 0      | Nero             |
| 1      | Blu scuro        |
| 2      | Verde scuro      |
| 3      | Azzurro scuro    |
| 4      | Rosso scuro      |
| 5      | Viola scuro      |
| 6      | Giallo scuro     |
| 7      | Grigio chiaro    |
| 8      | Grigio scuro     |
| 9      | Blu chiaro       |
| 10     | Verde chiaro     |
| 11     | Azzurro chiaro   |
| 12     | Rosso chiaro     |
| 13     | Viola chiaro     |
| 14     | Giallo chiaro    |
| 15     | Bianco           |
*/

void SetColor(int color)
{
    WORD wColor;
    HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO csbi;

    if(GetConsoleScreenBufferInfo(hStdOut, &csbi))
    {
        wColor = (csbi.wAttributes & 0xF0) + (color & 0x0F);
        SetConsoleTextAttribute(hStdOut, wColor);
    }
}
