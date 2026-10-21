#ifndef __RGB_H
#define __RGB_H

#include "main.h"

typedef enum
{
    RGB_OFF,
    RGB_RED,
    RGB_GREEN,
    RGB_BLUE,
    RGB_YELLOW,//R G B ɫ
    RGB_CYAN,//R  G  B ɫ
    RGB_MAGENTA,//R  B G𰵺ɫ
    RGB_WHITE//R G B      ɫ
}RGB_enum;


void RGB_action(RGB_enum RGB);
    
#endif

