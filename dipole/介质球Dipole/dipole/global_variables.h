#ifndef GLOBAL_VARIABLES_H_INCLUDED
#define GLOBAL_VARIABLES_H_INCLUDED
#include "main_display.h"
#include "Menu_display.h"
#include "sys.h"
#include "const.h"
#include "setting.h"

extern bool l_button_down;
enum cmode{SMENU,SSHOW,SSETTING,SINTRO};//��������
extern cmode MODE;

 struct Point                   /*���ڼ�¼���λ��*/
    {
        int x;
        int y;
    };
extern Point oldpt;

extern char* settingnum;

extern bool if_change;
#endif // GLOBAL_VARIABLES_H_INCLUDED
