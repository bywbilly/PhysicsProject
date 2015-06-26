#ifndef SETTING_H_INCLUDED
#define SETTING_H_INCLUDED
#include "sys.h"

class setting
{
public:
    setting();
    ~setting(){};

    struct still_button
    {
         double x,y;
    }stibutton[6];

    int mode;

    const double leng[5]={450,0.3,2.0,18.0,1.8};
    const double minimum[5]={50,0.3,1.0,12.0,0.2};//��������ÿ�������ķ�Χ
    double para[5];//0:������ 1�����Ӱ뾶 2���糡 3�������߳� 4������ϵ��
    bool setting_button_down[5];
    bool setting_mouse_on[2];

    void setting_reshape(int w,int h);
    void setting_idle();
    void setting_display();
    void setting_mouse(int button,int state,int x,int y,sys&s);
    void setting_mousemove(int x,int y);
    void setting_key(char c,int x,int y,sys &s);
    void setting_button_position();
    void setting_init();
    void setting_startbutton(float x,float y,float t);
    void setting_backbutton(float x,float y,float t);
    void setting_passivemotion(int x,int y);

    void setting_para_to_sys(sys &s);
};

#endif // SETTING_H_INCLUDED
