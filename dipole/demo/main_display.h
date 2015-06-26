#ifndef MAIN_DISPLAY_H_INCLUDED
#define MAIN_DISPLAY_H_INCLUDED
#include <GL/glut.h>
#include "sys.h"

class Main_display
{
public:
    Main_display();
   ~Main_display(){};


    struct Point                   /*���ڼ�¼���λ��*/
    {
        int x;
        int y;
    }oldpt;
    struct Camera                  /*�����������λ*/
    {
        GLfloat xeye;
        GLfloat yeye;
        GLfloat zeye;
    }camera;
    struct Polar                   /*������*/
    {
        double r;                   /*����r*/
        double alpha;               /*ˮƽƫ�Ǧ�*/
        double fy;                  /*��ֱƫ�Ǧգ���λ���ýǶȣ�*/
    }polar;

    void SetCamera(GLfloat x,GLfloat y);
    void main_display_keyboard(unsigned char c,int x,int y,sys &s);
    void main_display_mouse(int button,int state,int x,int y);
    void main_display_mouse_move(int x,int y);
    void main_display_play(sys &s);
    void main_display_init(void);
    void main_display_idle(sys &s1);
    void main_display_reshape(int w,int h,sys &s);

    int l_button_down;
    double drag_velocity;
};

#endif // MAIN_DISPLAY_H_INCLUDED
