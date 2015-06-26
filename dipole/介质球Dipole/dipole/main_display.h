#ifndef MAIN_DISPLAY_H_INCLUDED
#define MAIN_DISPLAY_H_INCLUDED
#include <GL/glut.h>
#include "sys.h"

class Main_display
{
public:
    Main_display();
   ~Main_display(){};

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
    void main_display_init();
    void main_display_idle(sys &s1);
    void main_display_reshape(int w,int h,sys &s);
    void main_display_passivemotion(int x,int y);

    void main_display_continue_button(int x,int y);
    void main_display_quit_button(int x,int y);//x��y�ǰ�ť�������꣬������Ϊ��λ
    void main_display_yn_button();//��Ϊλ�ù̶����Բ���Ҫ����
    void main_display_electricfield();//���糡
    void main_display_electric_button(int x,int y);//���Ƶ糡��ť

    double drag_velocity;
    float texture_r;//��ť�뾶��������Ϊ��λ
    bool main_display_pause;
    bool quit;
    float bigger;//��ť���ı���
    int M;
    bool play_mouse_on[3];//��pause��quit��ť�ǲ��������������
    bool quit_mouse_on[2];//��quit�����°�ť���Ƿ������
    bool show_electric;

    int pause_button_x;
    int pause_button_y;
    int leave_button_x;
    int leave_button_y;
};

#endif // MAIN_DISPLAY_H_INCLUDED
