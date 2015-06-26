#include <iostream>
#include <windows.h>
#include "sys.h"
#include "func.h"
#include "main_display.h"
#include "const.h"
#include "S_L.h"
#include <GL/glut.h>
using namespace std;

//ģʽ1�Ƕ���ѧ��2������㷨���������Ǹ��Ա߽磩��3�Ƕ���ѧ��4������㷨�������������ڱ߽磩
//�����ڱ߽�ĵ���ͼ�������������ˡ���

//�����ڵĴ������������Ų�����ô�������ǲ��ᱨ��ģ��������Կ��Կ����������⡣��
sys s=sys(part_r,ep,ef,E,rou,ttnum,f,dt,boundary,mode);
Main_display m;

void init()
{
    s.system_start();
    s.system_init();
    m.main_display_init();
}

void myReshape(int w,int h)
{
    m.main_display_reshape(w,h,s);
}

void myDisplay()
{
    m.main_display_play(s);
}

void myIdle()
{
    m.main_display_idle(s);
    myDisplay();
}

void myMouse(int button,int state,int x,int y)        /*����갴�»�����ʱ��ص��ú���*/
{
    m.main_display_mouse(button,state,x,y);
}

void OnMouseMove(int x,int y)          /*������ƶ�ʱ��ص��ú���*/
{
    m.main_display_mouse_move(x,y);
}

void myKeyboard(unsigned char c,int x,int y)
{
    m.main_display_keyboard(c,x,y,s);
}

int main(int argc,char*argv[])
{
    /*char str[200];
      GetConsoleTitle(str,200);
      HWND hd=FindWindow(NULL,str);
      ShowWindow(hd,SW_HIDE);*/
    SetConsoleTitle("Dipole_start");
    menu_temp(s,m);
    SetConsoleTitle("Dipole_info");
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);
    glutInitWindowPosition(250,150);
    glutInitWindowSize(la,lb);
    glutCreateWindow("Dipole");
    init();
    m.SetCamera(0.0f,0.0f);
    glutReshapeFunc(myReshape);
    glutDisplayFunc(myDisplay);
    glutMouseFunc(myMouse);
    glutMotionFunc(OnMouseMove);
    glutKeyboardFunc(myKeyboard);
    glutIdleFunc(myIdle);
    glutMainLoop();
    return 0;
}
