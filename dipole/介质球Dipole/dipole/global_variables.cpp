#include "global_variables.h"
#include "sys.h"
bool l_button_down=false;//������ʾ��������ǰ��״̬
cmode MODE=SMENU;//������ʾ��ǰ�ĳ���
Point oldpt={-1,-1};//�������������λ�ã�������Ϊ��λ
char* settingnum=new char[30] ;//����setting����display�У�������������ͼ
bool if_change=false;//��ǳ����Ƿ�ı䣬��glutmousefunc�о����Ƿ����³�ʼ��

