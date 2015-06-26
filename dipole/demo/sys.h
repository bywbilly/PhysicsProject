#ifndef SYS_H_INCLUDED
#define SYS_H_INCLUDED
#include "particle.h"
#include "Triple.h"

class sys
{
public:
    sys(double part_r,double ep,double ef,double E,double rou,int ttnum,double f,double dt,double boundary,int mode);
    void system_start(); //������ϵͳ��ʼ�����������ϴ��ж�ʱ�������������˺���
    void system_init();  //Ԥ����
    ~sys();

    int ttnum,N,mode; //������������ʼ��ʱÿһ�ŵ����ӣ�ģʽ
    double part_r,ep,ef,E,rou,f,dt,boundary;
    //���Ӱ뾶��������糣����Һ���糣������ǿ�������ܶȣ�����ϵ����ʱ�䲽�����߳�
    double part_p,part_m,dis; //��ż���أ���������ʼ��ʱ���Ӽ��

    int dis_mcmap,boundary_mcmap; //�����ϵĳ�ʼ���룬�����ϵı߳���MC�㷨���񻯺�����Ĳ�����

    bool whether_start; //�ж��Ƿ��ʼ�����������ϴ��ж�ʱ����������Ϊ�ѳ�ʼ�����Ӷ�����start

    int cal_total; //�Ѿ�����Ĳ���
    double Ek,Ep; //���ܣ�����

    particle *part_ob; //����ָ��
    Triple *k3; //Ԥ������

    int ***num_map; //MC����ָ��

    double **potential_map_yz;
    double **potential_map_xz;
    double **potential_map_xy;//����ͼָ�루���飩

    int **number_map_yz;
    int **number_map_xz;
    int **number_map_xy;//������ͼָ�루���飩
};

#endif // SYS_H_INCLUDED
