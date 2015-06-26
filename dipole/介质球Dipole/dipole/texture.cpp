#include "texture.h"
#include "global_variables.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


using namespace std;

 void * font=GLUT_BITMAP_TIMES_ROMAN_24;//����
 int B[4]={5,6,8,10};//B[i]��ģʽ��ť[i]������ֵ��ʾ��Ӧ����ͼ����

 GLuint theTexture[80];
char* textureLists[49] = {
     "image\\load.bmp",//0
     "image\\restart.bmp",//1
     "image\\reference.bmp",//2
     "image\\exit.bmp",//3
     "image\\1a.bmp",//4
     "image\\1b.bmp",//5
     "image\\2a.bmp",//6
     "image\\2b.bmp",//7
     "image\\3a.bmp",//8
     "image\\3b.bmp",//9
     "image\\4a.bmp",//10
     "image\\4b.bmp",//11
     "image\\mode.bmp",//12
     "image\\totalnum.bmp",//13
     "image\\part_r.bmp",//14
     "image\\E.bmp",//15
     "image\\boundary.bmp",//16
     "image\\friction.bmp",//17
     "image\\dt.bmp",//18
     "image\\epsilon.bmp",//19
     "image\\back_dark.bmp",//20
     "image\\procedure.bmp",//21
     "image\\control_bar.bmp",//22
     "image\\system_settings.bmp",//23
     "image\\background.bmp",//24
     "image\\intro.bmp",//25
     "image\\up.bmp",//26
     "image\\down.bmp",//27
     "image\\start_dark.bmp",//28
     "image\\logo.bmp",//29
     "image\\save.bmp",//30
     "image\\continue_dark.bmp",//31
     "image\\pause_dark.bmp",//32
     "image\\leave_dark.bmp",//33
     "image\\y_dark.bmp",//34
     "image\\n_dark.bmp",//35
     "image\\continue_bright.bmp",//36
     "image\\pause_bright.bmp",//37
     "image\\leave_bright.bmp",//38
     "image\\up_bright.bmp",//39
     "image\\down_bright.bmp",//40
     "image\\load_dark.bmp",//41
     "image\\restart_dark.bmp",//42
     "image\\reference_dark.bmp",//43
     "image\\exit_dark.bmp",//44
     "image\\y_bright.bmp",//45
     "image\\n_bright.bmp",//46
     "image\\start_bright.bmp",//47
     "image\\back_bright.bmp",//48
};
/*0-3,41-44��menu������dark��bright״̬��ť��4-11��setting�����ĵ���ģʽ�İ�ť��12-19��setting����ѡ��İ�ť��
20,48��back��ť��dark��bright״̬��21�ǿ�������22�ǿ���bar��23��setting�ı��⣬24��menu�ı�����25�ǲο����ϵ�ͼƬ
26-27,39-40�ֱ���up,down��dark��bright״̬��28,47�ֱ���back��ť��dark��bright״̬��29���ʼ��logo
30��maindisplayѯ���Ƿ񱣴�ı�����31-33,36-38�ֱ���maindisplay�����Ҳ�������ť��34-35,45-46�ֱ���y,n
*/
int ImageLoad(const char *filename, Image *image) {

    FILE *file;
    unsigned long size;
    unsigned long i;
    unsigned short int planes;
    unsigned short int bpp;
    char temp, finalName[80];

	//strcpy(finalName, "E:\\����\\opengl\\������ͼ2\\" );
	//strcat(finalName, filename );
    //finalName=filename;


    if ((file = fopen(filename, "rb"))==NULL) {	//(finalName, "rb"))==NULL) {			//��û���ҵ��ļ�ʱ����
		cout<<"File Not Found : "<<filename;
		return 0;
    }


    fseek(file, 0x0012, SEEK_CUR);			//�ҵ�λͼ�ĵ�18���ֽ��Զ�ȡ���

    if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {			//��ȡ���
		cout<<"Error reading width from "<< finalName;
		return 0;
    }

    if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {				//��ȡ�߶�
		cout<<"Error reading height from \n"<<finalName;
		return 0;
    }

    size = image->sizeX * image->sizeY * 3;

    if ((fread(&planes, 2, 1, file)) != 1) {
		cout<<"Error reading planes from \n"<< finalName;
		return 0;
    }

    if (planes != 1) {
		cout<<"Planes from xx is not 1:\n"<<finalName<< planes;
		return 0;
    }

    if ((i = fread(&bpp, 2, 1, file)) != 1) {
		cout<<"Error reading bpp from \n"<<finalName;
		return 0;
    }

    if (bpp != 24) {
		cout<<"Bpp from xx is not 24: \n"<<finalName<< bpp;
	return 0;
    }

    fseek(file, 24, SEEK_CUR);

    image->data = (char *) malloc(size);
    if (image->data == NULL) {
		printf("Error allocating memory for color-corrected image data");
		return 0;
    }

    if ((i = fread(image->data, size, 1, file)) != 1) {
		printf("Error reading image data from %s.\n", finalName);
		return 0;
    }

    for (i=0;i<size;i+=3) {
		temp = image->data[i];
		image->data[i] = image->data[i+2];
		image->data[i+2] = temp;
    }

    return 1;
}
// ��ȡbmpͼ
Image myTextureData[80];//ͼƷ���ݵĴ洢��ַ

void LoadGLTextures(int k) {

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		if (!ImageLoad(textureLists[k], &myTextureData[k])) exit(1);

		// �����������
		glGenTextures(1, &theTexture[k]);
	    glBindTexture(GL_TEXTURE_2D, theTexture[k]);
	   	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);			//
	   	glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);      //�������ص��ظ� ��ϸ�ο��̳�65
	   	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
	   	gluBuild2DMipmaps(GL_TEXTURE_2D, 3/*��ʾRGB*/, myTextureData[k].sizeX, myTextureData[k].sizeY, GL_RGB, GL_UNSIGNED_BYTE, myTextureData[k].data/*����OpenGL�������ݵ���Դ*/);			//�������λͼ�ļ�(*.bmp)ת����������ͼ
}

void glprint(float x, float y, char *string)//��Ļ����ʾ���ֵĺ���
{
     int len, i; glRasterPos2f(x, y);
     for (i = 0; i < (int) strlen(string); i++)
     {
        glutBitmapCharacter(font, string[i]);
     }
}



