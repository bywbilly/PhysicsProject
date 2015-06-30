#include <iostream>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <GLUT/glut.h>
#include <Box2D/Box2D.h>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include "Physics/Physics.h"
#include "Physics/Const.h"

using namespace std;

b2World* world;
vector<b2Vec2> gpoint;

b2Body* addRect(int x,int y,int w,int h,bool dyn=true)
{
	b2BodyDef bodydef;
	bodydef.position.Set(x*P2M,y*P2M);
	if(dyn)
		bodydef.type=b2_dynamicBody;
	bodydef.linearDamping = 0.0f;
	bodydef.angularDamping = 0.0f;

	b2Body* body=world->CreateBody(&bodydef);

	b2PolygonShape shape;
	shape.SetAsBox(P2M*w/2,P2M*h/2);

	b2FixtureDef fixturedef;
	fixturedef.shape=&shape;
	fixturedef.density=1.0;
	fixturedef.restitution = 1.0;
	fixturedef.friction = 0;
	body->CreateFixture(&fixturedef);
	return NULL;
}

void drawSquare(b2Vec2* points,b2Vec2 center,float angle)
{
	glColor3f(1,1,1);
	glPushMatrix();//glPushMatrix压入当前矩阵堆栈。
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_QUADS);
			for(int i=0;i<4;i++)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);//打印点
		glEnd();
	glPopMatrix();
}

void draw_poly(vector<b2Vec2> points,b2Vec2 center,float angle)
{
//	fprintf(stderr, "center = (%f, %f)\n", center.x, center.y);
	glColor3f(1,1,1);
	glPushMatrix();//glPushMatrix压入当前矩阵堆栈。
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_POLYGON);
			for(int i=0;i!=points.size();++i)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);//打印点
		glEnd();
	glPopMatrix();
}

void init()
{
	glMatrixMode(GL_PROJECTION);//mode 指定哪一个矩阵堆栈是下一个矩阵操作的目标 ; GL_PROJECTION,对投影矩阵应用随后的矩阵操作.
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);//GL_MODELVIEW,对模型视景矩阵堆栈应用随后的矩阵操作.
	glClearColor(0,0,0,1);//通过glClear使用红，绿，蓝以及AFA值来清除颜色缓冲区的，并且都被归一化在（0，1）之间的值，其实就是清空当前的所有颜色。

	world=new b2World(b2Vec2(0,g), false);
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();//重置当前指定的矩阵为单位矩阵.将之前矩阵变换导致变化过的栈顶矩阵重新归位，置为单位矩阵！消除之前的矩阵变换带来的影响
	b2Body* tmp=world->GetBodyList();
	vector<b2Vec2> points;
	int count = 0;
	while(tmp)
	{
		count++;
		int size = ((b2PolygonShape*)((tmp -> GetFixtureList()[0]).GetShape())) -> GetVertexCount();
		points.resize(size);
		for(int i=0;i<size;i++)
			points[i] =((b2PolygonShape*)tmp->GetFixtureList()->GetShape())->GetVertex(i);
		draw_poly(points,tmp->GetWorldCenter(),tmp->GetAngle());

		tmp=tmp->GetNext();
	}
//	fprintf(stderr, "Count = %d\n", count);
	glColor3f(1,1,1);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
	for(int i=0;i!=gpoint.size();++i)
            glVertex2f(gpoint[i].x,gpoint[i].y);//打印点
	glEnd();
	glPopMatrix();
}

void moveRect(double x, double y)
{
	b2Body* tmp = world -> GetBodyList();
	b2Vec2 points[4];
	while(tmp)
	{
		int fac1 = 1, fac2 = 1;
		if(rand() % 2) fac1 *= -1;
		if(rand() % 2) fac2 *= -1;
		tmp -> SetLinearVelocity(b2Vec2(rand() % 10  * fac1, rand() % 10  * fac2));
		tmp = tmp -> GetNext();
	}
}

int main(int argc,char** argv)
{
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(WIDTH,HEIGHT,32,SDL_OPENGL);
	Uint32 start;
	SDL_Event event;
	bool running=true;
	init();
	srand(2);
	//for(int i = 0; i < 10; ++i){addRect(rand() % 600 + 1, rand() % 400 + 1,30,30,true);}


	Simulator sb = Simulator(world);
	while(running)
	{
		start=SDL_GetTicks();//获取当前时间，以毫秒计时
		while(SDL_PollEvent(&event))//从事件队列里取出事件
		{
			switch(event.type)
			{
				case SDL_QUIT: { running=false; break;}

				case SDL_KEYDOWN:
					switch(event.key.keysym.sym)
					{
						case SDLK_ESCAPE: { running=false; break; }
					}
					break;

				case SDL_MOUSEBUTTONDOWN:
					int x,y;
					switch(event.button.button)
                    {
                	  case SDL_BUTTON_LEFT:
                	  fprintf(stderr, "LEFT DOWN\n");
                      x = event.button.x;//得到当前鼠标的坐标
                      y = event.button.y;
                      gpoint.push_back(b2Vec2(x, y));
                      /*点的过程中画线*/
                      break;
                      case SDL_BUTTON_RIGHT:
                	  fprintf(stderr, "RIGHT DOWN\n");
                      /*清理历史画的点*/
                      if(gpoint.size() == 0) break;
                      sb.addPolygon(gpoint, true);
                      gpoint.clear();
                      break;
                    }
					break;
				default:
					break;
			}
		}

		display();
		sb.simulateNextStep();
		SDL_GL_SwapBuffers();//这个是SDL的函数,前面开启了交换缓冲区,这里调用,交换前后缓冲区,图像就显示出来了。
		if(1000.0/60>SDL_GetTicks()-start)
			SDL_Delay(1000.0/60-(SDL_GetTicks()-start));
	}
	SDL_Quit();//main中的每个return前都要这个玩意儿
	return 0;
}
