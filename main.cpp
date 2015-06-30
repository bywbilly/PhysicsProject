#include <iostream>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <Box2D/Box2D.h>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include "Physics/Physics.h"
#include "Physics/Const.h"

using namespace std;

vector<b2Vec2> gpoint;
Simulator sb;

b2Body* addRect(int x,int y,int w,int h,bool dyn=true)
{
	b2BodyDef bodydef;
	bodydef.position.Set(x*P2M,y*P2M);
	if(dyn)
		bodydef.type=b2_dynamicBody;
	bodydef.linearDamping = 0.0f;
	bodydef.angularDamping = 0.0f;

	b2Body* body=sb.world->CreateBody(&bodydef);

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
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_QUADS);
			for(int i=0;i<4;i++)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}

void draw_poly(vector<b2Vec2> points,b2Vec2 center,float angle)
{
//	fprintf(stderr, "center = (%f, %f)\n", center.x, center.y);
	glColor3f(1,1,1);
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_POLYGON);
			for(int i=0;i!=points.size();++i)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}

void init()
{
	glMatrixMode(GL_PROJECTION);
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glClearColor(0,0,0,1);
	sb.create(vector<pair<vector<b2Vec2>, bool> >(), vector<pair<b2Vec2, double> >());
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	b2Body* tmp=sb.world->GetBodyList();
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
            glVertex2f(gpoint[i].x,gpoint[i].y);
	glEnd();
	glPopMatrix();
}

void moveRect(double x, double y)
{
	b2Body* tmp = sb.world -> GetBodyList();
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

	for(int i = 0; i < 360; ++i)
	{
		gpoint.push_back(b2Vec2(200 + cos((double)i * M_PI / 180) * 20, 20 + sin((double)i * M_PI / 180)  * 20));
	}
	sb.addPolygon(gpoint, true, 0, 0); gpoint.clear();

	while(running)
	{
		start=SDL_GetTicks();
		while(SDL_PollEvent(&event))
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
                      x = event.button.x;
                      y = event.button.y;
                      gpoint.push_back(b2Vec2(x, y));
                      break;
                      case SDL_BUTTON_RIGHT:
                	  fprintf(stderr, "RIGHT DOWN\n");
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
		SDL_GL_SwapBuffers();
		if(1000.0/60>SDL_GetTicks()-start)
			SDL_Delay(1000.0/60-(SDL_GetTicks()-start));
	}
	SDL_Quit();
	return 0;
}