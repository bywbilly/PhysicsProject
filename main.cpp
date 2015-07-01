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
#include <ctime>
#include "Physics/Physics.h"
#include "Physics/Const.h"

using namespace std;

vector<b2Vec2> gpoint;
Simulator sb;
const int MAXLEVEL = 7;
void (*init_func[MAXLEVEL])(int&, int&);
unsigned now_time;
class colorVector
{
public:
	double r, g, b;
	colorVector(){}
	colorVector(double r, double g, double b): r(r), g(g), b(b){}
	double len()
	{
		return sqrt(r * r + g * g + b * b);
	}
	colorVector operator - (const colorVector &x) const
	{
		return colorVector(r - x.r, g - x.g, b - x.b);
	}
	colorVector operator * (const double x) const
	{
		return colorVector(r * x, g * x, b * x);
	}
	colorVector operator / (const double x) const
	{
		return colorVector(r / x, g / x, b / x);
	}
	colorVector operator + (const colorVector &x) const
	{
		return colorVector(r + x.r, g + x.g, b + x.b);
	}
};
colorVector color[3];

void change_color(int size = 0)
{
	static double current_length = 0;
	static int start = 0;
	if(current_length > (color[(start + 1) % 3] - color[start]).len())
		start = (start + 1) % 3, current_length = 0;
	if (now_time != time(0))
	{
		current_length += 0.00005;
	}
	colorVector dir = color[(start + 1) % 3] - color[start];
	dir = dir / dir.len();
	dir = dir * current_length;
	dir = dir + color[start];
	if (size > 300)
		glColor3f(1, 0, 0);
	else
		glColor3f(dir.r, dir.g, dir.b);
}

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
	change_color(points.size());
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_POLYGON);
			for(int i=0;i!=points.size();++i)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}

void display(int dx,int dy)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	#ifdef __APPLE__
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	#endif
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

	//画目的地
	glColor3f(0,1,0);
	glPushMatrix();
	glBegin(GL_POLYGON);
	glVertex2f(dx-3,dy-3);	
	glVertex2f(dx-3,dy+3);
	glVertex2f(dx+3,dy+3);
	glVertex2f(dx+3,dy-3);
	glEnd();
	glPopMatrix();

	//画death地
	glColor3f(1,0,0);
	for(int i=0; i!=sb.deathPoint.size();++i)
	{
		glPushMatrix();
		glBegin(GL_POLYGON);
		glVertex2f(sb.deathPoint[i].x * M2P-3,sb.deathPoint[i].y * M2P-3);	
		glVertex2f(sb.deathPoint[i].x * M2P-3,sb.deathPoint[i].y * M2P+3);
		glVertex2f(sb.deathPoint[i].x * M2P+3,sb.deathPoint[i].y * M2P+3);
		glVertex2f(sb.deathPoint[i].x * M2P+3,sb.deathPoint[i].y * M2P-3);
		glEnd();
		glPopMatrix();
	}
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

void init_level0(int &dx, int &dy)
{
	g = 9.8;
	int centerx,centery;//开始点坐标
	
	centerx = WIDTH/7; centery = 20;
	dx=WIDTH-70; dy=HEIGHT-120; 
	vector< b2Vec2 > death_point;
	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
		//GameMap.push_back( pair< b2Vec2(,) , false> );
		goods.push_back( b2Vec2(WIDTH*2/5,HEIGHT-20) );
		goods.push_back( b2Vec2(WIDTH-20,HEIGHT-20) );
		goods.push_back( b2Vec2(WIDTH-20,HEIGHT-90) );
		goods.push_back( b2Vec2(WIDTH*2/3,HEIGHT-90) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		sb.create(GameMap , field, death_point , 0.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}

void init_level1(int &dx, int &dy)
{
	g = 9.8;
	int centerx,centery;//开始点坐标
	vector< b2Vec2 > death_point;
	death_point.push_back(b2Vec2(1100,-90+HEIGHT/3));
	//death_point.push_back(b2Vec2( , ));
	
	centerx = WIDTH/7; centery = 20;
	dx=WIDTH-70; dy=-40+5*HEIGHT/6; 

	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;

		goods.push_back( b2Vec2(0,HEIGHT/3) );
		goods.push_back( b2Vec2(0,-10+HEIGHT/3) );
		goods.push_back( b2Vec2(200,-10+HEIGHT/3) );
		goods.push_back( b2Vec2(200,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(200,HEIGHT/3) );
		goods.push_back( b2Vec2(200,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(300,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(300,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(300,HEIGHT/3) );
		goods.push_back( b2Vec2(300,-60+HEIGHT/3) );
		goods.push_back( b2Vec2(400,-60+HEIGHT/3) );
		goods.push_back( b2Vec2(400,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(400,HEIGHT/3) );
		goods.push_back( b2Vec2(400,-60+HEIGHT/3) );
		goods.push_back( b2Vec2(500,-90+HEIGHT/3) );
		goods.push_back( b2Vec2(600,-60+HEIGHT/3) );
		goods.push_back( b2Vec2(600,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(600,HEIGHT/3) );
		goods.push_back( b2Vec2(600,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(700,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(700,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(700,HEIGHT/3) );
		goods.push_back( b2Vec2(700,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(1000,-30+HEIGHT/3) );
		goods.push_back( b2Vec2(1000,HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(WIDTH,70+HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,60+HEIGHT/3) );
		goods.push_back( b2Vec2(500,110+HEIGHT/3) );
		goods.push_back( b2Vec2(500,120+HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(400,120+HEIGHT/3) );
		goods.push_back( b2Vec2(400,110+HEIGHT/3) );
		goods.push_back( b2Vec2(500,110+HEIGHT/3) );
		goods.push_back( b2Vec2(500,120+HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(150,120+HEIGHT/3) );
		goods.push_back( b2Vec2(150,110+HEIGHT/3) );
		goods.push_back( b2Vec2(250,110+HEIGHT/3) );
		goods.push_back( b2Vec2(250,120+HEIGHT/3) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(0,5*HEIGHT/6) );
		goods.push_back( b2Vec2(0,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(250,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(250,5*HEIGHT/6) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(410,5*HEIGHT/6) );
		goods.push_back( b2Vec2(410,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(800,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(800,5*HEIGHT/6) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		goods.push_back( b2Vec2(1000,5*HEIGHT/6) );
		goods.push_back( b2Vec2(1000,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(WIDTH,-10+5*HEIGHT/6) );
		goods.push_back( b2Vec2(WIDTH,5*HEIGHT/6) );
		GameMap.push_back( make_pair(goods , false) );
		goods.clear();
		
		///////////////////////////
		
		field.push_back(make_pair(make_pair(b2Vec2(500,20) , -3000.0), make_pair(3, false)));
		field.push_back(make_pair(make_pair(b2Vec2(330,-100 + 5*HEIGHT/6 ) , 3900.0), make_pair(3, false)));
		
		field.push_back(make_pair(make_pair(b2Vec2(900,-200 + 5*HEIGHT/6 ) , 3000.0), make_pair(3, false)));
		field.push_back(make_pair(make_pair(b2Vec2(900,+70 + 5*HEIGHT/6 ) , -3000.0), make_pair(3, false)));
		
		sb.create(GameMap , field, death_point , 1.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}

void init_level2(int &dx, int &dy)
{
	g = 9.8;
	int centerx, centery;
	centerx = WIDTH/7; centery = 20;
	dx=1000; dy=270; 
	vector<b2Vec2> death_point;
	
	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;

		goods.push_back(b2Vec2(150, 120));
		goods.push_back(b2Vec2(350, 120));
		goods.push_back(b2Vec2(150, 150));
		goods.push_back(b2Vec2(350, 150));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();
		
		goods.push_back(b2Vec2(500, 120));
		goods.push_back(b2Vec2(700, 120));
		goods.push_back(b2Vec2(500, 150));
		goods.push_back(b2Vec2(700, 150));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(500, 150));
		goods.push_back(b2Vec2(500, 185));
		goods.push_back(b2Vec2(450, 185));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(450, 185));
		goods.push_back(b2Vec2(450, 220));
		goods.push_back(b2Vec2(400, 220));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(400, 220));
		goods.push_back(b2Vec2(400, 255));
		goods.push_back(b2Vec2(350, 255));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(350, 255));
		goods.push_back(b2Vec2(350, 290));
		goods.push_back(b2Vec2(300, 290));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(300, 290));
		goods.push_back(b2Vec2(300, 500));
		goods.push_back(b2Vec2(350, 500));
		goods.push_back(b2Vec2(350, 290));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(150, 600));
		goods.push_back(b2Vec2(160, 600));
		goods.push_back(b2Vec2(160, 700));
		goods.push_back(b2Vec2(150, 700));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(160, 670));
		goods.push_back(b2Vec2(160, 700));
		goods.push_back(b2Vec2(700, 700));
		goods.push_back(b2Vec2(700, 670));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(670, 10));
		goods.push_back(b2Vec2(670, 120));
		goods.push_back(b2Vec2(700, 120));
		goods.push_back(b2Vec2(700, 10));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(450, 450));
		goods.push_back(b2Vec2(700, 450));
		goods.push_back(b2Vec2(700, 500));
		goods.push_back(b2Vec2(500, 500));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back(b2Vec2(800, 300));
		goods.push_back(b2Vec2(900, 300));
		goods.push_back(b2Vec2(900, 450));
		goods.push_back(b2Vec2(800, 450));
		GameMap.push_back(make_pair(goods, false));
		goods.clear();
		

		field.push_back(make_pair(make_pair(b2Vec2(475, 30), 10), make_pair(0, false)));
		field.push_back(make_pair(make_pair(b2Vec2(320, 600), 2), make_pair(0, false)));
		field.push_back(make_pair(make_pair(b2Vec2(450, 380), 3), make_pair(0, false)));
		field.push_back(make_pair(make_pair(b2Vec2(700, 380), 5), make_pair(0, false)));
		field.push_back(make_pair(make_pair(b2Vec2(730, 244), 7), make_pair(0, false)));
		
		//goods.clear();

		sb.create(GameMap , field, death_point, 0.5, false);

	}
	
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}


void init_level3(int &dx, int &dy)
{
	int centerx,centery;//开始点坐标
	
	centerx = 0; centery = 0;
	dx=WIDTH+1000; dy=HEIGHT+1000;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	srand(time(0));
	
	for(int i = 0; i < 500; ++i)
	{
		field.push_back(make_pair(make_pair(b2Vec2(rand() % WIDTH, rand() % HEIGHT), 0.1)
		, make_pair(2, true)));
	}
	g = 0;
	sb.create(GameMap , field, death_point , 0.5, true);
	sb.set_goal(dx,dy);

	b2Body* tmp = sb.world->GetBodyList();
	while(tmp)
	{
		int v = *((int*)tmp -> GetUserData());
		int fac1, fac2;
		fac1 = (rand() % 2) ? 1 : -1;
		fac2 = (rand() % 2) ? 1 : -1;
		if(v >= 10000)
		{
			tmp -> ApplyForce(b2Vec2(rand() % 100 * fac1, rand() % 100 * fac2), tmp -> GetWorldCenter());
		}
		tmp = tmp -> GetNext();
	}
}

void init_level4(int &dx, int &dy)
{
	int centerx,centery;//开始点坐标
	
	centerx = 0; centery = 0;
	dx=WIDTH+1000; dy=HEIGHT+1000;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	srand(time(0));
	
	for(int i = 0; i < 500; ++i)
	{
		field.push_back(make_pair(make_pair(b2Vec2(rand() % (WIDTH / 8), rand() % HEIGHT), 0)
		, make_pair(2, true)));
	}
	g = 0;
	sb.create(GameMap , field, death_point , 0.5, true);
	sb.set_goal(dx,dy);

	b2Body* tmp = sb.world->GetBodyList();
	while(tmp)
	{
		int v = *((int*)tmp -> GetUserData());
		int fac1, fac2;
		fac1 = (rand() % 2) ? 1 : -1;
		fac2 = (rand() % 2) ? 1 : -1;
		if(v >= 10000)
		{
			tmp -> ApplyForce(b2Vec2(rand() % 100 * fac1, rand() % 100 * fac2), tmp -> GetWorldCenter());
		}
		tmp = tmp -> GetNext();
	}
}
void init_level5(int &dx, int &dy)
{
	int centerx,centery;//开始点坐标
	
	centerx = 0; centery = 0;
	dx=WIDTH+1000; dy=HEIGHT+1000;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	srand(time(0));
	
	for(int i = 0; i < 500; ++i)
	{
		field.push_back(make_pair(make_pair(b2Vec2(rand() % WIDTH, rand() % HEIGHT), 0)
		, make_pair(2, true)));
	}
	g = 9.8;
	sb.create(GameMap , field, death_point , 0.5, true);
	sb.set_goal(dx,dy);

	b2Body* tmp = sb.world->GetBodyList();
	while(tmp)
	{
		int v = *((int*)tmp -> GetUserData());
		int fac1, fac2;
		fac1 = (rand() % 2) ? 1 : -1;
		fac2 = (rand() % 2) ? 1 : -1;
		if(v >= 10000)
		{
			tmp -> ApplyForce(b2Vec2((rand() * 2) % 200 * fac1, (rand() * 2) % 200 * fac2), tmp -> GetWorldCenter());
		}
		tmp = tmp -> GetNext();
	}
}


void init_level6(int &dx, int &dy)
{
	int centerx,centery;//开始点坐标
	
	centerx = 0; centery = 0;
	dx=WIDTH+1000; dy=HEIGHT+1000;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	srand(time(0));
	
	field.push_back(make_pair(make_pair(b2Vec2(WIDTH / 2, HEIGHT / 2), 100)
		, make_pair(2, false)));

	for(int i = 1; i < 2; ++i)
	{
		field.push_back(make_pair(make_pair(b2Vec2(WIDTH / 2, HEIGHT / 2 + rand() % 400), 4)
		, make_pair(2, true)));
	}
	g = 0;

	gpoint.clear();
	for(int i = 0; i < 360; ++i) 
	{
		gpoint.push_back(b2Vec2(WIDTH / 2 + cos((double)i*M_PI/180) * 60, HEIGHT / 2 + sin((double)i*M_PI/180)*60));
	}
	GameMap.push_back(make_pair(gpoint, false));
	gpoint.clear();


	sb.create(GameMap , field, death_point , 1, true, false);
	sb.set_goal(dx,dy);

	b2Body* tmp = sb.world->GetBodyList();
	while(tmp)
	{
		int v = *((int*)tmp -> GetUserData());
		int fac1, fac2;
		fac1 = (rand() % 2) ? 1 : -1;
		fac2 = (rand() % 2) ? 1 : -1;
		if(v >= 10001)
		{
			tmp -> ApplyForce(b2Vec2(60 * fac1, 0), tmp -> GetWorldCenter());
		}
		tmp = tmp -> GetNext();
	}
}

bool level(int &levelid)
{
	SDL_Event event;
	Uint32 start;
	bool running=true;

	int dx,dy;//目的地
	if(levelid >= MAXLEVEL) return true;
	gpoint.clear();
	init_func[levelid](dx, dy);
	
	while(running)
	{
		start=SDL_GetTicks();
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
				case SDL_QUIT: { running=false; sb.destroy(); break;}

				case SDL_KEYDOWN:
					switch(event.key.keysym.sym)
					{
						case SDLK_ESCAPE: { running=false; sb.destroy(); exit(0); }
						case SDLK_SPACE: 
						{ 
							sb.destroy();
							running=false;
							return false;
						}
						case SDLK_0:
						{
							levelid = 0;
							sb.destroy();
							return false;
						}
						case SDLK_1:
						{
							levelid = 1;
							sb.destroy();
							return false;
						}
						case SDLK_2:
						{
							levelid = 2;
							sb.destroy();
							return false;
						}
						case SDLK_3:
						{
							levelid = 3;
							sb.destroy();
							return false;
						}
						case SDLK_4:
						{
							levelid = 4;
							sb.destroy();
							return false;
						}
						case SDLK_5:
						{
							levelid = 5;
							sb.destroy();
							return false;
						}
						case SDLK_6:
						{
							levelid = 6;
							sb.destroy();
							return false;
						}
					}
					break;

				case SDL_MOUSEBUTTONDOWN:
					int x,y;
					switch(event.button.button)
					{
					case SDL_BUTTON_LEFT:
					//fprintf(stderr, "LEFT DOWN\n");
					x = event.button.x;
					y = event.button.y;
					gpoint.push_back(b2Vec2(x, y));
					break;
					case SDL_BUTTON_RIGHT:
					//fprintf(stderr, "RIGHT DOWN\n");
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

		display(dx,dy);
		int res = sb.simulateNextStep();
		if(res == 1)
		{
			sb.destroy();
			running = false;
			return true;
		}
		else if(res == -1)
		{
			sb.destroy();
			running = false;
			return false;
		}
		SDL_GL_SwapBuffers();
		if(1000.0/60>SDL_GetTicks()-start)
			SDL_Delay(1000.0/60-(SDL_GetTicks()-start));
	}
	return false;
}

void init()
{
	glMatrixMode(GL_PROJECTION);
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glClearColor(0,0,0,1);
	init_func[0] = init_level0;
	init_func[1] = init_level2;
	init_func[2] = init_level1;
	init_func[3] = init_level3;
	init_func[4] = init_level4;
	init_func[5] = init_level5;
	init_func[6] = init_level6;
	color[0].r = 0.293, color[0].g = 0, color[0].b = 0.508;
	color[1].r = 0, color[1].g = 0.801, color[1].b = 0.398;
	color[2].r = 0, color[2].g = 0.801, color[2].b = 0;
}

int main(int argc,char** argv)
{
	now_time = time(0);
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(WIDTH,HEIGHT,32,SDL_OPENGL);
	
	init();
	srand(2);
	
	for(int i = 0; i < MAXLEVEL; )
	{
		if(level(i))
			++i;
	}
	
	SDL_Quit();
	return 0;
}
