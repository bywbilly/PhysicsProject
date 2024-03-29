#ifndef _PHYSICS
#define _PHYSICS
#include <Box2D/Box2D.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <vector>
#include "convexhull.h"
#include "Const.h"
#include <algorithm>
#include <cassert>

using namespace std;

const int MAX_BODY_LIMIT = 1024;

class Simulator
{
private:
public:
	double goalx, goaly;
	double radius;
	b2World *world;
	int userdata[MAX_BODY_LIMIT], cnt;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	vector<b2Vec2> deathPoint;
	double Restore;
	bool isCreated;
	b2Body* addRect(int x,int y,int w,int h,bool dyn=true, double friction = 0.5)
	{
		assert(isCreated);
		b2BodyDef bodydef;
		bodydef.position.Set(x*P2M,y*P2M);
		if(dyn)
			bodydef.type=b2_dynamicBody;
		bodydef.linearDamping = 0.000f;
		bodydef.angularDamping = 0.000f;

		b2Body* body=world->CreateBody(&bodydef);

		b2PolygonShape shape;
		shape.SetAsBox(P2M*w/2,P2M*h/2);

		b2FixtureDef fixturedef;
		fixturedef.shape=&shape;
		fixturedef.density=1.0;
		fixturedef.restitution = Restore;
		fixturedef.friction = friction;
		body->CreateFixture(&fixturedef);
		body->SetUserData(&userdata[cnt]);
		userdata[cnt++] = 1;
		return NULL;
	}

	/*return true if the polygon is valid*/
	void addPolygon(vector<b2Vec2> points, bool canMove, double density = 1, double friction = 0.00, int usrdata = 1)
	{
		assert(isCreated);
		//fprintf(stderr, "ADD POLYGON\n");
		for (int i = 0; i < points.size(); ++i)
			points[i].x *= P2M, points[i].y *= P2M;
		points = convexhull(points);
		//fprintf(stderr, "check return\n");
		if (points.size() < 3)
			return;
		//fprintf(stderr, "convexhull SUCCESS %d\n", (int)points.size());

		b2BodyDef bodydef;
		b2Vec2 Gcenter;
		Gcenter = b2Vec2(0, 0);
		float area = 0;
		for (int i = 1; i < (int)points.size() - 1; ++i)
		{
			float now_area = det(points[i] - points[0], points[i + 1] - points[0]);
			area += now_area;
			b2Vec2 now_point = points[0] + points[i] + points[i + 1];
			now_point.x *= now_area / 3;
			now_point.y *= now_area / 3;
			Gcenter = Gcenter + now_point;
		}
		Gcenter.x /= area, Gcenter.y /= area;
		bodydef.position.Set(Gcenter.x, Gcenter.y);
		if(canMove)
			bodydef.type=b2_dynamicBody;
		bodydef.linearDamping = 0;
		bodydef.angularDamping = 0;

		b2Body* body=world->CreateBody(&bodydef);

		b2PolygonShape shape;
		b2Vec2 *f = new b2Vec2[points.size()];
		for(int i = 0; i < points.size(); ++i)
		{
			f[i] = points[i] - Gcenter;
		}
		//fprintf(stderr, "start Set\n");
		shape.Set(f, points.size());
		//fprintf(stderr, "end Set\n");
		delete f;

		b2FixtureDef fixturedef;
		fixturedef.shape=&shape;
		fixturedef.density=density;
		fixturedef.restitution = Restore;
		fixturedef.friction = friction;
		body->CreateFixture(&fixturedef);
		body->SetUserData(&userdata[cnt]);
		userdata[cnt++] = usrdata;
        //fprintf(stderr, "ADD POLYGON SUCCESS\n");
	}

	void init(const vector<pair<vector<b2Vec2>, bool> > &GameMap, const vector<pair<pair<b2Vec2, double>, pair<double, bool> > > &Field
		, bool needFence)
	{
		assert(isCreated);
		if(needFence)
		{
			addRect(WIDTH/2,0,WIDTH,10,false);
			addRect(WIDTH/2,HEIGHT-10,WIDTH,20,false);
			addRect(0,0,20,HEIGHT * 10,false);
			addRect(WIDTH,0,20,HEIGHT * 10,false);
		}
		for(int i = 0; i < GameMap.size(); ++i)
			addPolygon(GameMap[i].first, GameMap[i].second);
		field = Field;
		for(int i = 0; i < field.size(); ++i)
		{
			vector<b2Vec2> point;
			for(int j = 0; j < 360; ++j)
			{
				point.push_back(field[i].first.first + b2Vec2(5 * cos((double)j * M_PI / 180), 5 * sin((double)j * M_PI / 180)));
			}
			//fprintf(stderr, "add field poly\n");
			addPolygon(point, field[i].second.second, 1, 0, 10000 + i);
			field[i].first.first.x *= P2M, field[i].first.first.y *= P2M;
		}
	}
	Simulator()
	{
		isCreated = false;
	}
	~Simulator()
	{
		isCreated = false;
		delete world;
		world = NULL;
	}

	void create(vector<pair<vector<b2Vec2>, bool> > GameMap, vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field, vector<b2Vec2> dPoint, double r
			, bool isFullyRestore, bool needFence = true)
	{
		if(isFullyRestore)
			Restore = 1;
		else
			Restore = 0.5;
		isCreated = true;
		radius = r;
		cnt = 0;
		world = new b2World(b2Vec2(0, g), false);
		init(GameMap, field, needFence);
		deathPoint = dPoint;
		for(int i = 0; i < deathPoint.size(); ++i)
			deathPoint[i].x *= P2M, deathPoint[i].y *= P2M;
	}

	void destroy()
	{
		if(!isCreated)
			return;
		isCreated = false;
		delete world;
		world = NULL;
	}

	void set_goal(double dx, double dy)
	{
		assert(isCreated);
		goalx = dx * P2M;
		goaly = dy * P2M;
	}

	/*The return value should be a image*/
	int simulateNextStep()
	{
		assert(isCreated);
		world -> Step(1.0 / 60, 8, 8);
		b2Body* tmp=world->GetBodyList();
		vector<b2Vec2> points;
		int count = 0;
		while(tmp)
		{
			int v = *((int*)tmp -> GetUserData());
			if(v >= 10000)
			{
				field[v - 10000].first.first = tmp -> GetWorldCenter();
			}
			tmp = tmp -> GetNext();
		}
		tmp = world -> GetBodyList();
		while(tmp)
		{
			b2Vec2 pos = tmp -> GetWorldCenter();
			b2Vec2 force;
			force = b2Vec2(0, 0);
			if(*((int*)tmp -> GetUserData()) == -1)
			{
				if((pos.x - goalx) * (pos.x - goalx) + (pos.y - goaly) * (pos.y - goaly) <= radius * radius)
				{
					//fprintf(stderr, "WIN\n");
					return 1;
				}
				for(int i = 0; i < deathPoint.size(); ++i)
				{
					b2Vec2 dp = deathPoint[i];
					if((pos.x - dp.x) * (pos.x - dp.x) + (pos.y - dp.y) * (pos.y - dp.y) <= radius * radius)
					{
						//fprintf(stderr, "dead\n");
						return -1;
					}
				}
			}
			int v = *((int*)tmp -> GetUserData());
			for(int i = 0; i < field.size(); ++i)
			{
				if(i == v - 10000) continue;
				double r = sqrt((pos.x - field[i].first.first.x) * (pos.x - field[i].first.first.x) + (pos.y - field[i].first.first.y) * (pos.y - field[i].first.first.y));
				b2Vec2 direction = field[i].first.first - pos;
				double len = sqrt(direction.x * direction.x + direction.y * direction.y);
				direction.x /= len * pow(r, field[i].second.first), direction.y /= len * pow(r, field[i].second.first);
				direction.x *= field[i].first.second, direction.y *= field[i].first.second;
				force = force + direction;
			}
			//fprintf(stderr, "Force %f %f\n", force.x, force.y);
			tmp -> ApplyForce(force, pos);
			tmp=tmp->GetNext();
		}
		return 0;
	}
};

#endif
