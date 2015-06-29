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

class Simulator
{
private:
public:
	b2World *world;
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
	void init()
	{
		addRect(WIDTH/2,0,WIDTH,30,false);
		addRect(WIDTH/2,HEIGHT-50,WIDTH,30,false);
		addRect(10,0,10,HEIGHT * 10,false);
		addRect(WIDTH,0,10,HEIGHT * 10,false);
	}
	Simulator()
	{
		world = new b2World(b2Vec2(0,g), true);
		init();
	}
	Simulator(b2World *w)
	{
		world = w;
		init();
	}
	~Simulator()
	{
		delete world;
	}

	/*return true if the polygon is valid*/
	void addPolygon(vector<b2Vec2> points, bool canMove)
	{
        fprintf(stderr, "ADD POLYGON\n");
		for(int i = 0; i < points.size(); ++i)
			points[i].x *= P2M, points[i].y *= P2M;
		points = convexhull(points);
		fprintf(stderr, "check return\n");
		if(points.size() < 3)
			return;
		fprintf(stderr, "convexhull SUCCESS %d\n", (int)points.size());

		b2BodyDef bodydef;
		b2Vec2 Gcenter;
		Gcenter = b2Vec2(0, 0);
		for(int i = 0; i < points.size(); ++i)
			Gcenter = Gcenter + points[i];
		Gcenter.x /= points.size(), Gcenter.y /= points.size();
		bodydef.position.Set(Gcenter.x, Gcenter.y);
		if(canMove)
			bodydef.type=b2_dynamicBody;
		bodydef.linearDamping = 0.0f;
		bodydef.angularDamping = 0.0f;

		b2Body* body=world->CreateBody(&bodydef);

		fprintf(stderr, "fuck1\n");

		b2PolygonShape shape;
		b2Vec2 *f = new b2Vec2[points.size()];
		for(int i = 0; i < points.size(); ++i)
		{
			f[i] = points[i] - Gcenter;
		}
		shape.Set(f, points.size());
		delete f;

		fprintf(stderr, "fuck2\n");

		b2FixtureDef fixturedef;
		fixturedef.shape=&shape;
		fixturedef.density=1.0;
		fixturedef.restitution = 1.0;
		fixturedef.friction = 0;
		body->CreateFixture(&fixturedef);

        fprintf(stderr, "ADD POLYGON SUCCESS\n");
	}

	/*The return value should be a image*/
	void simulateNextStep()
	{
		world -> Step(1.0 / 30, 8, 3);
	}
};

#endif
