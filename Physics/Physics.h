#ifndef _PHYSICS
#define _PHYSICS
#include <Box2D/Box2D.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <vector>
#include "convexhull.h"
#include "Const.h"

class Simulator
{
private:
public:
	b2World *world;
	void init()
	{
		world = new b2World(b2Vec2(0,g), false);
	}
	Simulator()
	{
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
			points[i].x *= P2M;
		points = convexhull(points);

		b2BodyDef bodydef;
		bodydef.position.Set(points[0].x, points[0].y);
		if(canMove)
			bodydef.type=b2_dynamicBody;
		b2Body* body=world->CreateBody(&bodydef);

		b2PolygonShape shape;
		b2Vec2 *f = new b2Vec2[points.size()];
		for(int i = 0; i < points.size(); ++i)
			f[i] = points[i];
		shape.Set(f, points.size());
		delete f;

		b2FixtureDef fixturedef;
		fixturedef.shape=&shape;
		fixturedef.density=1.0;
		body->CreateFixture(&fixturedef);
        fprintf(stderr, "ADD POLYGON SUCCESS\n");
	}

	/*The return value should be a image*/
	void simulateNextStep()
	{
		world -> Step(1.0 / 30, 8, 8);
	}
};

#endif
