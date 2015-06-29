#ifndef _PHYSICS
#define _PHYSICS
#include <Box2D/Box2D.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <convexhull.h>

class Simulator
{
public:
	const double g = 9.8;
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
		points = convexhull(points);

		b2BodyDef bodydef;
		bodydef.position.Set(points[0].x, points[0].y);
		if(canMove)
			bodydef.type=b2_dynamicBody;
		b2Body* body=world->CreateBody(&bodydef);

		b2PolygonShape shape;
		shape.Set(0, 0);
		for(int i = 1; i < points.size(); ++i)
			shape.Set(points[i].x - points[0].x, points[i].y - points[0].y);

		b2FixtureDef fixturedef;
		fixturedef.shape=&shape;
		fixturedef.density=1.0;
		body->CreateFixture(&fixturedef);
	}

	/*The return value should be a image*/
	void simulateNextStep()
	{
		world -> Step(1.0 / 30, 8, 8);
	}
};

#endif
