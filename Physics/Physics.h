#ifndef _PHYSICS
#define _PHYSICS
#include <Box2D/Box2D.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <vector>

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
	bool addPolygon(vector<b2Vec2> points, bool canMove)
	{
		
	}

	/*The return value should be a image*/
	void simulateNextStep()
	{

	}
};

#endif
