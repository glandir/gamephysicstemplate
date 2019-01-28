#pragma once

#include <vector>
#include <functional>
#include "util/vectorbase.h"

struct Point {
	GamePhysics::Vec3 pos;
	GamePhysics::Vec3 vel;
	float mass;
};

struct Spring {
	float initialLength;
	float stiffness;
	int first, second;
};

using Points = std::vector<Point>;
using Springs = std::vector<Spring>;
using Forces = std::vector<GamePhysics::Vec3>;
using Kernel = std::function<float(float)>;

class CoupledSystem
{
public:
	void advance(float dt, float mass, float radius, float stiffness, float forceScaling,
		float damping, GamePhysics::Vec3 externalForce,
		int accelerator, Kernel kernel);
	
	void addPoint(Point p);
	void addSpring(Spring s);

	const Points& getPoints() const;

	const Springs& getSprings() const;

	int getNumSpringPoints() const { return numSpringPoints; }

	void reset(int n, float radius, float mass);

private:
	Points points;
	Springs springs;
	Forces forces;
	int numSpringPoints = 0;

};

