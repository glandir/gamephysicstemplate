#ifndef POINT_H
#define POINT_H

#include <vector>
#include "util/vectorbase.h"

struct Point {
	GamePhysics::Vec3 pos;
	GamePhysics::Vec3 vel;
	GamePhysics::Vec3 force;
	float mass;
	float dampening;
	bool isFixed;
};


struct Spring {
	int point1;
	int point2;
	float stiffness;
	float initialLength;
	float currentLength;
};

struct OldPoint {
	GamePhysics::Vec3 pos_old;
	GamePhysics::Vec3 vel_old;
};

using Points = std::vector<Point>;
using OldPoints = std::vector<OldPoint>;
using Springs = std::vector<Spring>;

void clearForce(Points& points);
void applyExternal(Points& points, GamePhysics::Vec3 force_ext);
void applySpringForce(Points& points, Springs& springs);

void truncatePositions(Points& points, GamePhysics::Vec3 limits);
#endif // !POINT_H
