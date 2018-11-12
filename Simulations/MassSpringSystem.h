#ifndef MASSSPRINGSYSTEM_H
#define MASSSPRINGSYSTEM_H

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

void clearForces(Points& points);
void applyExternalForces(Points& points, GamePhysics::Vec3 force_ext);
void applySpringForces(Points& points, Springs& springs);

// Implements a pseudo collision which simply pushes points back inside a rectangle
// with corners (-limits, limits)
// limits is assumed to be in octant I (+++)
void truncatePositions(Points& points, GamePhysics::Vec3 limits);

#endif //!MASSPRINGSYSTEM_H