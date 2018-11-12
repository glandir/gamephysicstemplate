#include "Point.h"
#include <vector>

void clearForce(Points& points)
{
	for (auto&& point : points) {
		point.force = {0.0};
	}
}


void applyExternal(Points& points, GamePhysics::Vec3 force_ext)
{
	for (auto&& point : points) {
		point.force += force_ext;
	}
}

void applySpringForce(Points&points, Springs& springs)
{
	for (auto&& spring : springs) {
		Point& point1 = points[spring.point1];
		Point& point2 = points[spring.point2];
		
		spring.currentLength = GamePhysics::norm(point1.pos - point2.pos);
		//std::cout << "l = " <<  spring.currentLength << std::endl;
		GamePhysics::Vec3 force = (point1.pos - point2.pos)
			/ spring.currentLength
			* spring.stiffness 
			* (spring.currentLength - spring.initialLength);
		
		//std::cout << "f = " << force.toString() << std::endl;

		point1.force -= force;
		point2.force += force;
	}
}

void truncatePositions(Points & points, GamePhysics::Vec3 limits)
{
	/*
	for (auto&& p : points) {
		p.pos.makeFloor(limits);
		p.pos.makeCeil(limits * -1);
	}
	*/
	const float eps = 1 / 32.f;
	for (auto&& p : points) {
		for (int i = 0; i < 3; i++) {
			auto&& val = p.pos.value[i];
			auto max = limits.value[i];
			if (val >= max) {
				val = max - eps;
				p.vel.value[i] = 0.f;
			}
			if (val <= -max) {
				val = -max + eps;
				p.vel.value[i] = 0.f;
			}
		}
	}
}

