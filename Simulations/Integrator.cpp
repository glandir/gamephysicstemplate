#include "Integrator.h"

void eulerIntegrate(Points & points, float dt)
{
	for (Point& p : points) {
		if (p.isFixed) {
			continue;
		}

		p.pos += dt * p.vel;
		p.vel += dt / p.mass * p.force;
	}
}

void midpointIntegrate1(Points & points, OldPoints& oldpoints, float dt)
{
	using namespace GamePhysics;
	for (int i = 0; i < points.size(); i++) {
		auto& p = points[i];
		if (p.isFixed) {
			continue;
		}
		oldpoints[i] = {p.pos, p.vel};
		p.pos += (dt / 2) * p.vel;
		p.vel += (dt / 2 / p.mass) * p.force;
	}
}

void midpointIntegrate2(Points & points, OldPoints & oldpoints, float dt)
{
	for (int i = 0; i < points.size(); i++) {
		auto& p = points[i];
		if (p.isFixed) {
			continue;
		}

		auto& oldP = oldpoints[i];
		p.pos = oldP.pos_old + dt * p.vel;
		p.vel = oldP.vel_old + dt / p.mass * p.force;
	}
}

void leapfrogIntegrate(Points & points, float dt)
{
	for (Point& p : points) {
		if (p.isFixed) {
			continue;
		}

		p.vel += dt / p.mass * p.force;
		p.pos += dt * p.vel;
	}
}
