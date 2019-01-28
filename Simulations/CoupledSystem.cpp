#include "CoupledSystem.h"
#include <random>

using namespace GamePhysics;


void applySpringForces(const Points &points, const Springs &springs, Forces& forces, float stiffness)
{
	for (auto&& spring : springs) {
		auto& point1 = points[spring.first];
		auto& point2 = points[spring.second];

		float currentLength = GamePhysics::norm(point1.pos - point2.pos);
		if (currentLength < 2E-10f) {
			continue;
		}
		GamePhysics::Vec3 force = (point1.pos - point2.pos)
			/ currentLength
			* stiffness
			* (currentLength - spring.initialLength);
		forces[spring.first] -= force;
		forces[spring.second] += force;
	}
}

void truncatePositions(Points &points, GamePhysics::Vec3 limits)
{
	for (auto&& p : points) {
		for (int axis = 0; axis < 3; ++axis)
		{
			float maxCoord = limits[axis];
			auto && coord = p.pos[0];

			if (coord < -maxCoord)
			{
				p.pos[axis] = -maxCoord;
				p.vel[axis] = -p.vel[axis];
			} else if (coord > maxCoord)
			{
				p.pos[axis] = maxCoord;
				p.vel[axis] = -p.vel[axis];
			}
		}
	}
}

void truncatePositions2(Points& points, float radius) {
	for(size_t i = 0; i < points.size(); ++i)
	{
		Vec3 pos = points[i].pos;
		Vec3 vel = points[i].vel;

		float off = -0.5f + radius;
		for (int f = 0; f < 6; f++)
		{
			float sign = (f % 2 == 0) ? -1.0f : 1.0f;
			if (sign * pos[f / 2] < off)
			{
				pos[f / 2] = sign * off;
				vel[f / 2] = -vel[f / 2];
			}
		}

		points[i].pos = pos;
		points[i].vel = vel;
	}
}

void updatePositions(Points& points, float dt)
{
	for (auto && p : points) {
		p.pos += dt * p.vel;
	}
}

void updateVelocities(Points& points, const Forces& forces, float dt)
{
	for (std::size_t i = 0; i < points.size(); ++i) {
		points[i].vel += dt / points[i].mass * forces[i];
	}
}

using Collision = std::pair<int, int>;
using Collisions = std::vector<Collision>;

Collisions getCollisions(const Points& points) {

	Collisions collisions;
	for (int i = 0; i < points.size(); ++i) {
		for (std::size_t j = 0; j < i; ++j) {
			collisions.emplace_back(i, j);
		}
	}

	return collisions;
}

void collisionResponse(const Collisions& collisions, const Points& points, Forces& forces,
	float radius, Kernel kernel, float forceScale)
{	
	float diameter = 2.f * radius;
	for (auto && collision : collisions) {
		auto & p1 = points[collision.first].pos;
		auto & p2 = points[collision.second].pos;

		Vec3 difference = p1 - p2;
		float distance = GamePhysics::norm(difference);
		if (distance > diameter || distance < 1E-10f) {
			continue;
		}
		Vec3 normDifference = difference / distance;

		Vec3 force = kernel(distance / diameter) * forceScale * normDifference;
		forces[collision.first] += force;
		forces[collision.second] -= force;
	}
}

void applyDampingForce(Forces& forces, const Points& points, float damping) {
	for (std::size_t i = 0; i < points.size(); ++i) {
		forces[i] -= damping * points[i].vel;
	}
}

void CoupledSystem::advance(float dt, float mass, float radius, float stiffness, float forceScaling,
	float damping, GamePhysics::Vec3 externalForce,
	int accelerator, std::function<float(float)> kernel)
{
	// Compute Forces
	forces.resize(points.size());
	std::fill(forces.begin(), forces.end(), externalForce);

	applySpringForces(points, springs, forces, stiffness);

	Collisions collisions = getCollisions(points);
	collisionResponse(collisions, points, forces, radius, kernel, forceScaling);
	applyDampingForce(forces, points, damping);

	// Integrate
	updateVelocities(points, forces, dt);
	updatePositions(points, dt);
	//truncatePositions(points, Vec3{0.5f - radius});
	truncatePositions2(points, radius);

}

void CoupledSystem::addPoint(Point p)
{
	points.emplace_back(p);
}

void CoupledSystem::addSpring(Spring s)
{
	springs.emplace_back(s);
}

const std::vector<Point>& CoupledSystem::getPoints() const
{
	return points;
}

const std::vector<Spring>& CoupledSystem::getSprings() const
{
	return springs;
}

void CoupledSystem::reset(int n, float radius, float mass)
{
	points.clear();
	springs.clear();

	std::mt19937 eng;
	float spread = 0.1f * radius;
	std::uniform_real_distribution<float> jitter(-spread, spread);

	Vec3 pt;

	float maxCoord = .5f - radius;
	int numPoints = 0;

	for (pt.y = -maxCoord; pt.y < maxCoord; pt.y += 2.0f*radius) {
		for (pt.x = -maxCoord; pt.x < maxCoord; pt.x += 2.0f*radius) {
			for (pt.z = -maxCoord; pt.z < maxCoord && numPoints < n; pt.z += 2.0f*radius) {
				Vec3 pos = pt + Vec3(jitter(eng), jitter(eng), jitter(eng));
				addPoint({ pos, {}, mass});
				numPoints++;
			}
		}
	}
	
	int firstSpringPoint = numPoints;

	int grid_n = 5;
	int grid_m = 4;
	float seperation = radius * 2.f + 0.01f;
	Vec3 offset = -seperation / 2 *Vec3{(float) grid_n, 0, (float) grid_m};

	for (int i = 0; i < grid_n*grid_m; ++i, ++numPoints) {
		int x = i % grid_n;
		int z = i / grid_n;
		
		Vec3 pos = offset + Vec3{ (float)x, .4f, (float)z } * seperation;

		addPoint({ pos, {jitter(eng), jitter(eng), jitter(eng)}, mass });
		if (x > 0) {
			addSpring({seperation, 0.f, numPoints - 1, numPoints});
		}
		if (z > 0) {
			addSpring({ seperation, 0.f, numPoints - grid_n, numPoints });
		}
		float distance = 1.4142135623730951 * seperation;
		if ((x > 0) && (z > 0)) {
			addSpring({ distance, 0.f, numPoints - grid_n - 1, numPoints });
		}
		if ((z > 0) && (x < grid_n - 1)) {
			addSpring({ distance, 0.f, numPoints - grid_n + 1, numPoints });
		}

	}
	numSpringPoints = numPoints - firstSpringPoint;
	
	return;
}
