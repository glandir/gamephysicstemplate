#pragma once

#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
#include <vector>

using namespace GamePhysics;

struct Force {
	int id;
	Vec3 p;
	Vec3 f;
};

struct Rigidbody {
	explicit Rigidbody(Vec3 position, Vec3 size, double mass);

	void applyForce(Force f);
	void applyImpulse(Vec3 J, Vec3 offset);
	void integrate(float dt);
	static void collide(Rigidbody & a, Rigidbody & b, bool bIsBarrier);

	Vec3 getVel(Vec3 offset);
	Mat4 getObjToWorldMap();

	Vec3 p = {};			// position
	Vec3 v = {};			// velocity
	Vec3 w = {};
	Quat r = {0, 0, 0, 1};
	double m_inv = 0;		// 1 / mass
private:
	Vec3 f = {};
	Vec3 q = {};
	Vec3 L = {};
	Mat4 I_inv_0 = {};		// I_0^-1
	Mat4 I_inv_curr = {};	// I^-1
	Vec3 s = {};			// size
};