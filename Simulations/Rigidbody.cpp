#include "Rigidbody.h"
#include "collisionDetect.h"


Rigidbody::Rigidbody(Vec3 position, Vec3 size, double mass_inv)
{
	p = position;
	m_inv = mass_inv;
	r = { 0,0,0,1 };

	I_inv_0 = {};
	for (int i : {0, 1, 2}) {
		double a = size[(i + 1) % 3], b = size[(i + 2) % 3];
		I_inv_0.value[i][i] = 12.0 * m_inv / (a*a + b*b); // cf. wikipedia
	}

	s = size;
}

void Rigidbody::applyForce(Force f)
{
	Vec3 p_osp = f.p - p;

	this->f += f.f;
	q += cross(p_osp, f.f);
}

void Rigidbody::applyImpulse(Vec3 J, Vec3 offset)
{
	v += m_inv * J;
	L += cross(offset, J);
}

void Rigidbody::integrate(float dt)
{
	p += dt * v;
	v += dt * m_inv * f;
	r += .5 * dt * Quat{ w[0], w[1], w[2] , 0 } * r;
	r /= r.norm(); // may have found it...
	L += dt * q;

	Mat4 Rot_r = r.getRotMat();
	Mat4 Rot_r_t = r.getRotMat();
	Rot_r_t.transpose();

	I_inv_curr = Rot_r * I_inv_0 * Rot_r_t;

	w = I_inv_curr * L;

	f = {};
	q = {};
}

void Rigidbody::collide(Rigidbody& a, Rigidbody& b, bool isBarrier)
{
	using namespace GamePhysics;
	CollisionInfo info = checkCollisionSAT(a.getObjToWorldMap(), b.getObjToWorldMap());
	if (!info.isValid) {
		return;
	}
	
	Vec3 n = info.normalWorld;
	Vec3 x_a = info.collisionPointWorld - a.p;
	Vec3 x_b = info.collisionPointWorld - b.p;

	Vec3 vrel = a.getVel(x_a) - b.getVel(x_b);

	if (dot(vrel, n) > 0.) {
		return;
	}

	double c = 1;

	double numinator = -(1 + c) * dot(vrel, n);
	double denominator = a.m_inv + b.m_inv +
		dot(
			cross(
				a.I_inv_curr * (cross(x_a, n)),
				x_a)
			+ cross(
				b.I_inv_curr * (cross(x_b, n)),
				x_b),
			n
		);
	
	Vec3 J = numinator / denominator * n;
	
	a.applyImpulse(J, x_a);
	if (!isBarrier) {
		b.applyImpulse(-J, x_b);
	}
}

Vec3 Rigidbody::getVel(Vec3 offset)
{
	return v  + cross(w, offset);
}

Mat4 Rigidbody::getObjToWorldMap()
{
	Mat4 scaleMat = {};
	scaleMat.initScaling(s[0], s[1], s[2]);

	Mat4 transMat = {};
	transMat.initTranslation(p[0], p[1], p[2]);

	return scaleMat * r.getRotMat() * transMat;
}

