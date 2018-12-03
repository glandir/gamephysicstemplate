#include "RigidBodySystemSimulator.h"
#include <iostream>

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = TESTCASEUSEDTORUNTEST;
}

const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "One Step Test,Single Body,Two Body,Complex Scene";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;


	switch (m_iTestCase) {
	case 0:
	case 1:
	case 2:
	default:
		break;
	}
}

void RigidBodySystemSimulator::reset()
{
	m_rigidbodies.clear();
	m_barriers.clear();
	m_forces.clear();
	m_gravity = {};
	switch (m_iTestCase) {
	case 1:
		applyForceOnBody(0, { 0.3, .5,  .25 }, { 1000, 1000, 0 });
	case 0:
		addRigidBody({}, { 1, .6, 0.5 }, 2.);
		setOrientationOf(0, { {0,0,1}, M_PI*.5 });
		applyForceOnBody(0, { 0.3, 0.5, 0.25 }, { 1, 1, 0 });
		break;
	case 2:
		addRigidBody({ -.2,-.15,0 }, { .15, .25, .15 }, 1);
		addRigidBody({ .2,-.05,.1 }, { .15, .25, .15 }, 1);
		setVelocityOf(0, { .8 , 0, 0 });
		setVelocityOf(1, { -0.8 , 0, 0 });
		break;
	case 3:
		m_gravity = {0, -10, 0};

		addRigidBody({ -.2, 0, 0 }, { .1, .2, .3 }, .1);
		addRigidBody({ 0,.1,.1 }, { 0.2, 0.1, 0.3 }, .1);
		addRigidBody({ .2,.1,-.1 }, { 0.2, 0.1, 0.3 }, .1);
		addRigidBody({ -.2,.4,-.1 }, { 0.2, 0.1, 0.3 }, .1);
		setOrientationOf(0, { {1,0,0}, 0 });
		setOrientationOf(1, {{ -1,1,-1 }, M_PI / 4});
		setVelocityOf(0, {0, -.8, .1});
		setVelocityOf(1, { -.6, 0, 0 });
		setVelocityOf(2, { -6, 0, .1});
		setVelocityOf(3, { .6, 2, 0 });

		// immovable walls
		m_barriers.push_back(Rigidbody{ {0,0,-.6}, {1,1,.2}, 0. });
		m_barriers.push_back(Rigidbody{ {0,0,.6}, {1,1,.2}, 0. });
		m_barriers.push_back(Rigidbody{ {0,-.6,0}, {1,.2,1}, 0. });
		m_barriers.push_back(Rigidbody{ {0,.6,0}, {1,.2,1}, 0. });
		m_barriers.push_back(Rigidbody{ {-.6,0,0}, {.2,1,1}, 0. });
		m_barriers.push_back(Rigidbody{ {.6,0,0}, {.2,1,1}, 0. });
		break;
	default:
		break;
	}
	
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
 	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (auto&& r : m_rigidbodies) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawRigidBody(r.getObjToWorldMap());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	doSimulation = true;

	switch (m_iTestCase) {
	case 0:
		if (!doSimulation) return;
		simulateTimestep(2.f);
		printResults();
		doSimulation = false;
		break;
	case 1: break;
	case 2: break;
	default: break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 1.f;
		inputWorld = inputWorld * inputScale;
		m_movableObjectPos = m_movableObjectFinalPos + inputWorld;
	}
	else {
		m_movableObjectFinalPos = m_movableObjectPos;
	}

	applyForceOnBody(0, m_rigidbodies[0].p + Vec3{ 0, 0, 0.3 }, m_movableObjectPos - m_movableObjectFinalPos);
}

void RigidBodySystemSimulator::simulateTimestep(float dt)
{
	if (!doSimulation) return;

	for (auto& f : m_forces) {
		m_rigidbodies[f.id].applyForce(f);
	}
	m_forces.clear();

	for (auto& r: m_rigidbodies) {
		// f.id is ignored anyways when the force is applied directly.
		r.applyForce({ -1, r.p, m_gravity / r.m_inv });
	}

	for (auto& r : m_rigidbodies) {
		r.integrate(dt);
	}

	if (!m_collision) {
		return;
	}
	
	for (int i = 0; i < getNumberOfRigidBodies(); i++) {
		for (int j = 0; j < i; j++) {
			Rigidbody::collide(m_rigidbodies[i], m_rigidbodies[j], false);
		}
	}

	for (auto& b : m_barriers) {
		for (auto& r : m_rigidbodies) {
			Rigidbody::collide(r, b, true);
		}
	}

}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_rigidbodies[i].p;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_rigidbodies[i].v;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_rigidbodies[i].w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_forces.push_back({i, loc, force});
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, double mass)
{
	m_rigidbodies.push_back(Rigidbody{position, size, 1./ (double)mass});
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_rigidbodies[i].r = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_rigidbodies[i].v = velocity;
}

void RigidBodySystemSimulator::printResults()
{
	auto& r = m_rigidbodies[0];
	Vec3 v_wsp = r.getVel(Vec3{ -.3, -.5, -.25 } -r.p);
	std::cout
		<< "Linear velocity v = " << getLinearVelocityOfRigidBody(0) << std::endl
		<< "Angular velocity w = " << getAngularVelocityOfRigidBody(0) << std::endl
		<< "World space velocity of (0.3, 0.5, 0.25) v_0 = " << v_wsp << std::endl;
}
