#include "MassSpringSystemSimulator.h"

#include <iostream>

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setupSystem1();
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "Simple Setup (one step),Simple Setup (animated),Complex Setup";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	
	
	switch (m_iTestCase) {
	case 0: break;
	case 1:
	case 2: {
		TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integration Method", "Euler,Leapfrog,Midpoint");
		TwAddVarRW(DUC->g_pTweakBar, "Integration Method", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		break;
		}
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse = m_trackmouse = m_oldtrackmouse = { 0, 0 };
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	
	DUC->beginLine();
	for (Spring s : m_springs) {
		Vec3 color = { 0.5f };
		DUC->drawLine(getPositionOfMassPoint(s.point1) * m_posScale, color, getPositionOfMassPoint(s.point2) * m_posScale, color);
		//std::cout << GamePhysics::norm(getPositionOfMassPoint(s.point1) - getPositionOfMassPoint(s.point2)) << std::endl;
	}
	DUC->endLine();

	for (Point p : m_points) {
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(p.pos * m_posScale, { m_massScale * float(p.mass) });
	}
	//std::cout << "-----." << std::endl;


}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase) {
	case 0:
		runSingleStepTests();
		break;
	case 1:
		setupSystem1();
		break;
	case 2:
		setupSystem2();
	default: break;
	}

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
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
		float inputScale = 100.f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}

	applyExternalForce(m_vfMovableObjectPos - m_vfMovableObjectFinalPos);
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) {
		return;
	}

	simulationStep(timeStep, m_iIntegrator);
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	m_points.push_back({ position, Velocity, {}, m_fMass, m_fDamping, isFixed });
	return m_points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.push_back({ masspoint1, masspoint2, m_fStiffness, initialLength, 0.f });
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_points[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_points[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}

void MassSpringSystemSimulator::setupSystem1()
{
	setMass(10);
	setStiffness(40);
	setDampingFactor(0);

	m_points.clear();
	addMassPoint({ 0.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, false);
	addMassPoint({ 0.f ,2.f, 0.f }, { 1.f, 0.f, 0.f }, false);
	
	m_springs.clear();
	addSpring(0, 1, 1.f);

	m_oldPoints.clear();
	m_oldPoints.resize(m_points.size());

	m_case1_done = false;
	m_truncatePositions = false;

	m_externalForce = {};
	m_gravity = {};
	m_limits = { std::numeric_limits<float>::infinity() };

}

void MassSpringSystemSimulator::setupSystem2()
{
	setMass(10);
	setStiffness(40);
	setDampingFactor(0);

	m_points.clear();
	m_springs.clear();

	std::mt19937 eng;
	std::uniform_real_distribution<float> pos(-4.f, 4.f);
	std::uniform_real_distribution<float> vel(0.f, 0.f); // TODO: change maybe?
	std::uniform_real_distribution<float> len(0.1f, 3.f);
	std::bernoulli_distribution spr(0.4);

	for (int i = 0; i < 10; i++) {
		addMassPoint({ pos(eng), pos(eng), pos(eng) }, { vel(eng), vel(eng), vel(eng) }, false);
	}

	for (int i = 0; i < m_points.size(); i++) {
		for (int j = 0; j < m_points.size(); j++) {
			if (i != j && spr(eng)) {
				addSpring(i, j, len(eng));
			}
		}
	}

	addMassPoint({ 0.f, 4.f, 0.f }, { 0.f }, true);
	setStiffness(80);
	addSpring(m_points.size() - 1, 0, 1.5f);

	m_oldPoints.clear();
	m_oldPoints.resize(m_points.size());

	m_case1_done = false;
	m_truncatePositions = false;

	m_externalForce = {};
	m_gravity = {0.f, -100.f, 0.f};
	m_limits = { 5.f };
}


void MassSpringSystemSimulator::simulationStep(float dt, int integrator)
{
	clearForces(m_points);
	applyExternalForces(m_points, m_externalForce + m_gravity);
	applySpringForces(m_points, m_springs);

	switch (integrator) {
	case EULER:
		eulerIntegrate(m_points, dt);
		break;
	case LEAPFROG:
		leapfrogIntegrate(m_points, dt);
		break;
	case MIDPOINT:
		midpointIntegrate1(m_points, m_oldPoints, dt);
		clearForces(m_points);
		applyExternalForces(m_points, m_externalForce + m_gravity);
		applySpringForces(m_points, m_springs);
		midpointIntegrate2(m_points, m_oldPoints, dt);
		break;
	default:
		break;
	}

	m_externalForce = {};
	
	truncatePositions(m_points, m_limits);
}

void MassSpringSystemSimulator::runSingleStepTests()
{
	if (m_case1_done) {
		return;
	}
	std::cout << "One Step Simulation: ignoring timeStep value and using dt = 0.1" << std::endl;

	std::string names[] = {"Euler", "Leapfrog (not implemented yet)", "Midpoint"};
	for (int method : {EULER, LEAPFROG, MIDPOINT}) {
		setupSystem1();
		simulationStep(0.1f, method);

		std::cout << "\n\nResult of the " << names[method] << " method:\n";
		int i = 0;
		for (auto p : m_points) {
			std::cout
				<< "Point #" << i++ << std::endl
				<< "\tp = " << p.pos.toString() << std::endl
				<< "\tv = " << p.vel.toString() << std::endl
				<< "\tf = " << p.force.toString() << std::endl;
		}
	}

	m_case1_done = true;
}
