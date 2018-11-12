#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h

#include "Simulator.h"
#include "Point.h"
#include "Integrator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// Simple two point system from excercise sheet
	void setupSystem1();
	// more complex system with more points and springs
	void setupSystem2();

	void simulationStep(float dt, int integrator);
	void runSingleStepTests();

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	int m_iTestCase;
	Vec3 m_externalForce;
	Vec3 m_gravity;
	Vec3 m_limits;

	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;

	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Points m_points;
	Springs m_springs;
	OldPoints m_oldPoints;

	bool m_case1_done = false;

	static constexpr float m_massScale = 0.001f;
	static constexpr float m_posScale = 0.1f;

	bool m_truncatePositions = false;
};

#endif