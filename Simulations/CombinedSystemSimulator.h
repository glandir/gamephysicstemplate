#pragma once
#include "Simulator.h"
#include "CoupledSystem.h"

#define NAIVEACC 0
#define GRIDACC 1

class CombinedSystemSimulator: public Simulator {
public:
		int numFrames = 0;
		// Construtors
		CombinedSystemSimulator();
		// Attributes
		Vec3 externalForce;
		Point2D m_mouse;
		Point2D m_trackmouse;
		Point2D m_oldtrackmouse;
		float m_fMass;
		float m_fRadius;
		float m_fStiffness;
		float m_fForceScaling;
		float m_fDamping;
		int   m_iNumSpheres;
		int   m_iAccelerator;
		int   m_iKernel;

		CoupledSystem m_coupledSystem;

		static std::function<float(float)> m_Kernels[5];

		// Functions
		const char * getTestCasesStr();
		void initUI(DrawingUtilitiesClass * DUC);
		void reset();
		void resetSystem();
		void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
		void notifyCaseChanged(int testCase);
		void externalForcesCalculations(float timeElapsed);
		void simulateTimestep(float timeStep);
		void onClick(int x, int y);
		void onMouse(int x, int y);
};
