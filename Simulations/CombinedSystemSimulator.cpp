#include "CombinedSystemSimulator.h"


std::function<float(float)> CombinedSystemSimulator::m_Kernels[5] = {
	[](float x) {return 1.0f; },              // Constant, m_iKernel = 0
	[](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
	[](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
	[](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
	[](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

// CombinedSystemSimulator member functions
CombinedSystemSimulator::CombinedSystemSimulator()
{
	m_iTestCase = 1;
	m_fMass = 0.01f;
	m_fRadius = 0.06f;
	m_fForceScaling = 1.0f;
	m_fDamping = 0.01f;
	m_fStiffness = 0.6f;
	m_iNumSpheres = 180;
	m_iAccelerator = 1;
	m_iKernel = 4;
}


const char * CombinedSystemSimulator::getTestCasesStr()
{
	return "BasicTest";
}

void CombinedSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void CombinedSystemSimulator::resetSystem()
{
	m_coupledSystem.reset(m_iNumSpheres, m_fRadius, m_fMass);
}

void CombinedSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	//TwAddButton(g_pTweakBar, "Reset Spheres", [](void *) {g_pSphereSystem = std::unique_ptr<SphereSystem>(new SphereSystem(g_iNumSpheres, g_fRadius)); }, nullptr, "");
	TwAddVarCB(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32,
		[](const void *value, void*  s) {
			CombinedSystemSimulator* this_ = (CombinedSystemSimulator*)s;
			this_->m_iNumSpheres = *(int*)value;
			this_->resetSystem();
		}, //SetCallback
		[](void *value, void* s) {
			CombinedSystemSimulator* this_ = (CombinedSystemSimulator*)s;
			*(int*)value = this_->m_iNumSpheres;
		}, //GetCallback
		this, "");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.001  min=0.001");
	TwAddVarRW(DUC->g_pTweakBar, "Radius", TW_TYPE_FLOAT, &m_fRadius, "step=0.005  min=0.005");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.001  min=0");
	TwAddVarRW(DUC->g_pTweakBar, "ForceScale", TW_TYPE_FLOAT, &m_fForceScaling, "step=0.01   min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.001  min=0");
	TwType TW_TYPE_ACCELERATOR = TwDefineEnumFromString("Accelerator", "None (n^2),Grid (m+n),KdTree (nlogn)");
	TwAddVarRW(DUC->g_pTweakBar, "Accelerator", TW_TYPE_ACCELERATOR, &m_iAccelerator, "");
	TwType TW_TYPE_KERNEL = TwDefineEnumFromString("Kernel", "Constant,Linear,Quadratic,WeakElectric,Electric");
	TwAddVarRW(DUC->g_pTweakBar, "Kernel", TW_TYPE_KERNEL, &m_iKernel, "");
}

void CombinedSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	resetSystem();
}


void CombinedSystemSimulator::externalForcesCalculations(float elapsedTime)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.2f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;

	Mat4 worldInv = Mat4(DUC->g_camera.GetWorldMatrix());
	worldInv = worldInv.inverse();
	Vec3 gravity = worldInv.transformVectorNormal(Vec3(0, -9.81f, 0));
	externalForce = gravity + pullforce;
}

void CombinedSystemSimulator::simulateTimestep(float timeStep)
{
	m_coupledSystem.advance(timeStep, m_fMass, m_fRadius, m_fForceScaling, m_fStiffness,
		m_fDamping, externalForce * m_fMass,
		m_iAccelerator, m_Kernels[m_iKernel]);
	//m_pSphereSystem->AdvanceLeapFrog(timeStep, m_iAccelerator, 10);
}
void CombinedSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	numFrames++;
	if ((numFrames % 50) == 0 || true) {
		std::clog << '.';
	}
	if ((numFrames % 1000) == 0) {
		std::clog << '\n';
	}

	vector<Point> const & points = m_coupledSystem.getPoints();
	Vec3 color = 0.6*Vec3(1, 0.92, 0.8);
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		auto & p = points[i];
		if (i == points.size() - m_coupledSystem.getNumSpringPoints()) {
			color = 0.6 * Vec3(1.0, .2, .2);
		}
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, color);
		DUC->drawSphere(p.pos, {m_fRadius});
	}

	auto springs = m_coupledSystem.getSprings();
	DUC->beginLine();
	for (auto&& s : springs) {
		auto& p1 = points[s.first];
		auto& p2 = points[s.second];
		DUC->drawLine(p1.pos, color, p2.pos, color);
	}
	DUC->endLine();
}


void CombinedSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void CombinedSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}