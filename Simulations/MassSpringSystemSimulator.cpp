#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Initialization
	m_iTestCase = 0; 
	switch (m_iTestCase)
	{
	case 0:
		initDemo1(); 
		break;
	case 1:break;
	case 2:break;
	default:break;
	}

	m_iAlgCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_pointScale = Vec3(0.02f, 0.02f, 0.02f); 
	m_lineColor = Vec3(1.0f, 0.1f, 0.6f);
	points = std::vector<Point>();
	springs= std::vector<Spring>();

}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "demo1,demo2,demo3,demo4";
}

const char* MassSpringSystemSimulator::getAlgCasesStr() {
	return "Explicit_Eular,Leap_Frog,Mid_Point";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Algorithm", getAlgCasesStr());
	TwAddVarRW(DUC->g_pTweakBar, "Algorithm", TW_TYPE_TESTCASE, &m_iAlgCase, "");

	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	switch (m_iTestCase)
	{
	case 0:
		initDemo1();
		break;
	case 1:break;
	case 2:break;
	default:break;
	}

}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
		drawMassSpring();
		break;
	case 1: break;
	case 2: break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "demo1\n";
		break;
	case 1:
		cout << "1!\n";
		break;
	case 2:
		cout << "2 !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		break;
	default:
		break;
	}
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

void MassSpringSystemSimulator::initDemo1()
{
	points.clear();
	springs.clear();
	// point 0
	points.push_back(Point(Vec3(0, 0, 0), Vec3(-1, 0, 0), 10));
	// point 1
	points.push_back(Point(Vec3(0, 2, 0), Vec3(1, 0, 0), 10));
	// spring 0
	springs.push_back(Spring(0, 1, 40, 1));
}

void MassSpringSystemSimulator::drawMassSpring()
{
	// draw points with spheres
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 0.8, 0.8), 100, 0.6 * Vec3(0.97, 0.86, 1));

	for (auto& p : points)
		DUC->drawSphere(p.position + m_vfMovableObjectPos, m_pointScale);
	DUC->beginLine();
	for (auto& s : springs)
		DUC->drawLine(points[s.point1].position + m_vfMovableObjectPos,
			m_lineColor,
			points[s.point2].position + m_vfMovableObjectPos,
			m_lineColor
		);
	DUC->endLine();

}

