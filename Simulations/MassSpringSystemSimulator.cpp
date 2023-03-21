#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Initialization
	m_iTestCase = 0;
	setIntegrator(EULER);
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_pointScale = Vec3(0.02f, 0.02f, 0.02f); 
	m_lineColor = Vec3(1.0f, 0.1f, 0.6f);
	points = std::vector<Point>();
	springs = std::vector<Spring>();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "demo1,demo4";
}

const char* MassSpringSystemSimulator::getAlgCasesStr() {
	return "Explicit_Eular,Leap_Frog,Mid_Point";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Algorithm", getAlgCasesStr());
	TwAddVarRW(DUC->g_pTweakBar, "Algorithm", TW_TYPE_TESTCASE, &m_iIntegrator, "");

	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
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
	default:break;
	}

}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawMassSpring();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		initDemo1();
		m_vfMovableObjectPos = Vec3(0, 0, 0);
		cout << "demo1\n";
		break;
	case 1:
		cout << "demo4\n";
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
	switch (m_iIntegrator)
	{
	case EULER:
		AdvanceEuler(timeStep);
		break;
	case LEAPFROG:
		AdvanceLeapFrog(timeStep);
		break;
	case MIDPOINT:
		AdvanceMidPoint(timeStep);
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::integratePositions(float timeStep)
{
	for (auto& p : points)
		p.integratePosition(timeStep);
}

void MassSpringSystemSimulator::integrateVelocity(float timeStep)
{
	for (auto& p : points)
		p.integrateVelocity(timeStep);
}

void MassSpringSystemSimulator::integratePositions(float timeStep, std::vector<Point>& ps)
{
	for (auto& p : ps)
		p.integratePosition(timeStep);
}

void MassSpringSystemSimulator::integrateVelocity(float timeStep, std::vector<Point>& ps)
{
	for (auto& p : ps)
		p.integrateVelocity(timeStep);
}

void MassSpringSystemSimulator::AdvanceEuler(float timeStep)
{
	for (auto& p : points)
	{
		p.clearForce();
		p.addGravity(m_externalForce);
	}
	for (auto& s : springs)
	{
		s.computeElasticForces(points);
		s.addToEndPoints(points);
	}

	integratePositions(timeStep);
	integrateVelocity(timeStep);
}

void MassSpringSystemSimulator::AdvanceLeapFrog(float timeStep)
{

}

void MassSpringSystemSimulator::AdvanceMidPoint(float timeStep)
{
	std::vector<Point> midPoints(points);

	for (auto& p : midPoints)
	{
		p.clearForce();
		p.addGravity(m_externalForce);
	}

	for (auto& s : springs)
	{
		s.computeElasticForces(midPoints);
		s.addToEndPoints(midPoints);
	}

	integratePositions(0.5 * timeStep, midPoints);
	integrateVelocity(0.5 * timeStep, midPoints);

	for (auto& p : points)
	{
		p.clearForce();
		p.addGravity(m_externalForce);
	}

	for (auto& s : springs)
	{
		s.computeElasticForces(midPoints);
		s.addToEndPoints(points);
	}

	int size = getNumberOfMassPoints();
	for (int i = 0; i < size; ++i)
		points[i].position += timeStep * midPoints[i].velocity;

	integrateVelocity(timeStep);
}


void MassSpringSystemSimulator::initDemo1()
{
	points.clear();
	springs.clear();
	setMass(10.f);
	setStiffness(40.f);
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), 0);
	int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), 0);
	addSpring(p0, p1, 1.f);
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	points.push_back(Point(position, Velocity, m_fMass, isFixed));
	return points.size() - 1;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, float m, bool isFixed) {
	points.push_back(Point(position, Velocity, m, isFixed));
	return points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springs.push_back(Spring(masspoint1, masspoint2, m_fStiffness, initialLength));
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float stiff, float initialLength)
{
	springs.push_back(Spring(masspoint1, masspoint2, stiff, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return points.size();
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	assert(index < points.size());
	return points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	assert(index < points.size());
	return points[index].velocity;
}

void MassSpringSystemSimulator::printPositionOfMassPoint(int index)
{
	assert(index < points.size());
	std::cout<< points[index].position << std::endl;
}

void MassSpringSystemSimulator::printVelocityOfMassPoint(int index)
{
	assert(index < points.size());
	std::cout << points[index].velocity << std::endl;
}

void MassSpringSystemSimulator::printSpring(int index)
{
	assert(index < springs.size());
	std::cout << "p1 : " << springs[index].point1 << "    p2 : " << springs[index].point2 
		<< "    l : " << springs[index].initialLength << "    k : " << springs[index].stiffness << std::endl;
}

void MassSpringSystemSimulator::newline()
{
	std::cout << std::endl;
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}


