#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Initialization
	m_iTestCase = 1;
	setIntegrator(MIDPOINT);
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_pointScale = Vec3(0.02f, 0.02f, 0.02f); 
	m_lineColor = Vec3(1.0f, 0.1f, 0.6f);
	points = std::vector<Point>();
	springs = std::vector<Spring>();
	setGravity(0);
	floor_boundary = 0;
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
		initDemo4();
		m_vfMovableObjectPos = Vec3(0, 0, 0);
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

void MassSpringSystemSimulator::checkBoundary()
{
	if (floor_boundary)
		for (auto& p : points)
			p.position[1] = p.position[1] < -1 ? -1 : p.position[1];
}

void MassSpringSystemSimulator::AdvanceEuler(float timeStep)
{
	for (auto& p : points)
	{
		p.clearForce();
		p.addGravity(m_fGravity);
	}
	for (auto& s : springs)
	{
		s.computeElasticForces(points);
		s.addToEndPoints(points);
	}

	integratePositions(timeStep);
	integrateVelocity(timeStep);
	checkBoundary();
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
		p.addGravity(m_fGravity);
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
		p.addGravity(m_fGravity);
	}

	for (auto& s : springs)
	{
		s.computeElasticForces(midPoints);
		s.addToEndPoints(points);
	}

	int size = getNumberOfMassPoints();
	for (int i = 0; i < size; ++i)
		points[i].position += (!points[i].fixed) * timeStep * midPoints[i].velocity;

	integrateVelocity(timeStep);
	checkBoundary();
}

void MassSpringSystemSimulator::initDemo1()
{
	points.clear();
	springs.clear();
	setMass(10.f);
	setStiffness(60.f);
	setGravity(0);
	floor_boundary = 0;
	applyExternalForce(Vec3(0, 0, 0));

	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), 0);
	int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), 0);
	addSpring(p0, p1, 1.f);
}

void MassSpringSystemSimulator::initDemo4()
{
	points.clear();
	springs.clear();
	setGravity(0.2f); 
	applyExternalForce(Vec3(0, 0, 0));
	floor_boundary = 1;

	int pCnt = 0;
	int p[20];

	setMass(10.f);
	p[pCnt++] = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), 1);


	setMass(5.f);
	// upper face
	p[pCnt++] = addMassPoint(Vec3(0, 0.8, 0), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0, 0.8, 0.3), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.3, 0.8, 0.3), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.3, 0.8, 0.0), Vec3(0, 0, 0), 0);
	// lower face
	p[pCnt++]= addMassPoint(Vec3(0.0, 0.5, 0.0), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.0, 0.5, 0.3), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.3, 0.5, 0.3), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.3, 0.5, 0.0), Vec3(0, 0, 0), 0);


	p[pCnt++] = addMassPoint(Vec3(0.3, -0.6, 0.0), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.0, -0.6, 0.3), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0.2, -0.6, 0.2), Vec3(0, 0, 0), 0);
	p[pCnt++] = addMassPoint(Vec3(0., -1, 0.), Vec3(0, 0, 0), 0);

	setStiffness(200.f);
	addSpring(p[0], p[1], 0.2);

	setStiffness(100.f);
	// upper face
	addSpring(p[1], p[2], 0.3);
	addSpring(p[2], p[3], 0.3);
	addSpring(p[3], p[4], 0.3);
	addSpring(p[4], p[1], 0.3);
	addSpring(p[1], p[3], 0.4243);
	addSpring(p[2], p[4], 0.4243);

	// middle
	addSpring(p[1], p[5], 0.3);
	addSpring(p[2], p[6], 0.3);
	addSpring(p[3], p[7], 0.3);
	addSpring(p[4], p[8], 0.3);

	addSpring(p[1], p[6], 0.4243);
	addSpring(p[2], p[7], 0.4243);
	addSpring(p[3], p[8], 0.4243);
	addSpring(p[4], p[5], 0.4243);

	addSpring(p[1], p[8], 0.4243);
	addSpring(p[2], p[5], 0.4243);
	addSpring(p[3], p[6], 0.4243);
	addSpring(p[4], p[7], 0.4243);

	//lower face
	addSpring(p[5], p[6], 0.3);
	addSpring(p[6], p[7], 0.3);
	addSpring(p[7], p[8], 0.3);
	addSpring(p[8], p[5], 0.3);
	addSpring(p[5], p[7], 0.4243);
	addSpring(p[6], p[8], 0.4243);

	setStiffness(80.f);
	//addSpring(p[7], p[9], 0.3);

	setStiffness(100.f);
	addSpring(p[9], p[10], 0.4);
	addSpring(p[10], p[11], 0.4);
	addSpring(p[11], p[9], 0.4);
	addSpring(p[9], p[12], 0.4);
	addSpring(p[10], p[12], 0.4);
	addSpring(p[11], p[12], 0.4);

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

void MassSpringSystemSimulator::setGravity(float g)
{
	m_fGravity = g;
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


