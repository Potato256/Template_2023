#include "RigidBodySystemSimulator.h" 

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "demo1";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "ObjRotation", TW_TYPE_QUAT4F, 
			&ControlledRot, " label='Object rotation' opened=true help='Change the object orientation.' axisz=-z");
		break;
	default:break;
	}
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

	// Set up the lighting
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 0.8, 0.8), 100, 0.6 * Vec3(0.97, 0.86, 1));

	// Draw the rigid body
	for (auto& rb : rigid_boxes) {
		DUC->drawRigidBody(rb.get_objToWorld());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		initDemo1();
		cout << "demo1\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) 
{

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	getControlledRot(0);
	for (auto& rb : rigid_boxes) {
		rb.update_position(timeStep);
		rb.update_rotation(timeStep);
		rb.update_linear_velocity(timeStep);
		rb.update_angular_velocity(timeStep);
		rb.clearForce();
	}
	setControlledRot(0);
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
	return rigid_boxes.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	assert(i < rigid_boxes.size());
	return rigid_boxes[i].center;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	assert(i < rigid_boxes.size());
	return rigid_boxes[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	assert(i < rigid_boxes.size());
	return rigid_boxes[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	assert(i < rigid_boxes.size());
	rigid_boxes[i].apply_force(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBox rb(position, size, mass);
	rigid_boxes.push_back(rb);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	assert(i < rigid_boxes.size());
	rigid_boxes[i].rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	assert(i < rigid_boxes.size());
	rigid_boxes[i].linear_velocity = velocity;
}

void RigidBodySystemSimulator::initDemo1()
{
	rigid_boxes.clear();
	// Add a rigid body
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
	setControlledRot(0);

	simulateTimestep(2.0);
}

void RigidBodySystemSimulator::setControlledRot(int i) {
	Quat& q = rigid_boxes[i].rotation;
	ControlledRot = XMVectorSet(q.x, q.y, q.z, q.w);
}

void RigidBodySystemSimulator::getControlledRot(int i) {
	Quat& q = rigid_boxes[i].rotation;
	q.x = XMVectorGetX(ControlledRot);
	q.y = XMVectorGetY(ControlledRot);
	q.z = XMVectorGetZ(ControlledRot);
	q.w = XMVectorGetW(ControlledRot);
}