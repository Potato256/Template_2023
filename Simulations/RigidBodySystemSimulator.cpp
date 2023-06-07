#include "RigidBodySystemSimulator.h" 
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
	gravity = Vec3(0, 0, 0);
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "demo1,demo2,demo3";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "ObjRotation", TW_TYPE_QUAT4F,
		&ControlledRot, " label='Object rotation' opened=true help='Change the object orientation.' axisz=-z");

	switch (m_iTestCase)
	{
	case 0:
	
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
	case 1:
		initDemo2();
		cout << "demo2\n";
		break;
	case 2:
		initDemo3();
		cout << "demo3\n";
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) 
{
	if (m_iTestCase < 1) {
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
			float inputScale = 20.f;
			inputWorld = inputWorld * inputScale * timeElapsed;
			//	std::cout << inputWorld << std::endl;
			applyForceOnBody(0, rigid_boxes[0].center + 0.5 * rigid_boxes[0].size, inputWorld);
		}
	}
	for (int i = 0; i < rigid_boxes.size(); ++i)
		applyForceOnBody(i, rigid_boxes[i].center, gravity * rigid_boxes[i].mass);
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	getControlledRot(0);
	for (auto& rb : rigid_boxes) {
		if (rb.fixed) continue;
		rb.update_position(timeStep);
		rb.update_rotation(timeStep);
		rb.update_linear_velocity(timeStep);
		rb.update_angular_velocity(timeStep);
		rb.clearForce();
	}
	detectCollision();
	setControlledRot(0);
}

void RigidBodySystemSimulator::detectCollision() 
{
	for (int i = 0; i < rigid_boxes.size(); i++) {
		for (int j = i + 1; j < rigid_boxes.size(); j++) {
			CollisionInfo info = checkCollisionSAT(rigid_boxes[i].get_objToWorld(),
				rigid_boxes[j].get_objToWorld());
			if (!info.isValid)
				continue;
			Vec3 v_rel = rigid_boxes[i].linear_velocity - rigid_boxes[j].linear_velocity;
			if (dot(v_rel, info.normalWorld) > 0)
				continue;
			float c = 1;
			float J = -(1 + c) * dot(v_rel, info.normalWorld) /
				(1 / rigid_boxes[i].mass + 1 / rigid_boxes[j].mass + dot(
					cross(rigid_boxes[i].inertia_tensor.inverse().transformVector(cross(info.collisionPointWorld - rigid_boxes[i].center, info.normalWorld)), info.collisionPointWorld - rigid_boxes[i].center) +
					cross(rigid_boxes[j].inertia_tensor.inverse().transformVector(cross(info.collisionPointWorld - rigid_boxes[j].center, info.normalWorld)), info.collisionPointWorld - rigid_boxes[j].center),
					info.normalWorld));
			if (!rigid_boxes[i].fixed)
			{
				rigid_boxes[i].linear_velocity += J * info.normalWorld / rigid_boxes[i].mass;
				rigid_boxes[i].angular_velocity += cross(info.collisionPointWorld - rigid_boxes[i].center, J * info.normalWorld);
			}
			if (!rigid_boxes[j].fixed)
			{
				rigid_boxes[j].linear_velocity -= J * info.normalWorld / rigid_boxes[j].mass;
				rigid_boxes[j].angular_velocity -= cross(info.collisionPointWorld - rigid_boxes[j].center, J * info.normalWorld);
			}
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
	gravity = Vec3(0, 0, 0);
	// Add a rigid body
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
	setControlledRot(0);

	simulateTimestep(2.0);
}
void RigidBodySystemSimulator::initDemo2()
{
	rigid_boxes.clear();
	gravity = Vec3(0, 0, 0);
	// Add a rigid body
	addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
	addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
	setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
	setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
	setControlledRot(0);

}
void RigidBodySystemSimulator::initDemo3()
{
	rigid_boxes.clear();
	gravity = Vec3(0, -0.3, 0);
	// Add a rigid body
	addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.2f, 0.1f, 0.1f), 100.0);
	setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
	addRigidBody(Vec3(0.0f, 0.4f, 0.0f), Vec3(0.1f, 0.1f, 0.2f), 100.0);
	addRigidBody(Vec3(0.0f, 0.6f, 0.0f), Vec3(0.1f, 0.1f, 0.2f), 100.0);
	setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
	// floor
	addRigidBody(Vec3(0.0f, -1.0f, 0.0f), Vec3(2.0f, 0.1f, 2.0f), 100.0f);
	rigid_boxes[4].fixed = true;

	setControlledRot(0);

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

