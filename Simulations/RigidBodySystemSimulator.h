#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
// add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBox {
public:
	RigidBox(Vec3 c, Vec3 s, float m) : center(c), size(s), mass(m) {
		rotation = Quat(0, 0, 0, 1);
		linear_velocity = Vec3(0, 0, 0);
		angular_velocity = Vec3(0, 0, 0);
		force = Vec3(0, 0, 0);
		fixed = false;
	}

	Vec3 center;
	Vec3 size;
	Quat rotation;

	Vec3 force;
	Vec3 linear_velocity;
	Vec3 angular_velocity;

	float mass;
	bool fixed;
	void clearForce() { force = Vec3(0, 0, 0); }

	Mat4 get_objToWorld() {
		Mat4 objToWorld, tmp;
		objToWorld.initId();
		tmp.initScaling(size.x, size.y, size.z);
		objToWorld = objToWorld * tmp;
		objToWorld = objToWorld * rotation.getRotMat();
		tmp.initTranslation(center.x, center.y, center.z);
		objToWorld = objToWorld * tmp;
		return objToWorld;

	}


};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void setControlledRot(int i);
	void getControlledRot(int i);

	void initDemo1();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;


	std::vector<RigidBox> rigid_boxes;
	float angle;
	DirectX::XMVECTOR ControlledRot;
	};
#endif