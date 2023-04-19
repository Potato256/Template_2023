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
		fixed = false;

		inertia_tensor.initScaling(size.y * size.y + size.z * size.z, size.x * size.x + size.z + size.z, size.x + size.x + size.y + size.y);
		inertia_tensor = inertia_tensor / 12.0f * mass;
	}

	class Force {
	public:
		Force (Vec3 l, Vec3 f) : loc(l), f(f) {}
		Vec3 loc;
		Vec3 f;
	};

	Vec3 center;
	Vec3 size;
	Quat rotation;
	Mat4d inertia_tensor;

	std::vector<Force> forces;
	Vec3 linear_velocity;
	Vec3 angular_velocity;

	float mass;
	bool fixed;
	
	void clearForce() { forces.clear(); }

	Mat4 get_objToWorld() {
		Mat4 trans, rot, scale;
		trans.initTranslation(center.x, center.y, center.z);
		rot = rotation.getRotMat();
		scale.initScaling(size.x, size.y, size.z);
		return scale * rot * trans;
	}

	void apply_force(Vec3 loc, Vec3 force) {
		forces.push_back(Force(loc, force));
	}

	void update_position(float dt) {
		center += linear_velocity * dt;
	}

	void update_rotation(float dt) {
		Quat q = Quat(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0) * rotation;
		rotation = rotation + q * (0.5f * dt);
		rotation.unit();
	}

	void update_linear_velocity(float dt) {
		for (auto& f : forces) {
			linear_velocity += f.f / mass * dt;
		}
	}

	void update_angular_velocity(float dt) {
		Vec3 torque = Vec3(0, 0, 0);
		Mat4d rot = rotation.getRotMat();
		Mat4d rot_T = rot;
		rot_T.transpose();
		for (auto& f : forces) {
			torque += cross(f.loc - center, f.f);
		}
		Mat4 inertia_tensor_real = rot * inertia_tensor * rot_T;
		angular_velocity += dt * inertia_tensor_real.inverse().transformVector(torque);
		//	std::cout << angular_velocity << std::endl;
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
	DirectX::XMVECTOR ControlledRot;
	};
#endif