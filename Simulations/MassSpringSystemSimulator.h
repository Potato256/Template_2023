#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class Spring {
public:
	Spring(int p1, int p2, float stiff, float initLen): 
		point1(p1), point2(p2), stiffness(stiff), initialLength(initLen), currentLength(0) {}
	int point1;
	int point2;
	float stiffness;
	float initialLength;
	float currentLength;
};

class Point {
public:
	Point(Vec3 p, Vec3 v, float m, bool fix): 
		position(p), velocity(v), force(Vec3(0,0,0)), mass(m), fixed(fix) {}
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	bool fixed;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	const char* getAlgCasesStr();
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
	int addMassPoint(Vec3 position, Vec3 Velocity, float m, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void addSpring(int masspoint1, int masspoint2, float stiff, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void initDemo1();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// Single step simulation
	void AdvanceEuler();
	void AdvanceLeapFrog();
	void AdvanceMidPoint();

	// Drawing Functions
	void drawMassSpring();

private:
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// Attributes
	Vec3 m_vfMovableObjectPos;
	Vec3 m_vfMovableObjectFinalPos;
	Vec3 m_pointScale;
	Vec3 m_lineColor;
	
	std::vector<Point> points;
	std::vector<Spring> springs;
};
#endif