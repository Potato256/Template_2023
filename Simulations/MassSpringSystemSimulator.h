#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>


// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
#define IMPLICIT_EULER 3
// Do Not Change

typedef Eigen::Triplet<float>             Tripletf;
typedef Eigen::Vector3f                      Vec3f;
typedef Eigen::VectorXf                      VecXf;
typedef Eigen::Matrix3f                      Mat3f;
typedef Eigen::SparseMatrix<float>  SpMatf;

class Point {
public:
	Point(Vec3 p, Vec3 v, float m, bool fix): 
		position(p), velocity(v), force(Vec3(0,0,0)), mass(m), fixed(fix) {}
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	float mass;
	bool fixed;

	void clearForce() { force = Vec3(0, 0, 0); }
	void addGravity(float g) { force += g * mass * Vec3(0,-1,0); }
	void integratePosition(float timeStep) { position += fixed? Vec3(0,0,0) : timeStep * velocity; };
	void integrateVelocity(float timeStep) { velocity += timeStep * force / mass; };
};

class Spring {
public:
	Spring(int p1, int p2, float stiff, float initLen) :
		point1(p1), point2(p2), stiffness(stiff), initialLength(initLen), currentLength(0), f(Vec3(0, 0, 0)) {}
	int point1;
	int point2;
	float stiffness;
	float initialLength;
private:
	float currentLength;
	// the direction of force is p2 - p1
	Vec3 f;

public:
	void computeElasticForces(std::vector<Point>& points) {
	    f = points[point2].position - points[point1].position;
		currentLength = std::sqrt(f[0]*f[0]+ f[1] * f[1]+ f[2] * f[2]);
		f = (initialLength - currentLength) * stiffness * f / currentLength;
	};
	void addToEndPoints(std::vector<Point>& points) {
		points[point1].force -= f;
		points[point2].force += f;
	};
	void compute_hessian_matrix(std::vector<Point>& points, Mat3f& hessian) {
		Vec3 tmp = points[point2].position - points[point1].position;
		Eigen::Matrix<float, 3, 1> x_ij = Vec3f(tmp.x, tmp.y, tmp.z).normalized();
		Mat3f M = x_ij * x_ij.transpose();
		Mat3f I{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
		hessian = (1 - initialLength / currentLength) * (I - M) + M;
		hessian = -stiffness * hessian;
	}
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
	void initDemo1();
	void initDemo2();
	void initDemo3();

	void applyExternalForce(Vec3 force);
	void integratePositions(float timeStep);
	void integrateVelocity(float timeStep);
	void integratePositions(float timeStep, std::vector<Point>& ps);
	void integrateVelocity(float timeStep, std::vector<Point>& ps);
	void checkBoundary();

	// Single step simulation
	void AdvanceEuler(float timeStep);
	void AdvanceLeapFrog(float timeStep);
	void AdvanceMidPoint(float timeStep);
	void AdvanceImplicitEuler(float timeStep);

	void add_matrix_block(const Mat3f& mat, std::vector<Tripletf>* mat_eles, int row, int col);
	bool CG(const SpMatf& A, VecXf& x, const VecXf& b);

	// Interfaces
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	void setGravity(float g);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	int addMassPoint(Vec3 position, Vec3 Velocity, float m, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void addSpring(int masspoint1, int masspoint2, float stiff, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void printPositionOfMassPoint(int index);
	void printVelocityOfMassPoint(int index);
	void printSpring(int index);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// Rendering
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
	float m_fGravity;
	int m_iIntegrator;
	bool floor_boundary;

	// Attributes
	Vec3 m_vfMovableObjectPos;
	Vec3 m_vfMovableObjectFinalPos;
	Vec3 m_pointScale;
	Vec3 m_lineColor;
	float u;
	float v;
	
	std::vector<Point> points;
	std::vector<Spring> springs;
};
#endif