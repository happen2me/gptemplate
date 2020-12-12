#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

static Mat4 computeInitInertiaTensor(const Vec3& size, const int mass)
{
	float I11 = mass * 1.0 / 12 * (size.y * size.y + size.z * size.z);
	float I22 = mass * 1.0 / 12 * (size.x * size.x + size.z * size.z);
	float I33 = mass * 1.0 / 12 * (size.x * size.x + size.y * size.y);
	Mat4 inertialTensor(I11, 0, 0, 0,
		0, I22, 0, 0,
		0, 0, I33, 0,
		0, 0, 0, 1);
	return inertialTensor;
}

struct RigidBody {
	Vec3 position;
	Vec3 size;
	int mass;
	Vec3 force;
	Vec3 forceLoc;
	Vec3 v; // Linear velovity
	Quat r; //Orientation in quatern
	Vec3 w; //Angular velocity
	const Mat4 I_inv_init; // Initial value of inertia tensor
	Mat4 I_inv; // Inverse of inertia tensor, because this is more useful
	Vec3 L; // Angular momentum
	RigidBody(Vec3 position, Vec3 size, int mass):
		I_inv_init(computeInitInertiaTensor(size, mass).inverse())
	{
		this->position = position;
		this->size = size;
		this->mass = mass;
		force = Vec3();
		forceLoc = Vec3();
		v = Vec3();
		r = Quat();
		w = Vec3();
		I_inv = I_inv_init;
		L = Vec3();
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

public: 
	// Helper functions
	//const static Mat4 computeInitInertiaTensor(const Vec3& size, const int mass);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	vector<RigidBody> rigidBoides;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif