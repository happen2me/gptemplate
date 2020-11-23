#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include <cmath>
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
		this->position = position;
		this->velocity = velocity;
		this->isFixed = isFixed;
		this->intForce = Vec3(0, 0, 0);
	};
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
	Vec3 intForce;
};
struct Spring {
	Spring(int mp1, int mp2, float length) {
		this->mp1 = mp1;
		this->mp2 = mp2;
		this->length = length;
	};
	int mp1;
	int mp2;
	float length;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	const char * getIntergratorsStr();

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void applyGravity(bool on);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	static Vec3 normalize(Vec3 v);
	void basicTest(int integrator);
	void simulateTwoPoint();
	void simulateComlexSystem();

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	int m_iTestCase;
	bool m_bApplyGravity;
	vector<MassPoint> m_points;
	vector<Spring> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif