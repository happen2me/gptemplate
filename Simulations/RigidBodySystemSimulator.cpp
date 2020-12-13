#include "RigidBodySystemSimulator.h"
#include "SimpleMath.h"

static int DEBUG = 0;

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_externalForce = Vec3();

	rigidBoides = vector<RigidBody>();

	// UI Attributes
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "One Step";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	int num_rigidbody = getNumberOfRigidBodies();
	for (int i = 0; i < num_rigidbody; i++) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		// TODO: convert to world transformation
		// BodyA.Obj2WorldMatrix = BodyA.scaleMat * BodyA.rotMat * BodyA.translatMat;
		Mat4 obj2WorldMatrix = composeToWorldMat(rigidBoides[i]);
		DUC->drawRigidBody(obj2WorldMatrix);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	int num = getNumberOfRigidBodies();
	
	for (int i = 0; i < num; i++) {
		RigidBody& body = rigidBoides[i];
		

		Vec3 extForce = body.force;
		Vec3 torque = cross(body.forceLoc, body.force);
		
		// Update linear position		
		body.position += timeStep * body.v;
		// Update linear velocity
		Vec3 linearAcc = extForce / body.mass;
		body.v += linearAcc * timeStep;
		// Update rotation: r <- r + h/2 * (0, w)^T * r
		Quat expandedAngularVelocity(0, body.w.x, body.w.y, body.w.z);
		body.r += (expandedAngularVelocity * body.r) * (timeStep / 2);
		body.r /= body.r.norm(); // re-normalize orientation quaternion
		// Update angular momentum: L <- L + h*q
		body.L += timeStep * torque;
		// Update inversed inertia tensor
		Mat4 rotation = body.r.getRotMat();
		Mat4 rotation_T = Mat4(rotation);
		rotation_T.transpose();
		body.I_inv = rotation * body.I_inv_init * rotation_T;
		// Update angular velocity
		body.w = body.I_inv * body.L;		

		if (DEBUG) {
			cout << "init inertia tensor" << body.I_inv_init << endl;
			cout << "init orientation in quat£º " << body.r << endl;
			cout << "torque£º " << torque << endl;
			cout << "body.r1" << body.r << endl;
			cout << "intertial1 inv" << body.I_inv << endl;
			cout << "angular velocity" << body.w << endl;
		}
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBoides.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidBoides[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidBoides[i].v;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidBoides[i].w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	rigidBoides[i].force = force;
	rigidBoides[i].forceLoc = loc;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidBoides.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidBoides[i].r = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidBoides[i].v = velocity;
}

Mat4 RigidBodySystemSimulator::composeToWorldMat(RigidBody& rigidBody)
{
	Vec3& size = rigidBody.size;
	Vec3& position = rigidBody.position;
	Mat4 scaleMat = Mat4();
	scaleMat.initScaling(size.x, size.y, size.z);
	Mat4 translateMat = Mat4();
	translateMat.initTranslation(position.x, position.y, position.z);
	Mat4 obj2WorldMatrix = scaleMat * rigidBody.r.getRotMat() * translateMat;
	return obj2WorldMatrix;
}


