#include "RigidBodySystemSimulator.h"
#include "SimpleMath.h"
#include "collisionDetect.h"

static int DEBUG = 0;

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_externalForce = Vec3();
	m_enableGravity = false;
	rigidBoides = vector<RigidBody>();

	// UI Attributes
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1, Demo 2, Demo 3, Demo4";
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
	switch (testCase)
	{
	case 0:
		simulateDemo1();
		break;
	case 1:
		simulateDemo2();
		break;
	case 2:
		simulateDemo3();
		break;
	case 3:
		simulateDemo4();
		break;
	default:
		simulateDemo1();
		break;
	}

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
		if (body.r.norm() != 0) {
			body.r /= body.r.norm(); // re-normalize orientation quaternion
		}
		// Update angular momentum: L <- L + h*q
		body.L += timeStep * torque;
		// Update inversed inertia tensor
		Mat4 rotation = body.r.getRotMat();
		Mat4 rotation_T = Mat4(rotation);
		rotation_T.transpose();
		body.I_inv = rotation * body.I_inv_init * rotation_T;
		// Update angular velocity
		body.w = body.I_inv * body.L;
		// Clear force
		body.force = Vec3();
		body.forceLoc = Vec3();

		if (DEBUG) {
			cout << "init inertia tensor" << body.I_inv_init << endl;
			cout << "init orientation in quat£º " << body.r << endl;
			cout << "torque£º " << torque << endl;
			cout << "body.r1" << body.r << endl;
			cout << "intertial1 inv" << body.I_inv << endl;
			cout << "angular velocity" << body.w << endl;
		}
	}

	// Collision detection
	for (int i = 0; i < num; i++) {
		for (int j = i + 1; j < num; j++) {
			RigidBody& A = rigidBoides[i];
			RigidBody& B = rigidBoides[j];
			Mat4 worldA = composeToWorldMat(A);
			Mat4 worldB = composeToWorldMat(B);
			CollisionInfo collision = checkCollisionSAT(worldA, worldB);
			// Handle collision event
			if (collision.isValid) {
				bool isSeparating = false;
				float impulse = computeImpulse(A, B, collision.collisionPointWorld, collision.normalWorld, isSeparating);
				if (isSeparating) {
					continue;
				}
				// cout << "Collision detected between " << i << " and " << j << endl;
				Vec3 relColA = collision.collisionPointWorld - A.position;
				Vec3 relColB = collision.collisionPointWorld - B.position;
				A.v += impulse * collision.normalWorld / A.mass;
				B.v -= impulse * collision.normalWorld / B.mass;
				A.L += cross(relColA, impulse * collision.normalWorld);
				B.L -= cross(relColB, impulse * collision.normalWorld);
			}
		}
	}
	// cout << "point0.position: " << rigidBoides[0].position << endl;
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

void RigidBodySystemSimulator::simulateDemo1()
{
	rigidBoides.clear();
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
}

void RigidBodySystemSimulator::simulateDemo2()
{
	rigidBoides.clear();
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
}

void RigidBodySystemSimulator::simulateDemo3()
{
	rigidBoides.clear();
	addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);

	addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
	setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
	setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
	// applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));
}

void RigidBodySystemSimulator::simulateDemo4()
{
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

Vec3 RigidBodySystemSimulator::computeWorldVelocity(RigidBody& rigidBody, Vec3& worldPosition)
{
	Vec3 relativePositionToRigidBody = worldPosition - rigidBody.position;
	Vec3 velocityP = rigidBody.v + cross(rigidBody.w, relativePositionToRigidBody);
	return velocityP;
}

float RigidBodySystemSimulator::computeImpulse(RigidBody& A, RigidBody& B, Vec3& collisionPointWorld, Vec3& collisionNorm, bool& isSeparating)
{
	// Compute velocities at collision point
	Vec3 worldVelocityA = computeWorldVelocity(A, collisionPointWorld);
	Vec3 worldVelocityB = computeWorldVelocity(B, collisionPointWorld);
	Vec3 relativeCollisionPointA = collisionPointWorld - A.position;
	Vec3 relativeCollisionPointB = collisionPointWorld - B.position;
	Vec3 relativeVelocity = worldVelocityA - worldVelocityB;
	Vec3 velocityDotNorm = relativeVelocity * collisionPointWorld;
	float projectedVal = velocityDotNorm[0] + velocityDotNorm[1] + velocityDotNorm[2];
	if (projectedVal > 0) { // two bodies are seperating, do nothing
		isSeparating = true;
		return 0.0f;
	}
	float bounciness = 0; // 0: hard; 1: bouncy
	float molecular = -(1 + bounciness) * projectedVal;
	Vec3 testA = Vec3(1, 2, 3);
	Vec3 testB = Vec3(4, 5, 6);
	Vec3 testMult = testA * testB;
	assert((testMult[0] - 4) < 0.00001);
	assert((testMult[1] - 10) < 0.00001);
	assert((testMult[2] - 18) < 0.00001);
	Vec3 vecCalcInDenominator = (cross(A.I_inv * cross(relativeCollisionPointA, collisionNorm), relativeCollisionPointA) + 
		cross(B.I_inv * cross(relativeCollisionPointB, collisionNorm), relativeCollisionPointB)) * collisionNorm;
	float vecCalcInDenominatorVal = vecCalcInDenominator[0] + vecCalcInDenominator[1] + vecCalcInDenominator[2];
	float denominator = 1 / A.mass + 1 / B.mass + vecCalcInDenominatorVal;
	return molecular / denominator;
}


