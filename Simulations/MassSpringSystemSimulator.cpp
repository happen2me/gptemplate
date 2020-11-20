#include "MassSpringSystemSimulator.h"

float sphereSize = 0.05f;
Vec3 lineColor = Vec3(0, 0, 0);

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_fMass = 0;
    m_fDamping = 0;
    m_fStiffness = 0;
    m_iIntegrator = EULER;

    m_springs = vector<Spring>();
    m_points = vector<MassPoint>();

    m_externalForce = Vec3();
    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return nullptr;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    int mp_num = getNumberOfMassPoints();
    int spring_num = getNumberOfSprings();
    for (int i = 0; i < mp_num; i++) {
        DUC->drawSphere(getPositionOfMassPoint(i), Vec3(sphereSize, sphereSize, sphereSize));
    }
    for (int i = 0; i < spring_num; i++) {
        DUC->drawLine(getPositionOfMassPoint(m_springs[i].mp1), lineColor, getPositionOfMassPoint(m_springs[i].mp2), lineColor);
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{  
    int numMassPoints = getNumberOfMassPoints();
    switch (m_iIntegrator)
    {
    case EULER: 
        for (int i = 0; i < numMassPoints; i++)
        {
            MassPoint& p = m_points[i];
            p.position += p.velocity * timeStep;
        }
        for each (const Spring &s in m_springs)
        {
            MassPoint &p1 = m_points[s.mp1];
            MassPoint &p2 = m_points[s.mp2];
            float curLen = sqrt(p1.position.squaredDistanceTo(p2.position));
            p1.intForce += -m_fStiffness * (curLen - s.length) * normalize(p1.position - p2.position);
            p2.intForce += -m_fStiffness * (curLen - s.length) * normalize(p2.position - p1.position);
        }
        for(int i = 0; i < numMassPoints; i++){
            MassPoint& p = m_points[i];
            Vec3 extAcc = m_externalForce / (m_fMass * numMassPoints);
            Vec3 dampAcc = -m_fDamping * p.velocity / m_fMass;
            Vec3 acc = p.intForce / m_fMass + extAcc + dampAcc;
            p.velocity += acc * timeStep;
            p.intForce = Vec3(0, 0, 0);
        }
        break;
    case MIDPOINT:
    { //bracket for defining local vars
        vector<Vec3> midPositions = vector<Vec3>();
        vector<Vec3> midVelocoties = vector<Vec3>(numMassPoints);
        vector<Vec3> initForces = vector<Vec3>(numMassPoints);
        vector<Vec3> initAccs = vector<Vec3>(numMassPoints);

        // calculate position at midpoint using init velocity
        for (int i = 0; i < numMassPoints; i++) {
            MassPoint& p = m_points[i];
            // Middle position
            Vec3 midPos = p.position + (timeStep / 2) * p.velocity;
            midPositions.push_back(midPos);
        }

        // calculate initial forces
        for each (const Spring & s in m_springs)
        {
            MassPoint& p1 = m_points[s.mp1];
            MassPoint& p2 = m_points[s.mp2];
            float curLen = sqrt(p1.position.squaredDistanceTo(p2.position));
            initForces[s.mp1] += -m_fStiffness * (curLen - s.length) * normalize(p1.position - p2.position);
            initForces[s.mp2] += -m_fStiffness * (curLen - s.length) * normalize(p2.position - p1.position);
        }

        // calculate initial acceleration
        for (int i = 0; i < numMassPoints; i++) {
            MassPoint& p = m_points[i];
            Vec3 extAcc = m_externalForce / (m_fMass * numMassPoints);
            Vec3 dampAcc = -m_fDamping * p.velocity / m_fMass;
            initAccs[i] = initForces[i] / m_fMass + extAcc + dampAcc;
        }
        
        // calculate velocity at midpoint using initial acc
        for (int i = 0; i < numMassPoints; i++) {
            const MassPoint& p = m_points[i];
            midVelocoties[i] = p.velocity + initAccs[i] * (timeStep / 2);
        }
        // Compute force at t+h/2 based on xtmp
        for each (const Spring & s in m_springs)
        {
            Vec3& pos1 = midPositions[s.mp1];
            Vec3& pos2 = midPositions[s.mp2];
            MassPoint& point1 = m_points[s.mp1];
            MassPoint& point2 = m_points[s.mp2];
            float curLen = sqrt(pos1.squaredDistanceTo(pos2));
            point1.intForce += -m_fStiffness * (curLen - s.length) * normalize(pos1 - pos2);
            point2.intForce += -m_fStiffness * (curLen - s.length) * normalize(pos2 - pos1);
        }

        //Update position and velocity
        for (int i = 0; i < numMassPoints; i++) 
        {
            MassPoint& p = m_points[i];
            p.position += midVelocoties[i] * timeStep;

            Vec3 extAcc = m_externalForce / (m_fMass * numMassPoints);
            Vec3 dampAcc = -m_fDamping * midVelocoties[i] / m_fMass;
            Vec3 midAcc = p.intForce / m_fMass + extAcc + dampAcc;
            p.velocity += midAcc * timeStep;
            p.intForce = Vec3(0, 0, 0);
        }
        
        break;
    }
    default:
        break;
    }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    m_points.push_back(MassPoint(position, Velocity, isFixed));
    return m_points.size()-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    m_springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return m_points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return m_points[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
    m_externalForce = force;
}

Vec3 MassSpringSystemSimulator::normalize(Vec3 v)
{
    float length = norm(v);
    return v/length;
}

void MassSpringSystemSimulator::simulateSimpleEuler()
{
    m_points.clear();
    m_springs.clear();
    setIntegrator(EULER);

    setMass(10.0f);
    setDampingFactor(0.0f);
    setStiffness(40.0f);
    applyExternalForce(Vec3(0, 0, 0));
    
    int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
    int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
    addSpring(p0, p1, 1.0);
}
