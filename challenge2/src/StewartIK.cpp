#include "StewartIK.h"

StewartIK::StewartIK(
    float bR,
    float pR,
    float bA,
    float pA,
    float basePlatformDist,
    float maxExtend) : m_baseRadius(bR), m_platformRadius(pR), m_baseAngle(bA), m_platformAngle(pA)
{
    float c1 = EIGEN_PI / 3;
    float c2 = 2 * c1;

    m_basePlatformVec.resize(3);
    m_basePlatformVec << 0, 0, basePlatformDist;

    // base anchor angles - anticlockwise - in order
    m_baseAnchorAngles.resize(NUM_JOINTS);
    m_baseAnchorAngles << -m_baseAngle,
                            m_baseAngle,
                            c2 - m_baseAngle,
                            c2 + m_baseAngle,
                            (-c2 - m_baseAngle),
                            (-c2 + m_baseAngle);

    // platform anchor angles - anticlockwise - in order
    m_platformAnchorAngles.resize(NUM_JOINTS);
    m_platformAnchorAngles << c1 + (-c2 + m_platformAngle),
                                c1 + -m_platformAngle,
                                c1 + m_platformAngle,
                                c1 + c2 - m_platformAngle,
                                c1 + c2 + m_platformAngle,
                                c1 + (-c2 - m_platformAngle);

    // x, y, z coords of base anchor points
    m_baseAnchorCoords.resize(NUM_JOINTS, 3);
    m_baseAnchorCoords << cos(m_baseAnchorAngles(0)), sin(m_baseAnchorAngles(0)), 0,
                            cos(m_baseAnchorAngles(1)), sin(m_baseAnchorAngles(1)), 0,
                            cos(m_baseAnchorAngles(2)), sin(m_baseAnchorAngles(2)), 0,
                            cos(m_baseAnchorAngles(3)), sin(m_baseAnchorAngles(3)), 0,
                            cos(m_baseAnchorAngles(4)), sin(m_baseAnchorAngles(4)), 0,
                            cos(m_baseAnchorAngles(5)), sin(m_baseAnchorAngles(5)), 0;

    m_baseAnchorCoords *= m_baseRadius;
    m_baseAnchorCoords.transposeInPlace(); // (6 x 3) -> (3 x 6)

    // x, y, z coords of platform anchor points
    m_platformAnchorCoords.resize(NUM_JOINTS, 3);
    m_platformAnchorCoords << cos(m_platformAnchorAngles(0)), sin(m_platformAnchorAngles(0)), 0,
                                cos(m_platformAnchorAngles(1)), sin(m_platformAnchorAngles(1)), 0,
                                cos(m_platformAnchorAngles(2)), sin(m_platformAnchorAngles(2)), 0,
                                cos(m_platformAnchorAngles(3)), sin(m_platformAnchorAngles(3)), 0,
                                cos(m_platformAnchorAngles(4)), sin(m_platformAnchorAngles(4)), 0,
                                cos(m_platformAnchorAngles(5)), sin(m_platformAnchorAngles(5)), 0;

    m_platformAnchorCoords *= m_platformRadius;
    m_platformAnchorCoords.transposeInPlace(); // (6 x 3) -> (3 x 6)

    MatrixXf initLegVecs = (m_platformAnchorCoords - m_baseAnchorCoords).colwise() + m_basePlatformVec;
    m_currLegLengths = initLegVecs.colwise().norm();
    std::cout << m_currLegLengths << std::endl;
}

int main()
{
    StewartIK ik = StewartIK(0.5, 0.2, 0.11, 0.11, 0.2, 0.3);
    return 0;
}