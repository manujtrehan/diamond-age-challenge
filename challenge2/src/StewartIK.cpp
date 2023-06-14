#include "StewartIK.h"

StewartIK::StewartIK(
    float bR,
    float pR,
    float bA,
    float pA,
    float basePlatformDist,
    float maxExtend) : m_baseRadius(bR), m_platformRadius(pR), m_baseAngle(bA),
                       m_platformAngle(pA), m_minLegLength(-1), m_maxLegLength(-1)
{
    float c1 = EIGEN_PI / 3;
    float c2 = 2 * c1;

    // set the default base to platform vector (only z separation)
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

    // platform anchor angles - anticlockwise - in order (platform rotated pi/3 anticlockwise wrt base in home position)
    m_platformAnchorAngles.resize(NUM_JOINTS);
    m_platformAnchorAngles << c1 + (-c2 + m_platformAngle),
                                c1 + -m_platformAngle,
                                c1 + m_platformAngle,
                                c1 + c2 - m_platformAngle,
                                c1 + c2 + m_platformAngle,
                                c1 + (-c2 - m_platformAngle);

    // set x, y, z coords of base anchor points
    setAnchorCoords(m_baseAnchorCoords, m_baseAnchorAngles, m_baseRadius);

    // set x, y, z coords of platform anchor points
    setAnchorCoords(m_platformAnchorCoords, m_platformAnchorAngles, m_platformRadius);

    // init inverse kinematics
    updateLengthsIK(0, 0, 0, 0, 0, 0);

    // set the max valid leg length based on the extension
    setMinMaxLength(maxExtend);
}

void StewartIK::updateLengthsIK(
    float x,
    float y,
    float z,
    float rollAngle,
    float pitchAngle,
    float yawAngle)
{
    VectorXf desiredTrans(3);
    desiredTrans << x, y, z;

    AngleAxisf roll(rollAngle, Vector3f::UnitZ());
    AngleAxisf yaw(yawAngle, Vector3f::UnitY());
    AngleAxisf pitch(pitchAngle, Vector3f::UnitX());
    Quaternionf quat = roll * yaw * pitch;

    // l = T + H + R.P - B
    MatrixXf legVecs = (quat.toRotationMatrix() * m_platformAnchorCoords) - m_baseAnchorCoords;
    legVecs = legVecs.colwise() + (desiredTrans + m_basePlatformVec);
    m_currLegLengths = legVecs.colwise().norm();

    // check for invalid lengths (less than min, or greater than max)
    // skip the check on initialization - minLegLength initialized to -1
    if(m_minLegLength > 0)
    {
        bool b1 = (m_currLegLengths.array() < m_minLegLength).any();
        bool b2 = (m_currLegLengths.array() > m_maxLegLength).any();
        if(b1 or b2)
            throw std::invalid_argument("Invalid input pose!");
    }
}

void StewartIK::setAnchorCoords(
    MatrixXf& anchorCoords,
    VectorXf& anchorAngles,
    float radius)
{
    anchorCoords.resize(NUM_JOINTS, 3);
    anchorCoords << anchorAngles.array().cos(),
                    anchorAngles.array().sin(),
                    VectorXf::Zero(NUM_JOINTS);
    anchorCoords *= radius;
    anchorCoords.transposeInPlace(); // (6 x 3) -> (3 x 6)
}

void StewartIK::setMinMaxLength(float maxExtend)
{
    m_minLegLength = m_currLegLengths(0);
    m_maxLegLength = m_minLegLength + maxExtend;
}

VectorXf StewartIK::getLengths() const { return m_currLegLengths; }

int main()
{
    /*
    * argument order
    * (base radius, platform radius, base anchor / 2, platform anchor / 2, base-platform distance, max extension)
    */
    StewartIK ik = StewartIK(1, 1, 0.1309, 0.1309, 1, 0.5);
    std::cout << "Init lengths:" << std::endl;
    std::cout << ik.getLengths() << std::endl;

    ik.updateLengthsIK(0, 0, 0.2, 0, 0, 0.1);
    std::cout << "Updated lengths:" << std::endl;
    std::cout << ik.getLengths() << std::endl;

    return 0;
}