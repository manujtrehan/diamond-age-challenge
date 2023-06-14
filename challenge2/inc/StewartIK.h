#ifndef _StewartIK_
#define _StewartIK_

#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

#define NUM_JOINTS 6

using namespace Eigen;

class StewartIK
{
private:
    // base and platform radii (m)
    float m_baseRadius;
    float m_platformRadius;
    // half of the anchor angles (rad)
    float m_baseAngle;
    float m_platformAngle;

    // Actuator limits
    // actuator length with 0 extension (m)
    float m_minLegLength;
    // actuator length with max extension (m)
    float m_maxLegLength;

    // joint angles in order for the base and platform anchors (rad)
    VectorXf m_baseAnchorAngles;
    VectorXf m_platformAnchorAngles;
    // vector from the base to the platform at the home position (0, 0, z) (m)
    VectorXf m_basePlatformVec;
    // current leg lengths (m)
    VectorXf m_currLegLengths;

    // x, y, z anchor coordinates for the base and platform anchors wrt their origins (m)
    MatrixXf m_baseAnchorCoords;
    MatrixXf m_platformAnchorCoords;

public:
    /*
     * Constructor
     * @brief Initializes the StewartIK class with the required angles, lengths and anchor coordinates
     * also sets the minLegLength based on the input base platform distance
     * @param bR base radius (m)
     * @param pR platform radius (m)
     * @param bA half of the base anchor angle (rad)
     * @param pA half of the platform anchor angle (rad)
     * @param basePlatformDist distance (z) between the base and platform at the home position (m)
     * @param maxExtend maximum extension of the actuators (m)
     */
    StewartIK(
        float bR,
        float pR,
        float bA,
        float pA,
        float basePlatformDist,
        float maxExtend);

    /*
     * IK solver
     * @brief Calculates acutator lengths to reach the input 6 DOF pose - l = T + H + R.P - B
     * @param x y z - desired platform translation in platform frame wrt platform home position (m)
     * @param roll pitch yaw - rotation of platform frame wrt base frame (rad)
     */
    void updateLengthsIK(
        float x,
        float y,
        float z,
        float roll,
        float pitch,
        float yaw);

    /*
     * Set default actuator lengths
     * @brief Sets the minimum and maximum actuator lengths - joint limits
     * @param maxExtend maximum extension of the actuators (m)
     */
    void setMinMaxLength(float maxExtend);

    /*
     * Set anchor coordinates
     * @brief Calculates and sets the x, y, z coordinates for anchor points of the base/platform
     * @param anchorCoords ref to the anchor coords variable that needs to be set (base or platform)
     * @param anchorAngles ref to the respective anchor angles
     * @param radius respective radius value (base or platform)
     */
    void setAnchorCoords(
        MatrixXf& anchorCoords,
        VectorXf& anchorAngles,
        float radius);

    /*
     * Get actuator lengths
     * @return returns the current actuator lengths
     */
    VectorXf getLengths() const;
};

#endif