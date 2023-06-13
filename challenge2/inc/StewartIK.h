#ifndef _StewartIK_
#define _StewartIK_

#include <iostream>
#include <Eigen/Dense>

#define NUM_JOINTS 6

using namespace Eigen;

class StewartIK
{
private:
    // base and platform radii
    float m_baseRadius;
    float m_platformRadius;
    // half of the anchor angles
    float m_baseAngle;
    float m_platformAngle;

    // Actuator limits
    // actuator length with 0 extension
    float m_minLegLength;
    // actuator length with max extension
    float m_maxLegLength;

    // joint angles in order for the base and platform anchors
    VectorXf m_baseAnchorAngles;
    VectorXf m_platformAnchorAngles;
    // vector from the base to the platform at the home position
    VectorXf m_basePlatformVec;
    // current leg lengths
    VectorXf m_currLegLengths;

    // x, y, z anchor coordinates for the base and platform anchors
    MatrixXf m_baseAnchorCoords;
    MatrixXf m_platformAnchorCoords;

public:
    /*
    * Constructor
    * @brief Initializes the StewartIK class with the required angles, lengths and anchor coordinates
    * also sets the minLegLength based on the input base platform distance
    */
    StewartIK(
        float bR,
        float pR,
        float bA,
        float pA,
        float basePlatformDist,
        float maxExtend);
};

#endif