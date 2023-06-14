## Challenge 2 - DiamondAge 3D

**Author: Manuj Trehan**

### Assumptions
1. The base and platform frame are aligned when the platform is at its home/retracted position (no relative rotation)
2. Axes: x pointing to the right, y pointing up, z pointing outwards
3. The base plate and platform plate thickness offsets are ignored
4. The platform is initialized in the home position
5. N_DOF is fixed - 6
6. All actuators have the same retracted and extended length

### Discussion
I spent approximately 4.5 hrs on reading up on literature, solving the challenge and documenting my approach.

I created a ```StewartIK``` class for the inverse kinematics solver. It requires and stores the default system parameters, like the plate radius, anchor angles and actuator limits. It has methods for initialization, solving IK for a given 6-DoF pose, and getting the current actuator lengths, apart from some other helper functions.

### Challenges
It was a little difficult to thoroughly test my algorithm to see if it was giving the correct ouputs. One way to check would by through a simulation, but that was out of scope of this challenge.