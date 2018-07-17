// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for collision checking

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// // Collision Checking Methods
bool inCollision(Eigen::MatrixXd inputPose, SkeletonPtr fullRobot);
bool inFirstParentJointLimits(Eigen::MatrixXd inputPose, SkeletonPtr robot);
bool isColliding(SkeletonPtr robot);

// // World creation for collision checking
SkeletonPtr createFloor();
