// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for generating poses that focuses on
// balancing and collision code

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "balance_collision.hpp"

// Namespaces
using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::simulation;
using namespace dart::utils;

// Global Variables
double kpi = M_PI;
double kbendLimit = 2.0944;

// // Limits of each joint to prevent collision with the previous joint
// // Below values obtained from hardware init script on Krang: https://github.gatech.edu/WholeBodyControlAttempt1/Krang-Software/blob/master/project/krang/util/scripts/initd/krang
// Format for below vectors
// qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// negative value (-) is clockwise from parent axis

double lowerJointLimit[] = {0, -1.57, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi};
double upperJointLimit[] = {2.88, 1.57, kpi, kbendLimit, kpi, kbendLimit, kpi, kbendLimit, kpi, kpi, kbendLimit, kpi, kbendLimit, kpi, kbendLimit, kpi};

// Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, SkeletonPtr fullRobot, SkeletonPtr fixedWheelRobot, Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck) {
    Eigen::MatrixXd balPoseParams;

    // Set position of full robot to the pose for transform and collision
    fullRobot->setPositions(balPoseParams);
    fullRobot->enableSelfCollisionCheck();
    fullRobot->disableAdjacentBodyCheck();

    // Check for first parent joint constraints throw exception if fails
    if (!inFirstParentJointLimits(balPoseParams, fullRobot)) {
        throw runtime_error("Pose violates first parent joint limits!");
    }

    // Run it through collision check
    bool isCollision = false;
    if (collisionCheck == true) {
        isCollision = isColliding(fullRobot);
    }
    // Throw exception if colliding
    if (isCollision) {
        throw runtime_error("Pose is in collision!");
    }

    return balPoseParams.transpose();
}

// // First Parent Collision Checking
bool inFirstParentJointLimits(Eigen::MatrixXd inputPose, SkeletonPtr robot) {
    // Check for base angle constraint (has to be b/t -pi/2 and pi/2)
    Eigen::MatrixXd baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double heading = atan2(baseTf(0, 0), -baseTf(1, 0));
    double qBase = atan2(baseTf(0,1)*cos(heading) + baseTf(1,1)*sin(heading), baseTf(2,1));

    if (qBase < 0 || qBase > M_PI) {
        return false;
    }

    int startJoint = 8;
    for (int i = 0; i < sizeof(lowerJointLimit)/sizeof(lowerJointLimit[0]); i++) {
        // qWaist and qTorso
        if (i < 2) {
            if (inputPose.row(startJoint + i)(0, 0) < lowerJointLimit[i] || inputPose.row(startJoint + i)(0, 0) > upperJointLimit[i]) {
                return false;
            }
        } else {
            // Skip the qKinect (add one to inputPose index to account for offset)
            // Continue with the arms
            if (inputPose.row(startJoint + 1 + i)(0, 0) < lowerJointLimit[i] || inputPose.row(startJoint + 1 + i)(0, 0) > upperJointLimit[i]) {
                return false;
            }
        }
    }
    return true;
}

// // Collision Check
// TODO: Need to fix implementation
bool isColliding(SkeletonPtr robot) {

    WorldPtr world(new World);
    SkeletonPtr floor = createFloor();

    world->addSkeleton(floor);
    world->addSkeleton(robot);

    auto constraintSolver = world->getConstraintSolver();
    auto group = constraintSolver->getCollisionGroup();
    auto& option = constraintSolver->getCollisionOption();
    auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
    option.collisionFilter = bodyNodeFilter;

    CollisionResult result;
    group->collide(option, &result);

    return result.isCollision();

}

// // World creation for collision checking
SkeletonPtr createFloor() {
    SkeletonPtr floor = Skeleton::create();

    // Give the floor a body
    BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 7.0;
    double floor_height = 0.05;
    std::shared_ptr<BoxShape> box(
          new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0.0, 0.0, floor_height+0.25);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}
