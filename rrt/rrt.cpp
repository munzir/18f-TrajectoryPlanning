// Author: Akash Patel (apatel435@gatech.edu)

// RRT
// Purpose: To find a series of safe interpose positions for trajectory planning
//
// Input: A set of poses
// Output: Trajectories for getting from one pose to the next pose in the input set

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

#include "../file_ops.hpp"
#include "collision.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Global Variables
double pi = M_PI;
double bendLimit = 2.0944;

// // Limits of each joint to prevent collision with the previous joint
// // Below values obtained from hardware init script on Krang: https://github.gatech.edu/WholeBodyControlAttempt1/Krang-Software/blob/master/project/krang/util/scripts/initd/krang
// Format for below vectors
// qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// negative value (-) is clockwise from parent axis

double lowerLimit[] = {0, -1.57, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi, -pi, -bendLimit, -pi, -bendLimit, -pi, -bendLimit, -pi};
double upperLimit[] = {2.88, 1.57, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi, pi, bendLimit, pi, bendLimit, pi, bendLimit, pi};

// Function Prototypes
// // Find all interpose trajectories
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses, string fullRobotPath, int branchFactor, double tolerance, double randomStep, double targetBias, double jointMoveBias);

// // Find interpose trajectory
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, double randomStep, double targetBias, double jointMoveBias);

// // Prune interpose trajectory
Eigen::MatrixXd pruneTrajectory(Eigen::MatrixXd trajectory);

// // Smooth interpose trajectory
Eigen::MatrixXd smoothTrajectory(Eigen::MatrixXd trajectory);

// // Create next safe random pose
Eigen::MatrixXd createNextSafeRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, double randomStep, double targetBias, double jointMoveBias);

// // Check if two poses are the same (close enough)
bool closeEnough(Eigen::MatrixXd pose1, Eigen::MatrixXd pose2, double tolerance);

// // Random Value
double fRand(double min, double max);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (random seed)
    //srand(time(0));
    srand(0);

    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../orderedrandom10fullbalance0.001000tolsafe.txt";
    string inputPosesFilename = "../sparsedorderedfinalSet.txt";
    //string inputPosesFilename = "../random2fullbalance0.001000tolsafe.txt";

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

    // INPUT on below line (branching factor for RRT) (how many children poses
    // for each parent (basically how wide to explore)
    int branchFactor = 1;

    // INPUT on below line (pose equality tolerance)
    double tolerance = 0.010;

    // INPUT on below line (step size for random (radians))
    double randomStep = 0.200;

    // INPUT on below line (target bias in decimal)
    double targetBias = 0.7;

    // INPUT on below line (whether to move the joint or not during the next
    // pose)
    double jointMoveBias = 0.80;

    // INPUT on below line (output filename)
    string outputBaseName = "poseTrajectories";

    Eigen::MatrixXd inputPoses;

    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    cout << "Generating Trajectories ...\n";
    Eigen::MatrixXd allTrajectories = createAllTrajectories(inputPoses, fullRobotPath, branchFactor, tolerance, randomStep, targetBias, jointMoveBias);
    cout << "|-> Done\n";

    // Write test xCOM values to file
    string outfilename;
    string inputPosesName = extractFilename(inputPosesFilename);
    string ext = ".txt";

    outfilename = outputBaseName + inputPosesName + ext;

    cout << "Writing trajectories to " << outfilename << " ...\n";

    ofstream outfile;
    outfile.open(outfilename);
    outfile << allTrajectories;
    outfile.close();

    cout << "|-> Done\n";
}

// // Find all interpose trajectories
//TODO
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses, string fullRobotPath, int branchFactor, double tolerance, double randomStep, double targetBias, double jointMoveBias) {

    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr robot = loader.parseSkeleton(fullRobotPath);

    //TODO what is upper limit on pose trajectory?
    //I guess i can just write it out directly
    Eigen::MatrixXd allInterPoseTraj = inputPoses.row(0);
    //for (int pose = 0; pose < inputPoses.rows() - 1; pose++) {
    for (int pose = 0; pose < 10; pose++) {
        cout << "Trajectory from " << pose << " to " << pose + 1 << endl;
        Eigen::MatrixXd interPoseTraj = createTrajectory(inputPoses.row(pose), inputPoses.row(pose + 1), robot, branchFactor, tolerance, randomStep, targetBias, jointMoveBias);
        Eigen::MatrixXd prunedInterPoseTraj = pruneTrajectory(interPoseTraj);
        Eigen::MatrixXd smoothedInterPoseTraj = smoothTrajectory(prunedInterPoseTraj);
        // How to add these to allInterPoseTraj without concatenating
        // Maybe just print them out directly?
        // TODO
        Eigen::MatrixXd allInterPoseTrajTmp(allInterPoseTraj.rows() + smoothedInterPoseTraj.rows(), allInterPoseTraj.cols());
        allInterPoseTrajTmp << allInterPoseTraj,
                               smoothedInterPoseTraj;
        allInterPoseTraj = allInterPoseTrajTmp;
    }
    return allInterPoseTraj;
}

// // Find interpose trajectory
//TODO
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, double randomStep, double targetBias, double jointMoveBias) {

    // Need to create the tree in this method
    // root is startPose
    // goal is endPose
    // Backtrace from goal to root after any one node in the tree is close
    // enough to root
    // Let's try just a binary tree first

    Eigen::MatrixXd trajectoryTree = startPose;
    Eigen::MatrixXd nextInterPose;
    Eigen::MatrixXd nextInterPoseParent;
    bool reachedGoal = false;
    int posesInTree = 1;
    while (reachedGoal == false) {
        // Add the random Poses
        //nextInterPoseParent = trajectoryTree.row((posesInTree)/branchFactor);
        //ONEBF
        nextInterPoseParent = trajectoryTree.row((posesInTree) - 1);

        cout << "Parent" << posesInTree/branchFactor << endl;
        cout << "PosesInTree" << posesInTree << endl;
        nextInterPose = createNextSafeRandomPose(nextInterPoseParent, endPose, robot, randomStep, targetBias, jointMoveBias);
        //cout << nextInterPose << endl;
        // TODO Add it to eigen Matrix in a better fashion
        Eigen::MatrixXd trajectoryTreeTmp(trajectoryTree.rows() + 1, trajectoryTree.cols());
        trajectoryTreeTmp << trajectoryTree,
                             nextInterPose;
        trajectoryTree = trajectoryTreeTmp;
        //trajectoryTree.row(posesInTree) = nextInterPose;

        if (closeEnough(nextInterPose, endPose, tolerance)) {
            reachedGoal = true;
        }
        posesInTree++;
    }

    // Backtrack the eigen::matrix
    Eigen::MatrixXd trajectory = endPose;
    Eigen::MatrixXd parentPose;
    int maxDepth = (int) (log(posesInTree) / log(branchFactor)) + 1;
    //ONEBF
    maxDepth = posesInTree;

    int currDepth = maxDepth - 1;
    int lastChildIndex = posesInTree - 1;
    int parentIndex = (lastChildIndex - 1)/branchFactor;
    while (currDepth > 0) {

        parentPose = trajectoryTree.row(parentIndex);
        Eigen::MatrixXd trajectoryTmp(trajectory.rows() + 1, trajectory.cols());

        trajectoryTmp << parentPose,
                         trajectory;
        trajectory = trajectoryTmp;

        currDepth--;
        parentIndex = (parentIndex - 1)/branchFactor;
    }

    return trajectory;
}

// // Prune interpose trajectory
//TODO
Eigen::MatrixXd pruneTrajectory(Eigen::MatrixXd trajectory) {
    return trajectory;
}

// // Smooth interpose trajectory
//TODO
Eigen::MatrixXd smoothTrajectory(Eigen::MatrixXd trajectory) {
    return trajectory;
}

// // Create next random pose
// TODO Need to create a random disturbance from initialPose
Eigen::MatrixXd createNextSafeRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, double randomStep, double targetBias, double jointMoveBias) {

    Eigen::MatrixXd randomPoseParams;
    //Eigen::MatrixXd randomPoseParams = initialPose.transpose();

    //double a = 0; //axis-angle1
    //double b = 0; //axis-angle2
    //double c = 0; //axis-angle3
    //double d = 0; //x
    //double e = 0; //y
    //double f = 0; //z
    //double g = 0; //qLWheel
    //double h = 0; //qRWheel
    //double k = 0; //qKinect

    bool isColliding = true;

    while (isColliding) {

        randomPoseParams = initialPose.transpose();

        // Target Bias
        double doubleTargetBias = fRand(0, 1);
        bool targetBiasFlag = false;
        if (doubleTargetBias <= targetBias) {
            targetBiasFlag = true;
        }

        // Decide which joints to move and in what direction
        // -1: away from target, 0: not moving, 1: toward target
        int jointMove[16];


        if (targetBiasFlag == true) {

            //cout << "Going to Target" << endl;

            int qBaseMove = 0;
            for (int i = 0; i < 16; i++) {
                double jointMoveProb = fRand(0, 1);
                if (i == 17 && jointMoveProb <= jointMoveBias) {
                    qBaseMove = 1;
                } else {
                    if (jointMoveProb <= jointMoveBias) {
                        jointMove[i] = 1;
                    } else {
                        jointMove[i] = 0;
                    }
                }
            }

            // Add the default values first
            // TODO: Need to allow change for qBase
            randomPoseParams(0, 0) = targetPose(0, 0);
            randomPoseParams(1, 0) = targetPose(0, 1);
            randomPoseParams(2, 0) = targetPose(0, 2);

            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);
            randomPoseParams(7, 0) = targetPose(0, 7);

            int index = 8;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso)
            int ii = 0;
            for (;ii < 2; ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, randomStep);
                if (step > abs(targetPose(0, index) - initialPose(0, index))) {
                    step = abs(targetPose(0, index) - initialPose(0, index));
                }

                if (initialPose(0, index) > targetPose(0, index)) {

                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }

            // Add qKinect
            randomPoseParams(index, 0) = targetPose(0, index);
            index++;

            //TODO: Need to allow change for arms
            // Add the rest of the values (qArms)
            for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, randomStep);
                if (step > abs(targetPose(0, index) - initialPose(0, index))) {
                    step = abs(targetPose(0, index) - initialPose(0, index));
                }

                if (initialPose(0, index) > targetPose(0, index)) {
                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }
        } else {

            //cout << "Going Random" << endl;

            int qBaseMove = 1;
            for (int i = 0; i < 16; i++) {
                double jointMoveProb = fRand(0, 3);
                if (i == 17) {
                    if (jointMoveProb < 1) {
                        qBaseMove = -1;
                    } else if (jointMoveProb < 2) {
                        qBaseMove = 0;
                    }
                } else {
                    if (jointMoveProb < 1) {
                        jointMove[i] = -1;
                    } else if (jointMoveProb < 2) {
                        jointMove[i] = 0;
                    } else {
                        jointMove[i] = 1;
                    }
                }
            }

            // Add the default values first
            // TODO: Need to allow change for qBase
            randomPoseParams(0, 0) = targetPose(0, 0);
            randomPoseParams(1, 0) = targetPose(0, 1);
            randomPoseParams(2, 0) = targetPose(0, 2);

            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);
            randomPoseParams(7, 0) = targetPose(0, 7);

            int index = 8;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso)
            int ii = 0;
            for (;ii < 2; ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, randomStep);

                if (initialPose(0, index) > targetPose(0, index)) {
                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }

            // Add qKinect
            randomPoseParams(index, 0) = targetPose(0, index);
            index++;

            //TODO: Need to allow change for arms
            // Add the rest of the values (qArms)
            for (;ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, randomStep);

                if (initialPose(0, index) > targetPose(0, index)) {
                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }
        }

        //cout << randomPoseParams.transpose() << endl;
        // Run it through collision check, if it passes then return
        isColliding = inCollision(randomPoseParams, robot);
        //cout << "Pose is colliding" << endl;

    }

    return randomPoseParams.transpose();
}

// // Check if two poses are the same (close enough)
bool closeEnough(Eigen::MatrixXd pose1, Eigen::MatrixXd pose2, double tolerance) {
    double poseNormed = (pose1 - pose2).norm();
    cout << "\rNorm " << poseNormed;
    for (int i = 0; i < pose1.cols(); i++) {
        if (abs(pose1.col(i)(0, 0) - pose2.col(i)(0, 0)) > tolerance) {
            return false;
        }
    }
    return true;
}

// // Random Value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
