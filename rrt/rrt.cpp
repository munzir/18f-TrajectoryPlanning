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

#include "../../18h-Util/collision.hpp"
#include "../../18h-Util/convert_pose_formats.hpp"
#include "../../18h-Util/file_ops.hpp"
#include "../../18h-Util/random.hpp"

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
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses, string fullRobotPath, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias, bool fixedWaist);

// // Find interpose trajectory
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias);

// // Find fixed waist interpose trajectory
Eigen::MatrixXd createFixedWaistTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias);

// // Prune interpose trajectory
Eigen::MatrixXd pruneTrajectory(Eigen::MatrixXd trajectory);

// // Smooth interpose trajectory
Eigen::MatrixXd smoothTrajectory(Eigen::MatrixXd trajectory);

// // Create next safe random pose
Eigen::MatrixXd createNextSafeRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias);

// // Create next safe fixed waist random pose
Eigen::MatrixXd createNextSafeFixedWaistRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias);

// // Move just the waist
bool moveWaistOnly(Eigen::MatrixXd startPose, double targetWaistPosition, Eigen::MatrixXd *nextPose, SkeletonPtr robot);

// // Check if two poses are the same (close enough)
bool closeEnough(Eigen::MatrixXd pose1, Eigen::MatrixXd pose2, double tolerance);

// // Check if two poses are the same (close enough)
bool allButWaistCloseEnough(Eigen::MatrixXd pose1, Eigen::MatrixXd pose2, double tolerance);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (random seed)
    //srand(time(0));
    srand(0);

    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../orderedrandom10fullbalance0.001000tolsafe.txt";
    //string inputPosesFilename = "../sparsedorderedfinalSet.txt";
    //string inputPosesFilename = "../rfinalSet.txt";
    string inputPosesFilename = "../random10anglebalance0.001000tolsafe.txt";

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/KrangCollision.urdf";

    // INPUT on below line (to move the waist independently or not)
    //bool fixedWaist = false;
    bool fixedWaist = true;

    // INPUT on below line (branching factor for RRT) (how many children poses
    // for each parent (basically how wide to explore)
    int branchFactor = 1;

    // INPUT on below line (pose equality tolerance)
    double tolerance = 0.005;

    // INPUT on below line (step size for random (radians))
    //double randomStep = 0.500;
    double randomStep = 2.000;
    Eigen::MatrixXd maxRandomSteps(1, 17);
    maxRandomSteps << 1, // qBase
                         1, 1, // qWaist, qTorso
          1, 1, 1, 1, 1, 1, 1, // qRArm0, ..., qRArm6 (from shoulder to the last joint)
          1, 1, 1, 1, 1, 1, 1; // qRArm0, ..., qRArm6 (from shoulder to the last joint)
    maxRandomSteps = randomStep * maxRandomSteps;

    // INPUT on below line (granulation) (how many steps to check in
    // between initial pose and the newly found pose)
    //int granulation = 10;
    //This assumes the time for each joint going from its start to end pos is
    //the same irrespective of its distance
    int granulation = 100;

    // INPUT on below line (target bias in decimal)
    double targetBias = 0.90;

    // INPUT on below line (whether to move the joint or not during the next
    // pose)
    double jointMoveBias = 0.90;

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
    Eigen::MatrixXd allTrajectories = createAllTrajectories(inputPoses, fullRobotPath, branchFactor, tolerance, maxRandomSteps, granulation, targetBias, jointMoveBias, fixedWaist);
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
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses, string fullRobotPath, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias, bool fixedWaist) {
    // Instantiate full robot
    DartLoader loader;
    SkeletonPtr robot = loader.parseSkeleton(fullRobotPath);

    //TODO what is upper limit on pose trajectory?
    //I guess i can just write it out directly
    Eigen::MatrixXd allInterPoseTraj = inputPoses.row(0);
    for (int pose = 0; pose < inputPoses.rows() - 1; pose++) {
        cout << "Trajectory from " << pose + 1 << " to " << pose + 2 << endl;
        Eigen::MatrixXd interPoseTraj;
        if (fixedWaist) {
            interPoseTraj = createFixedWaistTrajectory(inputPoses.row(pose), inputPoses.row(pose + 1), robot, branchFactor, tolerance, maxRandomSteps, granulation, targetBias, jointMoveBias);
        } else {
            interPoseTraj = createTrajectory(inputPoses.row(pose), inputPoses.row(pose + 1), robot, branchFactor, tolerance, maxRandomSteps, granulation, targetBias, jointMoveBias);
        }
        Eigen::MatrixXd prunedInterPoseTraj = pruneTrajectory(interPoseTraj);
        Eigen::MatrixXd smoothedInterPoseTraj = smoothTrajectory(prunedInterPoseTraj);

        // How to add these to allInterPoseTraj without concatenating
        // Maybe just print them out directly?
        // TODO
        ofstream interPoseTrajFile;
        string interPoseTrajFilename = "interposeTraj" + to_string(pose + 1) + "-" + to_string(pose + 2) + ".txt";
        interPoseTrajFile.open(interPoseTrajFilename);
        interPoseTrajFile << smoothTrajectory(smoothedInterPoseTraj);


        Eigen::MatrixXd allInterPoseTrajTmp(allInterPoseTraj.rows() + smoothedInterPoseTraj.rows(), allInterPoseTraj.cols());
        allInterPoseTrajTmp << allInterPoseTraj,
                               smoothedInterPoseTraj;
        allInterPoseTraj = allInterPoseTrajTmp;
    }
    return allInterPoseTraj;
}

// // Find interpose trajectory
//TODO
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias) {
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
        //TODO ONEBF
        nextInterPoseParent = trajectoryTree.row((posesInTree) - 1);

        //cout << "Parent" << posesInTree/branchFactor << endl;
        cout << "PosesInTree" << posesInTree << endl;
        nextInterPose = createNextSafeRandomPose(nextInterPoseParent, endPose, robot, maxRandomSteps, granulation, targetBias, jointMoveBias);
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
    //TODO ONEBF
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

// // Find fixed waist interpose trajectory
//TODO
Eigen::MatrixXd createFixedWaistTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose, SkeletonPtr robot, int branchFactor, double tolerance, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias) {
    // Need to create the tree in this method
    // root is startPose
    // goal is endPose
    // Backtrace from goal to root after any one node in the tree is close
    // enough to root
    // Let's try just a binary tree first

    Eigen::MatrixXd trajectoryTree = startPose;
    Eigen::MatrixXd nextInterPose;
    Eigen::MatrixXd nextInterPoseParent;

    // INPUT on below line (define a pose in which the waist is safe to move)
    // TODO need to come back to define reset position for arms
    double heading = startPose.coeff(0, 0);
    double qBase = startPose.coeff(0, 1);
    double x = startPose.coeff(0, 2);
    double y = startPose.coeff(0, 3);
    double z = startPose.coeff(0, 4);
    double qLWheel = startPose.coeff(0, 5);
    double qRWheel = startPose.coeff(0, 6);

    double waistStart = startPose.coeff(0, 7);

    Eigen::MatrixXd waistSafePose(1, 23);
    waistSafePose << heading, qBase, // heading, qBase
                     x, y, z, // x, y, z
            qLWheel, qRWheel, // qLWheel, qRWheel
                        waistStart, 0, // qWaist, qTorso
         0, 0, 0, 0, 0, 0, 0, // qRArm0, ..., qRArm6 (from shoulder to the last joint)
         0, 0, 0, 0, 0, 0, 0; // qRArm0, ..., qRArm6 (from shoulder to the last joint)

    bool reachedGoal = false;
    int posesInTree = 1;

    // Move pose to a waist-safe pose
    while (reachedGoal == false) {
        //nextInterPoseParent = trajectoryTree.row((posesInTree)/branchFactor);
        //TODO ONEBF
        nextInterPoseParent = trajectoryTree.row((posesInTree) - 1);

        //cout << "Parent" << posesInTree/branchFactor << endl;
        cout << "PosesInTree" << posesInTree << endl;

        nextInterPose = createNextSafeFixedWaistRandomPose(nextInterPoseParent, waistSafePose, robot, maxRandomSteps, granulation, targetBias, jointMoveBias);

        // TODO Add it to eigen Matrix in a better fashion
        Eigen::MatrixXd trajectoryTreeTmp(trajectoryTree.rows() + 1, trajectoryTree.cols());
        trajectoryTreeTmp << trajectoryTree,
                             nextInterPose;
        trajectoryTree = trajectoryTreeTmp;
        //trajectoryTree.row(posesInTree) = nextInterPose;

        if (closeEnough(nextInterPose, waistSafePose, tolerance)) {
            reachedGoal = true;
        }
        posesInTree++;
    }
    reachedGoal = false;

    // Move the safe pose only via waist to final waist position
    nextInterPoseParent = trajectoryTree.row((posesInTree) - 1);
    bool waistMovedSuccessfully = moveWaistOnly(nextInterPoseParent, endPose.col(7)(0, 0), &nextInterPose, robot);
    posesInTree++;

    if (!waistMovedSuccessfully) {
        // Maybe even program force exit or something
        nextInterPose = Eigen::MatrixXd::Zero(1, 23);
    }

    // TODO Add it to eigen Matrix in a better fashion
    Eigen::MatrixXd trajectoryTreeTmp(trajectoryTree.rows() + 1, trajectoryTree.cols());
    trajectoryTreeTmp << trajectoryTree,
                         nextInterPose;
    trajectoryTree = trajectoryTreeTmp;
    //trajectoryTree.row(posesInTree) = nextInterPose;

    // Move pose to final joints positions (waist is already in final position
    // at this point)
    while (reachedGoal == false) {
        //nextInterPoseParent = trajectoryTree.row((posesInTree)/branchFactor);
        //TODO ONEBF
        nextInterPoseParent = trajectoryTree.row((posesInTree) - 1);
        //cout << nextInterPoseParent << endl;

        //cout << "Parent" << posesInTree/branchFactor << endl;
        cout << "PosesInTree" << posesInTree << endl;

        nextInterPose = createNextSafeFixedWaistRandomPose(nextInterPoseParent, endPose, robot, maxRandomSteps, granulation, targetBias, jointMoveBias);

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
    //TODO ONEBF
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
Eigen::MatrixXd createNextSafeRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias) {
    Eigen::MatrixXd randomPoseParams;
    //Eigen::MatrixXd randomPoseParams = initialPose.transpose();

    //double a = 0; //heading
    //double b = 0; //qBase
    //double c = 0; //x
    //double d = 0; //y
    //double e = 0; //z
    //double f = 0; //qLWheel
    //double g = 0; //qRWheel

    bool inCollision = true;

    while (inCollision) {

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
            for (int i = 0; i < 17; i++) {
                double jointMoveProb = fRand(0, 1);
                if (i == 16 && jointMoveProb <= jointMoveBias) {
                    qBaseMove = 1;
                } else {
                    if (jointMoveProb <= jointMoveBias) {
                        jointMove[i] = 1;
                    } else {
                        jointMove[i] = 0;
                    }
                }
            }

            // TODO: Need to allow change for qBase
            // Find the target qBase and heading
            double headingTarget = targetPose.coeff(0, 0);
            double qBaseTarget = targetPose.coeff(0, 1);

            double headingInitial = initialPose.coeff(0, 0);
            double qBaseInitial = initialPose.coeff(0, 1);

            // Change qBase and heading
            double stepHeading = fRand(0.00, maxRandomSteps(0, 0));
            if (stepHeading > abs(headingTarget - headingInitial)) {
                stepHeading = abs(headingTarget - headingInitial);
            }
            if (headingInitial > headingTarget) {
                stepHeading *= -1;
            }

            double stepQBase = fRand(0.00, maxRandomSteps(0, 0));
            if (stepQBase > abs(qBaseTarget - qBaseInitial)) {
                stepQBase = abs(qBaseTarget - qBaseInitial);
            }
            if (qBaseInitial > qBaseTarget) {
                stepQBase *= -1;
            }

            //double heading = headingInitial + stepHeading * qBaseMove;
            double heading = headingTarget;
            //TODO
            double qBase = qBaseInitial + stepQBase * qBaseMove;

            randomPoseParams(0, 0) = heading;
            randomPoseParams(1, 0) = qBase;

            // Values that supposedly do not change
            randomPoseParams(2, 0) = targetPose(0, 2);
            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);

            int index = 7;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso, qArms)
            for (int ii = 0; ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, maxRandomSteps(0, index - 7 + 1));
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
            for (int i = 0; i < 17; i++) {
                double jointMoveProb = fRand(0, 3);
                if (i == 16) {
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
            double headingTarget = targetPose.coeff(0, 0);
            double qBaseTarget = targetPose.coeff(0, 1);

            // Convert the three to qBase and heading
            double headingInitial = initialPose.coeff(0, 0);
            double qBaseInitial = initialPose.coeff(0, 1);

            // Change qBase and heading
            double stepHeading = fRand(0.00, maxRandomSteps(0, 0));
            if (headingInitial > headingTarget) {
                stepHeading *= -1;
            }

            double stepQBase = fRand(0.00, maxRandomSteps(0, 0));
            if (qBaseInitial > qBaseTarget) {
                stepQBase *= -1;
            }

            //double heading = headingInitial + stepHeading * qBaseMove;
            double heading = headingTarget;
            //TODO
            double qBase = qBaseInitial + stepQBase * qBaseMove;

            randomPoseParams(0, 0) = heading;
            randomPoseParams(1, 0) = qBase;

            // Values that supposedly do not change
            randomPoseParams(2, 0) = targetPose(0, 2);
            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);

            int index = 7;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso, qArms)
            for (int ii = 0; ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, maxRandomSteps(0, index - 7 + 1));
                if (step > abs(targetPose(0, index) - initialPose(0, index))) {
                    step = abs(targetPose(0, index) - initialPose(0, index));
                }

                if (initialPose(0, index) > targetPose(0, index)) {
                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }
        }

        // Run it through collision check with granulation, if it passes then return
        robot->setPositions(munzirToDart(randomPoseParams.transpose()));
        inCollision = isColliding(robot);
        if (!inCollision) {
            for (int g = 1; g < granulation + 1; g++) {
                Eigen::MatrixXd nextStepPose = initialPose + (randomPoseParams.transpose() - initialPose) * (g/granulation);
                robot->setPositions(munzirToDart(nextStepPose));
                inCollision = isColliding(robot);
                if (inCollision == true) {
                    break;
                }
            }
        }
        //cout << "Pose is colliding" << endl;

    }

    return randomPoseParams.transpose();
}

// // Create next random pose
// TODO Need to create a random disturbance from initialPose, why? lol
// forgetting comments
// TODO Need to set waist to constant value for each new one created
Eigen::MatrixXd createNextSafeFixedWaistRandomPose(Eigen::MatrixXd initialPose, Eigen::MatrixXd targetPose, SkeletonPtr robot, Eigen::MatrixXd maxRandomSteps, int granulation, double targetBias, double jointMoveBias) {
    Eigen::MatrixXd randomPoseParams;

    //double a = 0; //heading
    //double b = 0; //qBase
    //double c = 0; //x
    //double d = 0; //y
    //double e = 0; //z
    //double f = 0; //qLWheel
    //double g = 0; //qRWheel

    bool inCollision = true;

    while (inCollision) {

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
            for (int i = 0; i < 18; i++) {
                double jointMoveProb = fRand(0, 1);
                if (i == 17 && jointMoveProb <= jointMoveBias) {
                    qBaseMove = 1;
                } else {
                    if (jointMoveProb <= jointMoveBias) {
                        jointMove[i] = 1; } else {
                        jointMove[i] = 0;
                    }
                }
            }

            // TODO: Need to allow change for qBase
            // Find the target qBase and heading
            double headingTarget = targetPose.coeff(0, 0);
            double qBaseTarget = targetPose.coeff(0, 1);

            // Find the initial qBase and heading
            double headingInitial = initialPose.coeff(0, 0);
            double qBaseInitial = initialPose.coeff(0, 1);

            // Change qBase and heading
            double stepHeading = fRand(0.00, maxRandomSteps(0, 0));
            if (stepHeading > abs(headingTarget - headingInitial)) {
                stepHeading = abs(headingTarget - headingInitial);
            }
            if (headingInitial > headingTarget) {
                stepHeading *= -1;
            }

            double stepQBase = fRand(0.00, maxRandomSteps(0, 0));
            if (stepQBase > abs(qBaseTarget - qBaseInitial)) {
                stepQBase = abs(qBaseTarget - qBaseInitial);
            }
            if (qBaseInitial > qBaseTarget) {
                stepQBase *= -1;
            }

            //double heading = headingInitial + stepHeading * qBaseMove;
            double heading = headingTarget;
            double qBase = qBaseInitial + stepQBase * qBaseMove;

            randomPoseParams(0, 0) = heading;
            randomPoseParams(1, 0) = qBase;

            // Values that supposedly do not change
            randomPoseParams(2, 0) = targetPose(0, 2);
            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);

            int index = 7;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso, qArms)
            for (int ii = 0; ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, maxRandomSteps(0, index - 7 + 1));
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
            for (int i = 0; i < 18; i++) {
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
            double headingTarget = targetPose.coeff(0, 0);
            double qBaseTarget = targetPose.coeff(0, 1);

            // Add the initial qBase and heading
            double headingInitial = initialPose.coeff(0, 0);
            double qBaseInitial = initialPose.coeff(0, 1);

            // Change qBase and heading
            double stepHeading = fRand(0.00, maxRandomSteps(0, 0));
            if (headingInitial > headingTarget) {
                stepHeading *= -1;
            }

            double stepQBase = fRand(0.00, maxRandomSteps(0, 0));
            if (qBaseInitial > qBaseTarget) {
                stepQBase *= -1;
            }

            //double heading = headingInitial + stepHeading * qBaseMove;
            double heading = headingTarget;
            double qBase = qBaseInitial + stepQBase * qBaseMove;

            randomPoseParams(0, 0) = heading;
            randomPoseParams(1, 0) = qBase;

            // Values that supposedly do not change
            randomPoseParams(2, 0) = targetPose(0, 2);
            randomPoseParams(3, 0) = targetPose(0, 3);
            randomPoseParams(4, 0) = targetPose(0, 4);
            randomPoseParams(5, 0) = targetPose(0, 5);
            randomPoseParams(6, 0) = targetPose(0, 6);

            int index = 7;

            //TODO: Need to allow change for waist and torso
            // Loop through adding the rest of the values (qWaist, qTorso, qArms)
            for (int ii = 0; ii < sizeof(lowerLimit)/sizeof(lowerLimit[0]); ii++) {
                // TODO Find closest direction

                double step = fRand(0.00, maxRandomSteps(0, index - 7 + 1));

                if (initialPose(0, index) > targetPose(0, index)) {
                    step *= -1;
                }

                randomPoseParams(index, 0) += step * jointMove[ii];
                index++;
            }
        }

        // Set the waist to be the same since that's what fixed waist means lol
        // AHHHHHHHHHHHHHHHH Why scream?
        // Cuz we should get same results using either initialPose or targetPose
        // But do we?
        // No, No we dont. A solution does not occur using initialPose
        // Hmmm, lets inspect out data for targetPose and see if the trajectory
        // follows what we want.
        // Seems like if the target waist was lower then the initial waist, the
        // method moveWaistOnly would not move the waist
        // Fixed now
        // Phew what a bug

        randomPoseParams(7, 0) = initialPose(0, 7);
        //randomPoseParams(7, 0) = targetPose(0, 7);

        // TODO fix the 69 to 70 Prob
        // Mystery might be the fact that it just takes a while
        // robot->setPositions(munzirToDart(initialPose));
        //inCollsion = isColliding(robot);
        //cout << initialPose << endl;
        //string isIniCol = to_string(inCollision);
        //cout << "Ini is col" + isIniCol << endl;

        // Run it through collision check with granulation, if it passes then return

        robot->setPositions(munzirToDart(randomPoseParams.transpose()));
        inCollision = isColliding(robot);
        if (!inCollision) {
            for (int g = 1; g < granulation + 1; g++) {
                Eigen::MatrixXd nextStepPose = initialPose + (randomPoseParams.transpose() - initialPose) * (g/granulation);
                robot->setPositions(munzirToDart(nextStepPose));
                inCollision = isColliding(robot);
                if (inCollision == true) {
                    break;
                }
            }
        }
        //cout << "Pose is colliding" << endl;

    }

    return randomPoseParams.transpose();
}

bool moveWaistOnly(Eigen::MatrixXd startPose, double targetWaistPosition, Eigen::MatrixXd *nextPose, SkeletonPtr robot) {
    Eigen::MatrixXd movingWaistPose = startPose;
    double waistStep = 0.010; //radians
    if (movingWaistPose.col(7)(0, 0) > targetWaistPosition) {
        waistStep *= -1;
    }

    robot->setPositions(munzirToDart(movingWaistPose));
    if (isColliding(robot)) {
        return false;
    }

    // Move the waist incrementally to see if collision happens
    while (movingWaistPose.col(7)(0, 0) - targetWaistPosition != 0) {
        if (abs(movingWaistPose.col(7)(0, 0) - targetWaistPosition) < abs(waistStep)) {
            movingWaistPose.col(7)(0, 0) = targetWaistPosition;
        } else {
            movingWaistPose.col(7)(0, 0) += waistStep;
        }
        robot->setPositions(munzirToDart(movingWaistPose));
        if (isColliding(robot)) {
            return false;
        }
    }
    *nextPose = movingWaistPose;

    return true;
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

// // Check if two poses are the same (close enough)
// Waist is ignored (heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, etc.
bool allButWaistCloseEnough(Eigen::MatrixXd pose1, Eigen::MatrixXd pose2, double tolerance) {
    pose1.col(7)(0, 0) = 0;
    pose2.col(7)(0, 0) = 0;
    //TODO oh so we can put the posesintree and the norm on one line to print it
    //nicely
    double poseNormed = (pose1 - pose2).norm();
    cout << "\rNorm " << poseNormed;
    for (int i = 0; i < pose1.cols(); i++) {
        if (abs(pose1.col(i)(0, 0) - pose2.col(i)(0, 0)) > tolerance) {
            return false;
        }
    }
    return true;
}
