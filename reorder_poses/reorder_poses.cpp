// Author: Akash Patel (apatel435@gatech.edu)

// Reorder Poses
// Purpose: To order an input set of poses in terms of closest distance (using
// norm of joint space metric)
//
// Input: A set of poses
// Output: Reordered set of poses based on closest distance

// Includes
#include <dart/dart.hpp>
#include <iostream>
#include <fstream>

#include "../file_ops.hpp"

// Namespaces
using namespace std;

// Function Prototypes
// // Reorder the poses in terms of closest distance
Eigen::MatrixXd reorderPoses(Eigen::MatrixXd inputPoses, Eigen::MatrixXd dofWeights);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (input poses filename)
    //string inputPosesFilename = "../random6003fullbalance0.001000tolsafe.txt";
    string inputPosesFilename = "../finalSet.txt";

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";


    // INPUT on below line (weights on each joint)
    // Order is the same as the dart format for pose
    Eigen::MatrixXd dofWeights(1, 25);
    dofWeights << 1, 1, 1,    // Axis Angle
                  1, 1, 1,    // x, y, z
                  1, 1,       // qLWheel, qRWheel
                  1, 1, 1,    // qWaist, qTorso, qKinect
      1, 1, 1, 1, 1, 1, 1,   // qLArm0, ..., qLArm6 (from shoulder to the last joint)
      1, 1, 1, 1, 1, 1, 1;   // qRArm0, ..., qRArm6 (from shoulder to the last joint)

    // INPUT on below line (output filename)
    string outputBaseName = "ordered";

    Eigen::MatrixXd inputPoses;

    try {
        cout << "Reading input poses ...\n";
        inputPoses = readInputFileAsMatrix(inputPosesFilename);
        cout << "|-> Done\n";
    } catch (exception& e) {
        cout << e.what() << endl;
        return EXIT_FAILURE;
    }

    cout << "Ordering Poses ...\n";
    Eigen::MatrixXd orderedPoses = reorderPoses(inputPoses, dofWeights);
    cout << "|-> Done\n";

    // Write test xCOM values to file
    string outfilename;
    string inputPosesName = extractFilename(inputPosesFilename);
    string ext = ".txt";

    outfilename = outputBaseName + inputPosesName + ext;

    cout << "Writing ordered poses to " << outfilename << " ...\n";

    ofstream outfile;
    outfile.open(outfilename);
    outfile << orderedPoses;
    outfile.close();

    cout << "|-> Done\n";
}

// // Reorder the poses in terms of closest distance
Eigen::MatrixXd reorderPoses(Eigen::MatrixXd inputPoses, Eigen::MatrixXd dofWeights) {
    Eigen::MatrixXd orderedPoses(inputPoses.rows(), inputPoses.cols());

    cout << "Pose: 0" << "/" << inputPoses.rows();

    // Add first pose
    orderedPoses.row(0) = inputPoses.row(0);
    Eigen::MatrixXd inputPosesTmp = inputPoses.bottomRows(inputPoses.rows() - 1);
    inputPoses = inputPosesTmp;

    for (int pose = 1; pose < orderedPoses.rows(); pose++) {
        Eigen::MatrixXd prevPose = orderedPoses.row(pose - 1);
        int nextClosePose = 0;
        double minNorm = (dofWeights.transpose() * (prevPose - inputPoses.row(0))).norm();
        int i = 0;
        while (i < inputPoses.rows()) {

            cout << "\rPose: " << pose + 1 << "/" << orderedPoses.rows() << " Finding Closest: " << i << "/" << inputPoses.rows() << " \t ";
            double norm = (dofWeights.transpose() * (prevPose - inputPoses.row(i))).norm();
            if (norm < minNorm) {
                minNorm = norm;
                nextClosePose = i;
            }
            i++;
        }

        orderedPoses.row(pose) = inputPoses.row(nextClosePose);

        // Remove the nextClosePose from inputPoses, since it is
        // already used (don't want to double count a single pose)

        Eigen::MatrixXd inputPosesTmp(inputPoses.rows() - 1, inputPoses.cols());

        if (nextClosePose == 0) {
            inputPosesTmp = inputPoses.bottomRows(inputPoses.rows() - 1);
        } else {
            inputPosesTmp.topRows(nextClosePose) = inputPoses.topRows(nextClosePose);
            inputPosesTmp.bottomRows(inputPosesTmp.rows() - nextClosePose) = inputPoses.bottomRows(inputPoses.rows() - 1 - nextClosePose);
        }
        inputPoses = inputPosesTmp;

    }

    cout << endl;
    return orderedPoses;
}
