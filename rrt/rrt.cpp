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

#include "file_ops.hpp"
#include "balance_collision.hpp"

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Function Prototypes
// // Find all interpose trajectories
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses);

// // Find interpose trajectory
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose);

// // Prune interpose trajectory
Eigen::MatrixXd pruneTrajectory(Eigen::MatrixXd trajectory);

// // Smooth interpose trajectory
Eigen::MatrixXd smoothTrajectory(Eigen::MatrixXd trajectory);

// TODO: Commandline arguments a default values
int main() {
    // INPUT on below line (input poses filename)
    string inputPosesFilename = "../random6003fullbalance0.001000tolsafe.txt";

    // INPUT on below line (absolute robot path)
    string fullRobotPath = "/home/apatel435/Desktop/WholeBodyControlAttempt1/09-URDF/Krang/Krang.urdf";

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
    Eigen::MatrixXd allTrajectories = createAllTrajectories(inputPoses);
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
Eigen::MatrixXd createAllTrajectories(Eigen::MatrixXd inputPoses) {
    return inputPoses;
}

// // Find interpose trajectory
//TODO
Eigen::MatrixXd createTrajectory(Eigen::MatrixXd startPose, Eigen::MatrixXd endPose) {
    return endPose;
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
