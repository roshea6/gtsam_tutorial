#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>

int main(int argc, char const* argv[])
{
    // Create an empty graph
    gtsam::NonlinearFactorGraph graph;

    // Create our starting pose
    gtsam::Pose2 prior(0.0, 0.0, 0.0);

    // create noise for the pose to make a Gaussian distribution
    gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));

    // Add the prior to the graph
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(0, prior, pose_noise));


    // Create odometry readings
    gtsam::Pose2 odom1(1.1, 0.0, 0.0); // Drive forward 1.1 units
    gtsam::Pose2 odom2(-0.9, 0.0, 0.0); // Drive backwards 0.9 units
    gtsam::Pose2 odom3(-1.0, 0.0, 0.0); // Drive backwards 0.8 units
    gtsam::Pose2 odom4(1.05, 0.0, 0.0); // Drive forwards 1.15 units

    // create the odometry noise
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

    // Add the odom readings to the graph
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(0, 1, odom1, odom_noise)); // First factor going between pose 0 and pose 1
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 0, odom2, odom_noise)); // Second factor going between pose 1 and pose 0
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(0, 2, odom3, odom_noise)); // Third factor going between pose 0 and pose 2
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, odom4, odom_noise)); // Fourth factor going between pose 2 and pose 0

    graph.print("Graph:\n");


    // Create guesses for the poses at each node
    gtsam::Values pose_guesses;
    pose_guesses.insert(0, gtsam::Pose2(0.0, 0.0, 0.0));
    pose_guesses.insert(1, gtsam::Pose2(0.0, 0.0, 0.0));
    pose_guesses.insert(2, gtsam::Pose2(0.0, 0.0, 0.0));
    pose_guesses.insert(3, gtsam::Pose2(0.0, 0.0, 0.0));

    // Optimize the graph to obtain the most probably poses
    gtsam::Values results = gtsam::LevenbergMarquardtOptimizer(graph, pose_guesses).optimize();

    results.print("Results:\n");


    return 0;
}