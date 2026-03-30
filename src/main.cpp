#include <Eigen/Dense>
#include <fstream>
#include <vector>

#include "slam_algorithm.hpp"

// DONT CHANGE THIS FILE
// SOLUTION SHOULD BE IN src/slam_algorithm.cpp and src/slam_algorithm.hpp

using namespace Eigen;

int main(int argc, char *argv[]) {
  SlamAlgorithm slam;

  const double elipse_major = 15.0;
  const double elipse_minor = 8.0;
  const int NUM_CONES = 30;
  const double LAP_TIME = 20.0;
  const double DT = 0.5;

  std::ofstream gt_file("ground_truth.csv");
  gt_file << "id,x,y" << std::endl;
  for (int i = 0; i < NUM_CONES; i++) {
    double cone_angle = (i / (double)NUM_CONES) * 2.0 * M_PI;
    double x = elipse_major * cos(cone_angle);
    double y = elipse_minor * sin(cone_angle);
    gt_file << i << "," << x << "," << y << std::endl;
  }
  gt_file.close();

  std::ofstream traj_file("trajectory.csv");
  traj_file << "x,y" << std::endl;

  // Simulate 2 full laps
  for (double t = 0; t < LAP_TIME * 2.0; t += DT) {

    // 1. True physical position of the car on the ellipse
    double true_angle = (t / LAP_TIME) * 2.0 * M_PI;
    Vector2d true_car_pos(elipse_major * cos(true_angle),
                          elipse_minor * sin(true_angle));

    // True heading (tangent to ellipse)
    double dx_true = -elipse_major * sin(true_angle);
    double dy_true = elipse_minor * cos(true_angle);
    double true_heading = atan2(dy_true, dx_true);

    // 2. Odometry position of the car
    // Accumulates an angular drift of ~23 degrees over the lap
    // and an outward lateral drift of 1.5 meters.
    double drift_angle = true_angle + (0.2 * t / LAP_TIME);
    double lat_drift = 1.0 * (t / LAP_TIME);

    double x_odom = (elipse_major + lat_drift) * cos(drift_angle);
    double y_odom = (elipse_minor + lat_drift) * sin(drift_angle);

    double dx_odom = -(elipse_major + lat_drift) * sin(drift_angle);
    double dy_odom = (elipse_minor + lat_drift) * cos(drift_angle);
    double odom_heading = atan2(dy_odom, dx_odom);

    Pose odom_pose{x_odom, y_odom, odom_heading};
    traj_file << x_odom << "," << y_odom << std::endl;

    // 3. Generate Mock LiDAR observations (Car sees cones within 6 meters)
    std::vector<Vector2d> local_observations;
    for (int i = 0; i < NUM_CONES; ++i) {
      double cone_angle = (i / (double)NUM_CONES) * 2.0 * M_PI;
      Vector2d true_cone_pos(elipse_major * cos(cone_angle),
                             elipse_minor * sin(cone_angle));

      if ((true_cone_pos - true_car_pos).norm() < 6.0) {
        // Perfect LiDAR measurement in local frame
        Matrix2d R_inv;
        R_inv << cos(-true_heading), -sin(-true_heading), sin(-true_heading),
            cos(-true_heading);
        Vector2d local_obs = R_inv * (true_cone_pos - true_car_pos);
        local_observations.push_back(local_obs);
      }
    }

    // 4. Send to Frontend
    slam.process_observations(odom_pose, local_observations, t);
  }

  traj_file.close();
  slam.export_to_csv("map.csv");
  return 0;
}
