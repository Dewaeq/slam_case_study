#pragma once

#include <Eigen/Dense>
#include <memory>

// In SLAM, we call the cones 'landmarks'
// DONT CHANGE
struct Landmark {
  int id;
  Eigen::Vector2d position;
  int times_seen;
  double first_time_seen;
};

// Store the position of the vehicle
// DONT CHANGE
struct Pose {
  double x;
  double y;
  double theta;

  // transform a point that's relative to the car to the world origin
  Eigen::Vector2d transform_to_map(const Eigen::Vector2d &local_point) const;
};

class SlamAlgorithm {
public:
  // DONT CHANGE here, but you *should* change this method inside
  // src/slam_algorithm.cpp
  void
  process_observations(const Pose &odom_pose,
                       const std::vector<Eigen::Vector2d> &local_observations,
                       double current_time);

  // Helper method to save the map for the visualization script
  // DONT CHANGE
  void export_to_csv(const std::string &file_name);

private:
  // DONT CHANGE
  std::vector<std::shared_ptr<Landmark>> global_map_;
  // DONT CHANGE
  int next_landmark_id_ = 0;
};
