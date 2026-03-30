#pragma once

#include <Eigen/Dense>
#include <memory>

struct Landmark {
  int id;
  Eigen::Vector2d position;
  int times_seen;
  double first_time_seen;
};

struct Pose {
  double x;
  double y;
  double theta;

  Eigen::Vector2d transform_to_map(const Eigen::Vector2d &local_point) const;
};

class SlamAlgorithm {
public:
  void
  process_observations(const Pose &pose,
                       const std::vector<Eigen::Vector2d> &local_observations,
                       double current_time);

  // Helper method to save the map for the visualization script
  // DONT CHANGE
  void export_to_csv(const std::string &file_name);

private:
  std::vector<std::shared_ptr<Landmark>> global_map_;
  int next_landmark_id_ = 0;
};
