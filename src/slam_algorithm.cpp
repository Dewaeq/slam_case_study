#include "slam_algorithm.hpp"

#include <fstream>
#include <iostream>
#include <ostream>

using namespace Eigen;

const double ASSOCIATION_THRESHOLD = 1.0; // meters

// transform a point that's relative to the car to the world origin
// DONT CHANGE
Vector2d Pose::transform_to_map(const Eigen::Vector2d &local_point) const {
  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta), sin(theta), cos(theta);
  return R * local_point + Eigen::Vector2d(x, y);
};

void SlamAlgorithm::process_observations(
    const Pose &odom_pose, const std::vector<Vector2d> &local_observations,
    double current_time) {

  for (const auto obs : local_observations) {
    Eigen::Vector2d global_obs = odom_pose.transform_to_map(obs);

    std::shared_ptr<Landmark> best_match = nullptr;
    double min_dist = ASSOCIATION_THRESHOLD;

    for (const auto &lm : global_map_) {
      double dist = (lm->position - global_obs).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_match = lm;
      }
    }

    if (best_match != nullptr) {
      // update existing landmark
      best_match->position = 0.8 * best_match->position + 0.2 * global_obs;
      best_match->times_seen++;
      std::cout << "[Time: " << current_time
                << "] Associated observation with cone ID: " << best_match->id
                << std::endl;
    } else {
      // create a new landmark
      auto new_lm = std::make_shared<Landmark>();
      new_lm->id = next_landmark_id_++;
      new_lm->position = global_obs;
      new_lm->times_seen = 1;
      new_lm->first_time_seen = current_time;
      global_map_.push_back(new_lm);
      std::cout << "[Time: " << current_time
                << "] Created NEW Cone ID: " << new_lm->id << std::endl;
    }
  }
}

// Helper method to save the map for the visualization script
// DONT CHANGE
void SlamAlgorithm::export_to_csv(const std::string &file_name) {
  std::ofstream file(file_name);

  file << "id,x,y" << std::endl;
  for (const auto &lm : global_map_) {
    file << lm->id << "," << lm->position.x() << "," << lm->position.y()
         << std::endl;
  }

  std::cout << "Map exported to " << file_name << " with " << global_map_.size()
            << " cones" << std::endl;
}
