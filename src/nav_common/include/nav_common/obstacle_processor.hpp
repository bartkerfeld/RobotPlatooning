#ifndef _OBSTACLE_PROCESSOR_HPP_
#define _OBSTACLE_PROCESSOR_HPP_

#include <deque>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_common/laser_observation.hpp"

struct Obstacle {
  bool operator<(const Obstacle &other) {
    int center_theta = std::abs(((theta_max - theta_min) / 2) - 90);
    int other_center_theta = std::abs(((other.theta_max - other.theta_min) / 2) - 90);
    return center_theta < other_center_theta;
  }

  int theta_min;
  int theta_max;
  int min_distance_theta;
  double average_distance;
  double min_distance;
  double width;
};

class ObstacleProcessor {
 public:
  /* constructors */
  ObstacleProcessor(
      double sector_size);

  bool sector_blocked(
      const std::vector<float> &ranges,
      int sector,
      float sector_max_distance);

  double min_theta_in_range(
      const std::vector<float> &ranges,
      int theta_low,
      int theta_high,
      float min_distance);

  bool sectors_blocked(
      const std::vector<float> &ranges,
      const std::vector<int> &sectors,
      float sector_max_distance);
  std::vector<Obstacle> get_obstacles(
      const std::vector<float> &ranges);

 private:
  /* member functions */

  /* data members */
  double sector_size_;
};

#endif /*  _OBSTACLE_PROCESSOR_HPP_  */
