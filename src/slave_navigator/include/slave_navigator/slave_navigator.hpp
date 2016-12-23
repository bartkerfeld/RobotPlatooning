#ifndef _SLAVE_NAVIGATOR_HPP_
#define _SLAVE_NAVIGATOR_HPP_

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_common/obstacle_processor.hpp"

class  SlaveNavigator {
 public:
  /* constructors */
  SlaveNavigator(
      const ros::Publisher& pub);
  void execute(
      const sensor_msgs::LaserScan::ConstPtr& scan);

 private:
  /* member functions */
  std::vector<Obstacle> get_obstacles(
      const std::vector<float> ranges);
  bool find_leader(
      std::vector<float> ranges);
  void execute_pid(
      double min_distance,
      int min_distance_angle,
      double* lin_speed,
      double* ang_speed);
  void determine_speeds(
      bool leader_found,
      bool left_near_blocked,
      bool right_near_blocked,
      bool left_far_blocked,
      bool center_getting_close);

  /* data members */
  const double FOLLOWING_DISTANCE = 0.5;
  const double MAX_LIN_SPEED = 0.5;
  const double MIN_LIN_SPEED = 0.0;
  const double MAX_ANG_SPEED = 0.5;
  const double MIN_ANG_SPEED = -0.5;
  const double LIN_P = 0.75;
  const double LIN_D = -0.5;
  const double ANG_P = 0.02;
  const double ANG_D = -0.1;

  double lin_speed_;
  double ang_speed_;
  int num_times_too_slow_;
  int prev_leader_theta_;
  Obstacle leader_;
  ros::Publisher publisher_;
  ObstacleProcessor obs_processor_;
};

/*******************************************************************************
 * Operater Definitions
 ******************************************************************************/

#endif /*  _SLAVE_NAVIGATOR_HPP_  */
