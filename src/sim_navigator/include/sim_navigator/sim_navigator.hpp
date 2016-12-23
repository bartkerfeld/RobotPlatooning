#ifndef _FOLLOWER_NAVIGATOR_HPP_
#define _FOLLOWER_NAVIGATOR_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "sim_navigator/laser_processor.hpp"

class  FollowerNavigator {
 public:
  FollowerNavigator(const ros::Publisher& pub);
  void execute(const sensor_msgs::LaserScan::ConstPtr& scan);

 private:
  long get_current_time_millis();
  double execute_pd(double current_error, const double &P, const double &D, double &previous_error, long &previous_time);

  ros::Publisher publisher;
  LaserProcessor laser_processor;
  double linear_speed;
  double angular_speed;

  long current_execute_time = 0;
  long previous_execute_time = 0;
  double previous_linear_speed = 0;
  double previous_angular_speed = 0;

  int motionless_leader_count = 0;
  bool ignore_leader = false;

  bool leader_found_on_previous_iteration = false;
  double previous_leader_distance = 0;

  int num_times_without_finding_wall = 0;

  const double LEADER_DISTANCE_P = 0.75;
  const double LEADER_DISTANCE_D = -0.5;
  double leader_distance_previous_error = 0;
  long leader_distance_previous_time = 0;
  long ignore_leader_start_time = 0;

  const double LEADER_ANG_P = 0.02;
  const double LEADER_ANG_D = -0.1;
  double leader_angle_previous_error = 0;
  long leader_angle_previous_time = 0;

  const double WALL_DISTANCE_P = 0.05;
  const double WALL_DISTANCE_D = -0.25;
  double wall_distance_previous_error = 0;
  long wall_distance_previous_time = 0;

  const double WALL_ANG_P = 0.05;
  const double WALL_ANG_D = -0.25;
  double wall_angle_previous_error = 0;
  long wall_angle_previous_time = 0;
};

#endif
