#include <vector>
#include <sys/time.h>
#include<geometry_msgs/Twist.h>
#include "sim_navigator/sim_navigator.hpp"

static const double MAX_LINEAR_SPEED = 0.5;
static const double MIN_LINEAR_SPEED = 0;
static const double MAX_ANGULAR_SPEED = 2.0;
static const double MIN_ANGULAR_SPEED = -2.0;

static const double STANDARD_LIN_SPEED = 0.3;

static const double FOLLOWING_DISTANCE = 0.75;
static const double NEAR_BLOCKED_DISTANCE = 0.75;
static const double FAR_BLOCKED_DISTANCE = 1.75;
static const double WALL_HUG_DISTANCE = (NEAR_BLOCKED_DISTANCE + FAR_BLOCKED_DISTANCE) / 2;

void FollowerNavigator::execute(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::vector<Point> points = laser_processor.get_points(scan);
  current_execute_time = get_current_time_millis();

  std::vector<HoughResult> maxima = laser_processor.hough_transform(points);
  std::vector<Line> lines = laser_processor.make_lines(maxima);

  ROS_INFO("---------------------------------------------------------");
  /* for (size_t i = 0; i < lines.size(); ++i) { */
  /*   Point start = lines[i].start_point; */
  /*   Point end = lines[i].end_point; */
  /*   /1* ROS_INFO("%f, (%f,%f)->(%f,%f)", lines[i].length, start.x, start.y, end.x, end.y); *1/ */
  /*   ROS_INFO("%f, (%f)->(%f)", lines[i].length, start.theta, end.theta); */
  /*   /1* ROS_INFO("%f, %f", lines[i].length, lines[i].angle); *1/ */
  /* } */

  if (laser_processor.find_leader(lines)) {
    Leader leader = laser_processor.get_leader();

    /* leader.distance *= std::sin(leader.angle * M_PI / 180); */
    ROS_INFO("leader.angle = %f", leader.angle);
    ROS_INFO("leader.distance = %f", leader.distance);

    leader.distance_error = (leader.distance) - FOLLOWING_DISTANCE;
    leader.angle_error = (leader.start_point.theta + leader.end_point.theta) / 2 - 90;

    ROS_INFO("LEADER_FOUND: direction:%f, distance:%f", leader.angle_error, leader.distance_error);

    linear_speed = execute_pd(leader.distance_error, LEADER_DISTANCE_P, LEADER_DISTANCE_D, leader_distance_previous_error, leader_distance_previous_time);
    angular_speed = execute_pd(leader.angle_error, LEADER_ANG_P, LEADER_ANG_D, leader_angle_previous_error, leader_angle_previous_time);
  }
  else {
    ROS_INFO("LEADER NOT FOUND");
    linear_speed = 0;
    angular_speed = 0;
  }

  /* geometry_msgs::Twist msg; */
  /* memset(&msg, 0, sizeof(msg)); */
  /* msg.linear.x = std::min(std::max(linear_speed, MIN_LINEAR_SPEED), MAX_LINEAR_SPEED); */
  /* msg.angular.z = std::min(std::max(angular_speed, MIN_ANGULAR_SPEED), MAX_ANGULAR_SPEED); */

  /* ROS_INFO("LIN_SPEED = %f, ANG_SPEED = %f", msg.linear.x, msg.angular.z); */

  /* publisher.publish(msg); */

  previous_linear_speed = linear_speed;
  previous_angular_speed = angular_speed;
  previous_execute_time = current_execute_time;
}

double FollowerNavigator::execute_pd(double current_error, const double &P, const double &D, double &previous_error, long &previous_time) {
  long current_time = get_current_time_millis();

  if (previous_time == 0) {
    previous_time = current_time;
  }

  long dt = current_time - previous_time;
  previous_time = current_time;

  double dx = current_error - previous_error;
  previous_error = current_error;

  return P * current_error + D * dx / dt;
}

long FollowerNavigator::get_current_time_millis() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

FollowerNavigator::FollowerNavigator(const ros::Publisher &pub) :
  publisher(pub), laser_processor(), linear_speed(0), angular_speed(0) {}
