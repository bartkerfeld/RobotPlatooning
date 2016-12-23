/*******************************************************************************
 * Name            : master_navigator.hpp
 * Project         : aero
 * Module          : navigator
 * Description     : Navigator node class
 * Creation Date   : Wed Nov  2 19:42:25 2016
 * Original Author : jharwell
 *
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "master_navigator/master_navigator.hpp"
#include <cfloat>

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
MasterNavigator::MasterNavigator(
    ros::Publisher vel_publisher,
    double sector_max_distance,
    double lin_speed,
    double ang_speed) :
    sector_max_distance_(sector_max_distance),
    lin_speed_(lin_speed),
    ang_speed_(ang_speed),
    obs_processor_(60),
    vel_publisher_(vel_publisher) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void MasterNavigator::prioritize_left(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
  geometry_msgs::Twist msg;
  memset(&msg,0,sizeof(msg));

  /*
   * Because each sector is 60 degrees, we have the following degree/sector
   * mapping:
   *
   * sector 0: 0 - 59 degrees
   * sector 1: 60 - 119 degrees
   * sector 2: 120 - 179 degrees
   */

  bool left_blocked_nom = obs_processor_.sector_blocked(scan->ranges, 2,
                                                        sector_max_distance_);

  bool center_blocked_nom = obs_processor_.sector_blocked(scan->ranges, 1,
                                                          sector_max_distance_);
  bool right_blocked_nom = obs_processor_.sector_blocked(scan->ranges, 0,
                                                         sector_max_distance_);

  bool right_blocked_close = obs_processor_.sector_blocked(
      scan->ranges, 2,
      sector_max_distance_/2);
  bool left_blocked_close = obs_processor_.sector_blocked(
      scan->ranges, 0,
      sector_max_distance_/2);

  /* The master is always moving forward by default */
  msg.linear.x = lin_speed_;

  /*
   * Anytime the left is open, turn left, UNLESS the center is blocked
   * (i.e. there is a wall in front of us). In that case do not turn left, as
   * that may result in you actually turning back the way you came.
   */
  if (!left_blocked_nom && !center_blocked_nom) {
    std::cout << "LEFT IS OPEN -> TURN LEFT" << std::endl;
    msg.angular.z = ang_speed_;
    msg.linear.x = lin_speed_ - 0.2;
  }
  else if (center_blocked_nom) {
    /*
     * If this is true, then the robot is facing some sort of obstacle (probably
     * a wall) orientated to the left, so reduce speed and course correct back
     * to the center
     */
    if (!left_blocked_close && right_blocked_close) {
      std::cout << "CENTER BLOCKED && ORIENTATED LEFT -> TURN RIGHT" << std::endl;
      msg.angular.z = -ang_speed_;
      msg.linear.x = lin_speed_ - 0.1;
    }

    /*
     * The robot is facing some sort of obstacle (probably
     * a wall) orientated to the right, so continue  current course of action
     * circumventing the obstacle
     */
    else {
      std::cout << "CENTER BLOCKED && ORIENTATED RIGHT -> NO ACTION" << std::endl;
    }
  }
  /*
   * This happens if you get to a dead end in a hallway--turn right to initiate
   * a U-turn and turn around
   */
  else if (center_blocked_nom && right_blocked_nom) {
    std::cout << "CENTER BLOCKED &&  LEFT_BLOCKED -> TURN RIGHT" << std::endl;
    msg.angular.z = ang_speed_;
    msg.linear.x = lin_speed_ - 0.1;
  } else {
    std::cout << "CENTER OPEN -> ALL AHEAD FULL" << std::endl;
  }

  vel_publisher_.publish(msg);
} /* MasterNavigator::prioritize_left() */
