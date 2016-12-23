/*******************************************************************************
 * Name            : master_navigator.hpp
 * Project         : aero
 * Module          : Master navigator
 * Description     : Header file for master navigator class
 * Creation Date   : Wed Nov  2 19:42:25 2016
 * Original Author : jharwell
 *
 ******************************************************************************/

#ifndef _MASTER_NAVIGATOR_HPP_
#define _MASTER_NAVIGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <deque>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_common/obstacle_processor.hpp"

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class  MasterNavigator {
 public:
  /* constructors */
  MasterNavigator(
      ros::Publisher vel_publisher,
      double sector_max_distance,
      double max_lin_speed,
      double max_ang_speed);

  /* member functions */
  void prioritize_left(const sensor_msgs::LaserScan::ConstPtr& scan);

 private:
  /* member functions */

  /* data members */
  const double MAX_LIN_SPEED = 0.3;
  const double MIN_LIN_SPEED = 0.3;
  const double MAX_ANG_SPEED = 0.4;
  const double MIN_ANG_SPEED = -0.4;

  double sector_max_distance_;
  double lin_speed_;
  double ang_speed_;
  ObstacleProcessor obs_processor_;
  ros::Publisher vel_publisher_;
};

/*******************************************************************************
 * Operater Definitions
 ******************************************************************************/

#endif /*  _MASTER_NAVIGATOR_HPP_  */
