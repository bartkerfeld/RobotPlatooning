/*******************************************************************************
 * Name            : master_main.cpp
 * Project         : aero
 * Module          : master
 * Description     : Master navigator entry point
 * Creation Date   : Sat Nov  5 10:55:04 2016
 * Original Author : jharwell
 *
 ******************************************************************************/

#define _MASTER_MAIN_CPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "master_navigator/master_navigator.hpp"
#include <functional>
#include <memory>
#include <signal.h>
#include <geometry_msgs/Twist.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static ros::Publisher* publisher_g; /* global only because of signal handling */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
void signal_handler(int s) {
  geometry_msgs::Twist msg;
  memset(&msg, 0, sizeof(msg));

  /* publish a bunch of times to make sure it gets through */
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);

  exit(1);
} /* signal_handler */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
int main(int argc, char **argv)
{
  /* initialize ROS */
  ros::init(argc, argv, "master_navigator");

  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>(
      "RosAria/cmd_vel",1000);
  publisher_g = &publisher;

  MasterNavigator navigator = MasterNavigator(publisher,  // publisher
                                              1.5,   // sector_max_distance,
                                              0.3,   // linear
                                              0.4);  // angular

  /*
   * Because we are using the SICK toolbox wrapper, we can use /scan directly,
   * rather than having to use /RosAria/lms2xx_1_laserscan
   */
  ros::Subscriber scan_sub = nh.subscribe("/scan", 1,
                                         &MasterNavigator::prioritize_left,
                                         &navigator);
  /*
   * Set up catching Ctrl-C (among other signals, so that we can shutdown
   * cleanly)
   */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGKILL, signal_handler);

  ros::spin();
  return 0;
} /* main() */
