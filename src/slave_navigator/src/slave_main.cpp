/*******************************************************************************
 * Name            : slave_main.cpp
 * Project         : aero
 * Module          : slave
 * Description     : Slave navigator entry point
 * Creation Date   : Sat Nov  5 10:55:04 2016
 * Original Author : bkerfeld
 *
 ******************************************************************************/

#define _SLAVE_MAIN_CPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "slave_navigator/slave_navigator.hpp"
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
  memset(&msg,0,sizeof(msg));

  /* publish a bunch of times to make sure it gets through */
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);

  exit(1);
} /* signal_handler */

int main(int argc, char **argv)
{
  /* initialize ROS */
  ros::init(argc, argv, "slave_navigator");

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<geometry_msgs::Twist>(
      "RosAria/cmd_vel",
      1000);
  publisher_g = &publisher;
  SlaveNavigator navigator = SlaveNavigator(publisher);

  /*
   * Because we are using the SICK toolbox wrapper, we can use /scan directly,
   * rather than having to use /RosAria/lms2xx_1_laserscan
   */
  ros::Subscriber scan_sub = n.subscribe("/scan", 1,
                                         &SlaveNavigator::execute,
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
