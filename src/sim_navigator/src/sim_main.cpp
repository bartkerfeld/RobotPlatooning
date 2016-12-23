#define _SLAVE_MAIN_CPP_

#include <signal.h>
#include <geometry_msgs/Twist.h>
#include "sim_navigator/sim_navigator.hpp"

static ros::Publisher* publisher_g;

void signal_handler(int s) {
  geometry_msgs::Twist msg;
  memset(&msg,0,sizeof(msg));

  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);
  publisher_g->publish(msg);

  exit(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim_navigator");
  ROS_INFO("SIM NAVIGATOR STARTED");

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<geometry_msgs::Twist>(
      "RosAria/cmd_vel",
      1000);

  publisher_g = &publisher;

  FollowerNavigator navigator = FollowerNavigator(publisher);
  /* ros::Subscriber scan_sub = n.subscribe("/RosAria/sim_lms2xx_1_laserscan", 1, */
  /*                                        &FollowerNavigator::execute, */
  /*                                        &navigator); */
  ros::Subscriber scan_sub = n.subscribe("/scan", 1,
                                         &FollowerNavigator::execute,
                                         &navigator);


  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGKILL, signal_handler);

  ros::spin();
  return 0;
}
