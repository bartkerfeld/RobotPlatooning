#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "slave_navigator/slave_navigator.hpp"
#include <cfloat>
#include <sys/time.h>
#include <unistd.h>



SlaveNavigator::SlaveNavigator(const ros::Publisher& pub) :
  lin_speed_(0.0),
  ang_speed_(0.0),
  num_times_too_slow_(0),
  prev_leader_theta_(90),
  leader_(),
  publisher_(pub),
  obs_processor_(30) {}

/**
 * SlaveNavigator::execute() - Execute the slave following action
 *
 * RETURN:
 *      void - N/A
 **/
void SlaveNavigator::execute(
    const sensor_msgs::LaserScan::ConstPtr& scan)
{
  geometry_msgs::Twist msg;
  memset(&msg,0,sizeof(msg));

  /*
   * Because each sector is 30 degrees, we have the following degree/sector
   * mapping:
   *
   * sector 0: 0 - 29 degrees
   * sector 1: 30 - 59 degrees
   * sector 2: 60 - 89 degrees
   * sector 3: 90 - 119 degrees
   * sector 4: 120 - 149 degrees
   * sector 5: 150 - 179 degrees
   */
  bool left_near_blocked = obs_processor_.sectors_blocked(
      scan->ranges,
      std::vector<int>({3, 4, 5}), /* 90 - 180  degrees */
      0.5);
  bool right_near_blocked = obs_processor_.sectors_blocked(
      scan->ranges,
      std::vector<int>({0, 1, 2}), /* 0 - 90 degrees */
      0.5);
  bool left_far_blocked = obs_processor_.sectors_blocked(
      scan->ranges,
      std::vector<int>({0, 1}), /* 0 - 60 degrees */
      0.7);

  bool center_getting_close = obs_processor_.sectors_blocked(
      scan->ranges,
      std::vector<int>({3, 4}), /* 60 - 120 degrees */
      0.7);

  bool leader_found = find_leader(scan->ranges);
  if (leader_found) {
    ROS_INFO("LEADER FOUND [ theta_min = %d, theta_max = %d, distance = %f, width = %f ]",
             leader_.theta_min, leader_.theta_max, leader_.min_distance, leader_.width);
  }

  /* Figure out what to do, based on where the currently detected obstacles are */
  determine_speeds(leader_found,left_near_blocked, right_near_blocked,
                   left_far_blocked, center_getting_close);
  ROS_INFO("LINEAR SPEED: %f | ANGULAR SPEED: %f", lin_speed_, ang_speed_);

  /* DO IT! */
  msg.linear.x = lin_speed_;
  msg.angular.z = ang_speed_;

  publisher_.publish(msg);
} /* SlaveNavigator::execute() */

/**
 * SlaveNavigator::determine_speeds() - Figure out how to direct the robot,
 * based on current/past detected obstacles
 *
 * RETURN:
 *      void - N/A
 **/
void SlaveNavigator::determine_speeds(
    bool leader_found,
    bool left_near_blocked,
    bool right_near_blocked,
    bool left_far_blocked,
    bool center_getting_close)
{
/*
   * If the center is blocked, we are too close to something (either the leader
   * or a wall), so stop to avoid crashing.
   */
  if (left_near_blocked & right_near_blocked) {
    ROS_INFO("ROBOT TOO CLOSE: STOPPING");
    ang_speed_ = 0.0;
    lin_speed_ = 0.0;
  }
  /*
   * If this is true, then we are too close to the wall on the left. Maintain
   * linear speed, but add angular speed to move us away from the wall.
   */
  else if (left_near_blocked & !right_near_blocked) {
    ROS_INFO("TOO CLOSE TO LEFT, TURNING RIGHT");
    ang_speed_ = -0.3;
    lin_speed_ = 0.2;
  }
  else if (!left_near_blocked & right_near_blocked) {
    ROS_INFO("TOO CLOSE TO RIGHT, TURNING LEFT");
    ang_speed_ = 0.3;
    lin_speed_ = 0.2;
  }
  /*
   * If this is true we have the leader in front of us somewhat, but have been
   * inside our target distance to the leader for too long. So, run the PID
   * loop to get back to proper distance. This is a distinct case from the one
   * when we get too close to a stationary object (i.e. our speed is too low for
   * a long time because we are facing a wall).
   *
   * Note that even though the PID loop also modifies the orientation of the
   * slave relative to the leader, it is not considered in the condition to
   * execute the if().
   *
   * We assume that the center of the leader is the midpoint between the min/max
   * thetas reported. This is not a bad assumption, but is not the best either.
   */
  else if ((leader_found && num_times_too_slow_ < 40)) {
    ROS_INFO("RUNNING PID");
    prev_leader_theta_ = (leader_.theta_min + leader_.theta_max) / 2;
    execute_pid(leader_.min_distance,
                (leader_.theta_max + leader_.theta_min) / 2 ,
                &lin_speed_, &ang_speed_);
  }
  /*
   * If this is true, we have veered to the center of the hallway. Maintain
   * linear speed, but add angular speed to get back to proper distance.
   */
  else if (!left_far_blocked) {
    ROS_INFO("TOO FAR FROM LEFT, TURNING LEFT");
    ang_speed_ = 0.3;
    lin_speed_ = 0.2;
  }
  /*
   * If this is true, we have somehow gotten stuck on a stationary object. Don't
   * do anything.
   */
  else if (num_times_too_slow_ > 40) {
    ROS_INFO("LIKELY STUCK ON A STATIONARY OBJECT");
  }
  else if (center_getting_close) {
    ROS_INFO("APPROACHING OBJECT: SLOWING DOWN");
    lin_speed_ = 0.2;
  }
  /*
   * If none of these conditions match, we have lost the leader, so drive
   * forward at a speed > that of the leader and hope for the best.
   */
  else {
    ROS_INFO("CANNOT FIND LEADER: DRIVING FORWARD");
    lin_speed_ = 0.4;
  }

  /*
   * Track how often the robot is stationary (or nearly so). If this happens too
   * often, that means we are stuck on a wall or other immobile object.
   */
  if (lin_speed_ < .005) {
    num_times_too_slow_++;
  } else {
    num_times_too_slow_ = 0;
  }
  ROS_INFO("num_times_too_slow = %d", num_times_too_slow_);

  /* Limit robot linear/angular speed */
  lin_speed_ = std::min(MAX_LIN_SPEED,lin_speed_);
  lin_speed_ = std::max(MIN_LIN_SPEED,lin_speed_);

  ang_speed_ = std::min(MAX_ANG_SPEED,ang_speed_);
  ang_speed_ = std::max(MIN_ANG_SPEED,ang_speed_);
} /* SlaveNavigator::determine_speeds() */

/**
 * SlaveNavigator::execute_pid() - Execute the slave PID loop to minimize the
 * deviation from the target distance/angle to the master
 *
 * RETURN:
 *      void - N/A
 **/
void SlaveNavigator::execute_pid(
    double min_distance,
    int min_distance_angle,
    double* lin_speed,
    double* ang_speed)
{
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  static long prev_ms = ms;
  long dt = ms - prev_ms;
  prev_ms = ms;

  // Static variables hurt my head sometimes, just a sanity check to make sure
  // it is not always zero
  ROS_INFO("dt = %ld", dt);

  double distance_diff = min_distance - FOLLOWING_DISTANCE;

  static double prev_distance_diff = distance_diff;
  double dx = distance_diff - prev_distance_diff;
  ROS_INFO("dx = %f", dx);
  *lin_speed = LIN_P * distance_diff + LIN_D * dx / dt;

  double angle_diff = (double) min_distance_angle - 90.0;
  ROS_INFO("MIN_DISTANCE_ANGLE = %d", min_distance_angle);
  ROS_INFO("ANGLE_DIFF = %f", angle_diff);

  static double prev_angle_diff = angle_diff;
  double dtheta = angle_diff - prev_angle_diff;
  ROS_INFO("dtheta = %f", dtheta);
  *ang_speed = ANG_P * angle_diff + ANG_D * dtheta / dt;
}

/**
 * SlaveNavigator::find_leader() - Process the most recent laser scan to figure
 * out where the leader is (may be determined to be where it was as of the
 * previous scan)
 *
 * RETURN:
 *      bool - TRUE if the leader was found, FALSE otherwise
 **/
bool SlaveNavigator::find_leader(const std::vector<float> ranges)
{
  std::vector<Obstacle> obstacles = obs_processor_.get_obstacles(ranges);

  /*
   * Iterate through all obstacles found in the vicinity. If they are comparable
   * in size to the known size of the leader, and within a set distance, then
   * keep them in the list as candidates for the location of the
   * master. Otherwise, remove them.
   */
  std::vector<Obstacle>::iterator iter = obstacles.begin();
  while (iter != obstacles.end()) {
    Obstacle obs = *iter;
    if (std::isinf(obs.average_distance) || obs.width < 0.2 || obs.width > 0.4 ||
        obs.average_distance > 3) {
      iter = obstacles.erase(iter);
    } else {
      ++iter;
    }
  } /* while() */

  /* If obstacles is empty, then there are no leader-like objects within range */
  if (obstacles.size() == 0) {
    return false;
  }

  // Print all the potential leaders (usually should be only one)
  std::string msg = "";
  for (Obstacle ob : obstacles) {
    std::cout << "[" << ob.theta_min << ", " << ob.theta_max << ", " << ob.average_distance << ob.width << "], ";
  }
  std::cout << std::endl;

  // Choose the closest potential leader
  std::sort(obstacles.begin(), obstacles.end());
  leader_ = obstacles[0];

  /*
   * If the new min/max thetas corresponding to the leader-like obstacle are
   * "better" than what we have now (i.e. we are getting closer/in better
   * alignment with the leader), then replace where we think the leader is.
   */
  for (Obstacle potential_leader : obstacles) {
    if (potential_leader.theta_min <= prev_leader_theta_ &&
        potential_leader.theta_max >= prev_leader_theta_) {
      leader_ = potential_leader;
      break;
    }
  }

  return true;
} /* SlaveNavigator::find_leader() */
