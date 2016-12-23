#define _OBSTACLE_PROCESSOR_CPP_

#include "nav_common/obstacle_processor.hpp"
#include <iostream>

ObstacleProcessor::ObstacleProcessor(
    double sector_size) :
    sector_size_(sector_size) {}

/**
 * ObstacleProcessor::sector_blocked() - Determine if a given sector [0,n] is
 * blocked when considered at a specific distance. Blocked is determined by
 * getting 3 hits of blocked at <= target distance.
 *
 * RETURN:
 *      bool - TRUE if the condition is met, FALSE otherwise
 **/
bool ObstacleProcessor::sector_blocked(
    const std::vector<float> &ranges,
    int sector,
    float sector_max_distance)
{
  int hit_count = 0;

  int theta = sector_size_*sector;
  while (theta < (sector+1)*sector_size_+1) {
    if (ranges[theta] < sector_max_distance) {
      hit_count++;
      if (hit_count > 5) {
        return true;
      }
    }

    theta++;
  }

  return false;
} /* ObstacleProcessor::sector_blocked */

/**
 * ObstacleProcessor::sectors_blocked() - Determine if a set of sectors [n,m]
 * are blocked when considered at a specific distance (same for all sectors)
 *
 * RETURN:
 *      bool - TRUE if the condition is met, FALSE otherwise
 **/
bool ObstacleProcessor::sectors_blocked(
    const std::vector<float> &ranges,
    const std::vector<int>& sectors,
    float sector_max_distance)
{
  for (int i = sectors[0]; i <= sectors[sectors.size()-1]; ++i) {
    if (sector_blocked(ranges, i, sector_max_distance)) {
      return true;
    }
  } /* for(i..) */
  return false;
} /* ObstacleProcess::sectors_blocked() */

/**
 * ObstacleProcessor::min_theta_in_range() - Get the theta corresponding to the
 * ray of minimum distance from the robot to an obstacle within a scan
 *
 * RETURN:
 *      double - The minimum theta
 **/
double ObstacleProcessor::min_theta_in_range(
    const std::vector<float> &ranges,
    int theta_low,
    int theta_high,
    float min_distance)
{
  double min_distance_angle = -1;
  for (int theta = theta_low; theta < theta_high; ++theta) {
    if (std::isnan(ranges[theta])) continue;

    if (ranges[theta] < min_distance) {
      min_distance_angle = theta;
    }
  }
  return min_distance_angle;
}

/**
 * ObstacleProcessor::get_obstacles() - Scan the results of a laser scan,
 * looking for large discontinuities in values, which are assumed to correspond
 * to the start/end of obstacles.
 *
 * RETURN:
 *      double - The minimum theta
 **/
std::vector<Obstacle> ObstacleProcessor::get_obstacles(
    const std::vector<float> &ranges)
{
  std::vector<Obstacle> obstacles;

  double distance_total = ranges[0];

  double min_distance = 25;
  int min_distance_theta = -1;


  Obstacle tmp;
  tmp.theta_min = 0;
  for (int theta = 1; theta < 181; theta++) {
    double current_distance = ranges[theta];
    double prev_distance_diff = ranges[theta - 1];

    double distance_diff = current_distance - prev_distance_diff;

    if (current_distance < min_distance) {
      min_distance = current_distance;
      min_distance_theta = theta;
    }

    // Must be a new obstacle or end of scan
    if (std::fabs(distance_diff) > 0.03 || theta == 180) {
      tmp.theta_max = theta - 1;
      double length_start_angle = ranges[tmp.theta_min];
      double length_end_angle = ranges[tmp.theta_max];

      // See SAS trig
      tmp.width = (length_end_angle * length_end_angle)
                  + (length_start_angle * length_start_angle)
                  - 2 * length_start_angle * length_end_angle * std::cos((tmp.theta_max - tmp.theta_min) * M_PI / 180);
      tmp.width = std::sqrt(tmp.width);

      tmp.average_distance = distance_total / (tmp.theta_max - tmp.theta_min);
      obstacles.push_back(tmp);

      tmp.min_distance_theta = min_distance_theta;
      tmp.min_distance = min_distance;

      // Reset for next obstacle
      tmp.theta_min = theta;
      distance_total = current_distance;
      min_distance = 25;
      min_distance_theta = -1;
    }
    else {
      distance_total += current_distance;
    }
  }
  return obstacles;
}

#undef _OBSTACLE_PROCESSOR_CPP_
