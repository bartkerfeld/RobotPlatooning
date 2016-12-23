#include <ros/ros.h>
#include <set>
#include "sim_navigator/laser_processor.hpp"

static double CUTOFF_DISTANCE = 3.0;
static double LEADER_WIDTH = 0.3;

/* HOUGH TRANSFORM PARAMETERS */
static double RHO_PRECISION = 0.005;
static double THETA_PRECISION = 0.25;
static int MIN_VOTE_COUNT = 10;

/*
 * Returns all laser scanner points within the cutoff distance
 */
std::vector<Point> LaserProcessor::get_points(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::vector<Point> points;

  Point tmp;
  double theta = 0.0;
  double distance;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    distance = scan->ranges[i];

    if (!std::isnan(distance) && distance < CUTOFF_DISTANCE) {
      tmp.theta = to_degrees(theta);
      tmp.r = distance;
      tmp.x = distance * std::cos(theta);
      tmp.y = distance * std::sin(theta);
      points.push_back(tmp);
    }

    theta += scan->angle_increment;
  }

  return points;
}

std::vector<HoughResult> LaserProcessor::hough_transform(const std::vector<Point> &points) {
  int rho_count = CUTOFF_DISTANCE / RHO_PRECISION;
  int theta_count = 360 / THETA_PRECISION;

  HoughResult **votes = new HoughResult*[theta_count];
  for (int i = 0; i < theta_count; ++i) {
    votes[i] = new HoughResult[rho_count] {};
  }

  /* Initialize Table */
  for (int t = 0; t < theta_count; ++t) {
    for (int r = 0; r < rho_count; ++r) {
      votes[t][r].count = 0;
      votes[t][r].theta = (double) t * THETA_PRECISION;
      votes[t][r].rho = (double) r * RHO_PRECISION;
    }
  }

  /* Apply Hough Transform */
  double rho;
  int rho_index = 0;
  int theta_index = 0;
  for (Point p : points) {
    for (double theta = -90; theta < 90; theta += THETA_PRECISION) {
      rho = p.x * std::cos(to_radians(theta)) + p.y * std::sin(to_radians(theta));
      rho_index = (rho + CUTOFF_DISTANCE) /  (2 * RHO_PRECISION);
      theta_index = (theta + 90) / THETA_PRECISION;
      votes[theta_index][rho_index].count++;
      votes[theta_index][rho_index].voters.push_back(p);
    }
  }

  /* Search for Maxima in Table */
  int t_min, r_min, t_max, r_max;
  std::vector<HoughResult> maxima;
  for (int t = 0; t < theta_count; ++t) {
    for (int r = 0; r < rho_count; ++r) {
      t_min = std::max(0, t - 1);
      r_min = std::max(0, r - 1);
      t_max = std::min(theta_count - 1, t + 1);
      r_max = std::min(rho_count - 1, r + 1);
      if (votes[t][r].count >= MIN_VOTE_COUNT &&
          votes[t][r].count >= votes[t_min][r_min].count &&
          votes[t][r].count >= votes[t_min][r].count  &&
          votes[t][r].count >= votes[t_min][r_max].count  &&
          votes[t][r].count >= votes[t][r_min].count  &&
          votes[t][r].count >= votes[t][r_max].count  &&
          votes[t][r].count >= votes[t_max][r_min].count  &&
          votes[t][r].count >= votes[t_max][r].count  &&
          votes[t][r].count >= votes[t_max][r_max].count) {
        maxima.push_back(votes[t][r]);
      }
    }
  }

  for (int i = 0; i < theta_count; ++i) {
    delete [] votes[i];
  }
  delete [] votes;

  return maxima;
}

/*
 * Returns collection of lines formed by analyzing the points in the Hough maxima
 */
std::vector<Line> LaserProcessor::make_lines(const std::vector<HoughResult> &hough_results) {
  std::vector<Line> lines;

  for (HoughResult h : hough_results) {
    std::vector<Line> b_lines = merge_line(break_line(h));

    for (Line l : b_lines) {
      lines.push_back(l);
    }
  }

  /* Remove lines that exists within other lines */
  auto outer_iter = lines.begin();
  auto inner_iter = lines.begin();
  bool removed = false;
  while (outer_iter != lines.end()) {
    removed = false;
    inner_iter = lines.begin();

    while (inner_iter != lines.end()) {
      if (inner_iter == outer_iter) {
        ++inner_iter;
        continue; 
      }

      if ((*outer_iter).start_point.theta >= (*inner_iter).start_point.theta &&
          (*outer_iter).end_point.theta <= (*inner_iter).end_point.theta) {
        removed = true;
        outer_iter = lines.erase(outer_iter);
        inner_iter = lines.begin();
        break;
      }

      ++inner_iter;
    }

    if (!removed) {
      ++outer_iter;
    }
  }

  return lines;
}

std::vector<Line> LaserProcessor::break_line(const HoughResult &hough_result) {
  std::vector<Line> lines;

  Line tmp;
  Point start_point = hough_result.voters[0];
  Point previous_point = start_point;

  size_t i = 0;
  for (Point current_point : hough_result.voters) {
    i++;
    double distance = std::sqrt(
                        std::pow((current_point.x - previous_point.x), 2) +
                        std::pow((current_point.y - previous_point.y), 2)
                      );

    if (distance > 0.10 || i == hough_result.voters.size()) {
      tmp = get_line(start_point, previous_point);
      tmp.distance = hough_result.rho;
      if (tmp.length > 0) {
        lines.push_back(tmp);
      }
      start_point = current_point;
    }

    previous_point = current_point;
  }

  return lines;
}

std::vector<Line> LaserProcessor::merge_line(const std::vector<Line> &lines) {
  std::vector<Line> merge_lines;

  if (lines.size() == 0) return merge_lines;

  Line previous = lines[0];
  Line tmp;

  size_t i = 0;
  for (Line l : lines) {
    i++;
    double distance = std::sqrt(
                        std::pow((previous.end_point.x - l.start_point.x), 2) +
                        std::pow((previous.end_point.y - l.start_point.y), 2)
                      );

    if (distance < 0.10 || i == lines.size()) {
      tmp = get_line(previous.start_point, l.end_point);
      tmp.distance = l.distance;
      merge_lines.push_back(tmp);
    }
  }

  return merge_lines;
}

bool LaserProcessor::find_leader(std::vector<Line> lines) {
  const double WIDTH_ERROR = .05;

  auto iter = lines.begin();
  while (iter != lines.end()) {
    Line line = *iter;
    if (line.length < LEADER_WIDTH - WIDTH_ERROR || line.length > LEADER_WIDTH + WIDTH_ERROR) {
         iter = lines.erase(iter); 
    }
    else {
      ++iter;
    }
  }

  if (lines.size() == 0) {
    return false;
  }

  set_leader(lines.back());

  for (Line line : lines) {
    if (line.start_point.theta <= previous_leader_theta && line.end_point.theta >= previous_leader_theta) {
      set_leader(line);
      break;
    }
  }

  return true;
}

/*
 * Checks points within angle range and returns true of any are within a certain distance
 */
bool LaserProcessor::sector_blocked(const std::vector<Point> &points, const double &theta_min, const double &theta_max, const double &min_distance) {
  int hit_count = 0;

  for (Point p : points) {
    if (p.theta >= theta_min && p.theta <= theta_max && p.r < min_distance) {
      hit_count++;
    }

    if (hit_count > 2) {
      return true;
    }
  }

  return false;
}

void LaserProcessor::set_left_wall(const Line &line) {
}

void LaserProcessor::set_leader(const Line &line) {
  previous_leader_theta = (line.start_point.theta + line.end_point.theta) / 2;

  leader.start_point = line.start_point;
  leader.end_point = line.end_point;
  leader.distance = line.distance;
  leader.angle = line.angle;
  leader.length = line.length;
}

Line LaserProcessor::get_line(const Point &start_point, const Point &end_point) {
  Line line;
  line.start_point = start_point;
  line.end_point = end_point;

  line.length = std::sqrt(
                std::pow((line.end_point.x - line.start_point.x), 2) +
                std::pow((line.end_point.y - line.start_point.y), 2)
              );

  double a = line.length;
  double b = line.end_point.r;
  double c = line.start_point.r;

  double A = line.end_point.theta - line.start_point.theta;
  double B = to_degrees(std::asin(std::sin(to_radians(A)) * b / a));
  double C = 180 - A - B;

  double angle;
  if (b < c) {
    angle = C;
  }
  else {
    angle = B;
  }

  double dtheta = 180 - line.end_point.theta;
  line.angle = angle - dtheta - 90;

  return line;
}

void LaserProcessor::print_laser_details(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_INFO("scan->angle_min: %f", scan->angle_min);
  ROS_INFO("scan->angle_max: %f", scan->angle_max);
  ROS_INFO("scan->angle_increment: %f", scan->angle_increment);
  ROS_INFO("scan->ranges.size(): %lu", scan->ranges.size());
}

LaserProcessor::LaserProcessor() : leader(), left_wall() {}
HoughResult::HoughResult() : rho(0), theta(0), count(0), voters() {}
