#ifndef _OBSTACLE_HPP
#define _OBSTACLE_HPP value

#include <sensor_msgs/LaserScan.h>
#include <vector>

struct Point {
  double x;
  double y;
  double r;
  double theta;
};

struct HoughResult {
  HoughResult();
  double rho;
  double theta;
  int count;
  std::vector<Point> voters;
};

struct Line {
  double length;
  double distance;
  double min_distance;
  double angle;
  Point start_point;
  Point end_point;
};

struct LeftWall : public Line {
  double angle_error;
};

struct Leader : public Line {
  double angle_error;
  double distance_error;
};

class LaserProcessor {
 public:
  LaserProcessor();
  void print_laser_details(const sensor_msgs::LaserScan::ConstPtr &scan);
  std::vector<Point> get_points(const sensor_msgs::LaserScan::ConstPtr &scan);
  std::vector<HoughResult> hough_transform(const std::vector<Point> &points);
  std::vector<Line> make_lines(const std::vector<HoughResult> &hough_results);
  std::vector<Line> break_line(const HoughResult &hough_result);
  std::vector<Line> merge_line(const std::vector<Line> &lines);
  bool sector_blocked(const std::vector<Point> &points, const double &theta_min, const double &theta_max, const double &min_distance);
  bool find_left_wall(std::vector<Line> lines);
  bool find_leader(std::vector<Line> lines);
  LeftWall get_left_wall() { return left_wall; }
  Leader get_leader() { return leader; }

 private:
  Line get_line(const Point &start_point, const Point &end_point);
  void set_left_wall(const Line &line);
  void set_leader(const Line &line);
  Leader leader;
  LeftWall left_wall;
  double previous_leader_theta = 90;
  double to_degrees(const double &radians) {
    return radians * 180 / M_PI;
  }
  double to_radians(const double &degrees) {
    return degrees * M_PI / 180;
  }
};

#endif
