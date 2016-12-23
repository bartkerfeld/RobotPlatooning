#ifndef _LASER_OBSERVATION_HPP_
#define _LASER_OBSERVATION_HPP_

#include <float.h>
#include <vector>

struct LaserObservation {
  LaserObservation() :
      sectors(3, true),
      min_obs_dist(FLT_MAX),
      n_nans(0) {}

  bool left_clear(void) const { return sectors[0]; }
  bool center_clear(void) const { return sectors[1]; }
  bool right_clear(void) const { return sectors[2]; }

  void set_left_blocked(void) { sectors[0] = false; }
  void set_center_blocked(void) { sectors[1] = false; }
  void set_right_blocked(void) { sectors[2] = false; }

  std::vector<bool> sectors;
  float min_obs_dist;
  int n_nans;
};

#endif /*  _LASER_OBSERVATION_HPP_  */
