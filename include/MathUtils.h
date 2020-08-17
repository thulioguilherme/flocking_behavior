#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <math.h>

namespace math_utils {

  double heading2Rad(const double a);

  double heading2Deg(const double a);
  
  double headingDiff(const double a, const double b);

  double relativeBearing(const double focalRbt_x, const double focalRbt_y, const double focal_heading, const double neighborRbt_x, const double neighborRbt_y);
  
} // namespace math_utils

#endif