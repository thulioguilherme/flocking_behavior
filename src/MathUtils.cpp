#include "MathUtils.h"

namespace math_utils {

  /* heading2Rad() //{ */

  double heading2Rad(const double a) {
    if (a > 180) {
      return (a - 180) * M_PI / 180 - M_PI;
    } else {
      return a * M_PI / 180;
    }
  }

  //}

  /* heading2Deg() //{ */
  double heading2Deg(const double a) {
    if (a < 0) {
      return 180 + 180 * (M_PI + a) / M_PI;
    } else {
      return 180 * a / M_PI;
    }
  }

  //}
  
  //}

  /* headingDiff() //{ */

  double headingDiff(const double a, const double b) {
    double a_deg = heading2Deg(a), b_deg = heading2Deg(b);
    double d = fmod(abs(a_deg - b_deg), 360);
    double dif = a_deg - b_deg;
    double r;

    if (d > 180) {
      r = 360 - d;
    } else {
      r = d;
    }

    if ((dif >= 0 && dif <= 180) || (dif <= -180 && dif >= -360)) {
      r *= -1;
    }

    return heading2Rad(r);
  }

  //}

  /* relativeBearing() //{ */

  double relativeBearing(const double focalRbt_x, const double focalRbt_y, const double focalRbt_heading, const double neighborRbt_x,
                         const double neighborRbt_y) {
    double angle_between = atan2(neighborRbt_y - focalRbt_y, neighborRbt_x - focalRbt_x);

    return headingDiff(focalRbt_heading, angle_between);
  }

  //}
  
} // namespace math_utils