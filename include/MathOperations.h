#ifndef MATH_OPERATIONS_H_
#define MATH_OPERATIONS_H_

#include <math.h>

namespace math_operations{
	double thetaDeg2Rad(double theta_deg);
	double thetaRad2Deg(double theta_rad);
	double thetaSum(double value_1, double value_2);
	double thetaDiff(double value_1, double value_2);
	double relativeBearing(double focalRbt_x, double focalRbt_y, double focalRbt_theta, double neighborRbt_x, double neighborRbt_y); 
} // namespace math_operations
#endif