#include "MathOperations.h"

namespace math_operations{
	// [deg] -> [rad]
	double thetaDeg2Rad(double theta_deg){
		double theta_rad;
		if(theta_deg > 180.0){
			theta_rad = (theta_deg - 180.0) * M_PI / 180.0 - M_PI;
		}else{
			theta_rad = theta_deg * M_PI / 180.0;
		}

		return theta_rad;
	}

	// [rad] -> [deg]
	double thetaRad2Deg(double theta_rad){
		double theta_deg;
		if(theta_rad < 0.0){
			theta_deg = 180.0 + 180.0 * (M_PI + theta_rad) / M_PI;
		}else{
			theta_deg = 180.0 * theta_rad / M_PI;
		}

		return theta_deg;
	}

	// [rad, rad] -> [rad]
	double thetaSum(double value_1, double value_2){
		double theta = fmod(thetaRad2Deg(value_1) + thetaRad2Deg(value_2), 360);
   
    	return thetaDeg2Rad(theta);
	}

	// [rad, rad] -> [rad]
	double thetaDiff(double value_1, double value_2){
		double r;
		double deg_begin = thetaRad2Deg(value_1);
		double deg_goal = thetaRad2Deg(value_2);

		double d = fmod(abs(deg_begin - deg_goal), 360);
		if(d > 180){
			r = 360 - d;
		}else{
			r = d;
		}

		// Calculate sign
		if((deg_begin - deg_goal >= 0 && deg_begin - deg_goal <= 180) || (deg_begin - deg_goal <= -180 && deg_begin - deg_goal >=-360)){
			r *= -1;
		}

		return thetaDeg2Rad(r);
	}

	// [m, m, rad, m, m] -> [rad]
	double relativeBearing(double focalRbt_x, double focalRbt_y, double focalRbt_theta, double neighborRbt_x, double neighborRbt_y){
		double x, y, relBearing, sign;
		x = neighborRbt_x - focalRbt_x;
		y = neighborRbt_y - focalRbt_y;

		double angle = thetaRad2Deg(atan2(y, x));
		double foc_theta = thetaRad2Deg(focalRbt_theta);

		double d = fmod(abs(foc_theta - angle), 360);
		if(d > 180){
			relBearing = 360 - d;
		}else{
			relBearing = d;
		}

		// Calculate sign
		if((foc_theta - angle >= 0 && foc_theta - angle <= 180) || (foc_theta - angle <= -180 && foc_theta - angle >=-360)){
			sign = -1;
		}else{
			sign = 1;
		}

		relBearing *= sign;

		return thetaDeg2Rad(relBearing);
	}

} // namespace math_operations