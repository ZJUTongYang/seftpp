#include "sparsity.h"
#include "map.h"
#include "tool.h"

double max_cos_value_in_interval(double min_t, double max_t)
{
	min_t = normalize_angle(min_t);
	max_t = min_t + normalize_angle_positive(max_t - min_t);
	
	if (min_t >= M_PI / 2)
	{
		return cos(min_t);
	}
	else if (min_t < M_PI / 2 && max_t >= M_PI / 2)
	{
		return 1.0;
	}
	else
	{
		return cos(max_t);
	}
}

double min_cos_value_in_interval(double min_t, double max_t)
{
	return -max_cos_value_in_interval(min_t + M_PI, max_t + M_PI);
}

bool isSEFGuaranteed(Map* pMap, double steer, double dir, double dis,
	double o_x, double o_y, double Delta_x, double Delta_y,
	double x_0, double y_0, double theta_0,
	double x_1, double y_1, double theta_1)
{
	if (fabs(steer) < 0.001)
	{
		// This is a straight motion, which is always sparse
		return true;
	}

	double R = fabs(pMap->chassis_L_ / tan(steer)); // always positive
	double turning_angle = dis / R; // always positive

	// When the motion is circular, the parameter t in the paper is the angular movement. 
	double t_max = fabs(turning_angle);

	enum ALL_MOTION_TYPE { FR, FL, BR, BL };
	ALL_MOTION_TYPE the_motion_type;
	if (steer > 0 && dir > 0)
	{
		the_motion_type = FL;
	}
	else if (steer > 0 && dir < 0)
	{
		the_motion_type = BR;
	}
	else if (steer < 0 && dir > 0)
	{
		the_motion_type = FR;
	}
	else if (steer < 0 && dir < 0)
	{
		the_motion_type = BL;
	}

	double A, B, C, D;
	switch (the_motion_type)
	{
	case FR:
		A = o_x - x_0 - R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_x + B * R + B * Delta_y;
		D = A * R + A * Delta_y - B * Delta_x;
		if ((A*A + B * B)*(A*A + B * B) > C*C + D * D)
		{
			return true;
		}
		else
		{
			double lhs = (A*A + B * B) / sqrt(C*C + D * D);
			double varphi = atan2(D, C);
			double min_t = 0 - theta_0 - varphi;
			double max_t = t_max - theta_0 - varphi;
			double max_rhs = max_cos_value_in_interval(min_t, max_t);
			double min_rhs = min_cos_value_in_interval(min_t, max_t);
			if (lhs > max_rhs || lhs < min_rhs)
			{
				return true;
			}
		}
		break;
	case FL:
		A = o_x - x_0 + R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_x - B * R + B * Delta_y;
		D = A * R - A * Delta_y + B * Delta_x;
		if ((A*A + B * B)*(A*A + B * B) > C*C + D * D)
		{
			return true;
		}
		else
		{
			double lhs = (A*A + B * B) / sqrt(C*C + D * D);
			double varphi = atan2(D, C);
			double min_t = 0 + theta_0 - varphi;
			double max_t = t_max + theta_0 - varphi;
			double max_rhs = max_cos_value_in_interval(min_t, max_t);
			double min_rhs = min_cos_value_in_interval(min_t, max_t);
			if (lhs > max_rhs || lhs < min_rhs)
			{
				return true;
			}
		}
		break;
	case BR:
		// This is the inverse of FL
		A = o_x - x_0 + R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_x - B * R + B * Delta_y;
		D = A * R - A * Delta_y + B * Delta_x;
		if ((A*A + B * B)*(A*A + B * B) > C*C + D * D)
		{
			return true;
		}
		else
		{
			double lhs = (A*A + B * B) / sqrt(C*C + D * D);
			double varphi = atan2(D, C);
			
			double min_t = -t_max + theta_0 - varphi;
			double max_t = 0 + theta_0 - varphi;
			double max_rhs = max_cos_value_in_interval(min_t, max_t);
			double min_rhs = min_cos_value_in_interval(min_t, max_t);
			if (lhs > max_rhs || lhs < min_rhs)
			{
				return true;
			}
		}
		break;
	case BL:
		// This is the inverse of FR
		A = o_x - x_0 - R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_x + B * R + B * Delta_y;
		D = A * R + A * Delta_y - B * Delta_x;
		if ((A*A + B * B)*(A*A + B * B) > C*C + D * D)
		{
			return true;
		}
		else
		{
			double lhs = (A*A + B * B) / sqrt(C*C + D * D);
			double varphi = atan2(D, C);
			double min_t = -t_max - theta_0 - varphi;
			double max_t = 0 - theta_0 - varphi;
			double max_rhs = max_cos_value_in_interval(min_t, max_t);
			double min_rhs = min_cos_value_in_interval(min_t, max_t);
			if (lhs > max_rhs || lhs < min_rhs)
			{
				return true;
			}
		}
		break;
	}
	return false;
}

bool isTLAGuaranteed(Map* pMap, double steer, double dir, double dis,
	double o_x, double o_y, double Delta_x, double Delta_y,
	double x_0, double y_0, double theta_0,
	double x_1, double y_1, double theta_1)
{
	double R = fabs(pMap->chassis_L_ / tan(steer)); // always positive
	double turning_angle = dis / R; // always positive
	// When the motion is circular, the parameter t in the paper is the angular movement. 
	double t_max = fabs(turning_angle);

	enum ALL_MOTION_TYPE { STRAIGHT, FR, FL, BR, BL };
	ALL_MOTION_TYPE the_motion_type;

	if (fabs(steer) < 0.001)
	{
		// This is a straight motion, which is always sparse
		the_motion_type = STRAIGHT;
	}
	else if (steer > 0 && dir > 0)
	{
		the_motion_type = FL;
	}
	else if (steer > 0 && dir < 0)
	{
		the_motion_type = BR;
	}
	else if (steer < 0 && dir > 0)
	{
		the_motion_type = FR;
	}
	else if (steer < 0 && dir < 0)
	{
		the_motion_type = BL;
	}

	double A, B, C, D;
	double varphi, rhs, min_t, max_t, max_lhs, min_lhs;
	switch(the_motion_type)
	{
	case STRAIGHT: 
		rhs = o_x + o_y - x_0 * cos(theta_0) - y_0 * sin(theta_0) - Delta_x;
		if (0 > rhs || t_max < rhs)
			return true;
		break;
	case FR: 
		A = o_x - x_0 - R * sin(theta_0);
		B = o_y - y_0 + R * sin(theta_0);
		C = B * Delta_x - A * R - A * Delta_y;
		D = A * Delta_x - B * R - B * Delta_y;
		varphi = atan2(D, C);
		min_t = 0 - theta_0 - varphi;
		max_t = t_max - theta_0 - varphi;
		max_lhs = max_cos_value_in_interval(min_t, max_t);
		min_lhs = min_cos_value_in_interval(min_t, max_t);
		if (min_lhs > 0 || max_lhs < 0)
		{
			return true;
		}
		break;
	case FL: 
		A = o_x - x_0 + R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_y - A * R - B * Delta_x;
		D = A * Delta_x - B * R + B * Delta_y;
		varphi = atan2(D, C);
		min_t = 0 + theta_0 + varphi;
		max_t = t_max + theta_0 + varphi;
		max_lhs = max_cos_value_in_interval(min_t, max_t);
		min_lhs = min_cos_value_in_interval(min_t, max_t);
		if (min_lhs > 0 || max_lhs < 0)
		{
			return true;
		}
		break;
	case BR: 
		// the same as FL
		A = o_x - x_0 + R * sin(theta_0);
		B = o_y - y_0 + R * cos(theta_0);
		C = A * Delta_y - A * R - B * Delta_x;
		D = A * Delta_x - B * R + B * Delta_y;
		varphi = atan2(D, C);
		min_t = -t_max + theta_0 + varphi;
		max_t = 0 + theta_0 + varphi;
		max_lhs = max_cos_value_in_interval(min_t, max_t);
		min_lhs = min_cos_value_in_interval(min_t, max_t);
		if (min_lhs > 0 || max_lhs < 0)
		{
			return true;
		}
		break;
	case BL:
		// The same as FR
		A = o_x - x_0 - R * sin(theta_0);
		B = o_y - y_0 + R * sin(theta_0);
		C = B * Delta_x - A * R - A * Delta_y;
		D = A * Delta_x - B * R - B * Delta_y;
		varphi = atan2(D, C);
		min_t = -t_max - theta_0 - varphi;
		max_t = 0 - theta_0 - varphi;
		max_lhs = max_cos_value_in_interval(min_t, max_t);
		min_lhs = min_cos_value_in_interval(min_t, max_t);
		if (min_lhs > 0 || max_lhs < 0)
		{
			return true;
		}
		break;
	}
	return false;
}