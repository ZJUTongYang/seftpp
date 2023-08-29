#pragma once
#include "map.h"

#define USE_IMPROVED_NODE_EXPANSION false

bool isSEFGuaranteed(Map* pMap, double steer, double dir, double dis,
	double o_x, double o_y, double Delta_x, double Delta_y, 
	double x_0, double y_0, double theta_0, 
	double x_1, double y_1, double theta_1);

bool isTLAGuaranteed(Map* pMap, double steer, double dir, double dis,
	double o_x, double o_y, double Delta_x, double Delta_y,
	double x_0, double y_0, double theta_0,
	double x_1, double y_1, double theta_1);
