#pragma once
#include <vector>
#include <utility>
#include "map.h"


std::vector<std::pair<double, double> > dijkstra_4dir( Map* theMap, 
	const std::pair<double, double>& start_pos, const std::pair<double, double>& end_pos);