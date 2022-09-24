#ifndef _PLANNER_
#define _PLANNER_

#include <vector>
#include "definitions.h"
#include "map.h"

typedef std::vector<int> Indices;


void planner(Map* theMap, Conf start_pose, std::pair<double, double> goal_loc, std::vector<Conf>& resulting_path, std::vector<Conf>& V, std::vector<std::vector<std::vector<Indices> > > & I,double & run_time, double& COUNT);


#endif