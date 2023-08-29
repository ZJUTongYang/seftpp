#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <ctime>
#include "tool.h"
#include "planner.h"
#include "map.h"
#include "definitions.h"
#include "math.h"
#include "dijkstra_4dir.h"
#include "case.h"
#include "sparsity.h"



std::vector<Conf> planner_wrapper(Map* theMap, double& run_time, double& V_size)
{
	std::pair<double, double> goal_loc = theMap->ran_goal_pose;

	std::pair<double, double>  start_loc = theMap->ran_base_point;

	std::pair<double, double> end_loc = theMap->ran_start_pose;

	//precheck
	if (!theMap->isCollisionFree(theMap->ran_start_pose.first, theMap->ran_start_pose.second, 0)) {
		std::cout << "StartPose is in collision\n";
		std::vector<Conf> resulting_path;
		return resulting_path;
	}

	if (!theMap->isCollisionFree(theMap->ran_goal_pose.first, theMap->ran_goal_pose.second, 0)) {
		std::cout << "EndPose is in collision\n";
		std::vector<Conf> resulting_path;
		return resulting_path;
	}

	// We find the initial tethered robot configuration
	// We use the naive dijkstra to generate a usable initial configuration
	std::vector<std::pair<double, double> > initial_path = dijkstra_4dir(theMap, start_loc, end_loc);

	//std::cout << "initial path size" << initial_path.size() << std::endl;
//	for (auto iter = initial_path.begin(); iter != initial_path.end(); ++iter)
//	{
//		std::cout << "[" << iter->first << ", " << iter->second << "], ";
//	}

	if (initial_path.size() == 0) {
		std::cout << "empty initial_path, return " << std::endl;
		std::vector<Conf> resulting_path;
		return resulting_path;
	}


	//std::ofstream ofs;
	//std::stringstream file_name;
	//file_name << "cpp_planner_path" << "_" << case_count << ".csv";
	//ofs.open(file_name.str(), std::ios::out);
	//for (auto iter = resulting_path.begin(); iter != resulting_path.end(); ++iter) {
	//	ofs << iter->x << "," << iter->y << "," << iter->theta;
	//	for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
	//	{
	//		ofs << "," << *iter2;
	//	}
	//	ofs << std::endl;
	//}


	// We estimate the local shortening of the path
	std::vector<double> old_tether_shape;
	old_tether_shape.resize(initial_path.size() * 2);
	for (unsigned int i = 0; i < initial_path.size(); ++i)
	{
		old_tether_shape[2 * i] = initial_path[i].first;
		old_tether_shape[i * 2 + 1] = initial_path[i].second;
	}

	std::vector<double> initial_cable;
	theMap->deformTether(old_tether_shape, initial_cable);
	std::cout << "We print the initial tether (including the robot's current pose)" << std::endl;
	int n = initial_cable.size() / 2;
	for (unsigned int i = 0; i < n; ++i)
	{
		std::cout << "[" << initial_cable[i * 2] << ", " << initial_cable[i * 2 + 1] << "], ";
	}
	std::cout << std::endl;

	// We set the base point to be an obstacle (but is not recorded into the obstacle list)
	theMap->setBasePointAsObstacle();

	// here the indices are not -1 and -2, because the robot center's position is still in the initial_cable variable
	int cable_size = initial_cable.size();
	double start_phi = atan2(initial_cable[cable_size - 3] - theMap->ran_start_pose.second, initial_cable[cable_size - 4] - theMap->ran_start_pose.first);

	Conf start_pose;
	start_pose.index = 0;
	start_pose.x = theMap->ran_start_pose.first;
	start_pose.y = theMap->ran_start_pose.second;
	start_pose.theta = normalize_angle(start_phi - (theMap->phi_1 + theMap->phi_2) / 2);

	std::cout << "[phi_1, phi_2, start_theta] = " << "[" << theMap->phi_1 << ", " << theMap->phi_2 << ", " << start_pose.theta << "]" << std::endl;
	// The initial tether-robot anchoring point should be calculated
	start_pose.s = theMap->polarRotateAndMoveS(start_pose.x, start_pose.y, start_pose.theta);

	// The initial phi also needs to be calculated
	// Theoretically, the initial tether state calculated as above cannot guarantee that the initial configuration is SEF. 
	// However, here we just want to give a starting configuration (in real application the initial tether shape is not optimised from a path but can be set by user) so it doesn't matter
	initial_cable.emplace_back(start_pose.s.first);
	initial_cable.emplace_back(start_pose.s.second);
	std::vector<double> calculated_cable_to_s;
	theMap->deformTether(initial_cable, calculated_cable_to_s);
	calculated_cable_to_s.pop_back();
	calculated_cable_to_s.pop_back();

	// We should double-check that the SEF constraint is OK
	int temp = calculated_cable_to_s.size();
	double calculated_phi = atan2(calculated_cable_to_s[temp - 1] - start_pose.s.second, calculated_cable_to_s[temp - 2] - start_pose.s.first);
	if (normalize_angle_positive(calculated_phi - start_pose.theta) > theMap->phi_1 &&
		normalize_angle_positive(calculated_phi - start_pose.theta) < theMap->phi_2)
	{
		std::cout << "The fake initial tether state calculated by 4-dir Dijkstra is SEF" << std::endl;
	}
	else
	{
		std::cout << "Error: we cannot use this start configuration because it is not SEF" << std::endl;
		std::vector<Conf> resulting_path;
		return resulting_path;
	}

	start_pose.phi = calculated_phi;
	start_pose.cost = 0;
	double dx, dy;
	dx = theMap->ran_goal_pose.first - start_pose.x;
	dy = theMap->ran_goal_pose.second - start_pose.y;

	start_pose.h = sqrt(dx*dx + dy * dy);
	start_pose.steer = 0;
	start_pose.motion_direction = 1;
	start_pose.open = true;
	start_pose.fatherindex = -1;
	start_pose.obs_vertices = calculated_cable_to_s;

	std::vector<Conf> resulting_path;
	std::vector<Conf> V;
	V.reserve(theMap->xsize_ * theMap->ysize_);

	// For each x, y, theta, we have a storage of the indices
	std::vector<std::vector<std::vector<Indices>>> I;

	planner(theMap, start_pose, goal_loc, resulting_path, V, I, run_time, V_size);

	return resulting_path;
}


int main(void) {

	std::string data_folder = "data_folder";
	Map* pMap = nullptr;
	double phi_1, phi_2;
	int footprint_subdivision;
	std::vector<std::pair<double, double> > footprint;

	enum all_case_num {_1, _2, _3};
	all_case_num case_num = _3;
	switch (case_num)
	{
	case _1:
		pMap = new Map(data_folder + "map\\original_map.bmp");
		pMap->robot_radius_ = 3;
		pMap->max_cable_length = 80;
		pMap->ran_base_point = {80.5, 44.5};
		pMap->ran_start_pose = {88.5, 9.5};
		pMap->ran_goal_pose = {41.5, 71.5};
		footprint = { {-1, 0}, {-2, -1}, {2, -1}, {2, 1}, {-2, 1} };
		footprint_subdivision = 3;
		break;
	case _2:
		pMap = new Map(data_folder + "map\\original_map.bmp");
		pMap->robot_radius_ = 3;
		pMap->max_cable_length = 80;
		pMap->ran_base_point = { 80.5, 44.5 };
		pMap->ran_start_pose = { 88.5, 9.5 };
		pMap->ran_goal_pose = { 41.5, 71.5 };		
		footprint = { {0, 0}, {1, -1}, {2, -1}, {2, 1}, {-2, 1}, {-2, -1}, {-1, -1}	};
		footprint_subdivision = 3;
		break;
	case _3:
		pMap = new Map(data_folder + "map\\original_map.bmp");
		pMap->robot_radius_ = 3;
		pMap->max_cable_length = 80;
		pMap->ran_base_point = { 80.5, 44.5 };
		pMap->ran_start_pose = { 88.5, 9.5 };
		pMap->ran_goal_pose = { 41.5, 71.5 };
		footprint = { {0, 0}, {0.5, 1}, {-2, 1}, {-2, -1}, {2, -1}, {2, 1}};
		footprint_subdivision = 3;
		break;
	}
	
	phi_1 = atan2(footprint.back().second - footprint[0].second, footprint.back().first - footprint[0].first);
	pMap->phi_1 = normalize_angle_positive(phi_1) + 0.05;
	phi_2 = atan2(footprint[1].second - footprint[0].second, footprint[1].first - footprint[0].first);
	pMap->phi_2 = normalize_angle_positive(phi_2) - 0.05;
	std::cout << "We show phi_1 and phi_2: " << pMap->phi_1 << ", " << pMap->phi_2 << std::endl;

	std::vector<std::pair<double, double> > refined_footprint;
	refined_footprint.reserve((footprint.size() + 1)*footprint_subdivision);
	for (unsigned int i = 0; i < footprint.size() - 1; ++i)
	{
		auto& prev_ver = footprint[i];
		auto& next_ver = footprint[i + 1];
		for (unsigned int j = 0; j < footprint_subdivision; ++j)
		{
			double temp_x = prev_ver.first + 1.0*j / footprint_subdivision * (next_ver.first - prev_ver.first);
			double temp_y = prev_ver.second + 1.0*j / footprint_subdivision * (next_ver.second - prev_ver.second);
			refined_footprint.emplace_back(std::pair<double, double>(temp_x, temp_y));
		}
	}
	pMap->footprint_.assign(refined_footprint.begin(), refined_footprint.end());
	pMap->init_footprint(); // initialize polar_footprint

	double run_time;
	double V_size;
	std::vector<Conf> resultant_path = planner_wrapper(pMap, run_time, V_size);
	
#if USE_IMPROVED_NODE_EXPANSION
	std::cout << std::endl << std::endl << "ours: " << std::endl;
#else
	std::cout << "prior work: " << std::endl;
#endif
	std::cout << "primitive path length: " << pMap->primitive_path_length_ << std::endl;
	std::cout << "validity checking resolution: " << pMap->validity_checking_resolution_ << std::endl;

	std::cout << "run_time: " << run_time << "s, number of expanded nodes: " << V_size << std::endl;
	double l = pathLength(resultant_path);
	std::cout << "path length: " << l << std::endl;
	
	std::cout << "number of dense/sparse/all validity checking and proportion: " 
			  << pMap->dense_checking_num_ << "/" << pMap->sparse_checking_num_  
			  //<< "/" << pMap->all_checking_num_ 
			  << std::endl;
	std::cout << "Proportion: " << 1.0*pMap->sparse_checking_num_ / (pMap->sparse_checking_num_ + pMap->dense_checking_num_) << std::endl;

	//// We need to visualise the final solution
	//std::ofstream of(data_folder + "result\\test.txt");
	//for (unsigned int i = 0; i < resultant_path.size(); ++i)
	//{
	//	of << resultant_path[i].x << " " << resultant_path[i].y << " " << resultant_path[i].theta << std::endl;
	//}
	//of.close();

	delete pMap;

}