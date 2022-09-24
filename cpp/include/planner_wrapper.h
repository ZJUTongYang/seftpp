#pragma once
#include "tool.h"
#include "planner.h"
#include "solver.h"
#include "map.h"
#include "definitions.h"
#include <iostream>
#include<fstream>
#include <string>
#include "math.h"
#include "dijkstra_4dir.h"

#include<cstdlib>
#include<ctime>


std::vector<Conf> planner_wrapper(Map* theMap,  double& run_time, double& V_size)
{
	std::pair<double, double> goal_loc = theMap->ran_goal_pose;

	std::pair<double, double>  start_loc = theMap->ran_base_point;
	//start_loc.first = round(theMap->ran_base_point.first);
	//start_loc.second = round(theMap->ran_base_point.second);

	std::pair<double, double> end_loc = theMap->ran_start_pose;
	//end_loc.first = round(theMap->ran_start_pose.first);
	//end_loc.second =  round(theMap->ran_start_pose.second);


		//std::cout << "test 3.1" << std::endl;

	// We find the initial tethered robot configuration
	// We use the naive dijkstra to generate a usable initial configuration

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



	std::vector<std::pair<double, double> > initial_path = dijkstra_4dir(theMap, start_loc, end_loc);

	std::cout << "initial path size" << initial_path.size() << std::endl;
	// std::vector<std::pair<int,int>> initial_path;

	//cv::namedWindow("map",cv::WINDOW_NORMAL);
	//cv::Mat img(theMap->ysize_, theMap->xsize_, CV_64FC1, cv::Scalar::all(0));

	//for(int i=0;i<theMap->ysize_;i++){
	//    for(int j=0;j<theMap->xsize_;j++){
	//        // std::cout<<i<<" "<<j<<std::endl;
	//        img.data[i*theMap->xsize_+j] = theMap->data_[(theMap->ysize_-1-i)*theMap->xsize_ + j];
	//    }
	//}
	if (initial_path.size() == 0) {
		std::cout << "empty initial_path,continue" << std::endl;
		std::vector<Conf> resulting_path;
		return resulting_path;
	}


	/*std::ofstream ofs;
	std::stringstream file_name;
	file_name << "cpp_planner_path" << "_" << case_count << ".csv";
	ofs.open(file_name.str(), std::ios::out);
	for (auto iter = resulting_path.begin(); iter != resulting_path.end(); ++iter) {
		ofs << iter->x << "," << iter->y << "," << iter->theta;
		for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
		{
			ofs << "," << *iter2;
		}
		ofs << std::endl;
	}*/

	//std::cout<<"point1"<<std::endl;

	//std::cout << "test 3.2" << std::endl;


	//std::vector<std::pair<double, double> > initial_cable;
	std::vector<double> initial_cable;
	/*initial_cable.clear();*/


	initial_cable.push_back(theMap->ran_base_point.first);
	initial_cable.push_back(theMap->ran_base_point.second);


	for (unsigned int i = 0; i < initial_path.size() - 1; ++i)
	{
		theMap->getCableState(initial_cable, std::pair<double, double>(initial_path[i].first, initial_path[i].second),
			std::pair<double, double>(initial_path[i + 1].first, initial_path[i + 1].second));
	}

	double temp[9][2] = { {-1,-1},{-1,0},{-1,1},{0,-1},{0,0},{0,1},{1,-1},{1,0},{1,1} };

	for (int i = 0; i < 9; i++) {
		int x, y;
		x = floor(theMap->ran_base_point.first) + temp[i][0];
		y = floor(theMap->ran_base_point.second) + temp[i][1];
		theMap->data_[y*theMap->xsize_ + x] = 254;

	}

	auto initial_end = initial_cable.back();

	int cable_size = initial_cable.size();
	//double start_phi  = atan2(initial_cable.back().second  - theMap->ran_start_pose.second, initial_cable.back().first - theMap->ran_start_pose.first);
	std::cout << "cable size" << cable_size << std::endl;

	double start_phi = atan2(initial_cable[cable_size - 1] - theMap->ran_start_pose.second, initial_cable[cable_size - 2] - theMap->ran_start_pose.first);

	//std::cout << "test 3.3" << std::endl;

	Conf start_pose;
	start_pose.index = 0;
	start_pose.x = theMap->ran_start_pose.first;
	start_pose.y = theMap->ran_start_pose.second;
	start_pose.theta = start_phi - (theMap->phi_1 + theMap->phi_2) / 2;
	start_pose.s = std::pair<double, double>(0, 0);
	start_pose.phi = start_phi;
	start_pose.cost = 0;
	double dx, dy;
	dx = theMap->ran_goal_pose.first - start_pose.x;
	dy = theMap->ran_goal_pose.second - start_pose.y;

	start_pose.h = sqrt(dx*dx + dy * dy);
	start_pose.steer = 0;
	start_pose.motion_direction = 1;
	start_pose.open = true;
	start_pose.fatherindex = -1;
	start_pose.obs_vertices = initial_cable;
	start_pose.s = theMap->polarRotateAndMoveToXy(start_pose.x, start_pose.y, start_pose.theta)[0];

	std::vector<Conf> resulting_path;
	std::vector<Conf> V;
	std::vector<std::vector<std::vector<Indices>>> I;

	//std::cout << "test 3.4" << std::endl;
	/*clock_t start, end;
	start = clock();*/

	planner(theMap, start_pose, goal_loc, resulting_path, V, I, run_time, V_size);
	/*end = clock();
	double endtime = (double)(end - start) / CLOCKS_PER_SEC;
	run_time = endtime;*/

	//std::cout << "test 3.5" << std::endl;

	/*double temp[9][2] = { {-1,-1},{-1,0},{-1,1},{0,-1},{0,0},{0,1},{1,-1},{1,0},{1,1} };*/

	for (int i = 0; i < 9; i++) {
		int x, y;
		x = floor(theMap->ran_base_point.first) + temp[i][0];
		y = floor(theMap->ran_base_point.second) + temp[i][1];
		theMap->data_[y*theMap->xsize_ + x] = 0;

	}
	/*V_size = V.size();*/

	return resulting_path;
}
