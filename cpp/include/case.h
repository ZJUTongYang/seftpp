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

#include "planner_wrapper.h"


void case_right(void) {
	//right case from 2-1 - 6-3
	std::string file_path = "../map/original_map.bmp";
	Map theMap(file_path);

	std::vector<std::pair<double, double>>  ran_base_point_vec = { {55,50} ,{25,20}, {76,31},{24,61}, {78,81} };
	std::vector<std::pair<double, double>>  ran_start_point_vec = { {60,26} ,{24,60}, {79,51},{29,45}, {51,69} };
	std::vector<std::pair<double, double>>  ran_end_point_vec = { {23,92} ,{88,40}, {41,52},{86,41}, {79,35} };

	int trible_case = 4;
	double bias = -0.5;
	std::ofstream ofs_sum;
	std::stringstream file_name_sum;
	file_name_sum << "cpp_planner_path_config" << ".csv";
	ofs_sum.open(file_name_sum.str(), std::ios::out);
	ofs_sum << "case_count" << "," << "phi_count" << "," << "phi1" << "," << "phi2" << "," << "base_point_x" << "," << "base_point_y" << ","
		<< "start_point_x" << "," << "start_point_y" << "," << "start_theta" << "," << "goal_point_x" << "," << "goal_point_y" << ","
		<< "path_length" << "," << "run_time" << "," << "node_expand" << std::endl;

	for (int case_count = 1; case_count < ran_base_point_vec.size(); case_count++) {

		//case_count = 4;

		std::cout << "---------------count " << case_count << "---------------------" << std::endl;
		theMap.ran_base_point.first = ran_base_point_vec[case_count].first + bias;
		theMap.ran_base_point.second = ran_base_point_vec[case_count].second + bias;

		theMap.ran_start_pose.first = ran_start_point_vec[case_count].first + bias;
		theMap.ran_start_pose.second = ran_start_point_vec[case_count].second + bias;

		theMap.ran_goal_pose.first = ran_end_point_vec[case_count].first + bias;
		theMap.ran_goal_pose.second = ran_end_point_vec[case_count].second + bias;


		theMap.max_cable_length = 80;// grids


		int inter_iter;
		if (case_count >= 1) {
			std::cout << "-----trible case-----" << std::endl;
			inter_iter = 3;
			//right 3.93 5.50


		}
		else {
			inter_iter = 1;
		}
		//back 2.36 3.93
		std::vector<double> f_x = { -1,-2,2,2,-2 };
		std::vector<double> f_y = { 0,-1,-1,1,1 };


		//rigth 3.93 5.50 
		//std::vector<double> f_x_r1 = { 0,1.5,-2,2,2,-2,-2,-1.5 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };

		//std::vector<double> f_x_r1 = { 0,1,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };


		std::vector<double> f_x_r1 = { 0,1,2,2,-2,-2,-1 };
		std::vector<double> f_y_r1 = { 0,-1,-1,1,1,-1,-1 };

		//right 3.73 5.70
		/*std::vector<double> f_x_r2 = { 0,1.5,-2, 2 ,2 ,-2, -2, -1.5 };
		std::vector<double> f_y_r2 = { 0, -1,-1,-1 ,1 , 1, -1, -1 };*/

		std::vector<double> f_x_r2 = { 0,1.5,-2,2,2,-2,-1 };
		std::vector<double> f_y_r2 = { 0,-1,-1,-1,1,1 ,-1 };



		//right 3.93 5.70
		//std::vector<double> f_x_r3 = { 0,1.5,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r3 = { 0,-1,-1,-1,1,1,-1,-1 };

		std::vector<double> f_x_r3 = { 0,1.5,-2, 2 ,2 ,-2, -1.5 };
		std::vector<double> f_y_r3 = { 0, -1,-1,-1 ,1 , 1,  -1 };



		/*	std::vector<double> f_x_l1 = { 0,-1,-2,-2,2,2,1.5 };
			std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };*/

			/*std::vector<double> f_x_l1 = { 0,0.5,-2,-2,2,2,1.5 };
			std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };*/


		std::vector<double> f_x_l1 = { 0,0.5,-2,-2,2,2,1.8 };
		std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };



		for (int j = 0; j < inter_iter; j++) {
			//j = 2;

			std::cout << "------- " << j << "------" << std::endl;
			if (case_count >= 1) {
				switch (j)
				{
				case(0): {
					//for left case

					f_x = f_x_r1;
					f_y = f_y_r1;
					break;
				};
				case(1): {
					f_x = f_x_r2;
					f_y = f_y_r2;
					break;
				};
				case(2): {
					f_x = f_x_r3;
					f_y = f_y_r3;
					break;
				};

				}
			}
			double phi_1 = atan2(f_y.back() - f_y[0], f_x.back() - f_x[0]);
			theMap.phi_1 = normalize_angle_positive(phi_1) + 0.05;

			double phi_2 = atan2(f_y[1] - f_y[0], f_x[1] - f_x[0]);
			theMap.phi_2 = normalize_angle_positive(phi_2) - 0.05;

			// Refine the robot footprint for collision checking
			std::vector<std::pair<double, double> > temp;

			//std::vector<std::pair<double, double> > footprint(theMap.footprint_.begin(), theMap.footprint_.end());
			std::vector<std::pair<double, double> > footprint;
			footprint.clear();
			for (unsigned int i = 0; i < f_x.size(); ++i)
			{
				footprint.push_back(std::pair<double, double>(f_x[i], f_y[i]));
			}

			footprint.insert(footprint.end(), footprint[0]);



			int n = 3;
			for (unsigned int i = 0; i < footprint.size() - 1; ++i)
			{
				for (unsigned int j = 0; j < n; ++j)
				{
					double tempx = footprint[i].first + 1.0*j / n * (footprint[i + 1].first - footprint[i].first);
					double tempy = footprint[i].second + 1.0*j / n * (footprint[i + 1].second - footprint[i].second);
					temp.push_back(std::pair<double, double>(tempx, tempy));
				}

			}
			theMap.footprint_.assign(temp.begin(), temp.end());

			theMap.init_footprint();
			std::cout << "phi_1:  " << theMap.phi_1 << std::endl;;
			std::cout << "phi_2 : " << theMap.phi_2 << std::endl;;


			// planner thePlanner;
			double run_time;
			double V_size;
			std::vector<Conf> resultant_path = planner_wrapper(&theMap, run_time, V_size);
			std::ofstream ofs2;
			std::stringstream file_name;
			file_name << "cpp_planner_path" << "_" << case_count << "_" << j << ".csv";
			ofs2.open(file_name.str(), std::ios::out);

			for (auto iter = resultant_path.begin(); iter != resultant_path.end(); ++iter) {
				ofs2 << iter->x << "," << iter->y << "," << iter->theta;
				ofs2 << "," << iter->obs_vertices.size() / 2 << "," << iter->mid_poses_x.size();
				for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
				{
					ofs2 << "," << *iter2;
				}
				for (int mid_i = 0; mid_i < iter->mid_poses_x.size(); mid_i++)
				{
					ofs2 << "," << iter->mid_poses_x[mid_i] << "," << iter->mid_poses_y[mid_i] << "," << iter->mid_poses_theta[mid_i];
				}
				ofs2 << std::endl;
			}

			ofs2.close();

			ofs_sum << case_count + 1 << "," << j + 1 << "," << theMap.phi_1 - 0.05 << "," << theMap.phi_2 + 0.05 << ",";
			ofs_sum << theMap.ran_base_point.first << "," << theMap.ran_base_point.second << ",";
			ofs_sum << theMap.ran_start_pose.first << "," << theMap.ran_start_pose.second << "," << resultant_path[1].theta << ",";
			ofs_sum << theMap.ran_goal_pose.first << "," << theMap.ran_goal_pose.second << ",";
			ofs_sum << pathLength(resultant_path) << ",";
			ofs_sum << run_time << ",";
			ofs_sum << V_size;
			ofs_sum << std::endl;

		}
	}

	ofs_sum.close();
}

void case_left(void) {
	//case from 1-1 to 1-3
	std::string file_path = "../map/original_map.bmp";
	Map theMap(file_path);

	std::vector<std::pair<double, double>>  ran_base_point_vec = { {81,45} };
	std::vector<std::pair<double, double>>  ran_start_point_vec = { {89,10} };
	std::vector<std::pair<double, double>>  ran_end_point_vec = { {42,72} };


	int trible_case = 4;
	double bias = -0.5;
	std::ofstream ofs_sum;
	std::stringstream file_name_sum;
	file_name_sum << "cpp_planner_path_config_left" << ".csv";
	ofs_sum.open(file_name_sum.str(), std::ios::out);
	ofs_sum << "case_count" << "," << "phi_count" << "," << "phi1" << "," << "phi2" << "," << "base_point_x" << "," << "base_point_y" << ","
		<< "start_point_x" << "," << "start_point_y" << "," << "start_theta" << "," << "goal_point_x" << "," << "goal_point_y" << ","
		<< "path_length" << "," << "run_time" << "," << "node_expand" << std::endl;

	for (int case_count = 0; case_count < ran_base_point_vec.size(); case_count++) {

		//case_count = 4;

		std::cout << "---------------count " << case_count << "---------------------" << std::endl;
		theMap.ran_base_point.first = ran_base_point_vec[case_count].first + bias;
		theMap.ran_base_point.second = ran_base_point_vec[case_count].second + bias;

		theMap.ran_start_pose.first = ran_start_point_vec[case_count].first + bias;
		theMap.ran_start_pose.second = ran_start_point_vec[case_count].second + bias;

		theMap.ran_goal_pose.first = ran_end_point_vec[case_count].first + bias;
		theMap.ran_goal_pose.second = ran_end_point_vec[case_count].second + bias;


		theMap.max_cable_length = 80;// grids


		int inter_iter;
		inter_iter = 3;
		//back 2.36 3.93
		std::vector<double> f_x = { -1,-2,2,2,-2 };
		std::vector<double> f_y = { 0,-1,-1,1,1 };


		//rigth 3.93 5.50 
		//std::vector<double> f_x_r1 = { 0,1.5,-2,2,2,-2,-2,-1.5 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };

		//std::vector<double> f_x_r1 = { 0,1,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };


		std::vector<double> f_x_r1 = { 0,1,2,2,-2,-2,-1 };
		std::vector<double> f_y_r1 = { 0,-1,-1,1,1,-1,-1 };

		//right 3.73 5.70
		/*std::vector<double> f_x_r2 = { 0,1.5,-2, 2 ,2 ,-2, -2, -1.5 };
		std::vector<double> f_y_r2 = { 0, -1,-1,-1 ,1 , 1, -1, -1 };*/

		std::vector<double> f_x_r2 = { 0,1.5,-2,2,2,-2,-1 };
		std::vector<double> f_y_r2 = { 0,-1,-1,-1,1,1 ,-1 };



		//right 3.93 5.70
		//std::vector<double> f_x_r3 = { 0,1.5,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r3 = { 0,-1,-1,-1,1,1,-1,-1 };

		std::vector<double> f_x_r3 = { 0,1.5,-2, 2 ,2 ,-2, -1.5 };
		std::vector<double> f_y_r3 = { 0, -1,-1,-1 ,1 , 1,  -1 };



		/*	std::vector<double> f_x_l1 = { 0,-1,-2,-2,2,2,1.5 };
			std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };*/

			//std::vector<double> f_x_l1 = { 0,0.2,-2,-2,2,2,1.8 };
			//std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };

		std::vector<double> f_x_l1 = { 0,0.5,-2,-2,2,2,1.8 };
		std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };


		for (int j = 0; j < inter_iter; j++) {
			//j = 2;

			std::cout << "------- " << j << "------" << std::endl;
			if (case_count >= 0) {
				switch (j)
				{
				case(0): {
					//for left case

				/*	f_x = f_x_r1;
					f_y = f_y_r1;*/
					break;
				};
				case(1): {
					f_x = f_x_r1;
					f_y = f_y_r1;
					break;
				};
				case(2): {
					f_x = f_x_l1;
					f_y = f_y_l1;
					break;
				};

				}
			}
			double phi_1 = atan2(f_y.back() - f_y[0], f_x.back() - f_x[0]);
			theMap.phi_1 = normalize_angle_positive(phi_1) + 0.05;

			double phi_2 = atan2(f_y[1] - f_y[0], f_x[1] - f_x[0]);
			theMap.phi_2 = normalize_angle_positive(phi_2) - 0.05;

			// Refine the robot footprint for collision checking
			std::vector<std::pair<double, double> > temp;

			//std::vector<std::pair<double, double> > footprint(theMap.footprint_.begin(), theMap.footprint_.end());
			std::vector<std::pair<double, double> > footprint;
			footprint.clear();
			for (unsigned int i = 0; i < f_x.size(); ++i)
			{
				footprint.push_back(std::pair<double, double>(f_x[i], f_y[i]));
			}

			footprint.insert(footprint.end(), footprint[0]);



			int n = 3;
			for (unsigned int i = 0; i < footprint.size() - 1; ++i)
			{
				for (unsigned int j = 0; j < n; ++j)
				{
					double tempx = footprint[i].first + 1.0*j / n * (footprint[i + 1].first - footprint[i].first);
					double tempy = footprint[i].second + 1.0*j / n * (footprint[i + 1].second - footprint[i].second);
					temp.push_back(std::pair<double, double>(tempx, tempy));
				}

			}
			theMap.footprint_.assign(temp.begin(), temp.end());

			theMap.init_footprint();
			std::cout << "phi_1:  " << theMap.phi_1 << std::endl;;
			std::cout << "phi_2 : " << theMap.phi_2 << std::endl;;


			// planner thePlanner;
			double run_time;
			double V_size;
			std::vector<Conf> resultant_path = planner_wrapper(&theMap, run_time, V_size);
			std::ofstream ofs2;
			std::stringstream file_name;
			file_name << "cpp_planner_path" << "_" << case_count << "_" << j << ".csv";
			ofs2.open(file_name.str(), std::ios::out);

			for (auto iter = resultant_path.begin(); iter != resultant_path.end(); ++iter) {
				ofs2 << iter->x << "," << iter->y << "," << iter->theta;
				ofs2 << "," << iter->obs_vertices.size() / 2 << "," << iter->mid_poses_x.size();
				for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
				{
					ofs2 << "," << *iter2;
				}
				for (int mid_i = 0; mid_i < iter->mid_poses_x.size(); mid_i++)
				{
					ofs2 << "," << iter->mid_poses_x[mid_i] << "," << iter->mid_poses_y[mid_i] << "," << iter->mid_poses_theta[mid_i];
				}
				ofs2 << std::endl;
			}

			ofs2.close();

			ofs_sum << case_count + 1 << "," << j + 1 << "," << theMap.phi_1 - 0.05 << "," << theMap.phi_2 + 0.05 << ",";
			ofs_sum << theMap.ran_base_point.first << "," << theMap.ran_base_point.second << ",";
			ofs_sum << theMap.ran_start_pose.first << "," << theMap.ran_start_pose.second << "," << resultant_path[1].theta << ",";
			ofs_sum << theMap.ran_goal_pose.first << "," << theMap.ran_goal_pose.second << ",";
			ofs_sum << pathLength(resultant_path) << ",";
			ofs_sum << run_time << ",";
			ofs_sum << V_size;
			ofs_sum << std::endl;

		}
	}

	ofs_sum.close();
}


void case_default(void) {
	//std::string file_path = "mymap3_matlab.png";

	//std::string file_path = "test.png";
	std::string file_path = "../map/original_map.bmp";
	Map theMap(file_path);

	//Map theMap(file_path);
	//std::vector<std::pair<double, double>>  ran_base_point_vec = { {55,50}, {81,45}, {14,61}, {16,18} ,{30,20} ,{30,20},{30,20},{76,31},{76,31},{76,31},{24,61},{24,61},{24,61},{78,81},{78,81},{78,81}};
	//std::vector<std::pair<double, double>>  ran_start_point_vec ={ {60,26}, {89,10}, {60,31}, {10,57} ,{24,60} ,{24,60},{24,60},{79,51},{79,51},{79,51},{29,45},{29,45},{29,45},{51,69},{51,69},{51,69} };
	//std::vector<std::pair<double, double>>  ran_end_point_vec = {  {23,92}, {42,72}, {68,84}, {64,11} ,{88,40} ,{88,40},{88,40},{41,52},{41,52},{41,52},{86,41},{86,41},{86,41},{79,35},{79,35},{79,35} };

	std::vector<std::pair<double, double>>  ran_base_point_vec = { {55,50}, {81,45}, {14,61}, {16,18} ,{25,20}, {76,31},{24,61}, {78,81} };
	std::vector<std::pair<double, double>>  ran_start_point_vec = { {60,26}, {89,10}, {60,31}, {10,57} ,{24,60}, {79,51},{29,45}, {51,69} };
	std::vector<std::pair<double, double>>  ran_end_point_vec = { {23,92}, {42,72}, {68,84}, {64,11} ,{88,40}, {41,52},{86,41}, {79,35} };


	int trible_case = 4;

	// 
	//std::cout << "theMap: xsize" << theMap.xsize_ << ", ysize: " << theMap.ysize_ << std::endl;

	//filp x and y
	 //theMap.ran_base_point = std::pair<double,double>(31, 98);
	 //theMap.ran_start_pose = std::pair<double, double> (134, 95);
	 //theMap.ran_goal_pose = std::pair<double, double> (108, 154);

	//theMap.ran_base_point = std::pair<double,double>(98, 31);
	//theMap.ran_start_pose = std::pair<double, double> (95, 134);
	//theMap.ran_goal_pose = std::pair<double, double> (154, 108);

	//real_world case 
	//theMap.ran_base_point = std::pair<double, double>(30.5, 97.5);
	//theMap.ran_start_pose = std::pair<double, double>(133.5, 94.5);
	//theMap.ran_goal_pose = std::pair<double, double>(107.5, 153.5);


	double bias = -0.5;

	std::ofstream ofs_sum;
	std::stringstream file_name_sum;
	file_name_sum << "cpp_planner_path_config" << ".csv";
	ofs_sum.open(file_name_sum.str(), std::ios::out);
	ofs_sum << "case_count" << "," << "phi_count" << "," << "phi1" << "," << "phi2" << "," << "base_point_x" << "," << "base_point_y" << ","
		<< "start_point_x" << "," << "start_point_y" << "," << "start_theta" << "," << "goal_point_x" << "," << "goal_point_y" << ","
		<< "path_length" << "," << "run_time" << "," << "node_expand" << std::endl;

	for (int case_count = 1; case_count < ran_base_point_vec.size(); case_count++) {

		//case_count = 4;

		std::cout << "---------------count " << case_count << "---------------------" << std::endl;
		theMap.ran_base_point.first = ran_base_point_vec[case_count].first + bias;
		theMap.ran_base_point.second = ran_base_point_vec[case_count].second + bias;

		theMap.ran_start_pose.first = ran_start_point_vec[case_count].first + bias;
		theMap.ran_start_pose.second = ran_start_point_vec[case_count].second + bias;

		theMap.ran_goal_pose.first = ran_end_point_vec[case_count].first + bias;
		theMap.ran_goal_pose.second = ran_end_point_vec[case_count].second + bias;







		theMap.max_cable_length = 80;// grids

		//for real world
		//std::vector<double> f_x = {5,-5,-5,-5,5,5,-5};
		//std::vector<double> f_y = {0,-5,5,-5,-5,5,5};

		int inter_iter;
		if (case_count >= 4) {
			std::cout << "-----trible case-----" << std::endl;
			inter_iter = 3;
			//right 3.93 5.50


		}
		else {
			inter_iter = 1;
		}
		//back 2.36 3.93
		std::vector<double> f_x = { -1,-2,2,2,-2 };
		std::vector<double> f_y = { 0,-1,-1,1,1 };


		//rigth 3.93 5.50 
		//std::vector<double> f_x_r1 = { 0,1.5,-2,2,2,-2,-2,-1.5 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };

		//std::vector<double> f_x_r1 = { 0,1,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r1 = { 0,-1,-1,-1,1,1,-1,-1 };


		std::vector<double> f_x_r1 = { 0,1,2,2,-2,-2,-1 };
		std::vector<double> f_y_r1 = { 0,-1,-1,1,1,-1,-1 };

		//right 3.73 5.70
		/*std::vector<double> f_x_r2 = { 0,1.5,-2, 2 ,2 ,-2, -2, -1.5 };
		std::vector<double> f_y_r2 = { 0, -1,-1,-1 ,1 , 1, -1, -1 };*/

		std::vector<double> f_x_r2 = { 0,1.5,-2,2,2,-2,-1 };
		std::vector<double> f_y_r2 = { 0,-1,-1,-1,1,1 ,-1 };



		//right 3.93 5.70
		//std::vector<double> f_x_r3 = { 0,1.5,-2,2,2,-2,-2,-1 };
		//std::vector<double> f_y_r3 = { 0,-1,-1,-1,1,1,-1,-1 };

		std::vector<double> f_x_r3 = { 0,1.5,-2, 2 ,2 ,-2, -1.5 };
		std::vector<double> f_y_r3 = { 0, -1,-1,-1 ,1 , 1,  -1 };



		/*	std::vector<double> f_x_l1 = { 0,-1,-2,-2,2,2,1.5 };
			std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };*/

		std::vector<double> f_x_l1 = { 0,0.5,-2,-2,2,2,1.5 };
		std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };


		//f_x = f_x_l1;
		//f_y= f_y_l1;

		//f_x = f_x_r1;
		//f_y = f_y_r1;

		//f_x_r1 = f_x_l1;
		//f_y_r1 = f_y_l1;
		for (int j = 0; j < inter_iter; j++) {
			//j = 2;

			std::cout << "------- " << j << "------" << std::endl;
			if (case_count >= 4) {
				switch (j)
				{
				case(0): {
					//for left case

					f_x = f_x_r1;
					f_y = f_y_r1;
					break;
				};
				case(1): {
					f_x = f_x_r2;
					f_y = f_y_r2;
					break;
				};
				case(2): {
					f_x = f_x_r3;
					f_y = f_y_r3;
					break;
				};

				}
			}
			double phi_1 = atan2(f_y.back() - f_y[0], f_x.back() - f_x[0]);
			theMap.phi_1 = normalize_angle_positive(phi_1) + 0.05;

			double phi_2 = atan2(f_y[1] - f_y[0], f_x[1] - f_x[0]);
			theMap.phi_2 = normalize_angle_positive(phi_2) - 0.05;

			// Refine the robot footprint for collision checking
			std::vector<std::pair<double, double> > temp;

			//std::vector<std::pair<double, double> > footprint(theMap.footprint_.begin(), theMap.footprint_.end());
			std::vector<std::pair<double, double> > footprint;
			footprint.clear();
			for (unsigned int i = 0; i < f_x.size(); ++i)
			{
				footprint.push_back(std::pair<double, double>(f_x[i], f_y[i]));
			}

			footprint.insert(footprint.end(), footprint[0]);



			int n = 3;
			for (unsigned int i = 0; i < footprint.size() - 1; ++i)
			{
				for (unsigned int j = 0; j < n; ++j)
				{
					double tempx = footprint[i].first + 1.0*j / n * (footprint[i + 1].first - footprint[i].first);
					double tempy = footprint[i].second + 1.0*j / n * (footprint[i + 1].second - footprint[i].second);
					temp.push_back(std::pair<double, double>(tempx, tempy));
				}

			}
			theMap.footprint_.assign(temp.begin(), temp.end());

			theMap.init_footprint();
			std::cout << "phi_1:  " << theMap.phi_1 << std::endl;;
			std::cout << "phi_2 : " << theMap.phi_2 << std::endl;;


			// planner thePlanner;
			double run_time;
			double V_size;
			std::vector<Conf> resultant_path = planner_wrapper(&theMap , run_time, V_size);
			std::ofstream ofs2;
			std::stringstream file_name;
			file_name << "cpp_planner_path" << "_" << case_count << "_" << j << ".csv";
			ofs2.open(file_name.str(), std::ios::out);

			for (auto iter = resultant_path.begin(); iter != resultant_path.end(); ++iter) {
				ofs2 << iter->x << "," << iter->y << "," << iter->theta;
				ofs2 << "," << iter->obs_vertices.size() / 2 << "," << iter->mid_poses_x.size();
				for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
				{
					ofs2 << "," << *iter2;
				}
				for (int mid_i = 0; mid_i < iter->mid_poses_x.size(); mid_i++)
				{
					ofs2 << "," << iter->mid_poses_x[mid_i] << "," << iter->mid_poses_y[mid_i] << "," << iter->mid_poses_theta[mid_i];
				}
				ofs2 << std::endl;
			}

			ofs2.close();

			ofs_sum << case_count + 1 << "," << j + 1 << "," << theMap.phi_1 - 0.05 << "," << theMap.phi_2 + 0.05 << ",";
			ofs_sum << theMap.ran_base_point.first << "," << theMap.ran_base_point.second << ",";
			ofs_sum << theMap.ran_start_pose.first << "," << theMap.ran_start_pose.second << "," << resultant_path[1].theta << ",";
			ofs_sum << theMap.ran_goal_pose.first << "," << theMap.ran_goal_pose.second << ",";
			ofs_sum << pathLength(resultant_path) << ",";
			ofs_sum << run_time << ",";
			ofs_sum << V_size;
			ofs_sum << std::endl;

		}
	}

	ofs_sum.close();
}



void case_real_world(void) {
	// case for real_world
	std::string file_path = "../map/original_map.bmp";
	Map theMap(file_path);

	int trible_case = 4;

	

	//real_world case 
	theMap.ran_base_point = std::pair<double, double>(30.5, 97.5);
	theMap.ran_start_pose = std::pair<double, double>(133.5, 94.5);
	theMap.ran_goal_pose = std::pair<double, double>(107.5, 153.5);


	
	//back 2.36 3.93
	std::vector<double> f_x = { -1,-2,2,2,-2 };
	std::vector<double> f_y = { 0,-1,-1,1,1 };


	//rigth 3.93 5.50 
	std::vector<double> f_x_r1 = { 0,1,2,2,-2,-2,-1 };
	std::vector<double> f_y_r1 = { 0,-1,-1,1,1,-1,-1 };

	//right 3.73 5.70
	std::vector<double> f_x_r2 = { 0,1.5,-2,2,2,-2,-1 };
	std::vector<double> f_y_r2 = { 0,-1,-1,-1,1,1 ,-1 };


	//right 3.93 5.70

	std::vector<double> f_x_r3 = { 0,1.5,-2, 2 ,2 ,-2, -1.5 };
	std::vector<double> f_y_r3 = { 0, -1,-1,-1 ,1 , 1,  -1 };





	std::vector<double> f_x_l1 = { 0,0.5,-2,-2,2,2,1.5 };
	std::vector<double> f_y_l1 = { 0,1,1,-1,-1,1,1 };


		
	//in real_world we set s back
	double phi_1 = atan2(f_y.back() - f_y[0], f_x.back() - f_x[0]);
	theMap.phi_1 = normalize_angle_positive(phi_1) + 0.05;

	double phi_2 = atan2(f_y[1] - f_y[0], f_x[1] - f_x[0]);
	theMap.phi_2 = normalize_angle_positive(phi_2) - 0.05;

	// Refine the robot footprint for collision checking
	std::vector<std::pair<double, double> > temp;

	//std::vector<std::pair<double, double> > footprint(theMap.footprint_.begin(), theMap.footprint_.end());
	std::vector<std::pair<double, double> > footprint;
	footprint.clear();
	for (unsigned int i = 0; i < f_x.size(); ++i)
	{
		footprint.push_back(std::pair<double, double>(f_x[i], f_y[i]));
	}

	footprint.insert(footprint.end(), footprint[0]);



	int n = 3;
	for (unsigned int i = 0; i < footprint.size() - 1; ++i)
	{
		for (unsigned int j = 0; j < n; ++j)
		{
			double tempx = footprint[i].first + 1.0*j / n * (footprint[i + 1].first - footprint[i].first);
			double tempy = footprint[i].second + 1.0*j / n * (footprint[i + 1].second - footprint[i].second);
			temp.push_back(std::pair<double, double>(tempx, tempy));
		}

	}
	theMap.footprint_.assign(temp.begin(), temp.end());

	theMap.init_footprint();
	std::cout << "phi_1:  " << theMap.phi_1 << std::endl;;
	std::cout << "phi_2 : " << theMap.phi_2 << std::endl;;


	// planner thePlanner;
	double run_time;
	double V_size;
	int case_count = 1;
	std::vector<Conf> resultant_path = planner_wrapper(&theMap, run_time, V_size);
		
}
