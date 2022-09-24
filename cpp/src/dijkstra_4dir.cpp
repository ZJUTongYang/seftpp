#include "dijkstra_4dir.h"
#include "map.h"
#include <vector>
#include <utility>
#include <queue>
#include <iostream>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<std::pair<double, double>> dijkstra_4dir( Map* theMap, 
	const std::pair<double, double>& start_loc, const std::pair<double, double>& goal_loc) {

	std::vector<std::pair<double, double> > initial_path;
	std::queue<std::pair<int, int>> pos_queue;
	pos_queue.push(std::pair<int, int>(floor(start_loc.first), floor(start_loc.second)));

	int* Mmap = theMap->data_;
	int xsize = theMap->xsize_;

	std::vector<int> father_index;
	father_index.resize(theMap->xsize_*theMap->ysize_, -1);

	std::vector<double> cost;
	cost.resize(theMap->xsize_*theMap->ysize_, 99999.0);
	cost[floor(start_loc.second)*xsize + floor(start_loc.first)] = 0;

	std::vector<std::pair<int, int>> sons;
	std::vector<double> theta_vec;

	while (1) {
		if (pos_queue.empty()) {
			std::cout << "empty queue" << std::endl;
			std::cout << "dijkstra failed" << std::endl;

			break;
		}

		int x = pos_queue.front().first;
		int y = pos_queue.front().second;

		//std::pair<int, int> cur_pos = pos_queue.front();
		// std::cout<<"cur pos"<<cur_pos(0)<<" "<<cur_pos(1)<<std::endl;
		pos_queue.pop();


		sons.clear();
		theta_vec.clear();
		std::pair<int, int> son_pos;

		son_pos.first = x - 1;
		son_pos.second = y;
		sons.push_back(son_pos);
		theta_vec.push_back(M_PI);

		son_pos.first = x;
		son_pos.second = y - 1;
		sons.push_back(son_pos);
		theta_vec.push_back(-M_PI/2);


		son_pos.first = x;
		son_pos.second = y + 1;
		sons.push_back(son_pos);
		theta_vec.push_back(M_PI/2);

		son_pos.first = x + 1;
		son_pos.second = y;
		sons.push_back(son_pos);
		theta_vec.push_back(0);




		// cout<<"x is"<<x<<"y is"<<y<<endl;
		// cout<<"end x is"<<end_pos[0]<<"end y is"<<end_pos[1]<<endl;

		if (x == floor(goal_loc.first) && y == floor(goal_loc.second)) {
			std::cout << "dijkstra:find_path" << std::endl;

			initial_path.clear();

			std::pair<int, int> cur;
			cur.first = x;
			cur.second = y;
			while (true) {
				//std::cout << "cur:[" << cur.first << "," << cur.second << "]" << std::endl;
				if (cur.first == floor(start_loc.first) && cur.second == floor(start_loc.second))
				{
					break;
				}
				int cur_index = cur.second * xsize + cur.first;
				int fatherindex = father_index[cur_index];
				int father_x = fatherindex % xsize;
				int father_y = (fatherindex - father_x) / xsize;

				initial_path.insert(initial_path.begin(), std::pair<double, double>(cur.first + 0.5, cur.second + 0.5));
				cur.first = father_x;
				cur.second = father_y;
			}
			//std::cout << "inside inital path" << initial_path.size() << std::endl;
			break;
		}


		for (int i = 0; i < sons.size(); i++) {
			std::pair<int, int> son_pos = sons[i];
			// cout<<"x "<<son_pos(0)<<"y "<<son_pos(1)<<endl;
			/*if (Mmap[son_pos.second * xsize + son_pos.first] == 254) {
				continue;
			}*/

			if (!theMap->isCollisionFree(son_pos.first, son_pos.second, theta_vec[i])) {
				continue;
			}

			if (cost[son_pos.second*xsize + son_pos.first] >= 0 && 
				cost[son_pos.second*xsize + son_pos.first] < 9998) 
			{
				// We are using BFS
				// If a node has been explored, then we cannot have a shortcut
				continue;
			}

			int son_index = son_pos.second *xsize + son_pos.first;
			int fatherindex = y * xsize + x;
			father_index[son_index] = fatherindex;
			cost[son_index] = cost[fatherindex] + 1;

			pos_queue.push(son_pos);

		}

	}
	return initial_path;
}