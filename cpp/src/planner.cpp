#include "planner.h"
#include <vector>
#include "definitions.h"
#include <cmath>
#include "solver.h"
#include "map.h"
#include "tool.h"
#include <iostream>
#include <fstream>

#include<cstdlib>
#include<ctime>
using namespace std;



void newMove(double x, double y, double theta, int direction, 
    double MOTION_RESOLUTION, double steer, int n, std::vector<double>& x_list, 
    std::vector<double>& y_list, std::vector<double>& yaw_list)
{
    double L = 3.0;
    double distance = direction * MOTION_RESOLUTION;
    double temp = normalize_angle(distance*tan(steer)/L);
    x_list.clear();
    y_list.clear();
    yaw_list.clear();
    x_list.push_back(x);
    y_list.push_back(y);


    for(unsigned int i = 0; i <= n; ++i)
    {
        yaw_list.push_back(theta + i*temp);
    }
    for(unsigned int i = 1; i <= n; ++i)
    {
        x_list.push_back(x_list.back() + distance*cos(yaw_list[i-1]));
        y_list.push_back(y_list.back() + distance*sin(yaw_list[i-1]));
    }

    x_list.erase(x_list.begin());
    y_list.erase(y_list.begin());
    yaw_list.erase(yaw_list.begin());

}

bool check_mid_pose(Map* theMap,Conf son_node) {
	for (int i = 0; i < son_node.mid_poses_x.size(); i++) {
		Conf tempnode;
		int obs_size = son_node.obs_vertices.size();
		double new_o_x = son_node.obs_vertices[obs_size - 2];
		double new_o_y = son_node.obs_vertices[obs_size - 1];
		tempnode.s = theMap->polarRotateAndMoveS(son_node.mid_poses_x[i], son_node.mid_poses_y[i], son_node.mid_poses_theta[i]);
		tempnode.theta = son_node.mid_poses_theta[i];
		tempnode.phi = atan2(new_o_y - son_node.s.second, new_o_x - son_node.s.first);
		if (!isSEF(tempnode, theMap->phi_1, theMap->phi_2))
		{
			return false;
		}
	
	}
	return true;
}

bool calc_next_node(Map* theMap, Conf cur_node, double steer, double direction, Conf& son_node, std::pair<double, double> goal_loc)
{
    double arc_l = 3.0; // arc length measured by grids
    double MOTION_RESOLUTION = 0.9;
    double SB_COST = 5;
    double STEER_COST = 1.0;
    double STEER_CHANGE_COST = 1.0;

    int n = floor(arc_l / MOTION_RESOLUTION) + 1;

    double x = cur_node.x;
    double y = cur_node.y;
    double theta = cur_node.theta;

    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> yaw_list;


    newMove(x, y, theta, direction, MOTION_RESOLUTION, steer, n, x_list, y_list, yaw_list);

    for(unsigned int i = 0; i < x_list.size(); ++i)
    {

        //if(!theMap->isCollisionFree(x, y, theta))
		if (!theMap->isCollisionFree(x_list[i], y_list[i], yaw_list[i]))
        {
            return false;
        }
    }

    double added_cost;
    if(direction == cur_node.motion_direction)
    {
        added_cost = 0;
    }
    else
    {
        added_cost = SB_COST;
    }

    // Steer Penalty
    added_cost += STEER_COST * fabs(steer);

    // Steer change penalty
    added_cost += STEER_CHANGE_COST * fabs(cur_node.steer - steer);

    //double cost = cur_node.cost + added_cost + arc_l;
	double cost = cur_node.cost + added_cost + n * MOTION_RESOLUTION;

    son_node.index = -1;
    son_node.x = x_list.back();
    son_node.y = y_list.back();
    son_node.theta = yaw_list.back();
    son_node.s.first = -1;
    son_node.s.second = -1;
    son_node.phi = -1;
    son_node.cost = cost;
    double x_diff = son_node.x - goal_loc.first;
    double y_diff = son_node.y - goal_loc.second;
    son_node.h = sqrt( x_diff*x_diff + y_diff*y_diff);
    son_node.hindex.clear();
    son_node.steer = steer;
    son_node.motion_direction = direction;

    son_node.open = true;
    son_node.fatherindex = cur_node.index;
    son_node.obs_vertices.clear();
    son_node.mid_poses_x.assign(x_list.begin(), x_list.end());
    son_node.mid_poses_y.assign(y_list.begin(), y_list.end());
    son_node.mid_poses_theta.assign(yaw_list.begin(), yaw_list.end());

    // We only need to verify the SEF of the last pose
    std::pair<double, double> old_s;
    old_s.first = cur_node.s.first;
    old_s.second = cur_node.s.second;

    son_node.s = theMap->polarRotateAndMoveS(x_list.back(), y_list.back(), yaw_list.back());
	
	std::vector<double> son_node_obs_vertices_temp(cur_node.obs_vertices.begin(), cur_node.obs_vertices.end());
	theMap->getCableState(son_node_obs_vertices_temp, old_s, son_node.s);
	son_node.obs_vertices = son_node_obs_vertices_temp;

    //son_node.obs_vertices = theMap->getCableState(cur_node.obs_vertices, old_s, son_node.s);

   // std::pair<double, double> new_o = son_node.obs_vertices.back();

	int obs_size = son_node.obs_vertices.size();
	
	double new_o_x = son_node.obs_vertices[obs_size - 2];
	double new_o_y = son_node.obs_vertices[obs_size - 1];


    son_node.phi = atan2(new_o_y - son_node.s.second, new_o_x - son_node.s.first);

	if (!check_mid_pose(theMap, son_node)) {
		return false;
	}

    if(!isSEF(son_node, theMap->phi_1, theMap->phi_2))
    {
        return false;
    }

    // max_cable_length
    // We check the maximum cable length

    //std::vector<std::pair<double, double> > temp(son_node.obs_vertices.begin(), son_node.obs_vertices.end());
	std::vector<double> temp(son_node.obs_vertices.begin(), son_node.obs_vertices.end());

    temp.insert(temp.end(), son_node.s.first);
	temp.insert(temp.end(), son_node.s.second);

    if(pathLength(temp) >= theMap->max_cable_length)
    {
        return false;
    }

    // We construct the hindex of the algorithm
    std::vector<int> cur_hindex = cur_node.hindex;
    temp.clear();
    //temp.push_back(std::pair<double, double>(cur_node.x, cur_node.y));
	temp.push_back(cur_node.x);
	temp.push_back(cur_node.y);

   /* for(unsigned int i = 0; i < son_node.mid_poses_x.size(); ++i)
    {
        temp.push_back(std::pair<double, double>(son_node.mid_poses_x[i], son_node.mid_poses_y[i]));
    }

    temp.push_back(std::pair<double, double>(son_node.x, son_node.y));

    for(unsigned int i = 0; i < temp.size()-1; ++i)
    {
        cur_hindex = calSwing(cur_hindex, std::pair<double, double>(temp[i].first, temp[i].second), 
            std::pair<double, double>(temp[i+1].first, temp[i+1].second), theMap->g_obs);
    }*/


    for(unsigned int i = 0; i < son_node.mid_poses_x.size(); ++i)
    {
		temp.push_back(son_node.mid_poses_x[i]);
		temp.push_back(son_node.mid_poses_y[i]);
    }

	temp.push_back(son_node.x);
	temp.push_back(son_node.y);


	int n_temp = temp.size() / 2;
   for(unsigned int i = 0; i < n_temp-1; ++i)
   {
	   cur_hindex = calSwing(cur_hindex, std::pair<double, double>(temp[2*i], temp[2*i+1]),
		   std::pair<double, double>(temp[2*(i+1)], temp[2*(i+1)+1]), theMap->g_obs);
   }


    son_node.hindex = cur_hindex;

    return true;

}

void get_neighbors(Map* theMap, Conf cur_node, std::vector<Conf>& sons_node, std::pair<double, double> goal_loc)
{
    double MIN_STEER = -1;
    double MAX_STEER = 1;
    // const int N_STEER = 11;
    // std::vector<double> STEER;
    // for(unsigned int i = 0; i <= N_STEER; ++i)
    // {
    //     STEER.push_back(MIN_STEER + i * (MAX_STEER - MIN_STEER) / (N_STEER - 1));
    // }

    std::vector<double> STEER = {-1.0000, -0.8000, -0.6000, -0.4000, -0.2000, 0, 0.2000, 0.4000, 0.6000, 0.8000, 1.0000};

    std::vector<int> D = {-1, 1};

    sons_node.clear();
    sons_node.reserve(STEER.size()*D.size());
    for(unsigned int i = 0; i < STEER.size(); ++i)
    {
        for(unsigned int j = 0; j < D.size(); ++j)
        {
            Conf son_node_temp;

            bool b = calc_next_node(theMap, cur_node, STEER[i], D[j], son_node_temp, goal_loc);
            if(b)
                sons_node.push_back(son_node_temp);
        }
    }
}

bool update_node_with_analytic_expansion(Conf cur_node, const std::pair<double, double>& goal_loc)
{
    double dx = goal_loc.first - cur_node.x;
    double dy = goal_loc.second - cur_node.y;
    double dis = sqrt(dx*dx + dy*dy);
    if(dis < 2)
        return true;

    return false;

}

void planner(Map* theMap, Conf start_pose, std::pair<double, double> goal_loc, std::vector<Conf>& resulting_path, 
    std::vector<Conf>& V, std::vector<std::vector<std::vector<Indices> > > & I,double& run_time,double& COUNT)
{

    auto comp = [](const std::pair<int, double>& a, const std::pair<int, double>& b)
        {return a.second > b.second;};

    V.clear();
    V.push_back(start_pose);
    int xnum = theMap->xsize_;
    int ynum = theMap->ysize_;
    int thetanum = 60;

    int x_gridsize = 1;// Should be the physical size of each grid measured in meters. Here we still use 1
    int y_gridsize = 1;
    double theta_gridsize = 2 * M_PI / thetanum;
    
    // We initialize I
    Indices single_temp;
    std::vector<Indices> row_temp;
    row_temp.resize(thetanum, single_temp);
    std::vector<std::vector<Indices> > two_temp;
    two_temp.resize(ynum, row_temp);
    I.resize(xnum, two_temp);

    std::vector<std::pair<int, double> > queue_;
    double dx = goal_loc.first - start_pose.x;
    double dy = goal_loc.second - start_pose.y;
    double h = sqrt(dx*dx+dy*dy);
    queue_.push_back(std::pair<int, double>(0, h));

    resulting_path.clear();

	std::ofstream ofs;
	ofs.open("cpp_planner_print.txt", std::ios::out);


	int count = 0;
	clock_t start, end;
	start = clock();
	bool find_path = false;
	//while(count++ <= 10000)
	COUNT = 0;
    while(1)
    {
        if(queue_.empty())
        {
            break;
        }

        // We find the least-cost motion
        auto top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), comp);
        queue_.pop_back();

        int cur_index = top.first;

        V[cur_index].open = false;

        Conf cur_node = V[cur_index];

        bool is_updated = update_node_with_analytic_expansion(cur_node, goal_loc);

        if(is_updated)
        {
            std::cout << "Path Found" << std::endl;
            std::cout << "Number of nodes: " << V.size() << std::endl;
			std::cout << "Cost of last node£º" << cur_node.cost << std::endl;
			find_path = true;
            resulting_path = tracePath(V, cur_node.index);
			break ;
        }

        // Here we calculate all elements of sons, 
        // Except their indices, because we haven't sure whether they will be added into the storage
        std::vector<Conf> sons;
        get_neighbors(theMap, cur_node, sons, goal_loc);

		
        for(unsigned int i = 0; i < sons.size(); ++i)
        {
            Conf son = sons[i];
			COUNT++;
            // Here the Hindex is the h signature of the cable AND THE ROBOT'S CENTER, 
            // or else different cable states do not share a common end
            std::vector<int> newHindex = son.hindex;

            double newcost = son.cost;// This cost is actually g-cost

            double dx = son.x - goal_loc.first;
            double dy = son.y - goal_loc.second;
            double hcost = sqrt(dx*dx + dy*dy);

            // Find the index of the son
            int newsonindex = V.size();
            son.index = newsonindex;

            int son_x_index;
            int son_y_index;
            int son_theta_index;
            calc_index(son, x_gridsize, y_gridsize, theta_gridsize, son_x_index, son_y_index, son_theta_index);

            if(I[son_x_index][son_y_index][son_theta_index].empty() || 
                !isEqualHC(newHindex, V, 
					I[son_x_index][son_y_index][son_theta_index]))
            {
                // If the voxel hasn't been visited
                // If we find non-homotopic paths to the same child voxel, 
                // We preserve them both

                queue_.push_back(std::pair<int, double>(newsonindex, newcost + hcost));
                std::push_heap(queue_.begin(), queue_.end(), comp);

                V.push_back(son);
                I[son_x_index][son_y_index][son_theta_index].push_back(newsonindex);
            }
            else
            {
                // We can find the equivalent Hindex in the I matrix
                // so we may carry out a shortcut
                int oldindex = findEqualHCNode(newHindex, V, I[son_x_index][son_y_index][son_theta_index]);

                if(V[oldindex].cost <= newcost)
                {
                    continue;
                }

     //           // If the old node is open, then it is in the queue
     //           if(V[oldindex].open)
     //           {
     //               int loc = std::find_if(queue_.begin(), queue_.end(), 
     //                   [&](const std::pair<int, double>& a){return a.first == oldindex;}
     //                   ) - queue_.begin();
     //               V[oldindex].open = false;
     //               queue_[loc].first = newsonindex;
     //               queue_[loc].second = newcost + hcost;
     //               std::make_heap(queue_.begin(), queue_.end(), comp);

					//V.push_back(son);
     //           }
     //           else
     //           {
     //               queue_.push_back(std::pair<int, double>(newsonindex, newcost + hcost));
     //               std::push_heap(queue_.begin(), queue_.end(), comp);

					//V.push_back(son);
     //           }

				if (V[oldindex].open == false)
				{
					queue_.push_back(std::pair<int, double>(oldindex, newcost + hcost));
					std::push_heap(queue_.begin(), queue_.end(), comp);
				}
				else
				{
					int loc = std::find_if(queue_.begin(), queue_.end(), 
						[&](const std::pair<int, double>& a){return a.first == oldindex;}
						) - queue_.begin();
					queue_[loc].second = newcost + hcost;
	                std::make_heap(queue_.begin(), queue_.end(), comp);
				}

				V[oldindex] = son;
				V[oldindex].index = oldindex;


            }
        }
    } // main while
	end = clock();
	double endtime = (double)(end - start) / CLOCKS_PER_SEC;
	run_time = endtime;
	cout << "Total time:" << endtime << endl;		
	cout << "Total time:" << endtime * 1000 << "ms" << endl;	

	if (!find_path) {
		std::cout << "Fail to find a path. We return" << std::endl;
		std::cout << "Vertices size" << COUNT << std::endl;
	}
    //resulting_path.clear();

	for (auto iter = V.begin(); iter != V.end(); ++iter)
	{
		ofs << iter - V.begin() << ", " << iter->x << ", " << iter->y << ", " << iter->theta << ", "
			<< iter->phi << ", " << iter->steer << ", " << iter->cost << ", " << iter->h << ", "
			<< "[" << iter->s.first << ", " << iter->s.second << "], " << iter->fatherindex << ", "
			<< "[";
		for (auto iter2 = iter->hindex.begin(); iter2 != iter->hindex.end(); ++iter2)
		{
			ofs << *iter2 << ", ";
		}
		ofs << "], [";
		for (auto iter2 = iter->obs_vertices.begin(); iter2 != iter->obs_vertices.end(); ++iter2)
		{
			ofs << *iter2 << ", ";
		}
		ofs << "]" << std::endl;
	}

	
	ofs.close();


}

