#include "planner.h"
#include <vector>
#include "definitions.h"
#include <cmath>
#include "map.h"
#include "tool.h"
#include <iostream>
#include <fstream>
#include "sparsity.h"

#include<cstdlib>
#include<ctime>
using namespace std;

void generateSMotion(Map* pMap, const std::vector<double>& x_list, const std::vector<double>& y_list,
	const std::vector<double>& theta_list, std::vector<double>& s_x_list,
	std::vector<double>& s_y_list, std::vector<double>& s_theta_list)
{
	unsigned int n = x_list.size();
	s_x_list.resize(n);
	s_y_list.resize(n);
	s_theta_list.resize(n);

	for (unsigned int i = 0; i < n; ++i)
	{
		double x = x_list[i];
		double y = y_list[i];
		double theta = theta_list[i];
		std::pair<double, double> temp_s = pMap->polarRotateAndMoveS(x, y, theta);
		s_x_list[i] = temp_s.first;
		s_y_list[i] = temp_s.second;
		s_theta_list[i] = theta;
	}
}

void generateMotion(Map* pMap, double x, double y, double theta, int dir,
	double steer, double dis, int n, std::vector<double>& x_list,
	std::vector<double>& y_list, std::vector<double>& theta_list)
{
	x_list.resize(n);
	y_list.resize(n);
	theta_list.resize(n, theta);

	if (fabs(steer) < 0.001)
	{
		// This is a straight motion
		double temp = dis / n;
		double dx = temp * cos(theta);
		double dy = temp * sin(theta);
		for (int i = 1; i <= n; ++i)
		{
			x_list[i-1] = x + i * dx;
			y_list[i-1] = y + i * dy;
		}
	}
	else
	{
		// turning radius is the inverse of tan(steer)
		double turning_radius = fabs(pMap->chassis_L_ / tan(steer)); // always positive
		double turning_angle = dis / turning_radius; // always positive
		if (steer > 0)
		{
			// This is a FL or BR motion
			double dtheta = dir * turning_angle / n; // NOT always positive
			double pivoting_x = x + turning_radius * cos(theta + M_PI / 2);
			double pivoting_y = y + turning_radius * sin(theta + M_PI / 2);
			for (int i = 1; i <= n; ++i)
			{
				x_list[i-1] = pivoting_x - turning_radius * cos(theta + M_PI / 2 + dtheta * i);
				y_list[i-1] = pivoting_y - turning_radius * sin(theta + M_PI / 2 + dtheta * i);
				theta_list[i-1] = theta + dtheta * i;
			}
		}
		else
		{
			// This is a FR or BL motion
			double dtheta = dir * turning_angle / n; // NOT	always positive
			double pivoting_x = x + turning_radius * cos(theta - M_PI / 2);
			double pivoting_y = y + turning_radius * sin(theta - M_PI / 2);
			for (int i = 1; i <= n; ++i)
			{
				x_list[i-1] = pivoting_x - turning_radius * cos(theta - M_PI / 2 + dtheta * i);
				y_list[i-1] = pivoting_y - turning_radius * sin(theta - M_PI / 2 + dtheta * i);
				theta_list[i-1] = theta + dtheta * i;
			}
		}
	}
}
//
//void generateMotion(Map* pMap, double x, double y, double theta, int dir, 
//    double steer, int n, std::vector<double>& x_list, 
//    std::vector<double>& y_list, std::vector<double>& yaw_list)
//{
////    double L = 3.0; // This is a fake parameter for car-like robot kinematics, not the distance of the robot center
//
//    double distance = dir * pMap->validity_checking_resolution_;
//    double temp = normalize_angle(distance*tan(steer)/pMap->chassis_L_);
//    x_list.clear();
//    y_list.clear();
//    yaw_list.clear();
//    x_list.emplace_back(x);
//    y_list.emplace_back(y);
//
//    for(unsigned int i = 0; i <= n; ++i)
//    {
//        yaw_list.emplace_back(theta + i*temp);
//    }
//    for(unsigned int i = 1; i <= n; ++i)
//    {
//        x_list.emplace_back(x_list.back() + distance*cos(yaw_list[i-1]));
//        y_list.emplace_back(y_list.back() + distance*sin(yaw_list[i-1]));
//    }
//
//    x_list.erase(x_list.begin());
//    y_list.erase(y_list.begin());
//    yaw_list.erase(yaw_list.begin());
//
//}

bool isCrossing(std::pair<double, double> s1, std::pair<double, double> g1,
	std::pair<double, double> s2, std::pair<double, double> g2)
{
	if ((s1.first > g1.first ? s1.first : g1.first) < (s2.first < g2.first ? s2.first : g2.first) ||
		(s1.second > g1.second ? s1.second : g1.second) < (s2.second < g2.second ? s2.second : g2.second) ||
		(s2.first > g2.first ? s2.first : g2.first) < (s1.first < g1.first ? s1.first : g1.first) ||
		(s2.second > g2.second ? s2.second : g2.second) < (s1.second < g1.second ? s1.second : g1.second))
	{
		return false;
	}

	if ((((s1.first - s2.first)*(g2.second - s2.second) - (s1.second - s2.second)*(g2.first - s2.first))*
		((g1.first - s2.first)*(g2.second - s2.second) - (g1.second - s2.second)*(g2.first - s2.first))) > 0 ||
		(((s2.first - s1.first)*(g1.second - s1.second) - (s2.second - s1.second)*(g1.first - s1.first))*
		((g2.first - s1.first)*(g1.second - s1.second) - (g2.second - s1.second)*(g1.first - s1.first))) > 0)
	{
		return false;
	}
	return true;
}

bool isOChanged(Map* pMap, const std::vector<double>& old_tether, const std::vector<double>& new_tether,
	double o_x, double o_y, std::vector<double>& s_x_list, std::vector<double>& s_y_list, std::vector<double>& s_theta_list)
{
	// First we need to make sure that the starting tether and the ending tether are unchanged
	if (old_tether.size() != new_tether.size())
		return true;
	for (int i = old_tether.size()-1; i >=0; --i)
	{
		if (fabs(old_tether[i] - new_tether[i]) > 0.5)
			return true;
	}

	std::vector<double> vertx(s_x_list);
	std::vector<double> verty(s_y_list);
	vertx.emplace_back(o_x);
	verty.emplace_back(o_y);

	for (unsigned int i = 0; i < pMap->critical_points_.size(); ++i)
	{
		bool b = pnpoly(vertx.size(), &(vertx[0]), &(verty[0]),
			pMap->critical_points_[i].first, pMap->critical_points_[i].second);
		if (b &&
			!(fabs(pMap->critical_points_[i].first - o_x) < 0.25 && fabs(pMap->critical_points_[i].second - o_y) < 0.25))
		{
			return true;
		}
	}
	return false;

//	return pMap->anyObstacleInPoly(vertx, verty);
}


bool isNS(Map* pMap, const std::vector<double>& old_tether, const double& x,
	const double& y, const double& theta)
{
	std::vector<std::pair<double, double> > theFootprint = pMap->polarRotateAndMoveToXy(x, y, theta);
	unsigned int num_footprint = theFootprint.size();
	unsigned int num_tether = old_tether.size() / 2;
	for (unsigned int j = 0; j < num_tether - 1; ++j)
	{
		for (unsigned int k = 0; k < num_footprint - 1; ++k)
		{
			if (isCrossing(theFootprint[k], theFootprint[k + 1],
				std::pair<double, double>(old_tether[2 * j], old_tether[2 * j + 1]),
				std::pair<double, double>(old_tether[2 * j + 2], old_tether[2 * j + 3])))
			{
				return false;
			}
		}
	}
	return true;
}

Conf generateConfWithoutCost(Map* pMap, const std::vector<double>& old_tether, 
	double old_x, double old_y, double old_theta, 
	double old_s_x, double old_s_y, 
	const std::vector<double>& x_list, const std::vector<double>& y_list, const std::vector<double>& theta_list, 
	const std::vector<double>& s_x_list, const std::vector<double>& s_y_list, const std::vector<double>& s_theta_list, 
	int index)
{
	Conf son_node;
	son_node.index = -1;
	son_node.x = x_list[index];
	son_node.y = y_list[index];
	son_node.theta = theta_list[index];
	son_node.s = pMap->polarRotateAndMoveS(son_node.x, son_node.y, son_node.theta);

	// We calculate the tether state
	// First we calculate the trajectory of S
	std::vector<double> new_tether = old_tether;
	new_tether.emplace_back(old_s_x);
	new_tether.emplace_back(old_s_y);
	for (unsigned int i = 0; i <= index; ++i)
	{
		new_tether.emplace_back(s_x_list[i]);
		new_tether.emplace_back(s_y_list[i]);
	}
	std::vector<double> deformed_tether;
	pMap->deformTether(new_tether, deformed_tether);

	son_node.obs_vertices.assign(deformed_tether.begin(), deformed_tether.end() - 2);

	int temp = son_node.obs_vertices.size();
	son_node.phi = atan2(son_node.obs_vertices[temp - 1] - son_node.s.second, son_node.obs_vertices[temp - 2] - son_node.s.first);

	return son_node;
}

bool isTLA(const Conf& a, double L)
{
	double length = 0;
	unsigned int n = a.obs_vertices.size()/2;
	for (unsigned int i = 0; i < n - 1; ++i)
	{
		length += sqrt((a.obs_vertices[2 * i] - a.obs_vertices[2 * i + 2])*(a.obs_vertices[2 * i] - a.obs_vertices[2 * i + 2])
			+ (a.obs_vertices[2 * i + 1] - a.obs_vertices[2 * i + 3])*(a.obs_vertices[2 * i + 1] - a.obs_vertices[2 * i + 3]));
	}
	length += sqrt((a.obs_vertices[2 * (n - 1)] - a.s.first)*(a.obs_vertices[2 * (n - 1)] - a.s.first)
		+ (a.obs_vertices[2 * n - 1] - a.s.second)*(a.obs_vertices[2 * n - 1] - a.s.second));

	if (length < L)
		return true;
	return false;
}

bool nodeExpansion(Map* pMap, const Conf& cur_node, double steer, double dir, Conf& son_node, std::pair<double, double>& goal_loc)
{
	static double SB_COST = 5;
	static double STEER_COST = 1.0;
	static double STEER_CHANGE_COST = 1.0;

    static unsigned int n = floor(pMap->primitive_path_length_ / pMap->validity_checking_resolution_) + 1;

    static std::vector<double> x_list;
    static std::vector<double> y_list;
    static std::vector<double> yaw_list;

	// This is the x-y-theta waypoints, not configurations
    generateMotion(pMap, cur_node.x, cur_node.y, cur_node.theta, dir, steer, pMap->primitive_path_length_, n, x_list, y_list, yaw_list);
	
	pMap->all_checking_num_++;
	// we check the non-selfcrossing property
	bool is_path_valid = true;
	for (unsigned int i = 0; i < n; ++i)
	{
		if (!isNS(pMap, cur_node.obs_vertices, x_list[i], y_list[i], yaw_list[i]))
		{
			is_path_valid = false;
			return is_path_valid;
		}
	}

	// we check the collision-free property
	for (unsigned int i = 0; i < n; ++i)
	{
		if (!pMap->isCollisionFree(x_list[i], y_list[i], yaw_list[i]))
		{
			is_path_valid = false;
			return is_path_valid;
		}
	}

	// ICRA23: we have to generate the waypoint configurations for verification
	static std::vector<double> s_x_list;
	static std::vector<double> s_y_list;
	static std::vector<double> s_theta_list;
	generateSMotion(pMap, x_list, y_list, yaw_list, s_x_list, s_y_list, s_theta_list);

	bool is_sef_guaranteed = false;
	bool is_tla_guaranteed = false;
	bool is_o_not_changed = false;

	// We first check that the ending configuration is SEF and TLA
	son_node = generateConfWithoutCost(pMap, cur_node.obs_vertices,
		cur_node.x, cur_node.y, cur_node.theta,
		cur_node.s.first, cur_node.s.second,
		x_list, y_list, yaw_list,
		s_x_list, s_y_list, s_theta_list, n - 1);

	if (!isSEF(son_node, pMap->phi_1, pMap->phi_2))
	{
		is_path_valid = false;
		return is_path_valid;
	}
	if (!isTLA(son_node, pMap->max_cable_length))
	{
		is_path_valid = false;
		return is_path_valid;
	}



#if	USE_IMPROVED_NODE_EXPANSION

	int tether_size_temp = cur_node.obs_vertices.size();
	double o_x = cur_node.obs_vertices[tether_size_temp - 2];
	double o_y = cur_node.obs_vertices[tether_size_temp - 1];

	is_o_not_changed = !isOChanged(pMap, cur_node.obs_vertices, son_node.obs_vertices,
		o_x, o_y, s_x_list, s_y_list, s_theta_list);

	is_sef_guaranteed = is_o_not_changed &&
		isSEFGuaranteed(pMap, steer, dir, pMap->primitive_path_length_, 
			o_x, o_y, pMap->footprint_[0].first, pMap->footprint_[0].second, 
			cur_node.x, cur_node.y, cur_node.theta, 
			son_node.x, son_node.y, son_node.theta);
	is_tla_guaranteed = is_o_not_changed && 
		isTLAGuaranteed(pMap, steer, dir, pMap->primitive_path_length_, 
			o_x, o_y, pMap->footprint_[0].first, pMap->footprint_[0].second, 
			cur_node.x, cur_node.y, cur_node.theta,
			son_node.x, son_node.y, son_node.theta);
#endif

	Conf temp_son_node;
	if (!is_sef_guaranteed || !is_tla_guaranteed)
	{
		pMap->dense_checking_num_++;
		for (unsigned int i = 0; i < n - 1; ++i)
		{
			temp_son_node = generateConfWithoutCost(pMap, cur_node.obs_vertices,
				cur_node.x, cur_node.y, cur_node.theta,
				cur_node.s.first, cur_node.s.second,
				x_list, y_list, yaw_list,
				s_x_list, s_y_list, s_theta_list, i);

			// We check the SEF property
			if (!isSEF(temp_son_node, pMap->phi_1, pMap->phi_2))
			{
				is_path_valid = false;
				return is_path_valid;
			}

			// We check the TLA property
			if (!isTLA(temp_son_node, pMap->max_cable_length))
			{
				is_path_valid = false;
				return is_path_valid;
			}
		}

	}
	else
	{
		pMap->sparse_checking_num_++;
//		std::cout << "yes" << std::endl;
	}

	double added_cost = 0;
	added_cost += (dir == cur_node.motion_direction) ? 0 : SB_COST;
    // Steer Penalty
    added_cost += STEER_COST * fabs(steer);
    // Steer change penalty
    added_cost += STEER_CHANGE_COST * fabs(cur_node.steer - steer);

	double cost = cur_node.cost + added_cost + n * pMap->validity_checking_resolution_;

    son_node.cost = cost;
    double x_diff = son_node.x - goal_loc.first;
    double y_diff = son_node.y - goal_loc.second;
    son_node.h = sqrt( x_diff*x_diff + y_diff*y_diff);
    son_node.hindex.clear();
    son_node.steer = steer;
    son_node.motion_direction = dir;

    son_node.open = true;
    son_node.fatherindex = cur_node.index;
    son_node.mid_poses_x.assign(x_list.begin(), x_list.end());
    son_node.mid_poses_y.assign(y_list.begin(), y_list.end());
    son_node.mid_poses_theta.assign(yaw_list.begin(), yaw_list.end());

    // We construct the hindex of the algorithm
    std::vector<int> cur_hindex = cur_node.hindex;
	std::vector<double> temp = {cur_node.x, cur_node.y};
    for(unsigned int i = 0; i < son_node.mid_poses_x.size(); ++i)
    {
		temp.emplace_back(son_node.mid_poses_x[i]);
		temp.emplace_back(son_node.mid_poses_y[i]);
    }
	temp.emplace_back(son_node.x);
	temp.emplace_back(son_node.y);

	int n_temp = temp.size() / 2;
   for(unsigned int i = 0; i < n_temp-1; ++i)
   {
	   cur_hindex = calSwing(cur_hindex, std::pair<double, double>(temp[2*i], temp[2*i+1]),
		   std::pair<double, double>(temp[2*(i+1)], temp[2*(i+1)+1]), pMap->obs_);
   }

    son_node.hindex = cur_hindex;

    return true;
}

void get_neighbors(Map* theMap, Conf cur_node, std::vector<Conf>& sons_node, std::pair<double, double> goal_loc)
{
    static const std::vector<double> STEER = {-0.6000, -0.5, -0.4000, -0.3, -0.2000, -0.1, 0, 0.1, 0.2000, 0.3, 0.4000, 0.5, 0.6000};

    static const std::vector<int> D = {-1, 1};

	static const int n_STEER = STEER.size();
	static const int n_D = D.size();

    sons_node.clear();
    sons_node.reserve(n_STEER*n_D);
	Conf son_node_temp;
	for(unsigned int i = 0; i < n_STEER; ++i)
    {
        for(unsigned int j = 0; j < n_D; ++j)
        {
            if(nodeExpansion(theMap, cur_node, STEER[i], D[j], son_node_temp, goal_loc))
//            if(improvedNodeExpansion(theMap, cur_node, STEER[i], D[j], son_node_temp, goal_loc))
                sons_node.emplace_back(son_node_temp);
        }
    }
}

bool isGoalReached(Conf cur_node, const std::pair<double, double>& goal_loc)
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
    V.emplace_back(start_pose);
    int xnum = theMap->xsize_;
    int ynum = theMap->ysize_;
    int thetanum = 30;

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
    queue_.emplace_back(std::pair<int, double>(0, h));

    resulting_path.clear();

	int count = 0;
	clock_t start, end;
	start = clock();
	bool find_path = false;
	COUNT = 0;
    while(!queue_.empty())
    {

        // We find the least-cost motion
        auto top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), comp);
        queue_.pop_back();

        int cur_index = top.first;

        V[cur_index].open = false;

        Conf cur_node = V[cur_index];

        bool is_goal_reached = isGoalReached(cur_node, goal_loc);

        if(is_goal_reached)
        {
   //         std::cout << "Path Found" << std::endl;
   //         std::cout << "Number of nodes: " << V.size() << std::endl;
			//std::cout << "Cost of last node£º" << cur_node.cost << std::endl;
			find_path = true;
            resulting_path = tracePath(V, cur_node.index);
			break ;
        }

        // Here we calculate all elements of sons, 
        // Except their indices, because we haven't sure whether they will be added into the storage
        std::vector<Conf> sons;
        get_neighbors(theMap, cur_node, sons, goal_loc);

//		std::cout << "size of sons: " << sons.size() << std::endl;
		
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

                queue_.emplace_back(std::pair<int, double>(newsonindex, newcost + hcost));
                std::push_heap(queue_.begin(), queue_.end(), comp);

                V.emplace_back(son);
                I[son_x_index][son_y_index][son_theta_index].emplace_back(newsonindex);
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

					//V.emplace_back(son);
     //           }
     //           else
     //           {
     //               queue_.emplace_back(std::pair<int, double>(newsonindex, newcost + hcost));
     //               std::push_heap(queue_.begin(), queue_.end(), comp);

					//V.emplace_back(son);
     //           }

				if (V[oldindex].open == false)
				{
					queue_.emplace_back(std::pair<int, double>(oldindex, newcost + hcost));
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

}

