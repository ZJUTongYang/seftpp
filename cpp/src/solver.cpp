#include "solver.h"
#include <vector>
#include <utility>
#include <iostream>
#include <cmath>
#include "tool.h"


void calc_index(Conf son, double x_gridsize, double y_gridsize, double theta_gridsize, int& son_x_index, int& son_y_index, int& son_theta_index)
{
    son_x_index = floor(son.x / x_gridsize);
    son_y_index = floor(son.y / y_gridsize);
    son_theta_index = floor(normalize_angle_positive(son.theta) / theta_gridsize);
}


std::vector<int> calSwing(std::vector<int> oldH, std::pair<double, double> oldp, std::pair<double, double> newp, const std::vector<std::pair<double, double> >& g_obs)
{
    std::vector<int> newH(oldH.begin(), oldH.end());
    for(int i = 0; i < g_obs.size(); ++i)
    {
        if(oldp.second >= g_obs[i].second && newp.second >= g_obs[i].second)
        {
            if(oldp.first > g_obs[i].first + 0.5 && newp.first < g_obs[i].first + 0.5)
            {
                if(!oldH.empty() && oldH.back() == i)
                {
                    newH.pop_back();
                }
                else
                {
                    newH.push_back(-i);
                }
                return newH;
            }
            else if(oldp.first < g_obs[i].first + 0.5 && newp.first > g_obs[i].first + 0.5)
            {
                if(!oldH.empty() && oldH.back() == -i)
                {
                    newH.pop_back();
                }
                else
                {
                    newH.push_back(i);
                }
                return newH;

            }
        }
    }

	return newH;
}

int findEqualHCNode(std::vector<int> newH, const std::vector<Conf>& V, std::vector<int> indices)
{
    if(indices.empty())
    {
        std::cout << "YT: we should not reach here" << std::endl;
        return -1;
    }

    for(unsigned int i = 0; i < indices.size(); ++i)
    {
        auto& oldH = V[indices[i]].hindex;
        if(oldH.size() == newH.size())
        {
            bool thesame = true;
            for(unsigned int j = 0; j < oldH.size(); ++j)
            {
                if(fabs((double(oldH[j]) - (double)(newH[j])) > 0.1))
                {
                    thesame = false;
                    break;
                }

            }
            if(thesame)
            {
                return indices[i];
            }
        }
    }

    std::cout << "YT: we should not reach here" << std::endl;
    return -1;
}

bool isEqualHC(std::vector<int> newH, const std::vector<Conf>& Storage, 
	std::vector<int> indices)
{
    for(unsigned int i = 0; i < indices.size(); ++i)
    {
        auto& oldH = Storage[indices[i]].hindex;
        if(oldH.size() == newH.size())
        {
            bool thesame = true;
            for(unsigned int j = 0; j < oldH.size(); ++j)
            {
                if(fabs((double(oldH[j]) - (double)(newH[j])) > 0.1))
                {
                    thesame = false;
                    break;
                }

            }
            if(thesame)
            {
                return true;
            }
        }
    }

    return false;
}

bool isSEF(const Conf& pose, const double phi_1, const double phi_2)
{
    double angle_diff = normalize_angle_positive(pose.phi - pose.theta);

    if(angle_diff >= phi_1 && angle_diff <= phi_2)
        return true;

    return false;
}

std::vector<Conf> tracePath(const std::vector<Conf>& V, int index)
{
    std::vector<Conf> waypoints;
    waypoints.clear();
    Conf cur = V[index];
    while(1)
    {
        if(cur.fatherindex == -1)
        {
            waypoints.insert(waypoints.begin(), cur);
            break;
        }

        int fatherindex = cur.fatherindex;
        waypoints.insert(waypoints.begin(), cur);
        cur = V[fatherindex];
    }

    return waypoints;
}