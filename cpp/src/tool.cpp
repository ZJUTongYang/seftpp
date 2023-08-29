#include "tool.h"
#include <vector>
#include <queue>
#include "map.h"
#include "definitions.h"

#include <opencv2/opencv.hpp>

double normalize_angle(const double angle)
{
	const double result = fmod(angle + M_PI, 2.0*M_PI);
	if (result <= 0.0) return result + M_PI;
	return result - M_PI;
}

double normalize_angle_positive(const double angle)
{
	const double result = fmod(angle, 2.0*M_PI);
	if (result < 0.0) return result + 2.0*M_PI;
	return result;
}

double crossProduct2D(const std::pair<double, double>& a, const std::pair<double, double>& b)
{
	return a.first*b.second - a.second*b.first;
}

bool isTriVertex(const double* tri, const double& px, const double& py)
{
	if (fabs(tri[0] - px) < 0.5 && fabs(tri[1] - py) < 0.5)
		return true;
	if (fabs(tri[2] - px) < 0.5 && fabs(tri[3] - py) < 0.5)
		return true;
	if (fabs(tri[4] - px) < 0.5 && fabs(tri[5] - py) < 0.5)
		return true;
	return false;
}

bool isPointInOrOnTri(const double* tri, const double& px, const double& py)
{
	std::pair<double, double> ab(tri[2] - tri[0], tri[3] - tri[1]),
		bc(tri[4] - tri[2], tri[5] - tri[3]), ca(tri[0] - tri[4], tri[1] - tri[5]),
		ap(px - tri[0], py - tri[1]), bp(px - tri[2], py - tri[3]), cp(px - tri[4], py - tri[5]);
	double cross0 = crossProduct2D(ab, ap), cross1 = crossProduct2D(bc, bp), cross2 = crossProduct2D(ca, cp);

	return (cross0 >= 0 && cross1 >= 0 && cross2 >= 0) || (cross0 <= 0 && cross1 <= 0 && cross2 <= 0);
}

bool pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
	int i, j;
	bool c = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
			c = !c;
	}
	return c;
}
   
int sign(float x){
    if(x>0){
        return 1;
    }
    else if(x<0){
        return -1;
    }
    else{
        return 0;
    }
}

double pathLength(const std::vector<double >& thePath)
{
	if (thePath.empty())
	{
		std::cout << "check here" << std::endl;
		return 10000.0;
	}

	double L = 0;
	double dx, dy;

	int n = thePath.size() / 2;

	for (unsigned int i = 0; i < n - 1; ++i)
	{
		dx = thePath[2*(i + 1)] - thePath[2*i];
		dy = thePath[2 * (i + 1) + 1] - thePath[2 * i + 1];
		L += sqrt(dx*dx + dy * dy);
	}
	return L;
}

double pathLength(std::vector<Conf>& thePath)
{
	std::vector<double> double_path;
	for (auto iter = thePath.begin(); iter != thePath.end(); ++iter) {
		double_path.emplace_back(iter->x);
		double_path.emplace_back(iter->y);
	}
	return pathLength(double_path);
}

void calc_index(Conf son, double x_gridsize, double y_gridsize, double theta_gridsize, int& son_x_index, int& son_y_index, int& son_theta_index)
{
	son_x_index = floor(son.x / x_gridsize);
	son_y_index = floor(son.y / y_gridsize);
	son_theta_index = floor(normalize_angle_positive(son.theta) / theta_gridsize);
}

std::vector<int> calSwing(std::vector<int> oldH, std::pair<double, double> oldp, std::pair<double, double> newp, const std::vector<Obs>& obs)
{
	std::vector<int> newH(oldH.begin(), oldH.end());
	for (int i = 0; i < obs.size(); ++i)
	{
		if (oldp.second >= obs[i].ObsPosition.second && newp.second >= obs[i].ObsPosition.second)
		{
			if (oldp.first > obs[i].ObsPosition.first + 0.5 && newp.first < obs[i].ObsPosition.first + 0.5)
			{
				if (!oldH.empty() && oldH.back() == i)
				{
					newH.pop_back();
				}
				else
				{
					newH.emplace_back(-i);
				}
				return newH;
			}
			else if (oldp.first < obs[i].ObsPosition.first + 0.5 && newp.first > obs[i].ObsPosition.first + 0.5)
			{
				if (!oldH.empty() && oldH.back() == -i)
				{
					newH.pop_back();
				}
				else
				{
					newH.emplace_back(i);
				}
				return newH;

			}
		}
	}

	return newH;
}

int findEqualHCNode(std::vector<int> newH, const std::vector<Conf>& V, std::vector<int> indices)
{
	if (indices.empty())
	{
		std::cout << "YT: we should not reach here" << std::endl;
		return -1;
	}

	for (unsigned int i = 0; i < indices.size(); ++i)
	{
		auto& oldH = V[indices[i]].hindex;
		if (oldH.size() == newH.size())
		{
			bool thesame = true;
			for (unsigned int j = 0; j < oldH.size(); ++j)
			{
				if (fabs((double(oldH[j]) - (double)(newH[j])) > 0.1))
				{
					thesame = false;
					break;
				}

			}
			if (thesame)
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
	for (unsigned int i = 0; i < indices.size(); ++i)
	{
		auto& oldH = Storage[indices[i]].hindex;
		if (oldH.size() == newH.size())
		{
			bool thesame = true;
			for (unsigned int j = 0; j < oldH.size(); ++j)
			{
				if (fabs((double(oldH[j]) - (double)(newH[j])) > 0.1))
				{
					thesame = false;
					break;
				}
			}
			if (thesame)
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

	if (angle_diff >= phi_1 && angle_diff <= phi_2)
		return true;

	return false;
}

std::vector<Conf> tracePath(const std::vector<Conf>& V, int index)
{
	std::vector<Conf> waypoints;
	waypoints.clear();
	Conf cur = V[index];
	while (1)
	{
		if (cur.fatherindex == -1)
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