#pragma once

#include <vector>
#include <utility>
#include <string>

// to deal with colinear obstacle vertices
#define SEFTPP_EPS 0.0000000001

class Obs
{
public: 
	std::pair<double, double> ObsPosition;
	std::vector<std::pair<int, int> > grids_;
};

class Map
{
public: 
    int xsize_;
	int ysize_;

	std::vector<Obs> obs_;

	int* data_;

    double phi_1;
    double phi_2;

    double max_cable_length;

    std::pair<double, double> ran_base_point;
	std::pair<double, double> ran_start_pose;
	std::pair<double, double> ran_goal_pose;

	std::vector<std::pair<double, double> > footprint_;
	std::vector<std::pair<double, double> > polar_footprint_;
	double robot_radius_; //  just set a value which is larger than the truth

	inline void setBasePointAsObstacle()
	{
		double temp[9][2] = { {-1,-1},{-1,0},{-1,1},{0,-1},{0,0},{0,1},{1,-1},{1,0},{1,1} };
		for (unsigned int i = 0; i < 9; i++) {
			int x, y;
			x = floor(ran_base_point.first) + temp[i][0];
			y = floor(ran_base_point.second) + temp[i][1];
			data_[y*xsize_ + x] = 254;
		}
	}

	bool isCollisionFree(double x, double y, double theta)
	{
		if (x < robot_radius_ || x > xsize_ - robot_radius_ || y < robot_radius_ || y > ysize_ - robot_radius_)
			return false;

		std::vector<std::pair<double, double> > footprint = polarRotateAndMoveToXy(x, y, theta);
		for (unsigned int i = 0; i < footprint.size(); ++i)
		{
			// Here the robot vertices will not go outside the map matrix, because we have filtered the map boundaries above. 
			if (data_[(int)(floor(footprint[i].second)*xsize_ + floor(footprint[i].first))] != 0)
			{
				return false;
			}
		}
		return true;
	}

	inline std::pair<double, double> polarRotateAndMoveS(double x, double y, double theta)
	{
		return std::pair<double, double>(polar_footprint_[0].first * cos(polar_footprint_[0].second+theta) + x,
			polar_footprint_[0].first * sin(polar_footprint_[0].second+theta) + y);
	}

	std::vector<std::pair<double, double> >  polarRotateAndMoveToXy(double x, double y, double theta)
	{
		std::vector<std::pair<double, double> > after_polar(polar_footprint_.begin(), polar_footprint_.end());
		for (unsigned int i = 0; i < after_polar.size(); ++i)
		{
			after_polar[i].second += theta;
		}

		std::vector<std::pair<double, double> > footprint(after_polar.begin(), after_polar.end());
		for (unsigned int i = 0; i < after_polar.size(); ++i)
		{
			footprint[i].first = after_polar[i].first * cos(after_polar[i].second) + x;
			footprint[i].second = after_polar[i].first * sin(after_polar[i].second) + y;
		}
		return footprint;
	}

	bool straightenATwoSegmentCurve(std::vector<double>& old_curve, std::vector<double>& new_curve);

	void deformTether(const std::vector<double>& old_curve,
		std::vector<double>& new_curve);

	void newGetCableState(std::vector<double>& old_tether_shape,
		std::pair<double, double> olds, std::pair<double, double> news);

    void generatePolyMap(std::string map_filename);

	void sedFill();

	Map(std::string map_filename);

	~Map()
	{
		delete[] data_;
	}
    
    void init_footprint();

	double chassis_L_; // This is a fake parameter for car-like robot kinematics
	double validity_checking_resolution_;
	double primitive_path_length_;

	std::vector<std::pair<double, double> > critical_points_;

	int sparse_checking_num_;
	int dense_checking_num_;
	int all_checking_num_;

private: 

	int critical_points_num_;
	std::vector<double> critical_points_orientation;
};
