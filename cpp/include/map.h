#pragma once

#include <vector>
#include <utility>
#include <string>

class Map
{
public: 
    int xsize_;
    int ysize_;

    std::vector<std::pair<double, double> > g_obs;

	int* data_;

    double phi_1;
    double phi_2;

    double max_cable_length;

	std::vector<std::pair<double, double> > critical_points;
	std::vector<double> critical_points_orientation;

    std::pair<double, double> ran_base_point;
    std::pair<double, double> ran_start_pose;
    std::pair<double, double> ran_goal_pose;

    std::vector<std::pair<double, double> > footprint_;
    std::vector<std::pair<double, double> > polar_footprint_;

    bool isCollisionFree(double x, double y, double theta);

    std::vector<std::pair<double, double> >  polarRotateAndMoveToXy(double x, double y, double theta);

	void getCableState(std::vector<double>& obs_list, std::pair<double, double> old_s_pos, std::pair<double, double>  new_s_pos);

    std::pair<double, double>  polarRotateAndMoveS(double x, double y, double theta);

    void generatePolyMap(std::string map_filename);

	void sedFill(std::vector<std::pair<double, double> >& ObsPosition);

	Map(std::string map_filename);

	~Map()
	{
		delete[] data_;
		delete[] critical_points_x_;
		delete[] critical_points_y_;
	}
    
    void init_footprint();

	//std::vector<std::pair<double, double> > tryRemoveObstacles(std::vector<std::pair<double, double> > obs_list, std::pair<double, double> new_s_point);
	std::vector<double> tryRemoveObstacles(std::vector<double> obs_list, std::pair<double, double> news);

private: 

	// Just to be used for fast reference
	// In C++, critical points may be "int"
	int* critical_points_x_;
	int* critical_points_y_;
	int critical_points_num_;

};

