#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Conf{

    double x;
    double y;
    double theta;
    double phi;
    
    double steer;

    double h;
    double cost;
    
    std::vector<int> hindex;
    int index;

    bool open;

    std::pair<double, double> s;

    int motion_direction; // 1: forward, -1: backward

    int fatherindex;

	std::vector<double> obs_vertices;

    std::vector<double> mid_poses_x;
    std::vector<double> mid_poses_y;
    std::vector<double> mid_poses_theta;
};
