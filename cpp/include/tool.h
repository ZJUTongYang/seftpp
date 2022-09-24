#ifndef _TOOL_
#define _TOOL_

#include<iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "solver.h"
#define INF 10000000

double normalize_angle(const double theta);

double normalize_angle_positive(const double theta);


void newbresenham(double x1, double y1, double x2, double y2, int xsize, std::vector<int>& result);

void newbresenham(double x1, double y1, double x2, double y2, std::vector<std::pair<int, int> >& result);

bool pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);
//bool isCollisionFree(double x, double y, double theta);
// std::vector<Eigen::Vector2f> tryRemoveObstacles(std::vector<Eigen::Vector2f> obs_list,Eigen::Vector2f new_s_point);
//std::vector<std::pair<double,double>> tryRemoveObstacles(std::vector<std::pair<double, double>> obs_list, std::pair<double, double> new_s_point);
int sign(float x);

//double pathLength(const std::vector<std::pair<double, double> >& thePath);

double pathLength(const std::vector<double >& thePath);
double pathLength(std::vector<Conf>& thePath);



#endif





