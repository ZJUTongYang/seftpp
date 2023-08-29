#pragma once

#include<iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "definitions.h"
#include "map.h"

void calc_index(Conf son, double x_gridsize, double y_gridsize, double theta_gridsize, int& son_x_index, int& son_y_index, int& son_theta_index);

std::vector<int> calSwing(std::vector<int> oldH, std::pair<double, double> oldp, std::pair<double, double> oldp2, const std::vector<Obs>& obs);

int findEqualHCNode(std::vector<int> newH, const std::vector<Conf>& V, std::vector<int> indices);

bool isEqualHC(std::vector<int> newH, const std::vector<Conf>& Storage, std::vector<int> indices);

bool isPointInOrOnTri(const double* tri, const double& px, const double& py);

bool isSEF(const Conf& pose, const double phi_1, const double phi_2);

bool isTriVertex(const double* tri, const double& px, const double& py);

double normalize_angle(const double theta);

double normalize_angle_positive(const double theta);

double pathLength(const std::vector<double >& thePath);

double pathLength(std::vector<Conf>& thePath);

bool pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);

int sign(float x);

std::vector<Conf> tracePath(const std::vector<Conf>& V, int index);
