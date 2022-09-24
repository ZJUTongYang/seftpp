#ifndef _SOLVER_
#define _SOLVER_
#include <vector>
#include <utility>
#include "definitions.h"



void calc_index(Conf son, double x_gridsize, double y_gridsize, double theta_gridsize, int& son_x_index, int& son_y_index, int& son_theta_index);

std::vector<int> calSwing(std::vector<int> oldH, std::pair<double, double> oldp, std::pair<double, double> oldp2, const std::vector<std::pair<double, double> >& g_obs);

int findEqualHCNode(std::vector<int> newH, const std::vector<Conf>& V, std::vector<int> indices);

bool isEqualHC(std::vector<int> newH, const std::vector<Conf>& Storage, std::vector<int> indices);

bool isSEF(const Conf& pose, const double phi_1, const double phi_2);

std::vector<Conf> tracePath(const std::vector<Conf>& V, int index);

#endif