#pragma once

#include "map.h"
#include "tool.h"
#include <string>
#include <opencv2/opencv.hpp>



Map::Map(std::string map_filename) {
	critical_points.clear();
	critical_points_orientation.clear();
	generatePolyMap(map_filename);

	//// We de-duplicate critical points
	//std::sort(critical_points.begin(), critical_points.end(), 
	//	[](const auto& a, const auto& b) {return a.first < b.first || (a.first == b.first && a.second < b.second); });
	//critical_points.erase(
	//	std::unique(critical_points.begin(), critical_points.end(), 
	//		[](const auto& a, const auto& b) {return a.first == b.first && a.second == b.second; }
	//		), critical_points.end());

	critical_points_num_ = critical_points.size();
	critical_points_x_ = new int[critical_points_num_];
	critical_points_y_ = new int[critical_points_num_];
	for (unsigned int i = 0; i < critical_points.size(); ++i)
	{
		// YT: here we can safely use round, because the critical points in C++ must be near an integer
		critical_points_x_[i] = round(critical_points[i].first);
		critical_points_y_[i] = round(critical_points[i].second);
	}
}

bool Map::isCollisionFree(double x, double y, double theta)
{
	if (x < 4 || x > xsize_ - 4 || y < 4 || y > ysize_ - 4)
		return false;

	std::vector<std::pair<double, double> > footprint = polarRotateAndMoveToXy(x, y, theta);
	for (unsigned int i = 0; i < footprint.size(); ++i)
	{
		if (data_[(int)(floor(footprint[i].second)*xsize_ + floor(footprint[i].first))] != 0) {
			std::cout << "";
			return false;

		}
	}
	return true;
}

void Map::generatePolyMap(std::string map_filename)
{

	// std::tuple<Eigen::MatrixXi,std::vector<Eigen::Vector2f>>

	cv::Mat map_img = cv::imread(map_filename, cv::IMREAD_GRAYSCALE);
	if (map_img.empty())
	{
		std::cout << "We cannot open the map png file. " << std::endl;
	}

	xsize_ = map_img.cols;
	ysize_ = map_img.rows;

	// To be consistent with previous codes
	int height = map_img.rows; // ysize_
	int width = map_img.cols; // xsize_

	data_ = new int[xsize_*ysize_];

	// We need to store the costmap in Euclidean X-Y coordinate
	for (unsigned int row = 0; row < ysize_; ++row)
	{
		uchar *current_row = map_img.ptr<uchar>(row);
		for (unsigned int col = 0; col < xsize_; ++col)
		{
			data_[(ysize_ - 1 - row)*xsize_ + col] = *(current_row + col);
		}
	}

	// We transform the image map (free:255, obstacle:0) to the costmap (free: 0, obstacle: 254, inflation: 127)
	for (unsigned int x = 0; x < xsize_; ++x)
	{
		for (unsigned int y = 0; y < ysize_; ++y)
		{
			int temp = data_[y*xsize_ + x];
			if (data_[y*xsize_ + x] == 0)
			{
				data_[y*xsize_ + x] = 254;
			}
			else if (data_[y*xsize_ + x] >= 253) // a white grid, which corresponds to the obstacle-free point
			{
				data_[y*xsize_ + x] = 0;
			}
		}
	}


	//for (int i = 0; i < map.rows(); i++) {
	//	for (int j = 0; j < map.cols(); j++) {
	//		if (map(i, j) == 0)
	//			map(i, j) = 254;
	//		else if (map(i, j) >= 254)
	//			map(i, j) = 0;
	//	}
	//}


	// We add the boundary of the map
	for (unsigned int row = 0; row < ysize_; ++row)
	{
		for (unsigned int col = 0; col < xsize_; ++col)
		{
			if (row == 0 || row == ysize_ - 1)
			{
				data_[row*xsize_ + col] = 254;
			}
			else
			{
				if (col == 0 || col == xsize_ - 1)
				{
					data_[row*xsize_ + col] = 254;
				}
			}
		}
	}

	//for (int i = 0; i < map.rows(); i++) {
	//	for (int j = 0; j < map.cols(); j++) {
	//		if (i == 0 || j == 0 || i == map.rows() - 1 || j == map.cols() - 1) {
	//			map(i, j) = 254;
	//		}
	//	}
	//}


	//std::vector<Eigen::Vector2i> obstacle = sedFill(map);

	//std::vector<std::pair<int, int> > g_obs;
	sedFill(g_obs);


	// We prepare the critical obstacles (grids)
	// For each obstacle, we check whether its 8-dir neighbour is a 4-neighbor obstacle-free grid

	std::vector<std::pair<int, int> > all_temp;
	all_temp.reserve(10000);
	for (unsigned int i = 0; i < xsize_; ++i)
	{
		for (unsigned int j = 0; j < ysize_; ++j)
		{
			if (data_[j*xsize_ + i] == 254)
				all_temp.push_back(std::pair<int, int>(i, j));
		}
	}
	int n_temp = all_temp.size();
	all_temp.insert(all_temp.end(), all_temp.begin(), all_temp.end());
	all_temp.insert(all_temp.end(), all_temp.begin(), all_temp.end());

	for (unsigned int i = 0; i < n_temp; ++i)
	{
		all_temp[i].first += 1;
		all_temp[i].second += 1;
		all_temp[i + n_temp].first -= 1;
		all_temp[i + n_temp].second -= 1;
		all_temp[i + 2 * n_temp].first += 1;
		all_temp[i + 2 * n_temp].second -= 1;
		all_temp[i + 3 * n_temp].first -= 1;
		all_temp[i + 3 * n_temp].second += 1;
	}

	//// Here we must remove the boundary obstacles, i.e., <= 0 but not < 0
	//// This is because we will find their adjacent obstacles in the next step
	//all_temp.erase(std::remove_if(all_temp.begin(), all_temp.end(),
	//	[&](const auto& a) {return a.first <= 0 || a.first >= xsize_ - 1 || a.second <= 0 || a.second >= ysize_ - 1; }), 
	//	all_temp.end());

	auto all_temp_temp = all_temp;
	all_temp.clear();
	for (auto iter = all_temp_temp.begin(); iter != all_temp_temp.end(); ++iter)
	{
		if (iter->first <= 0 || iter->first >= xsize_ - 1 || iter->second <= 0 || iter->second >= ysize_ - 1)
			continue;
		all_temp.push_back(*iter);
	}


	std::vector<std::pair<int, int> > updated_all_temp;
	updated_all_temp.reserve(all_temp.size());
	for (auto iter = all_temp.begin(); iter != all_temp.end(); ++iter)
	{
		if (data_[iter->second*xsize_ + (iter->first - 1)] == 254 ||
			data_[iter->second*xsize_ + (iter->first + 1)] == 254 ||
			data_[(iter->second - 1)*xsize_ + iter->first] == 254 ||
			data_[(iter->second + 1)*xsize_ + iter->first] == 254)
		{
			continue;
		}
		updated_all_temp.push_back(*iter);
	}
	


	// We use precise location of obstacle corners

	int x, y;
	for (auto iter = updated_all_temp.begin(); iter != updated_all_temp.end(); ++iter)
	{
		x = iter->first;
		y = iter->second;

		int count = 0;
		if (data_[(y - 1)*xsize_ + (x - 1)] == 254)
			count++;
		if (data_[(y - 1)*xsize_ + (x + 1)] == 254)
			count++;
		if (data_[(y + 1)*xsize_ + (x - 1)] == 254)
			count++;
		if (data_[(y + 1)*xsize_ + (x + 1)] == 254)
			count++;

		if (count >= 2)
			continue;


		if (data_[(y - 1)*xsize_ + (x - 1)] == 254)
		{
			//			critical_points.push_back(std::pair<double, double>(x - 0.5, y - 0.5));
			critical_points.push_back(std::pair<double, double>(x, y));
			critical_points_orientation.push_back(-3.0 / 4 * M_PI);
		}
		else if (data_[(y + 1)*xsize_ + (x - 1)] == 254)
		{
			//critical_points.push_back(std::pair<double, double>(x - 0.5, y + 0.5));
			critical_points.push_back(std::pair<double, double>(x, y + 1));
			critical_points_orientation.push_back(3.0 / 4 * M_PI);

		}
		else if (data_[(y - 1)*xsize_ + (x + 1)] == 254)
		{
			//critical_points.push_back(std::pair<double, double>(x + 0.5, y - 0.5));
			critical_points.push_back(std::pair<double, double>(x + 1, y));
			critical_points_orientation.push_back(-1.0 / 4 * M_PI);

		}
		else
		{
			//critical_points.push_back(std::pair<double, double>(x + 0.5, y + 0.5));
			critical_points.push_back(std::pair<double, double>(x + 1, y + 1));
			critical_points_orientation.push_back(1.0 / 4 * M_PI);

		}
	}

	//for (int i = 0; i < temp_obs.size(); i++) {
	//	int x = temp_obs[i][0];
	//	int y = temp_obs[i][1];
	//	int temp_x, temp_y;
	//	temp_x = x - 1, temp_y = y - 1;
	//	if (temp_x >= 0 && temp_x < map.rows() && temp_y >= 0 && temp_y < map.cols()) {
	//		if (map(x - 1, y - 1) == 254) {
	//			Eigen::Vector2f pos;
	//			pos << x - 0.5, y - 0.5;
	//			critical_points.push_back(pos);
	//		}
	//	}

	//	temp_x = x - 1, temp_y = y + 1;
	//	if (temp_x >= 0 && temp_x < map.rows() && temp_y >= 0 && temp_y < map.cols()) {
	//		if (map(x - 1, y + 1) == 254) {
	//			Eigen::Vector2f pos;
	//			pos << x - 0.5, y + 0.5;
	//			critical_points.push_back(pos);
	//		}
	//	}

	//	temp_x = x + 1, temp_y = y + 1;
	//	if (temp_x >= 0 && temp_x < map.rows() && temp_y >= 0 && temp_y < map.cols()) {
	//		if (map(x + 1, y + 1) == 254) {
	//			Eigen::Vector2f pos;
	//			pos << x + 0.5, y + 0.5;
	//			critical_points.push_back(pos);
	//		}
	//	}

	//	temp_x = x + 1, temp_y = y - 1;
	//	if (temp_x >= 0 && temp_x < map.rows() && temp_y >= 0 && temp_y < map.cols()) {
	//		if (map(x + 1, y - 1) == 254) {
	//			Eigen::Vector2f pos;
	//			pos << x + 0.5, y - 0.5;
	//			critical_points.push_back(pos);
	//		}
	//	}

	//}


	//Mmap_ = map;
	//for (int i = 0; i < critical_points.size(); i++) {
	//	std::pair<double, double> temp;
	//	temp.first = critical_points[i](0);
	//	temp.second = critical_points[i](1);

	//	g_obs.push_back(temp);
	//}

}


void Map::sedFill(std::vector<std::pair<double, double> >& ObsPosition)
{
	ObsPosition.clear();
	int nx = xsize_;
	int ny = ysize_;

	// We first copy the map cost value to another 2D array
	int** mask;
	mask = new int*[nx];
	for (unsigned int i = 0; i < nx; ++i)
	{
		mask[i] = new int[ny];
	}
	// Copy value
	for (unsigned int i = 0; i < nx; ++i)
	{
		for (unsigned int j = 0; j < ny; ++j)
		{
			mask[i][j] = data_[i + j * xsize_] == 254 ? 254 : 0;// In matlab, we use the original map, so here we do the same as in matlab
		}
	}



	//for (unsigned int i = 0; i < xsize_; ++i)
	//{
	//	mask[i][0] = 0;
	//	mask[i][ysize_ - 1] = 0;
	//}
	//for (unsigned int j = 0; j < ysize_; ++j)
	//{
	//	mask[0][j] = 0;
	//	mask[xsize_ - 1][j] = 0;
	//}
	std::pair<int, int> seed(0, 0);
	std::vector<std::pair<int, int> > queue;
	queue.push_back(seed);
	while (!queue.empty())
	{
		auto cur = queue.back();
		queue.pop_back();
		if (cur.first >= 0 && cur.second >= 0 && cur.first <= nx - 1 && cur.second <= ny - 1
			&& mask[cur.first][cur.second] >= 254)
		{
			mask[cur.first][cur.second] = 0;
			queue.push_back(std::pair<int, int>(cur.first - 1, cur.second));
			queue.push_back(std::pair<int, int>(cur.first + 1, cur.second));
			queue.push_back(std::pair<int, int>(cur.first, cur.second-1));
			queue.push_back(std::pair<int, int>(cur.first, cur.second+1));
		}
	}



	// We identify obstacles
	for (unsigned int i = 1; i < nx - 1; ++i)
	{
		for (unsigned int j = 1; j < ny - 1; ++j)
		{
			if (mask[i][j] == 0)
			{
				// free space 
				continue;
			}

			std::pair<int, int> seed(i, j);
			// std::cout << "seed: (" << i << ", " << j << "): " << (int)mask[i][j] << std::endl;

			//ObsPosition.push_back(seed);
			ObsPosition.push_back(std::pair<double, double>(seed.first + 0.5, seed.second + 0.5));

			// We start a sedFill to eliminate all connected obstacles;
			std::vector<std::pair<int, int> > queue;
			queue.push_back(seed);
			
			while (!queue.empty())
			{
				auto cur = queue.back();
				if (cur.first >= 0 && cur.first < xsize_ && 
					cur.second >= 0 && cur.second < ysize_ && 
					mask[cur.first][cur.second] == 254)
				{
					mask[cur.first][cur.second] = 0;

					queue.pop_back();
					queue.push_back(std::pair<int, int>(cur.first - 1, cur.second));
					queue.push_back(std::pair<int, int>(cur.first + 1, cur.second));
					queue.push_back(std::pair<int, int>(cur.first, cur.second - 1));
					queue.push_back(std::pair<int, int>(cur.first, cur.second + 1));
				}
				else
				{
					queue.pop_back();
				}
			}//while
		}
	}

	// delete mask
	for (unsigned int i = 0; i < nx; ++i)
		delete[] mask[i];

	delete[] mask;
}


float min(float* array,int size) {
	float min = 10000;
	for (int i = 0; i < size; i++) {
		if (array[i] < min) {
			min = array[i];
		}
	}
	return min;
}

float max(float* array, int size) {
	float max = -10000;
	for (int i = 0; i < size; i++) {
		if (array[i] >max) {
			max = array[i];
		}
	}
	return max;
}

double min(double* array, int size) {
	double min = 10000;
	for (int i = 0; i < size; i++) {
		if (array[i] < min) {
			min = array[i];
		}
	}
	return min;
}

double max(double* array, int size) {
	double max = -10000;
	for (int i = 0; i < size; i++) {
		if (array[i] > max) {
			max = array[i];
		}
	}
	return max;
}


void Map::getCableState(std::vector<double>& obs_list,
	std::pair<double, double> olds, std::pair<double, double> news)
{

	//std::vector<std::pair<double, double> > tri;
	static double tri[6];
	float tri_vertx[3];
	float tri_verty[3];
	/*tri.push_back(obs_list.back());
	tri.push_back(olds);
	tri.push_back(news);*/

	/*tri[0] = obs_list.back().first;
	tri[1] = obs_list.back().second;
	tri[2] = olds.first;
	tri[3] = olds.second;
	tri[4] = news.first;
	tri[5] = news.second;*/
	int obs_size = obs_list.size();
	
	tri_vertx[0] = obs_list[obs_size - 2];
	tri_vertx[1] = olds.first;
	tri_vertx[2] = news.first;

	tri_verty[0] = obs_list[obs_size - 1];
	tri_verty[1] = olds.second;
	tri_verty[2] = news.second;
	

	//std::vector<double> result_obs_list;

	//std::vector<std::pair<double, double> > temp_critical_points(critical_points.begin(), critical_points.end());

	int temp_x[1000];
	int temp_y[1000];



	// Avoiding insert the last obstacle again
	/*for (auto iter = temp_critical_points.begin(); iter != temp_critical_points.end(); ++iter)
	{
		if (fabs(iter->first - tri[0].first) < 0.1 && fabs(iter->second - tri[0].second) < 0.1)
		{
			iter->first = -1;
			iter->second = -1;
			break;
		}
	}*/

	//for (int i = 0; i < critical_points.size(); ++i)
	//{
	//	temp_x[i] = critical_points[i].first;
	//	temp_y[i] = critical_points[i].second;
	//	if (fabs(temp_x[i] - tri_vertx[0]) < 0.1 && fabs(temp_y[i] - tri_verty[0]) < 0.1)
	//	{
	//		temp_x[i] = -1;
	//		temp_y[i] = -1;
	//		break;
	//	}

	//}

	// 2022.08.29 YT: We use memcpy
	memcpy(temp_x, critical_points_x_, sizeof(int)*critical_points_num_);
	memcpy(temp_y, critical_points_y_, sizeof(int)*critical_points_num_);
	int int_tri_vertx_0 = (int)(tri_vertx[0]);
	int int_tri_verty_0 = (int)(tri_verty[0]);
	for (unsigned int i = 0; i < critical_points_num_; ++i)
	{
		if (temp_x[i] == int_tri_vertx_0 && temp_y[i] == int_tri_verty_0)
		{
			temp_x[i] = -1;
			temp_y[i] = -1;
			break;
		}
	}

	//std::vector<bool> in_or_on;

	static bool in_or_on[1000];
	int n = critical_points.size();


	/*std::vector<float> vertx;
	std::vector<float> verty;

	for (auto iter = tri.begin(); iter != tri.end(); ++iter)
	{
		vertx.push_back(iter->first);
		verty.push_back(iter->second);
	}*/

	//bool Map::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)

	float min_tri_x = min(tri_vertx, 3);
	float min_tri_y = min(tri_verty, 3);
	float max_tri_x = max(tri_vertx, 3);
	float max_tri_y = max(tri_verty, 3);
	//in_or_on.resize(temp_critical_points.size(),false);

	/*for (auto iter = temp_critical_points.begin(); iter != temp_critical_points.end(); ++iter)
	{
		if (iter->first > max_tri_x || iter->first<min_tri_x || iter->second>max_tri_y || iter->second < min_tri_y) {
			in_or_on.push_back(false);
		}
		else {
			in_or_on.push_back(pnpoly(tri.size(), &(vertx[0]), &(verty[0]), iter->first, iter->second));
		}
	}*/

	for (int i = 0; i < n; ++i)
	{
		if (temp_x[i] > max_tri_x || temp_x[i]<min_tri_x || temp_y[i]>max_tri_y || temp_y[i] < min_tri_y) {
			in_or_on[i] = (false);
		}
		else {
			//in_or_on[i] = (pnpoly(3, &(vertx[0]), &(verty[0]), temp_x[i], temp_y[i]));
			in_or_on[i] = (pnpoly(3, tri_vertx, tri_verty, temp_x[i], temp_y[i]));
		}
	}


	bool any_in_or_on = false;
	//std::vector<Eigen::Vector2f> corner_list;
	//for (int i = 0; i < tri.size(); i++) {
	//	bool In = pnpoly(temp_critical_points, tri[i](0), tri[i](1));
	//	if (In) {
	//		in_flag = false;
	//		Eigen::Vector2f pos;
	//		pos << temp_critical_points[i](0), temp_critical_points[i](1);
	//		corner_list.push_back(pos);
	//	}

	//}

	for (int i = 0; i < n; i++)
	{
		if (in_or_on[i])
		{
			any_in_or_on = true;
			break;
		}
	}


	if (!any_in_or_on) {
		//remove obstacle
		//result_obs_list = tryRemoveObstacles(obs_list, news);
		obs_list = tryRemoveObstacles(obs_list, news);
	}
	else
	{
		double pre_theta = atan2(tri_verty[1] - tri_verty[0], tri_vertx[1] - tri_vertx[0]);

		std::pair<double, double> temp_point;
		std::vector<std::pair<double, double> > corners;
		for (unsigned int i = 0; i < n; ++i)
		{
			if (in_or_on[i])
			{
				temp_point.first = temp_x[i];
				temp_point.second = temp_y[i];
				corners.push_back(temp_point);
			}
		}

		// We find the corner with least angle diff
		std::vector<std::pair<double, double> > diff(corners.begin(), corners.end());
		std::vector<double> theta;
		std::vector<double> temp_theta;
		for (auto iter = diff.begin(); iter != diff.end(); ++iter)
		{
			iter->first -= tri_vertx[0];
			iter->second -= tri_verty[0];
			theta.push_back(atan2(iter->second, iter->first));
			temp_theta.push_back(fabs(normalize_angle(theta.back() - pre_theta)));
		}

		// Find min_theta, and all corners that have min_theta
		double min_theta = temp_theta[0];
		std::vector<std::pair<double, double> > corner;
		//corner.push_back(corners[0]);

		for (int i = 0; i < temp_theta.size(); i++) {
			if (temp_theta[i] < min_theta) {
				corner.clear();
				corner.push_back(corners[i]);
				min_theta = temp_theta[i];
			}
			else if (temp_theta[i] == min_theta)
			{
				corner.push_back(corners[i]);
			}
		}

		// If there are multiple least-angle-diff corners, we choose the nearest one
		std::pair<double, double> single_corner;
		if (corner.size() == 1)
		{
			single_corner = corner[0];
		}
		else
		{
//			std::vector<double> dis;
			double dx = corner[0].first - tri_vertx[0];
			double dy = corner[0].second - tri_verty[0];
			double min_dis = sqrt(dx*dx + dy * dy);
			single_corner = corner[0];
			for (auto iter = corner.begin(); iter != corner.end(); ++iter)
			{
				dx = iter->first - tri_vertx[0];
				dy = iter->second - tri_verty[0];
				double new_dis = sqrt(dx*dx + dy * dy);
				if (new_dis < min_dis)
				{
					min_dis = new_dis;
					single_corner = *iter;
				}
			}
		}

		//result_obs_list.reserve(obs_list.size() + 10);
		//result_obs_list.clear();
		//result_obs_list.insert(result_obs_list.begin(), obs_list.begin(), obs_list.end());
		//result_obs_list.push_back(single_corner.first);
		//result_obs_list.push_back(single_corner.second);
		obs_list.push_back(single_corner.first);
		obs_list.push_back(single_corner.second);


		//result_obs_list = getCableState(result_obs_list, olds, news);
		//getCableState(result_obs_list, olds, news);
		getCableState(obs_list, olds, news);

	}

	//return result_obs_list;
	return;
}

void Map::init_footprint()
//void Map::init_footprint(std::vector<double> f_x, std::vector<double> f_y)
{
//	for (unsigned int i = 0; i < f_x.size(); ++i)
	for(auto iter = footprint_.begin(); iter != footprint_.end(); ++iter)
	{
		//double r = sqrt(f_x[i] * f_x[i] + f_y[i] * f_y[i]);
		//double theta = atan2(f_y[i], f_x[i]);
		double r = sqrt(iter->first*iter->first + iter->second*iter->second);
		double theta = atan2(iter->second, iter->first);
		polar_footprint_.push_back(std::pair<double, double>(r, theta));
	}
}

std::vector<double> Map::tryRemoveObstacles(std::vector<double> obs_list, std::pair<double, double> news)
{
	std::vector<double> result_obs_list(obs_list.begin(), obs_list.end());

	if (obs_list.size() < 4)
	{
		return result_obs_list;
	}

	std::vector<double> tri;

	int n = obs_list.size();
	tri.push_back(obs_list[n - 4]);
	tri.push_back(obs_list[n - 3]);
	tri.push_back(obs_list[n - 2]);
	tri.push_back(obs_list[n - 1]);

	tri.push_back(news.first);
	tri.push_back(news.second);


	//std::vector<std::pair<int, int> > result;
	//newbresenham(tri[0], tri[1], tri[4], tri[5], result);
	//for (int i = 0; i < result.size(); i++) {
	//	if (data_[result[i].second * xsize_ + result[i].first] != 0) {
	//		return obs_list;
	//	}
	//}

	//// 2022.08.29 YT: we use std::vector<int> bresenham
	//std::vector<int> result;
	//newbresenham(tri[0], tri[1], tri[4], tri[5], xsize_, result);
	//for (int i = 0; i < result.size(); i++) {
	//	if (data_[result[i]] != 0) {
	//		return obs_list;
	//	}
	//}

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	double pre_theta = atan2(tri[3] - tri[1], tri[2] - tri[0]);
	//	pre_theta = atan2(tri(2, 2) - tri(1, 2), tri(2, 1) - tri(1, 1));
	double post_theta = atan2(tri[5] - tri[3], tri[4] - tri[2]);
	//post_theta = atan2(tri(3, 2) - tri(2, 2), tri(3, 1) - tri(2, 1));
	double d_theta = normalize_angle(post_theta - pre_theta);
	//d_theta = wrapToPi(post_theta - pre_theta);
	double test_theta;
	if (d_theta < 0)
		//if d_theta < 0
		test_theta = pre_theta - M_PI / 2;
	//	test_theta = pre_theta - pi / 2;
	//else
	else
		//	test_theta = pre_theta + pi / 2;
		test_theta = pre_theta + M_PI / 2;
	//end

	int loc = std::find_if(critical_points.begin(), critical_points.end(),
		[&](const auto& a) {return a.first == tri[2] && a.second == tri[3]; }) - critical_points.begin();
	//	temp = critical_points(:, 1) == tri(2, 1);
	//if any(temp)
	//	temp = temp & critical_points(:, 2) == tri(2, 2);
	//if any(temp)
	//	loc = find(temp);
	//obs_theta = critical_points_orientation(loc);
	//end
	//	end

	double obs_theta = critical_points_orientation[loc];

	double result_theta = fabs(normalize_angle(test_theta - obs_theta));
	//	result_theta = abs(wrapToPi(test_theta - obs_theta));
	if (result_theta < M_PI / 2)
		//if result_theta < pi / 2
		return result_obs_list;
	//	return;
	//end

	//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	//std::vector<std::pair<double, double>> temp_critical_points = critical_points;

	int temp_x[1000];
	int temp_y[1000];

	// Avoiding insert the obstacle again
	//for (int i = 0; i < critical_points.size(); i++) {
	//	if (critical_points[i].first == tri[0].first) {
	//		if (critical_points[i].second == tri[0].second)
	//			temp_critical_points[i].first = -1;
	//		temp_critical_points[i].second = -1;
	//	}

	//}
	/*for (auto iter = critical_points.begin(); iter != critical_points.end(); ++iter)
	{
		if (fabs(iter->first - tri[0].first) < 0.1 && fabs(iter->second - tri[0].second) < 0.1)
		{
			temp_x = -1;
			iter->second = -1;
		}
		if (fabs(iter->first - tri[1].first) < 0.1 && fabs(iter->second - tri[1].second) < 0.1)
		{
			iter->first = -1;
			iter->second = -1;
		}
	}*/

	for (int i = 0; i < critical_points.size(); i++) {
		temp_x[i] = round(critical_points[i].first);
		temp_y[i] = round(critical_points[i].second);
		//x0 y0
		if (fabs(critical_points[i].first - tri[0]) < 0.1&& fabs(critical_points[i].second - tri[1]) < 0.1) {
			temp_x[i] = -1;
			temp_y[i] = -1;
		}

		//x1 y1

		if (fabs(critical_points[i].first - tri[2]) < 0.1&& fabs(critical_points[i].second - tri[3]) < 0.1) {
			temp_x[i] = -1;
			temp_y[i] = -1;
		}
	}


	//std::vector<bool> in_or_on;
	std::vector<float> vertx;
	std::vector<float> verty;

	static bool in_or_on[1000];
	int size = critical_points.size();
	/*for (auto iter = tri.begin(); iter != tri.end(); ++iter)
	{
		vertx.push_back(iter->first);
		verty.push_back(iter->second);
	}*/

	for (int i = 0; i < 3; i++) {
		vertx.push_back(tri[2 * i]);
		verty.push_back(tri[2 * i+1]);
	}

	//bool Map::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)


	float min_tri_x = min(&(vertx[0]), vertx.size());
	float min_tri_y = min(&(verty[0]), verty.size());
	float max_tri_x = max(&(vertx[0]), vertx.size());
	float max_tri_y = max(&(verty[0]), verty.size());
	//in_or_on.resize(temp_critical_points.size(),false);


	for (int i = 0; i < size; ++i)
	{
		if (temp_x[i] > max_tri_x || temp_x[i]<min_tri_x || temp_y[i]>max_tri_y || temp_y[i] < min_tri_y) {
			in_or_on[i] = (false);
		}
		else {
			in_or_on[i] = (pnpoly(tri.size(), &(vertx[0]), &(verty[0]), temp_x[i], temp_y[i]));
		}
	}


	bool any_in_or_on = false;

	//for (int i = 0; i < tri.size(); i++) {
	//	bool In = pnpoly(temp_critical_points, tri[0].first, tri[0].second);
	//	if (In) {
	//		in_flag = false;
	//		break;
	//	}

	//}

	for (int i = 0; i < size; i++)
	{
		if (in_or_on[i])
		{
			any_in_or_on = true;
			break;
		}
	}


	if (!any_in_or_on) {
		result_obs_list.pop_back();
		result_obs_list.pop_back();

		result_obs_list = tryRemoveObstacles(result_obs_list, news);
	}

	return result_obs_list;
}



