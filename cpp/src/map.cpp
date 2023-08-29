
#include "map.h"
#include "tool.h"
#include <string>
#include <opencv2/opencv.hpp>

Map::Map(std::string map_filename) 
{
	chassis_L_ = 3.0;
	primitive_path_length_ = 6.0;
	validity_checking_resolution_ = 1.0; // Change here to try different settings

	critical_points_.clear();
	critical_points_orientation.clear();
	generatePolyMap(map_filename);

	critical_points_num_ = critical_points_.size();

	sparse_checking_num_ = 0;
	dense_checking_num_ = 0;
	all_checking_num_ = 0;
}

void Map::generatePolyMap(std::string map_filename)
{
	// opencv is only used here for generating grid-map
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

	// We obtain distinct obstacles
	sedFill();

	// We calculate the critical obstacles (grids)
	// For each obstacle, we check whether its 8-dir neighbour is a 4-neighbor obstacle-free grid

	// We collect all obstacle grids
	std::vector<std::pair<int, int> > all_temp;
	all_temp.reserve(xsize_*ysize_);
	for (unsigned int i = 0; i < xsize_; ++i)
	{
		for (unsigned int j = 0; j < ysize_; ++j)
		{
			if (data_[j*xsize_ + i] == 254)
				all_temp.emplace_back(std::pair<int, int>(i, j));
		}
	}
	
	// We get the 4-neighbours of obstacle grids
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
	auto all_temp_temp = all_temp;
	all_temp.clear();
	all_temp.reserve(all_temp_temp.size());
	for (auto iter = all_temp_temp.begin(); iter != all_temp_temp.end(); ++iter)
	{
		if (iter->first <= 0 || iter->first >= xsize_ - 1 || iter->second <= 0 || iter->second >= ysize_ - 1)
			continue;
		all_temp.emplace_back(*iter);
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
		updated_all_temp.emplace_back(*iter);
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

		// Here we don't add 0.5 because in C++ the obstacle occupies integer grids
		// The orientation is pointing INSIDE the obstacle corner
		if (data_[(y - 1)*xsize_ + (x - 1)] == 254)
		{
			critical_points_.emplace_back(std::pair<double, double>(x, y));
			critical_points_orientation.emplace_back(-3.0 / 4 * M_PI);
		}
		else if (data_[(y + 1)*xsize_ + (x - 1)] == 254)
		{
			critical_points_.emplace_back(std::pair<double, double>(x, y + 1));
			critical_points_orientation.emplace_back(3.0 / 4 * M_PI);
		}
		else if (data_[(y - 1)*xsize_ + (x + 1)] == 254)
		{
			critical_points_.emplace_back(std::pair<double, double>(x + 1, y));
			critical_points_orientation.emplace_back(-1.0 / 4 * M_PI);
		}
		else
		{
			critical_points_.emplace_back(std::pair<double, double>(x + 1, y + 1));
			critical_points_orientation.emplace_back(1.0 / 4 * M_PI);
		}
	}
}

void Map::sedFill()
{
	obs_.clear();
	unsigned int nx = xsize_;
	unsigned int ny = ysize_;

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

	// We clear the outer obstacle (if it exists)
	std::pair<int, int> seed(0, 0);
	std::vector<std::pair<int, int> > queue;
	queue.reserve(nx*ny);
	queue.emplace_back(seed);
	while (!queue.empty())
	{
		auto cur = queue.back();
		queue.pop_back();
		if (cur.first >= 0 && cur.second >= 0 && cur.first <= nx - 1 && cur.second <= ny - 1
			&& mask[cur.first][cur.second] >= 254)
		{
			mask[cur.first][cur.second] = 0;
			if(cur.first-1 >= 0 && mask[cur.first - 1][cur.second] >= 254) 
				queue.emplace_back(std::pair<int, int>(cur.first - 1, cur.second));
			if (cur.first +1 <= nx-1 && mask[cur.first + 1][cur.second] >= 254)
				queue.emplace_back(std::pair<int, int>(cur.first + 1, cur.second));
			if (cur.second-1 >=0 && mask[cur.first][cur.second - 1] >= 254)
				queue.emplace_back(std::pair<int, int>(cur.first, cur.second-1));
			if (cur.second +1 <= ny-1 && mask[cur.first][cur.second + 1] >= 254)
				queue.emplace_back(std::pair<int, int>(cur.first, cur.second+1));
		}
	}

	// We identify internal obstacles
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

			obs_.emplace_back(Obs());
			obs_.back().ObsPosition.first = seed.first + 0.5;
			obs_.back().ObsPosition.second = seed.second + 0.5;

			// We start a sedFill to eliminate all connected obstacles;
			std::vector<std::pair<int, int> > queue;
			queue.reserve(nx*ny);
			queue.emplace_back(seed);			
			while (!queue.empty())
			{
				auto cur = queue.back();
				if (cur.first > 0 && cur.first < xsize_-1 && 
					cur.second > 0 && cur.second < ysize_-1 && 
					mask[cur.first][cur.second] >= 254)
				{
					mask[cur.first][cur.second] = 0;

					// We store the grids of a connected obstacle (maybe we can develop visibility graph from it)
					obs_.back().grids_.emplace_back(std::pair<int, int>(cur.first, cur.second));

					queue.pop_back();
					if(mask[cur.first -1][cur.second] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first - 1, cur.second));
					if (mask[cur.first + 1][cur.second] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first + 1, cur.second));
					if (mask[cur.first][cur.second - 1] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first, cur.second - 1));
					if (mask[cur.first][cur.second + 1] >= 254)
						queue.emplace_back(std::pair<int, int>(cur.first, cur.second + 1));
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

bool Map::straightenATwoSegmentCurve(std::vector<double>& old_curve, std::vector<double>& new_curve)
{
	// The old curve must have only three elements (i.e., 6 numbers)
	static double tri_x[3] = {old_curve[0], old_curve[2], old_curve[4]};
	static double tri_y[3] = {old_curve[1], old_curve[3], old_curve[5]};
	static double tri[6] = { old_curve[0], old_curve[1], old_curve[2], old_curve[3], old_curve[4], old_curve[5] };

	// We need to make sure that the middle obstacle can be straightened
	int critical_point_index = -1;
	for (unsigned int i = 0; i < critical_points_.size(); ++i)
	{
		if (fabs(critical_points_[i].first - tri_x[1]) < 0.25 &&
			fabs(critical_points_[i].second - tri_y[1]) < 0.25)
		{
			critical_point_index = i;
			break;
		}
	}
	if (critical_point_index != -1)
	{
		double inside_ori = critical_points_orientation[critical_point_index];
		double pre_dis = sqrt((tri_x[0] - tri_x[1])*(tri_x[0] - tri_x[1]) + (tri_y[0] - tri_y[1])*(tri_y[0] - tri_y[1]));
		double post_dis = sqrt((tri_x[2] - tri_x[1])*(tri_x[2] - tri_x[1]) + (tri_y[2] - tri_y[1])*(tri_y[2] - tri_y[1]));
		std::pair<double, double> stretch_pre_dir((tri_x[0] - tri_x[1]) / pre_dis, (tri_y[0] - tri_y[1]) / pre_dis);
		std::pair<double, double> stretch_post_dir((tri_x[2] - tri_x[1]) / post_dis, (tri_y[2] - tri_y[1]) / post_dis);
		std::pair<double, double> stretch_dir(stretch_pre_dir.first + stretch_post_dir.first, stretch_pre_dir.second + stretch_post_dir.second);
		if (stretch_dir.first*cos(inside_ori) + stretch_dir.second*sin(inside_ori) > 0)
		{
			// cannot stretch. we just return the same tether
			new_curve = old_curve;
			return true;
		}
	}

	// We calculate the boundary orientations
	double pre_theta = atan2(tri_y[1] - tri_y[0], tri_x[1] - tri_x[0]);
	double post_theta = atan2(tri_y[2] - tri_y[0], tri_x[2] - tri_x[0]);
	double d_theta = normalize_angle(post_theta - pre_theta);

	// We check whether there exist obstacle vertices that are in the triangle
	std::vector<double> in_or_on_theta;
	in_or_on_theta.reserve(critical_points_.size());
	std::vector<std::pair<double, double> > in_or_on_obs_list;
	in_or_on_obs_list.reserve(critical_points_.size());
	for (auto iter = critical_points_.begin(); iter != critical_points_.end(); ++iter)
	{
		if (isPointInOrOnTri(&(tri[0]), iter->first, iter->second))
		{
			// Here we need to remove the three vertices of the triangle (if they are not in then better)
			if (isTriVertex(&(tri[0]), iter->first, iter->second))
			{
				continue;
			}
			double the_theta = atan2(iter->second - tri[1], iter->first - tri[0]);
			the_theta = pre_theta + normalize_angle(the_theta - pre_theta);
			in_or_on_theta.emplace_back(the_theta);
			in_or_on_obs_list.emplace_back(*iter);
		}
	}

	new_curve.clear();
	if (in_or_on_theta.empty())
	{
		// There is no need to keep three vertices. We just remove the middle one
		new_curve = { old_curve[0], old_curve[1], old_curve[4], old_curve[5] };
		return true;
	}
	else
	{
		// We have to detemine which obstacle vertex is to be contacted NEXT
		// If there is only one internal point, we just choose it
		double min_index = 0;
		double min_dis = sqrt((tri[0] - in_or_on_obs_list[min_index].first)*(tri[0] - in_or_on_obs_list[min_index].first)
			+ (tri[1] - in_or_on_obs_list[min_index].second)*(tri[1] - in_or_on_obs_list[min_index].second));
		double min_theta = in_or_on_theta[0];

		// If there are multiple internal points, we have to choose one
		if (in_or_on_theta.size() > 1)
		{
			if (d_theta > 0)
			{
				for (unsigned int i = 1; i < in_or_on_theta.size(); ++i)
				{
					double the_theta = in_or_on_theta[i];
					if (the_theta > min_theta + SEFTPP_EPS)
					{
						continue;
					}
					else if (the_theta < min_theta - SEFTPP_EPS)
					{
						min_index = i;
						min_theta = the_theta;
						min_dis = sqrt((tri[0] - in_or_on_obs_list[min_index].first)*(tri[0] - in_or_on_obs_list[min_index].first)
							+ (tri[1] - in_or_on_obs_list[min_index].second)*(tri[1] - in_or_on_obs_list[min_index].second));
					}
					else
					{
						// Maybe multiple vertices are colinear. So we have to choose the nearest one
						double the_dis = sqrt((tri[0] - in_or_on_obs_list[i].first)*(tri[0] - in_or_on_obs_list[i].first)
							+ (tri[1] - in_or_on_obs_list[i].second)*(tri[1] - in_or_on_obs_list[i].second));
						if (the_dis < min_dis - SEFTPP_EPS)
						{
							min_index = i;
							min_theta = the_theta;
							min_dis = sqrt((tri[0] - in_or_on_obs_list[min_index].first)*(tri[0] - in_or_on_obs_list[min_index].first)
								+ (tri[1] - in_or_on_obs_list[min_index].second)*(tri[1] - in_or_on_obs_list[min_index].second));
						}
					}
				}
			}
			else // d_theta < 0
			{
				for (unsigned int i = 1; i < in_or_on_theta.size(); ++i)
				{
					double the_theta = in_or_on_theta[i];
					if (the_theta < min_theta - SEFTPP_EPS)
					{
						continue;
					}
					else if (the_theta < min_theta + SEFTPP_EPS)
					{
						min_index = i;
						min_theta = the_theta;
						min_dis = sqrt((tri[0] - in_or_on_obs_list[min_index].first)*(tri[0] - in_or_on_obs_list[min_index].first)
							+ (tri[1] - in_or_on_obs_list[min_index].second)*(tri[1] - in_or_on_obs_list[min_index].second));
					}
					else
					{
						// Maybe multiple vertices are colinear. So we have to choose the nearest one
						double the_dis = sqrt((tri[0] - in_or_on_obs_list[i].first)*(tri[0] - in_or_on_obs_list[i].first)
							+ (tri[1] - in_or_on_obs_list[i].second)*(tri[1] - in_or_on_obs_list[i].second));
						if (the_dis < min_dis - SEFTPP_EPS)
						{
							min_index = i;
							min_theta = the_theta;
							min_dis = sqrt((tri[0] - in_or_on_obs_list[min_index].first)*(tri[0] - in_or_on_obs_list[min_index].first)
								+ (tri[1] - in_or_on_obs_list[min_index].second)*(tri[1] - in_or_on_obs_list[min_index].second));
						}
					}
				}
			}
		}// end if there are multiple internal points. We've chosen the next tether-obstacle contact point

		new_curve = { old_curve[0], old_curve[1] };
		std::vector<double> temp_old_curve = { in_or_on_obs_list[min_index].first, in_or_on_obs_list[min_index].second,
			old_curve[2], old_curve[3], old_curve[4], old_curve[5] };

		std::vector<double> temp_new_curve;
		straightenATwoSegmentCurve(temp_old_curve, temp_new_curve);
		new_curve.insert(new_curve.end(), temp_new_curve.begin(), temp_new_curve.end());
		return false;
	}
}

void Map::deformTether(const std::vector<double>& old_tether_shape,
	std::vector<double>& new_tether_shape)
{
	// The input contains the location of the robot's S position. 

	new_tether_shape.clear();
	new_tether_shape.reserve(old_tether_shape.size() + 10);
	new_tether_shape = old_tether_shape;

	if (old_tether_shape.size() <= 4)
	{
		return ;
	}

	int n = new_tether_shape.size();
	int i = n - 6;
	while (1)
	{
		std::vector<double> old_part_tether(new_tether_shape.begin() + i, new_tether_shape.begin() + i + 6);
		std::vector<double> new_part_tether;

		bool b = straightenATwoSegmentCurve(old_part_tether, new_part_tether);
		// the new part tether is just removing the middle obstacle point
		// or the tether is not changed
		// then b is true
		if (b && new_part_tether.size() == 4)
		{
			new_tether_shape.erase(new_tether_shape.begin() + i + 3);
			new_tether_shape.erase(new_tether_shape.begin() + i + 2);
			n = new_tether_shape.size();
			i = std::min(i - 2, n - 6);
			if (i < 0)
				break;
		}
		else if (b && new_part_tether.size() == 6)
		{
			n = new_tether_shape.size();
			i = std::min(i - 2, n - 6);
			if (i < 0)
				break;
		}
		else
		{
			// The tether encounters new obstacles. We should add them
			new_tether_shape.erase(new_tether_shape.begin() + i, new_tether_shape.begin() + i + 6);
			new_tether_shape.insert(new_tether_shape.begin() + i, new_part_tether.begin(), new_part_tether.end());
			n = new_tether_shape.size();

			i = std::min((int)(i + new_part_tether.size()), n - 6);
			if (i < 0)
				break;
		}
	}
}

void Map::newGetCableState(std::vector<double>& old_tether_shape,
	std::pair<double, double> olds, std::pair<double, double> news)
{
	std::vector<double> result_tether;
	old_tether_shape.emplace_back(olds.first);
	old_tether_shape.emplace_back(olds.second);
	old_tether_shape.emplace_back(news.first);
	old_tether_shape.emplace_back(news.second);
	deformTether(old_tether_shape, result_tether);
	old_tether_shape.assign(result_tether.begin(), result_tether.end());
}

void Map::init_footprint()
{
	for(auto iter = footprint_.begin(); iter != footprint_.end(); ++iter)
	{
		double r = sqrt(iter->first*iter->first + iter->second*iter->second);
		double theta = atan2(iter->second, iter->first);
		polar_footprint_.emplace_back(std::pair<double, double>(r, theta));
	}
}