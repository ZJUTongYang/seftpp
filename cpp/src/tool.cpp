#include "tool.h"
#include <vector>
#include <queue>
#include "map.h"
#include "solver.h"

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

    std::vector<std::pair<double, double> >  Map::polarRotateAndMoveToXy(double x, double y, double theta)
    {
        std::vector<std::pair<double, double> > after_polar(polar_footprint_.begin(), polar_footprint_.end());
        for(unsigned int i = 0; i < after_polar.size(); ++i)
        {
            after_polar[i].second += theta;
        }

        std::vector<std::pair<double, double> > footprint(after_polar.begin(), after_polar.end());
        for(unsigned int i = 0; i < after_polar.size(); ++i)
        {
            footprint[i].first = after_polar[i].first * cos(after_polar[i].second) + x;
            footprint[i].second = after_polar[i].first * sin(after_polar[i].second) + y;
        }
        return footprint;

    }

    std::pair<double, double>  Map::polarRotateAndMoveS(double x, double y, double theta)
    {
        std::pair<double, double> after_polar = polar_footprint_[0];

        after_polar.second += theta;

        std::pair<double, double> footprint;

        footprint.first = after_polar.first * cos(after_polar.second) + x;
        footprint.second = after_polar.first * sin(after_polar.second) + y;

        return footprint;

    }

	bool pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
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


	void newbresenham(double x1, double y1, double x2, double y2, int xsize, std::vector<int>& result)
	{
		std::vector<std::pair<int, int> > temp_result;
		newbresenham(x1, y1, x2, y2, temp_result);
		result.clear();
		for (auto iter = temp_result.begin(); iter != temp_result.end(); ++iter)
		{
			result.push_back(iter->second*xsize + iter->first);
		}

	}


	//void newbresenham(double x1, double y1, double x2, double y2, int xsize, std::vector<int>& result) {
	//	// std::vector<std::pair<int, int> > result;
	//	result.clear();

	//	int floorx1 = floor(x1);
	//	int floorx2 = floor(x2);
	//	int floory1 = floor(y1);
	//	int floory2 = floor(y2);

	//	result.reserve(abs(floorx2 - floorx1) + 1 + abs(floory2 - floory1) + 1);

	//	// if they lie vertically (one above another), we cannot calculate k
	//	// so we direct solve this case
	//	if (floorx1 == floorx2)
	//	{
	//		// std::vector<int> L = intelist(y1, y2);
	//		if (floory1 < floory2)
	//		{
	//			for (int i = floory1; i <= floory2; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx1, i));
	//				result.push_back(i*xsize + floorx1);
	//			}
	//		}
	//		else if (floory1 == floory2)
	//		{
	//			//result.push_back(std::pair<int, int>(floorx1, floory1));
	//			result.push_back(floory1*xsize + floorx1);
	//		}
	//		else
	//		{
	//			for (int i = floory1; i >= floory2; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx1, i));
	//				result.push_back(i*xsize + floorx1);
	//			}
	//		}
	//		return;
	//	}

	//	if (floory1 == floory2)
	//	{
	//		if (floorx1 < floorx2)
	//		{
	//			for (int i = floorx1; i <= floorx2; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(i, floory1));
	//				result.push_back(floory1*xsize + i);
	//			}
	//		}
	//		else if (floorx1 == floorx2)
	//		{
	//			//result.push_back(std::pair<int, int>(floorx1, floory1));
	//			result.push_back(floory1*xsize + floorx1);
	//		}
	//		else
	//		{
	//			for (int i = floorx1; i >= floorx2; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(i, floory1));
	//				result.push_back(floory1*xsize + i);
	//			}
	//		}
	//		return;
	//	}

	//	// we get the function of the ray in y=kx+b
	//	double k = (y2 - y1) / (x2 - x1);
	//	double b = y1 - k * x1;

	//	int flag;


	//	std::vector<std::pair<int, int> > v;
	//	//v.reserve(abs(floorx2 - floorx1));
	//	v.resize(abs(floorx2 - floorx1));
	//	int count = 0;
	//	if (x1 < x2)
	//	{
	//		flag = 1;
	//		for (int i = floorx1 + 1; i <= floorx2; ++i)
	//		{
	//			//v.push_back(std::pair<int, int>(i, floor(k*i + b)));
	//			v[count++] = std::pair<int, int>(i, floor(k*i + b));
	//		}
	//	}
	//	else
	//	{
	//		flag = -1;
	//		for (int i = floorx1; i >= floorx2 + 1; --i)
	//		{
	//			//v.push_back(std::pair<int, int>(i, floor(k*i + b)));
	//			v[count++] = std::pair<int, int>(i, floor(k*i + b));
	//		}
	//	}

	//	int temp = sign(k)*flag;

	//	if (floorx2 > floorx1)
	//	{
	//		// we insert the start column
	//		if (floory1 < v.front().second)
	//		{
	//			for (int i = floory1; i <= v.front().second; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx1, i));
	//				result.push_back(i*xsize + floorx1);
	//			}
	//		}
	//		else
	//		{
	//			for (int i = floory1; i >= v.front().second; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx1, i));
	//				result.push_back(i*xsize + floorx1);
	//			}
	//		}

	//		// we insert the intermediate column
	//		if (abs(floorx2 - floorx1) > 1)
	//		{

	//			// if the ray only passes two columns, then there is no intermediate columns
	//			for (unsigned int i = 0; i < v.size() - 1; ++i)
	//			{
	//				int xinsert = v[i].first;
	//				int ystart = v[i].second;
	//				int yend = v[i + 1].second;
	//				if (ystart < yend)
	//				{
	//					for (int j = ystart; j <= yend; ++j)
	//					{
	//						//result.push_back(std::pair<int, int>(xinsert, j));
	//						result.push_back(j*xsize + xinsert);
	//					}
	//				}
	//				else
	//				{
	//					for (int j = ystart; j >= yend; --j)
	//					{
	//						//result.push_back(std::pair<int, int>(xinsert, j));
	//						result.push_back(j*xsize + xinsert);
	//					}
	//				}
	//			}

	//		}

	//		// we insert the final column
	//		if (floory2 > v.back().second)
	//		{
	//			int xinsert = v.back().first;
	//			for (int i = v.back().second; i <= floory2; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(xinsert, i));
	//				result.push_back(i*xsize + xinsert);
	//			}
	//		}
	//		else
	//		{
	//			int xinsert = v.back().first;
	//			for (int i = v.back().second; i >= floory2; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(xinsert, i));
	//				result.push_back(i*xsize + xinsert);
	//			}
	//		}

	//	}//floorx2 > floorx1
	//	else // floorx1 is greater than floorx2. We do things backwards
	//	{
	//		int xinsert = v.front().first;
	//		// we insert the start column
	//		if (floory1 < v.front().second)
	//		{
	//			for (int i = floory1; i <= v.front().second; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(xinsert, i));
	//				result.push_back(i*xsize + xinsert);
	//			}
	//		}
	//		else
	//		{
	//			for (int i = floory1; i >= v.front().second; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(xinsert, i));
	//				result.push_back(i*xsize + xinsert);
	//			}
	//		}

	//		// we insert the intermediate column
	//		if (abs(floorx2 - floorx1) > 1)
	//		{

	//			// if the ray only passes two columns, then there is no intermediate columns
	//			for (unsigned int i = 0; i < v.size() - 1; ++i)
	//			{
	//				int xinsert = v[i + 1].first;//Here is different
	//				int ystart = v[i].second;
	//				int yend = v[i + 1].second;
	//				if (ystart < yend)
	//				{
	//					for (int j = ystart; j <= yend; ++j)
	//					{
	//						//result.push_back(std::pair<int, int>(xinsert, j));
	//						result.push_back(j*xsize + xinsert);
	//					}
	//				}
	//				else
	//				{
	//					for (int j = ystart; j >= yend; --j)
	//					{
	//						//result.push_back(std::pair<int, int>(xinsert, j));
	//						result.push_back(j*xsize + xinsert);
	//					}
	//				}
	//			}

	//		}

	//		// we insert the final column
	//		if (floory2 > v.back().second)
	//		{
	//			// int xinsert = v.back().first;
	//			for (int i = v.back().second; i <= floory2; ++i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx2, i));
	//				result.push_back(i*xsize + floorx2);
	//			}
	//		}
	//		else
	//		{
	//			// int xinsert = v.back().first;
	//			for (int i = v.back().second; i >= floory2; --i)
	//			{
	//				//result.push_back(std::pair<int, int>(floorx2, i));
	//				result.push_back(i*xsize + floorx2);
	//			}
	//		}
	//	}//floorx2 < floorx1
	//}





    void newbresenham(double x1, double y1, double x2, double y2, std::vector<std::pair<int, int> >& result){
	// std::vector<std::pair<int, int> > result;
	result.clear();

	int floorx1 = floor(x1);
	int floorx2 = floor(x2);
	int floory1 = floor(y1);
	int floory2 = floor(y2);

    result.reserve(abs(floorx2 - floorx1) + 1 + abs(floory2 - floory1) + 1);

	// if they lie vertically (one above another), we cannot calculate k
	// so we direct solve this case
	if(floorx1 == floorx2)
	{
		// std::vector<int> L = intelist(y1, y2);
        if(floory1 < floory2)
        {
            for(int i = floory1; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        else if(floory1 == floory2)
        {
            result.push_back(std::pair<int, int>(floorx1, floory1));
        }
        else
        {
            for(int i = floory1; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        return ;
	}

    if(floory1 == floory2)
    {
        if(floorx1 < floorx2)
        {
            for(int i = floorx1; i <= floorx2; ++i)
            {
                result.push_back(std::pair<int, int>(i, floory1));
            }
        }
        else if(floorx1 == floorx2)
        {
            result.push_back(std::pair<int, int>(floorx1, floory1));
        }
        else
        {
            for(int i = floorx1; i >= floorx2; --i)
            {
                result.push_back(std::pair<int, int>(i, floory1));
            }
        }
        return ;
    }

    // we get the function of the ray in y=kx+b
	double k = (y2 - y1)/(x2 - x1);
	double b = y1 - k*x1;

    int flag;
    

    std::vector<std::pair<int, int> > v;
    v.reserve(abs(floorx2 - floorx1));
    if(x1 < x2)
    {
        flag = 1;
        for(int i = floorx1+1; i <= floorx2; ++i)
        {
            v.push_back(std::pair<int, int>(i, floor(k*i+b)));
        }
    }
    else
    {
        flag = -1;
        for(int i = floorx1; i >= floorx2 + 1; --i)
        {
            v.push_back(std::pair<int, int>(i, floor(k*i+b)));
        }
    }
    
    int temp = sign(k)*flag;

    if(floorx2 > floorx1)
    {
        // we insert the start column
        if(floory1 < v.front().second)
        {
            for(int i = floory1; i <= v.front().second; ++i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        else
        {
            for(int i = floory1; i >= v.front().second; --i)
            {
                result.push_back(std::pair<int, int>(floorx1, i));
            }
        }
        
        // we insert the intermediate column
        if(abs(floorx2 - floorx1) > 1)
        {

            // if the ray only passes two columns, then there is no intermediate columns
            for(unsigned int i = 0; i < v.size()-1; ++i)
            {
                int xinsert = v.at(i).first;
                int ystart = v.at(i).second;
                int yend = v.at(i+1).second;
                if(ystart < yend)
                {
                    for(int j = ystart; j <= yend; ++j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
                else
                {
                    for(int j = ystart; j >= yend; --j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
            }
            
        }
        
        // we insert the final column
        if(floory2 > v.back().second)
        {
            int xinsert = v.back().first;
            for(int i = v.back().second; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        else
        {
            int xinsert = v.back().first;
            for(int i = v.back().second; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }

    }//floorx2 > floorx1
    else // floorx1 is greater than floorx2. We do things backwards
    {
        int xinsert = v.front().first;
        // we insert the start column
        if(floory1 < v.front().second)
        {
            for(int i = floory1; i <= v.front().second; ++i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        else
        {
            for(int i = floory1; i >= v.front().second; --i)
            {
                result.push_back(std::pair<int, int>(xinsert, i));
            }
        }
        
        // we insert the intermediate column
        if(abs(floorx2 - floorx1) > 1)
        {

            // if the ray only passes two columns, then there is no intermediate columns
            for(unsigned int i = 0; i < v.size()-1; ++i)
            {
                int xinsert = v.at(i+1).first;//Here is different

                int ystart = v.at(i).second;
                int yend = v.at(i+1).second;
                if(ystart < yend)
                {
                    for(int j = ystart; j <= yend; ++j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
                else
                {
                    for(int j = ystart; j >= yend; --j)
                    {
                        result.push_back(std::pair<int, int>(xinsert, j));
                    }
                }
            }
            
        }
        
        // we insert the final column
        if(floory2 > v.back().second)
        {
            // int xinsert = v.back().first;
            for(int i = v.back().second; i <= floory2; ++i)
            {
                result.push_back(std::pair<int, int>(floorx2, i));
            }
        }
        else
        {
            // int xinsert = v.back().first;
            for(int i = v.back().second; i >= floory2; --i)
            {
                result.push_back(std::pair<int, int>(floorx2, i));
            }
        }
    }//floorx2 < floorx1
}


void newbresenham_same_as_matlab(double x1, double y1, double x2, double y2, 
	std::vector<std::pair<int, int> >& result) {
	// std::vector<std::pair<int, int> > result;
	result.clear();

	int floorx1 = floor(x1);
	int floorx2 = floor(x2);
	int floory1 = floor(y1);
	int floory2 = floor(y2);

	result.reserve(abs(floorx2 - floorx1) + 1 + abs(floory2 - floory1) + 1);

	// if they lie vertically (one above another), we cannot calculate k
	// so we direct solve this case
	if (floorx1 == floorx2)
	{
		// std::vector<int> L = intelist(y1, y2);
		if (floory1 < floory2)
		{
			for (int i = floory1; i <= floory2; ++i)
			{
				result.push_back(std::pair<int, int>(floorx1, i));
			}
		}
		else if (floory1 == floory2)
		{
			result.push_back(std::pair<int, int>(floorx1, floory1));
		}
		else
		{
			for (int i = floory1; i >= floory2; --i)
			{
				result.push_back(std::pair<int, int>(floorx1, i));
			}
		}
		return;
	}

	if (floory1 == floory2)
	{
		if (floorx1 < floorx2)
		{
			for (int i = floorx1; i <= floorx2; ++i)
			{
				result.push_back(std::pair<int, int>(i, floory1));
			}
		}
		else if (floorx1 == floorx2)
		{
			result.push_back(std::pair<int, int>(floorx1, floory1));
		}
		else
		{
			for (int i = floorx1; i >= floorx2; --i)
			{
				result.push_back(std::pair<int, int>(i, floory1));
			}
		}
		return;
	}

	// we get the function of the ray in y=kx+b
	double k = (y2 - y1) / (x2 - x1);
	double b = y1 - k * x1;

	int flag;


	std::vector<std::pair<int, int> > v;
	v.reserve(abs(floorx2 - floorx1));
	if (x1 < x2)
	{
		flag = 1;
		for (int i = floorx1 + 1; i <= floorx2; ++i)
		{
			v.push_back(std::pair<int, int>(i, floor(k*i + b)));
		}
	}
	else
	{
		flag = -1;
		for (int i = floorx1; i >= floorx2 + 1; --i)
		{
			v.push_back(std::pair<int, int>(i, floor(k*i + b)));
		}
	}

	int temp = sign(k)*flag;

	if (floorx2 > floorx1)
	{
		// we insert the start column
		if (floory1 < v.front().second)
		{
			for (int i = floory1; i <= v.front().second; ++i)
			{
				result.push_back(std::pair<int, int>(floorx1, i));
			}
		}
		else
		{
			for (int i = floory1; i >= v.front().second; --i)
			{
				result.push_back(std::pair<int, int>(floorx1, i));
			}
		}

		// we insert the intermediate column
		if (abs(floorx2 - floorx1) > 1)
		{

			// if the ray only passes two columns, then there is no intermediate columns
			for (unsigned int i = 0; i < v.size() - 1; ++i)
			{
				int xinsert = v.at(i).first;
				int ystart = v.at(i).second;
				int yend = v.at(i + 1).second;
				if (ystart < yend)
				{
					for (int j = ystart; j <= yend; ++j)
					{
						result.push_back(std::pair<int, int>(xinsert, j));
					}
				}
				else
				{
					for (int j = ystart; j >= yend; --j)
					{
						result.push_back(std::pair<int, int>(xinsert, j));
					}
				}
			}

		}

		// we insert the final column
		if (floory2 > v.back().second)
		{
			int xinsert = v.back().first;
			for (int i = v.back().second; i <= floory2; ++i)
			{
				result.push_back(std::pair<int, int>(xinsert, i));
			}
		}
		else
		{
			int xinsert = v.back().first;
			for (int i = v.back().second; i >= floory2; --i)
			{
				result.push_back(std::pair<int, int>(xinsert, i));
			}
		}

	}//floorx2 > floorx1
	else // floorx1 is greater than floorx2. We do things backwards
	{
		int xinsert = v.front().first;
		// we insert the start column
		if (floory1 < v.front().second)
		{
			for (int i = floory1; i <= v.front().second; ++i)
			{
				result.push_back(std::pair<int, int>(xinsert, i));
			}
		}
		else
		{
			for (int i = floory1; i >= v.front().second; --i)
			{
				result.push_back(std::pair<int, int>(xinsert, i));
			}
		}

		// we insert the intermediate column
		if (abs(floorx2 - floorx1) > 1)
		{

			// if the ray only passes two columns, then there is no intermediate columns
			for (unsigned int i = 0; i < v.size() - 1; ++i)
			{
				int xinsert = v.at(i + 1).first;//Here is different

				int ystart = v.at(i).second;
				int yend = v.at(i + 1).second;
				if (ystart < yend)
				{
					for (int j = ystart; j <= yend; ++j)
					{
						result.push_back(std::pair<int, int>(xinsert, j));
					}
				}
				else
				{
					for (int j = ystart; j >= yend; --j)
					{
						result.push_back(std::pair<int, int>(xinsert, j));
					}
				}
			}

		}

		// we insert the final column
		if (floory2 > v.back().second)
		{
			// int xinsert = v.back().first;
			for (int i = v.back().second; i <= floory2; ++i)
			{
				result.push_back(std::pair<int, int>(floorx2, i));
			}
		}
		else
		{
			// int xinsert = v.back().first;
			for (int i = v.back().second; i >= floory2; --i)
			{
				result.push_back(std::pair<int, int>(floorx2, i));
			}
		}
	}//floorx2 < floorx1
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
		double_path.push_back(iter->x);
		double_path.push_back(iter->y);
	}
	double L = pathLength(double_path);
	return L;

}


    


    
    
    
