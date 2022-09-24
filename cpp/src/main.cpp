#include "tool.h"
#include "planner.h"
#include "solver.h"
#include "map.h"
#include "definitions.h"
#include <iostream>
#include<fstream>
#include <string>
#include "math.h"
#include "dijkstra_4dir.h"

#include<cstdlib>
#include<ctime>
#include "case.h"




void drawPath(std::vector<std::pair<int,int>> path,cv::Mat& img){
	//visualize path for debug
    cv::Scalar color = cv::Scalar(255,12,255);
    for(int i=0;i<path.size()-1;i++){

        cv::line(img,cv::Point(path[i].first,path[i].second),cv::Point(path[i+1].first,path[i+1].second),color,1.5);
    }
    
    
}




int main(void) {
	//case_default();
	//case_right();

	int num;
	std::cout << "right case or left case" << std::endl;
	std::cout << "left 1,right 2"<< std::endl;
	std::cin >> num;
	
	if (num == 1) {
		for (int i = 0; i < 1; i++) {
			case_left();

		}
	}
	else {
		case_right();
	}
	
	// system("pause");


}