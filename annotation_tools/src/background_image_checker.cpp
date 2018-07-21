#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>


void getParam();
bool isImage(const std::string &image_path);

std::string node_name_ = "background_image_checker";
std::string path_ = "";

void getParam()
{
	ros::param::param<std::string>(node_name_ + "/path", path_, path_);
	
	ROS_INFO("path_: %s", path_.c_str());
}

void checkBackGrounds()
{
	std::ifstream in(path_.c_str());
	std::string str;
	int cnt = 0;
	ROS_INFO("file is opened");
	std::vector<std::string> images;
	while (std::getline(in, str))
	{
		images.push_back(str);
	}
	// output the line
	std::vector<std::string> invalid_images;	
	printf("\n");	
	for(int i = 0; i < images.size(); i++)
	{
		printf("\r %% %f Completed ", i * 100.0 / images.size() );
		if(!isImage(images[i]))
		{
			invalid_images.push_back(images[i]);
		}
	}
	printf("\r %% %f Completed ", 100.0);
	printf("\n");	
	ROS_INFO("Search is completed: %d with error", cnt);
	for(auto p : invalid_images)
	{
		ROS_INFO("%s", p.c_str());
	}
}

bool isImage(const std::string &image_path)
{
	cv::Mat image = cv::imread(image_path);
	if(!image.empty())
	{
		return true;
	}
	return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "background_image_checker");
	getParam();
	checkBackGrounds();
	ros::spin();
	return 0;
}
