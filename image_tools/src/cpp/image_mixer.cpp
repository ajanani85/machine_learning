/*
 * image_mixer.cpp
 *
 *  Created on: Sep 14, 2018
 *      Author: ubuntu
 */
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

std::string node_name_ = "image_mixer";
std::string destination_folder_ = "";

std::vector<std::string> image_folders_;
std::vector<std::string> image_locations_;

bool convert_to_gray_scale_ = false;

bool createPath(const std::string &destination)
{
	boost::filesystem::path p(destination);
	boost::filesystem::create_directories(p);

	if(boost::filesystem::exists(p))
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::vector<std::string> getImages(std::string folder)
{
	std::vector<cv::String> filenames;
	std::vector<std::string> image_locations;
	cv::String cv_folder(folder);
	cv::glob(cv_folder, filenames);

	for(auto filename : filenames)
	{
		image_locations.push_back(filename.operator std::string());
	}

	return image_locations;
}

int writeImages(const std::string &destination, std::vector<std::string> &images)
{
	int img_cnt = 0;
	for(auto image : images)
	{
		boost::filesystem::path img_path(image);
		std::string img_new_location = destination + "/img-" + std::to_string(img_cnt) + ".png";

		cv::imwrite(img_new_location, cv::imread(image));
		img_cnt++;
		ROS_INFO("Writing Images: %% %f is Completed", img_cnt * 100.0 / (int)images.size());
	}
	if(img_cnt == images.size())
	{
		return 0;
	}
	else return -1;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, node_name_);
	ros::NodeHandle nh;

	ros::param::param<std::vector<std::string>>(node_name_ + "/image_locations", image_folders_, image_folders_);
	ros::param::param<std::string>(node_name_ + "/destination_folder", destination_folder_, destination_folder_);
	ros::param::param<bool>(node_name_ + "/convert_to_gray_scale", convert_to_gray_scale_, convert_to_gray_scale_);

	for(auto image_folder : image_folders_)
	{
		ROS_INFO("image_folder: %s", image_folder.c_str());
	}

	ROS_INFO("destination_folder_: %s", destination_folder_.c_str());
	ROS_INFO("convert_to_gray_scale_: %s", convert_to_gray_scale_?"True":"False");


	for(auto image_folder : image_folders_)
	{
		if(!boost::filesystem::exists(boost::filesystem::path(image_folder)))
		{
			ROS_INFO("Folder %s Does Not Exist", image_folder.c_str());
			return -1;
		}
		image_locations_ = getImages(image_folder);
	}

	if(image_locations_.size() == 0)
	{
		ROS_INFO("No Image Is Found In Any Folders");
		return -1;
	}
	if(!createPath(destination_folder_))
	{
		ROS_INFO("Something Went Wrong. Destination folder is not created");
		return -1;
	}

	return writeImages(destination_folder_, image_locations_);

	ros::spin();
	return 0;
}
