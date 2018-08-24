/*
 * to_darknet_annotation_converter.cpp
 *
 *  Created on: Aug 23, 2018
 *      Author: ubuntu
 */

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <annotation_tools/Annotation.h>

std::string NODE_NAME_ = "to_darknet_annotaiton_converter";

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME_);
	ros::NodeHandle nh;

	std::string annotation_file_ = "";
	std::string destination_folder_ = "";
	float test_percentage = 0.0;

	ros::param::param<std::string>(NODE_NAME_ + "/annotation_file", annotation_file_, annotation_file_);
	ros::param::param<std::string>(NODE_NAME_ + "/destination_folder", destination_folder_, destination_folder_);
	ros::param::param<float>(NODE_NAME_ + "/test_percentage", test_percentage, test_percentage);
	if(annotation_file_ == "")
	{
		ROS_INFO("Invalid Annotation File Location");
		return 0;
	}
	if(destination_folder_ == "")
	{
		ROS_INFO("Invalid Destination Location");
		return 0;
	}

	am::Annotation annotation = am::Annotation();
	annotation.createPath(destination_folder_);
	annotation.OpencvToDarknetAnnotation(annotation_file_, destination_folder_, test_percentage);



	ros::spin();
	return 0;
}


