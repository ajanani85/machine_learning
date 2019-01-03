/*
 * annotation_cleaner.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: ubuntu
 */

#include <annotation_tools/Annotation.h>
#include <ros/ros.h>
#include <string>
#include <vector>

std::string node_name_ = "annotation_cleaner";

int main(int argc, char** argv)
{
	ros::init(argc, argv, node_name_);

	std::string annotation_directory = "";
	std::string image_directory="";
	bool grayscale_ = false;
	float test_percentage;

	ros::param::param<std::string>(node_name_ + "/annotation_directory", annotation_directory, annotation_directory);
	ros::param::param<std::string>(node_name_ + "/image_directory", image_directory, image_directory);
	ros::param::param<bool>(node_name_ + "/grayscale", grayscale_, grayscale_);
	ros::param::param<float>(node_name_ + "/test_percentage", test_percentage, test_percentage);

	am::Annotation annotation = am::Annotation();

	//annotation.review(annotation_directory, image_directory);

	annotation.cleanAnnotations(annotation_directory, image_directory, grayscale_, test_percentage);


	ros::spin();
	return 0;
}


