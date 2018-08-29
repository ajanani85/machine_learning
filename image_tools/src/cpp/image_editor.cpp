/*
 * image_editor.cpp
 *
 *  Created on: Aug 28, 2018
 *      Author: ubuntu
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <sstream>

std::string node_name_ ="image_editor";
std::string image_folder_ = "";

/*
 * target image color space:
 * 0 -> grayscale
 * 1 -> rgb
*/
int color_space_ = 0;
int max_rotation_angle_ = 359;
int rotation_step_ = 1;
std::string destination_folder_="";


int read_images(std::string &image_folder, std::vector<std::string> &image_locations, std::vector<cv::Mat> &images)
{
	std::vector<cv::String> filenames;
	cv::String folder(image_folder);
	cv::glob(folder, filenames);
	ROS_INFO("Reading through the images ...");
	for(auto filename : filenames)
	{
		std::string filename_std = filename.operator std::string();
		cv::Mat img;
		img = cv::imread(filename_std);
		if (img.empty())
		{
			continue;
		}
		image_locations.push_back(filename_std);
		images.push_back(img);
	}
	ROS_INFO("Reading is completed ... image count: %d", (int)images.size());
	return (int)images.size();
}


void rotate_images(std::vector<cv::Mat> &images, int max_rotation_angle_, int rotation_step_)
{
	ROS_INFO("Rotating Images ...");
	std::vector<cv::Mat> src = images;
	images.erase(images.begin(), images.end());
	int img_cnt = 0;

	for(auto image : src)
	{
		images.push_back(image);
		img_cnt++;
		cv::Point2f center((image.cols-1)/2.0,(image.rows-1)/2.0);
		for(int i = 0; i <= max_rotation_angle_; i += rotation_step_)
		{
			cv::Mat rot_p = cv::getRotationMatrix2D(center, i, 1.0);
			//cv::Mat rot_n = cv::getRotationMatrix2D(center, -i, 1.0);

		    cv::Rect2f bbox_p = cv::RotatedRect(cv::Point2f(), image.size(), i).boundingRect2f();
		    //cv::Rect2f bbox_n = cv::RotatedRect(cv::Point2f(), image.size(), -i).boundingRect2f();

		    rot_p.at<double>(0,2) += bbox_p.width/2.0 - image.cols/2.0;
		    rot_p.at<double>(1,2) += bbox_p.height/2.0 - image.rows/2.0;

		    //rot_n.at<double>(0,2) += bbox_n.width/2.0 - image.cols/2.0;
		    //rot_n.at<double>(1,2) += bbox_n.height/2.0 - image.rows/2.0;

		    cv::Mat dst_p;
		    cv::warpAffine(image, dst_p, rot_p, bbox_p.size());

		   // cv::Mat dst_n;
		    //cv::warpAffine(image, dst_n, rot_n, bbox_n.size());



		    images.push_back(dst_p);
		    //images.push_back(dst_n);
		}
		ROS_INFO("Rotating Images ... Image %d of %d is done.", img_cnt, (int)src.size());

	}
	ROS_INFO("Rotating Images ... is Completed.");
}

void change_colorspace(std::vector<cv::Mat> &images, int color_space)
{

	for(int i = 0; i < images.size(); i++)
	{
		cv::Mat img = images[i];

		switch(color_space)
		{
			case 0:
			{
				if(images[i].channels() > 1)
				{
					//Only RGB to Gray
					cv::cvtColor(images[i], img, cv::COLOR_RGB2GRAY);
				}
				break;
			}
		}
		images[i] = img;
	}
}

void write_images(std::vector<cv::Mat> &images, std::string &destination_folder)
{
	std::string size_string = std::to_string((int)images.size());
	ROS_INFO("Writing %d Images", (int)images.size());
	for(int i=0; i < images.size(); i++ )
	{
		std::stringstream ss;
		ss << "/img-" << std::setw((int)size_string.length()) << std::setfill('0') << i << ".png";
		std::string img_name_ = ss.str();

		cv::imwrite(destination_folder + img_name_, images[i]);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, node_name_);
	ros::NodeHandle nh_;

	ros::param::param<std::string>(node_name_ + "/image_folder", image_folder_, image_folder_);
	ros::param::param<int>(node_name_ + "/max_rotation_angle", max_rotation_angle_, max_rotation_angle_);
	ros::param::param<int>(node_name_ + "/rotation_step", rotation_step_, rotation_step_);
	ros::param::param<int>(node_name_ + "/color_space", color_space_, color_space_);


	if(image_folder_ == "")
	{
		ROS_INFO("Error in reading the image_folder");
		return -1;
	}

	destination_folder_ = boost::filesystem::path(image_folder_).parent_path().string() + "/image_editor_output";
	if(!boost::filesystem::exists(boost::filesystem::path(destination_folder_)))
	{
		boost::filesystem::create_directory(boost::filesystem::path(destination_folder_));
	}

	std::vector<std::string> image_locations;
	std::vector<cv::Mat> images;
	read_images(image_folder_, image_locations, images);
	rotate_images(images, max_rotation_angle_, rotation_step_);
	change_colorspace(images, color_space_);
	write_images(images, destination_folder_);
	ROS_INFO("Done!");
	return 0;
}
