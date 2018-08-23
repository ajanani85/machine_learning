/*
 * Annotation.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: ubuntu
 */
#include <annotation_tools/Annotation.h>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <stdio.h>

namespace am
{
Annotation::Annotation()
{

}
Annotation::Annotation(const std::string &imageset_location, int class_id)
{
	imageset_location_ = new boost::filesystem::path(imageset_location);
	parent = imageset_location_->parent_path();
	//printf("imageset_location: %s \n", imageset_location.c_str());
	//printf("annotations_path_: %s \n", std::string(parent.string() + std::string("/label")).c_str());
	annotations_path_ = createPath(parent.string() + std::string("/label"));


	class_id_ = class_id;
}
Annotation::~Annotation()
{

}
std::string Annotation::getParent()
{
	return parent.string();
}
std::vector<std::string> Annotation::getAnnotationFileLocations()
{
	return {parent.string(), annotations_path_.string()};
}

boost::filesystem::path Annotation::createPath(const std::string &path)
{
	boost::filesystem::path p(path);
	if(!boost::filesystem::exists(p))
	{
		boost::filesystem::create_directories(p);
	}

	return p;
}

void Annotation::addAnnotation(const std::string &file_name, std::vector<cv::Rect> &annotations, int img_height, int img_width, bool debug)
{
	if (annotations.size() == 0)
	{
		return;
	}
	//Saving annotation file in opencv format
	boost::filesystem::path file_path(file_name);
	std::string file_name_no_path = file_path.stem().string();
	if(debug)
	{
		printf("image name: %s\n", file_name_no_path.c_str());
		printf("passed filename: %s \n", file_name.c_str());
	}
	std::ofstream opencv_annotation_file(parent.string() + std::string("/annotations.txt"), std::ios::out | std::ios::app);
	opencv_annotation_file << file_name << " " << annotations.size();
	for(auto an : annotations)
	{
		opencv_annotation_file << " " << an.x << " " << an.y << " " << an.width << " " << an.height;
	}
	opencv_annotation_file << std::endl;
	opencv_annotation_file.close();


	//Converting annotation file to Darknet Format
	auto darknet_annotations = OpencvToDarknet(annotations, img_height, img_width);
	//Creating annotation file
	std::string image_path = annotations_path_.string() + "/" + file_name_no_path + ".txt";
	std::ofstream darknet_annotation_file(image_path, std::ios::out);

	for(int i = 0; i < darknet_annotations.size(); i++)
	{
		darknet_annotation_file << class_id_;
		for(int j = 0; j <= 3; j++)
		{
			darknet_annotation_file << " " << darknet_annotations[i][j];
		}
		if(i > 0 && i != annotations.size() - 1)
		{
			darknet_annotation_file << std::endl;
		}
	}
	darknet_annotation_file.close();
}

/*
 * OpenCV Format is repect to the top left corner of the annotation. X, Y and Width and Height are based on absolute pixel coordinates
 * Darknet Format is respect to the center of the annotation. X, Y and width and height are based on the relative pixel coordinates
 */
std::vector<std::vector<float>> Annotation::OpencvToDarknet(std::vector<cv::Rect> &src, int img_height, int img_width)
{

	std::vector<std::vector<float>> annotations;
	if(src.size() == 0) return annotations;
	for(int i = 0; i < src.size(); i++)
	{
		float dw = 1.0 / img_width;
		float dh = 1.0 / img_height;
		//getting the center in pixel:
		cv::Point center(src[i].x + src[i].width / 2, src[i].y + src[i].height / 2);

		printf("center: %d, %d\n", center.x, center.y);

		std::vector<float> annotate;
		annotate.push_back(center.x * dw);
		annotate.push_back(center.y * dh);
		annotate.push_back(src[i].width * dw);
		annotate.push_back(src[i].height * dh);

		printf("d_annotation: %f, %f, %f, %f\n", annotate[0], annotate[1], annotate[2], annotate[3]);
		annotations.push_back(annotate);
	}
	return annotations;
}

bool Annotation::fileExist(const std::string &loc)
{
  if (FILE *file = fopen(loc.c_str(), "r"))
  {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool Annotation::pathExist(const std::string &path)
{
	return boost::filesystem::exists(boost::filesystem::path(path));
}

bool Annotation::pathExist(const boost::filesystem::path &path)
{
	return boost::filesystem::exists(path);
}

bool Annotation::createPath(const boost::filesystem::path &path)
{
	bool path_status = pathExist(path);
	if(!path_status)
	{
		boost::filesystem::create_directory(boost::filesystem::path(path));
		path_status = pathExist(path);
	}
	return path_status;
}

//This function converts the opencv cascade style annotation to darknet annotation format.
void Annotation::OpencvToDarknetAnnotation(const std::string &cv_annotation_file, bool debug)
{

	if(!fileExist(cv_annotation_file))
	{
		return;
	}

	std::ifstream in(cv_annotation_file);
	std::string line;
	int line_cnt = 0;
	bool potentialJump = false;
	while(std::getline(in, line))
	{

		boost::filesystem::path image_path(line.substr(0, line.find(" ")));

		cv::Mat img = cv::imread(image_path.string());
		if (img.empty())
		{
			continue;
		}

		boost::filesystem::path parent_path = image_path.parent_path();

		std::string annotation_file_path_ = parent_path.string() + std::string("/label/") + image_path.stem().string() + std::string(".txt");

		if(!pathExist((parent_path.string() + std::string("/label"))))
		{
			createPath(parent_path.string() + std::string("/label"));
		}

		std::vector<cv::Rect> rects = getOpencvRect(line);
		auto darknet_rects = OpencvToDarknet(rects, img.size().height, img.size().width);

		std::ofstream annotation_file(annotation_file_path_, std::ios::out);

		for(int i = 0; i < darknet_rects.size(); i++)
		{
			annotation_file << class_id_;
			for(int j = 0; j <= 3; j++)
			{
				annotation_file << " " << darknet_rects[i][j];
			}
			if(i != rects.size() - 1)
			{
				annotation_file << std::endl;
			}
		}
		annotation_file.close();
	}

}

std::vector<cv::Rect> Annotation::getOpencvRect(const std::string &annotation_line)
{
	std::vector<cv::Rect> rects;
	int annotations_cnt = 0;
	if(annotation_line == "")
	{
		return rects;
	}
	std::vector<std::string> annotation_components = getSubStrings(annotation_line, ' ');
	annotations_cnt = std::stoi(annotation_components[1]);

	for(int i = 0; i < annotations_cnt; i++)
	{
		cv::Rect rect(std::stoi(annotation_components[2 + i * 4]),
					  std::stoi(annotation_components[2 + i * 4 + 1]),
					  std::stoi(annotation_components[2 + i * 4 + 2]),
					  std::stoi(annotation_components[2 + i * 4 + 3]));
		rects.push_back(rect);
	}
	return rects;
}

std::vector<std::string> Annotation::getSubStrings(const std::string &src, const char &delim)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(src);
	while (std::getline(tokenStream, token, delim))
	{
		tokens.push_back(token);
	}
	return tokens;
}

}



