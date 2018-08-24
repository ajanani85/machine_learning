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
#include <algorithm>

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

		//printf("center: %d, %d\n", center.x, center.y);

		std::vector<float> annotate;
		annotate.push_back(center.x * dw);
		annotate.push_back(center.y * dh);
		annotate.push_back(src[i].width * dw);
		annotate.push_back(src[i].height * dh);

		//printf("d_annotation: %f, %f, %f, %f\n", annotate[0], annotate[1], annotate[2], annotate[3]);
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

int Annotation::getNumberOfLines(const std::string &file_location)
{
	//Getting the number of lines
	std::ifstream in(file_location);
	int numLines = 0;
	std::string unused;

	while ( std::getline(in, unused) )
	{
		++numLines;
	}

	in.close();
	return numLines;
}


int Annotation::getFileContent(const std::string &file_location, std::vector<std::string> &content)
{
	//Getting the number of lines
	std::ifstream in(file_location);
	std::string unused;

	while ( std::getline(in, unused) )
	{
		content.push_back(unused);
	}
	in.close();
	//printf("Compelted Reading The Annotation File\n");
	return (int)content.size();
}

//This function converts the opencv cascade style annotation to darknet annotation format.
void Annotation::OpencvToDarknetAnnotation(const std::string &cv_annotation_file, const std::string &destination, float test_percentage, bool debug)
{

	if(!fileExist(cv_annotation_file))
	{
		printf("Annotation File Does Not Exist\n");
		return;
	}

	//create paths for images and labels
	//
	//boost::filesystem::path default_label_path = createPath(boost::filesystem::path(cv_annotation_file).parent_path().string() + "/labels");

	printf("Getting ready ....\n");
	//int number_of_lines = getNumberOfLines(cv_annotation_file);
	std::vector<std::string> content;
	std::vector<std::string> accepted_images;
	int number_of_lines = getFileContent(cv_annotation_file, content);

	int line_cnt = 1;
	int invalid_images_cnt = 0;
	printf("\r Processing image %d out of %d ", line_cnt, number_of_lines);
	for(auto line : content)
	{
		boost::filesystem::path image_path(line.substr(0, line.find(" ")));
		boost::filesystem::path default_image_path = createPath(destination + "/images");
		boost::filesystem::path default_label_path = createPath(destination + "/labels");



		cv::Mat img = cv::imread(image_path.string());


		if (img.empty())
		{
			invalid_images_cnt++;
			//printf("Annotation: Could Not Find The Image: %s \n", image_path.string().c_str());
			continue;
		}
		//Writing the image in destination directory
		std::string image_name = std::string("/img-") + std::to_string(line_cnt) + std::string(".png");
		std::string annotation_name = std::string("/img-") + std::to_string(line_cnt) + std::string(".txt");
		std::string img_new_location_ = default_image_path.string() + image_name;

		cv::imwrite(img_new_location_, img);


		std::string annotation_file_path_ = default_label_path.string() + annotation_name;
		//printf("annotation_file_path_: %s \n",annotation_file_path_.c_str());
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
		accepted_images.push_back(img_new_location_);

		printf("\r Processing image %d out of %d ", line_cnt, number_of_lines);
		line_cnt++;
	}
	printf("Processing Is Completed.\n");
	printf("Invalid Images: %d, %f\n", invalid_images_cnt, invalid_images_cnt * 100.0/number_of_lines);
	printf("Accepted Images: %d, %f\n", line_cnt, line_cnt * 100.0/number_of_lines);

	printf("Allocating Testing and Training Bucket ...\n");
	//creating test.txt and train.txt in the parent of the destination folder
	std::ofstream training_images_file(destination + "/train.txt");
	std::ofstream testing_images_file(destination + "/test.txt");

	int number_of_test_components = test_percentage * accepted_images.size() / 100.0;
	printf("Number of test components = %d\n", number_of_test_components);
	std::vector<int> test_images_indices;

	std::srand((unsigned)time(0));
	//getting the random indices
	for(int i = 0; i <= number_of_test_components; i++)
	{
		printf("size of the list sofar: %d", (int)test_images_indices.size());
		bool itIsRepeated = true;

		while(itIsRepeated)
		{
			int index = std::rand() % accepted_images.size();
			//printf("new random: %d \n", index);
			//only if the random index is not repeated, add to the list
			if(!contains(test_images_indices, index))
			{
				//printf("adding random: %d \n", index);
				test_images_indices.push_back(index);
				itIsRepeated = false;
				break;
			}
			//usleep(10000);
		}

	}
	printf("test_images_indices count = %d\n", (int)test_images_indices.size());
	//writing into file
	for(int i = 0; i < accepted_images.size(); i++)
	{
		//the i is found as part of the index
		if(contains(test_images_indices, i))
		{
			testing_images_file << accepted_images[i] << std::endl;
		}
		else
		{
			training_images_file << accepted_images[i] << std::endl;
		}
	}

	testing_images_file.close();
	training_images_file.close();

	printf("Allocation Is Completed ...\n");

}

bool Annotation::contains( std::vector<int> &Vec, int Element )
{
	if(Vec.size() == 0)
	{
		return false;
	}
	for(int i=0; i < Vec.size(); i++)
	{
		//std::cout << e << std::endl;
		if(Element == Vec[i])
		{
			printf("%d is equal to Vec[%d] = %d\n", Element, i, Vec[i]);
			return true;
		}
	}
	//printf("Could not find the element in the list\n");
	return false;
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



