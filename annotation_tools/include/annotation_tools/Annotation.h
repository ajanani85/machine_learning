/*
 * Annotation.h
 *
 *  Created on: Aug 20, 2018
 *      Author: ubuntu
 */

#ifndef MACHINE_LEARNING_ANNOTATION_TOOLS_INCLUDE_ANNOTATION_TOOLS_ANNOTATION_H_
#define MACHINE_LEARNING_ANNOTATION_TOOLS_INCLUDE_ANNOTATION_TOOLS_ANNOTATION_H_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

namespace am
{
class Annotation
{
private:
	boost::filesystem::path *imageset_location_ = nullptr;
	boost::filesystem::path annotations_path_;
	boost::filesystem::path parent;

	std::string opencv_annotation_file_location_ = "";
	int class_id_=0;


public:
	Annotation();
	Annotation(const std::string &imageset_location, int class_id = 0);
	~Annotation();

	void addAnnotation(const std::string &file_name, std::vector<cv::Rect> &annotations, int img_height, int img_width, bool debug = false);

	std::string getParent();
	std::vector<std::string> getAnnotationFileLocations();
	std::vector<std::vector<float>> OpencvToDarknet(std::vector<cv::Rect> &src, int img_height, int img_width);

	bool fileExist(const std::string &loc);
	bool pathExist(const std::string &path);
	bool pathExist(const boost::filesystem::path &path);
	bool createPath(const boost::filesystem::path &path);
	boost::filesystem::path createPath(const std::string &path);

	void OpencvToDarknetAnnotation(const std::string &cv_annotation_file, const std::string &destination, bool debug = false);
	std::vector<cv::Rect> getOpencvRect(const std::string &annotation_line);
	std::vector<std::string> getSubStrings(const std::string &src, const char &delim);
	int getNumberOfLines(const std::string &file_location);
	int getFileContent(const std::string &file_location, std::vector<std::string> &content);

};
}



#endif /* MACHINE_LEARNING_ANNOTATION_TOOLS_INCLUDE_ANNOTATION_TOOLS_ANNOTATION_H_ */
