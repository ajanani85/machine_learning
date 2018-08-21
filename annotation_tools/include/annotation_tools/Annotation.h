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

namespace ml
{
class Annotation
{
private:
	boost::filesystem::path *imageset_location_ = nullptr;
	boost::filesystem::path annotations_path_;
	std::string opencv_annotation_file_location_ = "";

	boost::filesystem::path createPath(const std::string &path);
public:
	Annotation();
	Annotation(const std::string &imageset_location, const std::string &opencv_annotation_file_location);
	~Annotation();
};
}



#endif /* MACHINE_LEARNING_ANNOTATION_TOOLS_INCLUDE_ANNOTATION_TOOLS_ANNOTATION_H_ */
