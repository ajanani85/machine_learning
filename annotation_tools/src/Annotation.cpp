/*
 * Annotation.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: ubuntu
 */
#include <annotation_tools/Annotation.h>
#include <boost/filesystem.hpp>

namespace ml
{
Annotation::Annotation()
{

}
Annotation::Annotation(const std::string &imageset_location, const std::string &opencv_annotation_file_location)
{
	imageset_location_ = new boost::filesystem::path(imageset_location);
	boost::filesystem::path parent = imageset_location_->parent_path();
	annotations_path_ = createPath(parent.string() + std::string("/label"));
	opencv_annotation_file_location_ = opencv_annotation_file_location;

}
Annotation::~Annotation()
{

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
}



