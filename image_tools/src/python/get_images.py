#!/usr/bin/env python

from google_images_download import google_images_download   #importing the library
import rospy
import cv2
import os

node_name_ = "get_images"

def getImages():
	#collecting all the parameters
	keyword_ = rospy.get_param("/get_images/keyword")
	number_of_images_ = rospy.get_param("/get_images/number_of_images")
	output_directory_ = rospy.get_param("/get_images/output_directory")
	
	#downloading the images
	response = google_images_download.googleimagesdownload()   #class instantiation
	arguments = {"keywords":keyword_,"limit":number_of_images_,"print_urls":False , "print_paths":True, "chromedriver":"/home/ubuntu/catkin_ws/src/machine_learning/image_tools/chromedriver/chromedriver", "output_directory":output_directory_}   #creating list of arguments
	paths = response.download(arguments)   #passing the arguments to the function
	print(paths)
	
	#renaming the images
	
if __name__ == '__main__':
	rospy.init_node(node_name_, anonymous=False)
	getImages()
	rospy.spin()
