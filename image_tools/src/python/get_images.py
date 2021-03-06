#!/usr/bin/env python

from google_images_download import google_images_download   #importing the library
import rospy
import cv2
import os

node_name_ = "get_images"

def getImages():
	#collecting all the parameters
	resize_cols_ = 0
	resize_rows_ = 0
	color_space_ = 0
	
	keyword_ = rospy.get_param("/get_images/keyword")
	number_of_images_ = rospy.get_param("/get_images/number_of_images")
	output_directory_ = rospy.get_param("/get_images/output_directory")
	chrome_driver_ = rospy.get_param("/get_images/chrome_driver")
	color_space_ = rospy.get_param("/get_images/color_space")
	height_ = rospy.get_param("/get_images/height")
	width_ = rospy.get_param("/get_images/width")
	exact_size_ = str(width_) + ", " + str(height_)
	
	#downloading the images
	response = google_images_download.googleimagesdownload()   #class instantiation
	arguments = {"keywords":keyword_,"limit":number_of_images_,"print_urls":False , "print_paths":False, "chromedriver":chrome_driver_, "output_directory":output_directory_, "exact_size": exact_size_}   #creating list of arguments
	paths = response.download(arguments)   #passing the arguments to the function
	
	#renaming the images:
	#1. listing all the image files
	path = output_directory_ + "/" + keyword_ 
	files = os.listdir(path)
	
	number_of_zero_fill = len(str(number_of_images_))
	
	img_cnt=0
	for file in files:
		#getting file extension
		#filename, file_extension = os.path.splitext(file)
		img = cv2.imread(path + "/" + file)
		if img is None:
			os.remove(path + "/" + file) 
			continue
		#if(resize_cols_ > 0 and resize_rows_ > 0):
		#	resized_img = cv2.resize(img, (resize_cols_, resize_rows_))	
		#	img = resized_img	
		
		if color_space_ == 1:
			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)	
		
		#print path + "/" + file
		#check colorspace

		result = cv2.imwrite(path + "/img-" + str(img_cnt).zfill(number_of_zero_fill) + ".png", img)
		print path + "/img-" + str(img_cnt).zfill(number_of_zero_fill)  + ".png"
		if(result):
			img_cnt = img_cnt + 1
		
		# remove the old image
		os.remove(path + "/" + file) 
	
	print("Done")
	#Exit the program
	rospy.signal_shutdown("Compelted")
	
if __name__ == '__main__':
	rospy.init_node(node_name_, anonymous=False)
	getImages()
	rospy.spin()
