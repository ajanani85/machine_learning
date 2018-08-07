#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <csignal>

#define RED     "\x1b[31m"
#define GREEN   "\x1b[32m"
#define YELLOW  "\x1b[33m"
#define BLUE    "\x1b[34m"
#define MAGENTA "\x1b[35m"
#define CYAN    "\x1b[36m"
#define BRED    "\x1b[31;1m"
#define BGREEN   "\x1b[32;1m"
#define BYELLOW  "\x1b[33;1m"
#define BBLUE    "\x1b[34;1m"
#define BMAGENTA "\x1b[35;1m"
#define BCYAN    "\x1b[36;1m"
#define COLOR_RESET   "\x1b[0m"

void getParam();
bool isImage(const std::string &image_path);

std::string node_name_ = "image_list_analyser";
std::string path_ = "";
bool isAnnotation_ = false;

void getParam()
{
  ros::param::param<std::string>(node_name_ + "/path", path_, path_);

  ROS_INFO("path_: %s", path_.c_str());
}

void checkBackGrounds()
{
  std::ifstream in(path_.c_str());



  std::string str;
  int cnt = 0;
  printf("file is opened.");
  std::vector<std::string> images;
  std::vector<std::string> lines;
  std::vector<std::string> str_list;
  //reading the file
  printf("\nReading the content...");
  while (std::getline(in, str))
  {
    lines.push_back(str);
  }
  in.close();
  if(lines.size() == 0)
  {
    ROS_INFO(RED " FAILED: File is empty or the path is incorrect. Passed Path: %s" COLOR_RESET, path_.c_str());
    raise(SIGINT);
  }
  printf(GREEN "\rReading the content...Completed." COLOR_RESET);
  //Processing each line
  printf("\nProcessing...");
  int line_cnt = 0;
  for (auto line : lines)
  {
    line_cnt++;
    //going through each line and finding out the number of components in the line
    std::istringstream iss(line);
    std::vector<std::string> sub_str((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
    //bg file
    if(sub_str.size() == 1)
    {
      if(isImage(str))
      {
        images.push_back(line);
      }
    }

    //annotaion files
    else if(sub_str.size() > 1)
    {
      bool imgFound = false;
      for(auto s : sub_str)
      {
        if(isImage(s))
        {
          imgFound = true;
          break;
        }
      }
      if(imgFound)
      {
        images.push_back(line);
      }
    }
    printf("\rProcessing...%%%.2f", line_cnt * 100.0 / lines.size());
  }
  printf(GREEN "\rProcessing...Completed." COLOR_RESET);
  printf("\nWriting into file...");
  std::ofstream out;
  out.open(path_.c_str(), std::ios::out);
  //edit the file
  for(int i = 0; i < images.size(); i++)
  {
    out << images[i] << "\n";
    printf("\rWriting into file...%% %.2f", i * 100.00/ images.size());
  }
  printf(GREEN "\rWriting into file...Completed\n "COLOR_RESET);
  /*
  // output the line
  std::vector<std::string> invalid_images;
  printf("\n");
  ROS_INFO("Scanning the images: ");
  for(int i = 0; i < images.size(); i++)
  {
    printf("\r %% %f Completed ", i * 100.0 / images.size() );
    if(!isImage(images[i]))
    {
      invalid_images.push_back(images[i]);
    }
  }

  printf("\r %% %f Completed ", 100.0);
  printf("\n");
  ROS_INFO("Search is completed: %d with error", cnt);
  for(auto p : invalid_images)
  {
    ROS_INFO(RED "%s" COLOR_RESET, p.c_str());
  }*/

  printf("\nDone.");
  raise(SIGINT);
}

bool isImage(const std::string &image_path)
{
  cv::Mat image = cv::imread(image_path);
  if(!image.empty())
  {
    return true;
  }
  return false;
}

void signal_handler(int signal_id)
{
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_handler);
  ros::init(argc, argv, "image_list_analyser", ros::init_options::NoSigintHandler);

  getParam();
  checkBackGrounds();
  ros::spin();
  return 0;
}
