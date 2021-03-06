#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <csignal>
#include <X11/Xlib.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <annotation_tools/Annotation.h>



using namespace std;
using namespace cv;


// Function prototypes
void on_mouse(int, int, int, int, void*);
std::vector<cv::Rect> get_annotations(cv::Mat);
void drawPreviousAnnotations(cv::Mat);
std::string image_folder_ = "";
std::string annotation_location_ = "";
std::string backup_file_ = "/backup.txt";
int first_image_index_ = 0;
std::string NODE_NAME_="annotation_tools";

am::Annotation *annotation = nullptr;

// Public parameters
Mat image;
int roi_x0 = 0, roi_y0 = 0, roi_x1 = 0, roi_y1 = 0, num_of_rec = 0;
bool start_draw = false, stop = false;
bool helper_ = false;

Display* d;
Screen*  s;
int image_width;
int image_height;
int class_id_ = 0;

std::vector<cv::Rect> previous_annotations;

// Window name for visualisation purposes
const string window_name = "OpenCV Based Annotation Tool";


// FUNCTION : Mouse response for selecting objects in images
// If left button is clicked, start drawing a rectangle as long as mouse moves
// Stop drawing once a new left click is detected by the on_mouse function
void on_mouse(int event, int x, int y, int data, void * param = 0)
{
	
    // Action when left button is clicked
    if(event == EVENT_LBUTTONDOWN)
    {
        if(!start_draw)
        {
            roi_x0 = x;
            roi_y0 = y;
            start_draw = true;
            //ROS_INFO("EVENT_LBUTTONDOWN true");
        } else {
            roi_x1 = x;
            roi_y1 = y;
            start_draw = false;
            //ROS_INFO("EVENT_LBUTTONDOWN false");
        }
    }

    // Action when mouse is moving and drawing is enabled
    if((event == EVENT_MOUSEMOVE) && start_draw)
    {
        // Redraw bounding box for annotation
        cv::Mat current_view;
        image.copyTo(current_view);
        //ROS_INFO("Drawing from [%d, %d] to [%d, %d]", roi_x0, roi_y0, x, y);
        rectangle(current_view, cv::Point(roi_x0,roi_y0), cv::Point(x,y), cv::Scalar(0,0,255));
        cv::imshow(window_name, current_view);
        cv::moveWindow(window_name, s->width / 2 - image_width, s->height / 2 - image_height);
        //ROS_INFO("EVENT_MOUSEMOVE true");
    }
}

// FUNCTION : returns a vector of Rect objects given an image containing positive object instances
std::vector<cv::Rect> get_annotations(cv::Mat input_image)
{
    vector<Rect> current_annotations;
    /*if(previous_annotations.size() > 0)
    {
		current_annotations = previous_annotations;
    }*/

    // Make it possible to exit the annotation process
    stop = false;

    // Init window interface and couple mouse actions
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, on_mouse);
    cv::Mat src;
    if(previous_annotations.size() == 1 && helper_ )
	{		
		//ROS_INFO("x: %d, y: %d, width: %d, height: %d", previous_annotations[0].x,
		//previous_annotations[0].y, previous_annotations[0].width, previous_annotations[0].height);
		
		start_draw = false;
		on_mouse(cv::EVENT_LBUTTONDOWN, previous_annotations[0].x,previous_annotations[0].y, 0);
		//start_draw = true;
		//on_mouse(cv::EVENT_LBUTTONDOWN, previous_annotations[0].x + previous_annotations[0].width,previous_annotations[0].y + previous_annotations[0].height, 0);
		
		start_draw = true;
		on_mouse(cv::EVENT_MOUSEMOVE, previous_annotations[0].x + previous_annotations[0].width,previous_annotations[0].y + previous_annotations[0].height, 0);
		on_mouse(cv::EVENT_MOUSEMOVE, previous_annotations[0].x + previous_annotations[0].width,previous_annotations[0].y + previous_annotations[0].height, 0);
				
	}
    
	
    image = input_image;
    imshow(window_name, image);
    cv::moveWindow(window_name, s->width / 2 - image_width, s->height / 2 - image_height);
    int key_pressed = 0;
	//bool prev_processed = false;
    do
    {
        // Get a temporary image clone
        Mat temp_image = input_image.clone();
        Rect currentRect(0, 0, 0, 0);
		/*if(previous_annotations.size() == 1 && prev_processed)
	   	{
			currentRect.x = previous_annotations[0].x;
		    currentRect.y = previous_annotations[0].y;	
		    currentRect.width = previous_annotations[0].width;
		    currentRect.height = previous_annotations[0].height;
		    prev_processed = false;		
	   	}*/
	   	
        // Keys for processing
        // You need to select one for confirming a selection and one to continue to the next image
        // Based on the universal ASCII code of the keystroke: http://www.asciitable.com/
        //      c = 99              add rectangle to current image
        //          n = 110                 save added rectangles and show next image
        //      d = 100         delete the last annotation made
        //          <ESC> = 27      exit program
        key_pressed = 0xFF & waitKey(0);
        switch( key_pressed )
        {
        case 27:
                stop = true;
                break;
        case 99:
                // Draw initiated from top left corner
                if(roi_x0<roi_x1 && roi_y0<roi_y1)
                {
                    currentRect.x = roi_x0;
                    currentRect.y = roi_y0;
                    currentRect.width = roi_x1-roi_x0;
                    currentRect.height = roi_y1-roi_y0;
                }
                // Draw initiated from bottom right corner
                if(roi_x0>roi_x1 && roi_y0>roi_y1)
                {
                    currentRect.x = roi_x1;
                    currentRect.y = roi_y1;
                    currentRect.width = roi_x0-roi_x1;
                    currentRect.height = roi_y0-roi_y1;
                }
                // Draw initiated from top right corner
                if(roi_x0>roi_x1 && roi_y0<roi_y1)
                {
                    currentRect.x = roi_x1;
                    currentRect.y = roi_y0;
                    currentRect.width = roi_x0-roi_x1;
                    currentRect.height = roi_y1-roi_y0;
                }
                // Draw initiated from bottom left corner
                if(roi_x0<roi_x1 && roi_y0>roi_y1)
                {
                    currentRect.x = roi_x0;
                    currentRect.y = roi_y1;
                    currentRect.width = roi_x1-roi_x0;
                    currentRect.height = roi_y0-roi_y1;
                }
                // Draw the rectangle on the canvas
                // Add the rectangle to the vector of annotations
                current_annotations.push_back(currentRect);
                break;
        case 100:
                // Remove the last annotation
                if(current_annotations.size() > 0){
                    current_annotations.pop_back();
                }
                break;
        default:
                // Default case --> do nothing at all
                // Other keystrokes can simply be ignored
                break;
        }

        // Check if escape has been pressed
        if(stop)
        {
            break;
        }

        // Draw all the current rectangles onto the top image and make sure that the global image is linked
        for(int i=0; i < (int)current_annotations.size(); i++){
            rectangle(temp_image, current_annotations[i], Scalar(0,255,0), 1);
        }
        image = temp_image;

        // Force an explicit redraw of the canvas --> necessary to visualize delete correctly
        imshow(window_name, image);
        cv::moveWindow(window_name, s->width / 2 - image_width, s->height / 2 - image_height);
    }
    // Continue as long as the next image key has not been pressed
    while(key_pressed != 110);

    // Close down the window
    destroyWindow(window_name);

    // Return the data
    return current_annotations;
}

int saveAnnotation(const std::string &annotation_file_location, std::vector<cv::Rect> &annotation, const String &image_path)
{
  if(annotation.size() > 0)
  {
    ofstream output(annotation_file_location.c_str(), std::ofstream::out | std::ofstream::app);
    if ( !output.is_open() )
    {
      cerr << "The path for the output file contains an error and could not be opened. Please check again!" << endl;
      return -1;
    }
    //vector<Rect> &anno = it->second;
    output << image_path << " " << annotation.size();
    for(size_t j=0; j < annotation.size(); j++){
      Rect temp = annotation[j];
      output << " " << temp.x << " " << temp.y << " " << temp.width << " " << temp.height;
    }
    output << endl;
    output.close();
    return 0;
  }
  return -1;

}

bool fileExist(std::string &loc)
{
  if (FILE *file = fopen(loc.c_str(), "r"))
  {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

void processBackUp()
{
  std::string backup_file_location = boost::filesystem::path(image_folder_).parent_path().string() + backup_file_;
  if(fileExist(backup_file_location))
  {
    std::ifstream in(backup_file_location);
    std::string line;
    int line_cnt = 0;
    while(std::getline(in, line))
    {
    	if(line_cnt == 1)
    	{
    		first_image_index_ = std::stoi(line) + 1;
    	}
    	line_cnt++;
    }
    in.close();
  }

}

void getParam()
{
  ros::param::param<std::string>(NODE_NAME_ + "/image_location", image_folder_, image_folder_);
  ros::param::param<bool>(NODE_NAME_ + "/helper", helper_, helper_);
  ros::param::param<int>(NODE_NAME_ + "/class_id", class_id_, class_id_);
  
  annotation = new am::Annotation(image_folder_, class_id_);
  auto locations = annotation->getAnnotationFileLocations();

  processBackUp();
}
void saveBackup(std::string &img_location, int img_index)
{
  std::string backup_file_location = boost::filesystem::path(image_folder_).parent_path().string() + backup_file_;
  std::ofstream f(backup_file_location);
  f << img_location << "\n";
  f << img_index << "\n";
  f.close();
}
void signal_handler(int signal)
{
  cv::destroyAllWindows();
  return;
}
int main( int argc, char** argv )
{
    signal(SIGINT, signal_handler);
    ros::init(argc, argv, NODE_NAME_.c_str(), ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
	
	d = XOpenDisplay(NULL);
	s = DefaultScreenOfDisplay(d);

    getParam();
	
    string image_folder(image_folder_);
    string annotations_file(annotation_location_);
    /*if (annotations_file.empty())
    {
        ROS_INFO("Annotation file path is invalid");
        return -1;
    }*/
    if (image_folder.empty() )
    {
      ROS_INFO("image file path is invalid");
      return -1;
    }

    // Start by processing the data
    // Return the image filenames inside the image folder
    map< String, vector<Rect> > annotations;
    std::vector<String> filenames;
    String folder(image_folder);
    glob(folder, filenames);

    // Add key tips on how to use the software when running it
    cout << "* mark rectangles with the left mouse button," << endl;
    cout << "* press 'c' to accept a selection," << endl;
    cout << "* press 'd' to delete the latest selection," << endl;
    cout << "* press 'n' to proceed with next image," << endl;
    cout << "* press 'esc' to stop." << endl;

    // Loop through each image stored in the images folder
    // Create and temporarily store the annotations
    // At the end write everything to the annotations file

    printf("Image Left: %d", (int)(filenames.size() - first_image_index_));
    fflush(stdout);
    for (size_t i = first_image_index_; i < filenames.size(); i++)
    {
        // Read in an image
        cv::Mat current_image = imread(filenames[i]);
        image_width = current_image.rows;
        image_height = current_image.cols;
        // Check if the image is actually read - avoid other files in the folder, because glob() takes them all
        // If not then simply skip this iteration
        if(current_image.empty()){
            continue;
        }



        // Perform annotations & store the result inside the vectorized structure
        // If the image was resized before, then resize the found annotations back to original dimensions
        
        std::vector<cv::Rect> current_annotations = get_annotations(current_image);
		
		if(current_annotations.size() > 0 )
		{
			previous_annotations = current_annotations;
		}
		
        annotations[filenames[i]] = current_annotations;
        printf("\rImage Left: %d", (int)(filenames.size() - i - 1));
        fflush(stdout);
        //saveAnnotation(annotations_file, current_annotations, filenames[i]);
        annotation->addAnnotation(std::string(filenames[i]), current_annotations, current_image.size().height, current_image.size().width, false);
        saveBackup(image_folder_,i);
        // Check if the ESC key was hit, then exit earlier then expected
        if(stop){
            break;
        }
    }

    // When all data is processed, store the data gathered inside the proper file
    // This now even gets called when the ESC button was hit to store preliminary results
    /*ofstream output(annotations_file.c_str());
    if ( !output.is_open() ){
        cerr << "The path for the output file contains an error and could not be opened. Please check again!" << endl;
        return 0;
    }

    // Store the annotations, write to the output file
    for(map<String, vector<Rect> >::iterator it = annotations.begin(); it != annotations.end(); it++)
    {
        vector<Rect> &anno = it->second;
        output << it->first << " " << anno.size();
        for(size_t j=0; j < anno.size(); j++){
            Rect temp = anno[j];
            output << " " << temp.x << " " << temp.y << " " << temp.width << " " << temp.height;
        }
        output << endl;
    }*/

    return 0;
}
