#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "std_srvs/Empty.h"
#include <std_msgs/String.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "finding_big_zebro/getScreenshot.h"

#define DBG_WINDOW_NAME "Debug Window"
#define WAITKEYTIME 30

#define SCREENSHOT_PATH "/home/pi/finding-big-zebro/screenshots/"
const std::string EXTENSION = ".png";

const int DEFAULT_COLS = 1024;
const int DEFAULT_ROWS = 768;

const cv::Size KERNEL_SIZE = cv::Size( 9, 9);
const double THRESHOLD = 40;
const cv::Scalar INDICATOR_COLOR = cv::Scalar(255,0,192);
const cv::Scalar RECTANGLE_COLOR = cv::Scalar(0x99,0x84,0x46);
const int BACKGROUND_REFRESH_RATE = 5;

const cv::Point RECTANGLE_UL = cv::Point(20,47);
const cv::Point RECTANGLE_LR = cv::Point(600,433);

const int PLACEMENT_STATE = 0;
const int DETECTION_STATE = 1;

class motionDetector {
private:
  int foo_, bar_;
  
  ros::NodeHandle node_;

  ros::Subscriber imageSub_;

  ros::ServiceServer saveImageServer_;
    
  cv::Mat image_, imageOriginal_, background_;
  //cv_bridge::CvImage bridge_;

  bool imageCaptured_, backgroundSet_, getScreenshot_;
  int backgroundIterator_;
  std::string screenshotFilename_;

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr bridge;
    try
      {
	bridge = cv_bridge::toCvCopy(msg, "mono8");
      }
    catch (cv_bridge::Exception& e)
      {
	std::cout << "Failed to transform image-bridge!" << std::endl;
	  //<< " Error: " <<  e				 \
	//		  << std::endl;
      }

    //imageOriginal_ = cv::Mat::zeros(bridge->image.rows,
    //bridge->image.cols,
    //		        CV_8UC3);
    
    imageOriginal_ = bridge->image.clone();
    image_ = imageOriginal_.clone();

    if(!backgroundSet_ ||
       backgroundIterator_++ >= BACKGROUND_REFRESH_RATE)                        // Updage the background, or set it
      {
	setBackground(imageOriginal_.clone());
	backgroundIterator_ = 0;
      }

    imageCaptured_ = true;                 // Captured flag
    
    foo_ = imageOriginal_.rows;
    bar_ = imageOriginal_.cols;
  }

  // bool saveImageCallback(finding_big_zebro::getScreenshot::Request& request,
  //			 finding_big_zebro::getScreenshot::Response& response)
  bool saveImageCallback (std_srvs::Empty::Request& request,
			  std_srvs::Empty::Response& response)
  {
    //    screenshotFilename_ = request.filename;
    getScreenshot_ = true;
    
    return true;
  }
  
public:
  
  motionDetector() {
    imageSub_ = node_.subscribe("/camera/image", 
				1, 
				&motionDetector::imageCallback,
				this);
    
    saveImageServer_ = node_.advertiseService("motiondetector/save_image",
					      &motionDetector::saveImageCallback,
					      this);// &motionDetector::saveImage, this);
    
    imageCaptured_ = false;
    backgroundSet_ = false;
    getScreenshot_ = false;

    backgroundIterator_ = 0;

    //TODO: activate the camera service
  }

  bool captured()
  {
    return imageCaptured_;
  }
  
  void displayImage()
  {
    if (!imageCaptured_)
      {
	std::cout << "Image wasn't captured. Is the camera running (call camera start service)." << std::endl;
      }
    else if(image_.rows > 0 && image_.cols > 0) // Image available
      {
	// std::cout << "Displaying image..." << std::endl;
	cv::imshow(DBG_WINDOW_NAME, image_);
	cv::waitKey(WAITKEYTIME);

	if(getScreenshot_)
	  {
	    std::string filePath = SCREENSHOT_PATH;

	    if (screenshotFilename_.empty())
	      screenshotFilename_ = "screenshot.png"; // Check if a filename is given

	    std::string fileExtension = screenshotFilename_.substr(screenshotFilename_.length() - EXTENSION.length(),
								   EXTENSION.length());// check the file extension name
	      //	    if (screenshotFilename_.substr())
	    std::cout << "File extension = " << std::endl;
	    
	    filePath += screenshotFilename_;
	    
	    cv::imwrite(filePath, image_);
	    
	    std::cout << "Image saved to:" << std::endl \
		      << SCREENSHOT_PATH << screenshotFilename_ << std::endl;

	    getScreenshot_ = false;
	  }
    
    // TODO (nice to have) not overwriting, time stamp
      }
    else if (imageOriginal_.rows > 0 && imageOriginal_.cols > 0)
      {
	std::cout << "Displaying original image..." << std::endl;
	cv::imshow(DBG_WINDOW_NAME, imageOriginal_);
	cv::waitKey(WAITKEYTIME);
      }
    else
      {
	std::cout << "Error! Window to show too small!" << std::endl;
	//<< "Window dimension: " << image_.cols << " x " << image_.rows << std::endl;
	displayImageInfo();
      }
  }

  void displayImageInfo()
  {
    std::cout << "Rows:\t " << image_.rows		\
	      << "Cols:\t" << image_.cols << std::endl	\
      //<< "Type:\t" << image_.type			
	      << std::endl \
	      << "Original Rows:\t" << foo_ << std::endl	\
	      << "Original Cols:\t" << bar_ \
	      << std::endl;
  }

  void setBackground(const cv::Mat &image)
  {
    cv::GaussianBlur(image, background_, KERNEL_SIZE, 0, 0);
    backgroundSet_ = true;
    //std::cout << "Background set!" << std::endl;
  }

  void processImage()
  {
    if (!imageCaptured_)
      return;
    
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    
    cv::GaussianBlur(imageOriginal_, image_, KERNEL_SIZE, 0,0);
    //std::cout << "Substracting background" << std::endl;
    image_ = background_ - image_;

    cv::threshold(image_, image_, THRESHOLD, 255, CV_THRESH_BINARY);

    //std::cout << "Matrix type: " << image_.type() << std::endl;
    cv::Mat imageCopy = image_.clone();
    cv::findContours(imageCopy, contours,
		     CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE,
		     cv::Point(0,0));

    //std::cout << "Mystery value:" << contours.size() << std::endl;
    
    cv::cvtColor(image_, image_, CV_GRAY2RGB);    // Convert back to color

    std::vector<cv::Point2f> movementCenters(contours.size() ); // Initializing the center point vector
    
    if (movementCenters.size() > 0)
      {
	//cv::drawContours(image_, contours, -1, INDICATOR_COLOR, 2, 8); // For identifying the contours
	
	//std::cout << "Contour center points:" << std::endl;

	float dummy;
	for (int i=0; i<movementCenters.size(); i++)
	  {
	    cv::minEnclosingCircle(contours[i], movementCenters[i], dummy);
	    
	    cv::circle(image_, movementCenters[i], 2, INDICATOR_COLOR, -1, 8, 0);
	    /* std::cout << "\t"			\
		      << movementCenters[i].x << ","	\
		      << movementCenters[i].y << std::endl;*/
	  }
	//std::cout << std::endl;
	std::cout << "Motion detected!" << std::endl;
      }
    else
      {
	//std::cout << "No contours found!" << std::endl; 
      }
  }

  void drawPlacement()
  {
    if (!imageCaptured_)
	return; // No image recorded or dimensions aren't right.
	else if (image_.cols < DEFAULT_COLS ||
		 image_.rows < DEFAULT_ROWS)
	  {
	    std::cout << "Image resolution doesn't match for placement verification!" << std::endl;
	    return;
	  }

    int imageWidth = image_.cols, imageHeight = image_.rows;
    int placementRadius;
    cv::Point placementCenter;

    placementCenter = cv::Point(image_.cols/2,image_.rows/2);
    placementRadius = image_.cols/6;

    cv::cvtColor(imageOriginal_, image_, CV_GRAY2RGB); // Convert back to color
    
    cv::circle(image_, placementCenter, placementRadius, RECTANGLE_COLOR, 4);
  }

  ros::NodeHandle getNode()
  {
    return node_;
  }
};


int main(int argc, char **argv)
{
  int state = -1;
  while(state == -1)
    {
      std::cout << "Select state, please." << DETECTION_STATE << ": Detection (default), " << PLACEMENT_STATE << ": Placement." ;
      std::cin >> state;
      if (state == DETECTION_STATE)
	std::cout << "Detection state set.";
      else if (state == PLACEMENT_STATE)
	std::cout << "Detection state set.";
      else
	{
	  std::cout << "Unknown or no state argument given. Default state of detection set." << std::endl;
	  state = DETECTION_STATE;
	}
    }
  //int state = PLACEMENT_STATE;
  //int state = DETECTION_STATE;
  bool placementPrompted = false;
  
  ros::init(argc, argv, "finding_big_zebro_node");

  motionDetector detector;

  ros::Rate loop_rate(30);

  ros::ServiceClient client = detector.getNode().serviceClient<std_srvs::Empty::Request>("/camera/start_capture");

  {
    std_srvs::Empty::Request dummyReq;
    std_srvs::Empty::Response dummyRes;
    if (client.call<std_srvs::Empty::Request, std_srvs::Empty::Response>(dummyReq, dummyRes))
	std::cout << "Camera service called!" << std::endl;
    else
	std::cout << "Camera service failed!" << std::endl;
  }
  
  std::cout << "Camera started" << std::endl;

  std::cout << "Please place the Testplate in the image as shown and press enter...";
  
  
  while(ros::ok())
    {
      //      detector.displayImageInfo();
      // detector.displayImage();
      ros::spinOnce();

      switch(state)
	{
	case PLACEMENT_STATE:
	  if (!placementPrompted && detector.captured())
	    {
	      std::cout << "Please place the Testplate in the image as shown and press enter...";
	      placementPrompted = true;
	    }
	  if (detector.captured())
	    detector.drawPlacement();
	  detector.displayImage();
	  break;
	case DETECTION_STATE:
	  detector.processImage();
	  detector.displayImage();
	  break;
	default:
	  break;
	}
	  
      loop_rate.sleep();
      //detector.displayImageInfo();
    }
  
  return 0;
}
