#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DBG_WINDOW_NAME "Debug Window"
#define WAITKEYTIME 30

const cv::Size KERNEL_SIZE = cv::Size( 9, 9);
const double THRESHOLD = 40;
const cv::Scalar INDICATOR_COLOR = cv::Scalar(255,0,192);

class motionDetector {
private:
  int foo_, bar_;
  
  ros::NodeHandle node_;

  ros::Subscriber imageSub_;
    
  cv::Mat image_, imageOriginal_, background_;
  //cv_bridge::CvImage bridge_;

  bool imageCaptured_, backgroundSet_;

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

    if(!backgroundSet_)                        // That's the first image (=Background)
      {
	setBackground(imageOriginal_.clone());
      }

    imageCaptured_ = true;                 // Captured flag
    
    //bridge->image.copyTo(imageOriginal_);
    //foo_ = bridge->image.rows;
    //bar_ = bridge->image.cols;
    foo_ = imageOriginal_.rows;
    bar_ = imageOriginal_.cols;
  }
  
public:
  
  motionDetector() {
    imageSub_ = node_.subscribe("/camera/image", 
				1, 
				&motionDetector::imageCallback,
				this);
    imageCaptured_ = false;
    backgroundSet_ = false;

    
    //TODO: activate the camera service
  }

  bool captured()
  {
    return imageCaptured_;
  }
  
  void displayImage()
  {
    if(image_.rows > 0 && image_.cols > 0 && imageCaptured_)
      {
	// std::cout << "Displaying image..." << std::endl;
	cv::imshow(DBG_WINDOW_NAME, image_);
	cv::waitKey(WAITKEYTIME);
      }
    else if (!imageCaptured_)
      {
	std::cout << "Image wasn't captured." << std::endl;
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
    std::cout << "Background set!" << std::endl;
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

    std::cout << "Mystery value:" << contours.size() << std::endl;
    
    cv::cvtColor(image_, image_, CV_GRAY2RGB);    // Convert back to color

    std::vector<cv::Point2f> movementCenters(contours.size() ); // Initializing the center point vector
    
    if (movementCenters.size() > 0)
      {
	//cv::drawContours(image_, contours, -1, INDICATOR_COLOR, 2, 8); // For identifying the contours
	
	std::cout << "Contour center points:" << std::endl;
	float dummy;
	for (int i=0; i<movementCenters.size(); i++)
	  {
	    cv::minEnclosingCircle(contours[i], movementCenters[i], dummy);
	    
	    cv::circle(image_, movementCenters[i], 2, INDICATOR_COLOR, -1, 8, 0);
	    std::cout << "\t" \
		      << movementCenters[i].x << ","	\
		      << movementCenters[i].y << std::endl;
	  }
	std::cout << std::endl;
      }
    else
      {
	std::cout << "No contours found!" << std::endl; 
      }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "finding_big_zebro_node");

  motionDetector detector;

  ros::Rate loop_rate(30);
  
  while(ros::ok())
    {
      //      detector.displayImageInfo();
      // detector.displayImage();
      ros::spinOnce();

      detector.processImage();
      detector.displayImage();

      loop_rate.sleep();
      //detector.displayImageInfo();
    }
  
  return 0;
}
