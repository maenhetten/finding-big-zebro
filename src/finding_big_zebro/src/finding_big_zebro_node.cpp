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

class motionDetector {
private:
  int foo_, bar_;
  
  ros::NodeHandle node_;

  ros::Subscriber imageSub_;
    
  cv::Mat image_, imageOriginal_;
  //cv_bridge::CvImage bridge_;

  bool imageCaptured_;

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
    //bridge->image.copyTo(imageOriginal_);
    //foo_ = bridge->image.rows;
    //bar_ = bridge->image.cols;
    foo_ = imageOriginal_.rows;
    bar_ = imageOriginal_.cols;

    imageCaptured_ = true;                        // Captured flag
  }
  
public:
  
  motionDetector() {
    imageSub_ = node_.subscribe("/camera/image", 
				1, 
				&motionDetector::imageCallback,
				this);
    imageCaptured_ = false;

    
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

  void processImage()
  {
    std::cout << "Applying Gaussian filter..." << std::endl;
    cv:: GaussianBlur(imageOriginal_, image_, KERNEL_SIZE, 0,0);
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
