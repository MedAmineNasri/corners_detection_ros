#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Mat src_gray;
Mat dst, dst_norm, dst_norm_scaled;
int thresh = 180;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/stereo/left/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/corners_detection/output_video", 1);

  
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
  
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr,cv_ptr2;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr2=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    
    cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );

   
    dst = Mat::zeros( cv_ptr->image.size(), CV_32FC1);
    
    cornerHarris( src_gray, dst, 2, 3 , 0.04 );

    // Normalizing
    normalize( dst,dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

    convertScaleAbs( dst_norm, dst_norm_scaled );
 

    /// Drawing a circle around corners
    for( int i = 0; i < dst_norm.rows ; i++ )
     { for( int j= 0; j < dst_norm.cols; j++ )
       {// window with a score R greater than 180 is considered a corner
        if( (int) dst_norm.at<float>(i,j) > thresh )
          {//The detected corners are surrounded by a small black circle
           circle( cv_ptr2->image, Point( j, i ), 10,  Scalar(0), 2,8,0);
          }
       }
     }

     

    // Update GUI Window
    cv_ptr->image = dst_norm_scaled ;
    cvtColor( dst_norm_scaled, cv_ptr->image, COLOR_GRAY2BGR );
    cv::imshow(OPENCV_WINDOW,  cv_ptr2->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr2->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "corners_detection");
  ImageConverter ic;
  ros::spin();
  return 0;
}
