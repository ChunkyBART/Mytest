#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cvaux.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <cmath>
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>  //for std::vector
#include <std_msgs/Int16.h>
#include <math.h>
#include "opencv2/nonfree/features2d.hpp"
#define Path_radious 50
bool state_pointer = false;
bool im_rdy=false;
bool depth_rdy=false;
using namespace cv;
using namespace std;
int global_grid_origin_x=0;
int global_grid_origin_y=0;
image_transport::Publisher image_pub;
namespace enc = sensor_msgs::image_encodings;
int sliderPos = 70;
Mat image;
Mat dst;
Mat im;
Mat depth_im;
Mat Dis;
bool _isR = true;
RotatedRect r;
int score[20] ={};
bool start = false;
int morph_elem = 0;
int morph_size = 4;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;
int x_bar,y_bar;
int posx,posy;
ros::Publisher pos_pub;
ros::Publisher com_pub;
ros::Publisher dis_pub;
ros::Publisher state_pub;
cv_bridge::CvImage ptr;
cv_bridge::CvImageConstPtr cv_ptr;
std_msgs::Int16 dist,state_msg;
SurfFeatureDetector detector(2000);
vector<KeyPoint> keypoints1;
vector<KeyPoint> keypoints2[20];
//Initialise wrapping class for descriptors computing using SURF() class.
SurfDescriptorExtractor extractor;
//Compute: Input:image, keypoints Output:descriptors
Mat descriptors1,descriptors2[20];
//Initialise BruteForceMatcher: For each descriptor in the first set, this matcher finds the closest descriptor in the second set by trying each on (=brute)
//BruteForceMatcher< L2<float> > matcher;
FlannBasedMatcher matcher;
vector< DMatch > matches;
double max_dist = 0; double min_dist = 150;
Mat sign_template[20] ;
//cv_bridge::CvImagePtr cv_ptr;

void command_callback(const std_msgs::StringConstPtr& msg)
{
    if(msg->data == "go")
    {
        ROS_ERROR("debug");
        state_pointer = true;
    }
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //std_msgs::String imsignal;
    //std_msgs::String comsignal;
    cv_bridge::CvImageConstPtr cv_ptr;
    //char filename[40];
    cv_ptr =  cv_bridge::toCvShare(msg, enc::BGR8);
    Mat im,biimage;
    im = cv_ptr->image;
    //cvtColor(cv_ptr->image,im,CV_BGR2GRAY);
    inRange(im,Scalar(80,0,0), Scalar(255,255,50),biimage); //(85,80,80);
   // p.Uppercolor = Scalar(125,255,255);
    //imshow( "before", im );
    //waitKey(1);
    //imshow("after",biimage);
    SimpleBlobDetector::Params params;

    params.filterByColor =true;
    params.filterByCircularity =false;
    params.filterByConvexity =false;
    params.filterByInertia = false;
    params.blobColor = 255;
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 10000;
    params.maxArea = 800000;
    // Filter by Inertia
    params.minInertiaRatio = 10;
    params.minDistBetweenBlobs = 10;
    SimpleBlobDetector detector(params);
    std::vector<KeyPoint> keypoints;
    detector.detect(biimage,keypoints);
    Mat im_with_keypoints;
    drawKeypoints( biimage, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("test",im_with_keypoints);
    waitKey(1);
    for
    (
    std::vector<KeyPoint>::iterator it = keypoints.begin();
    it != keypoints.end();
    ++it
    )
    {
        KeyPoint k =  *it;
        cout << k.pt << endl;
        x_bar=k.pt.x;
        y_bar=k.pt.y;
    if(x_bar>((im_with_keypoints.cols/2)-(im_with_keypoints.cols/15)) && x_bar<(im_with_keypoints.cols/2)+(im_with_keypoints.cols/15))
    {
        std_msgs::String sent;
        sent.data =  "getposition";
        com_pub.publish(sent);
        ros::Duration(0.05).sleep();
        sent.data =  "BR";
        com_pub.publish(sent);
        ros::Duration(2).sleep();
    }
}
}
void audio_callback(const std_msgs::Int16MultiArray & test)
{
    int sum;
    for(int i=1;i<test.data.size();i++)
    sum = test.data[i];
    ROS_INFO("%d",sum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "barrel");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
 // cv::namedWindow("gray");
  //  cv::namedWindow("depth");
//  image_transport::ImageTransport it(nh);
  ros::Subscriber com_sub = nh.subscribe("/audio", 1, audio_callback);
 // image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
 // com_pub = nh.advertise<std_msgs>("/audio",1);
  //image_transport::Subscriber subasus = it.subscribe("/camera/rgb/image_raw",1,imageCallback);
  //image_pub = it.advertise("/feature",1);
  //ros::Timer timer2 = im.createTimer(ros::Duration(0.05), motion_detection);
  //state_pub = nh.advertise<std_msgs::String>("/image_hazard",1);
  //spinner.start();
  //ros::waitForShutdown();
  ros::spin();
}
//}
