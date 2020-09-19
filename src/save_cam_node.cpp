#include <iostream>

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>

#define LOOP_TIME   			0.1

using namespace std;
using namespace cv;

Mat mat_original = Mat(1920, 1024, CV_8UC4);        // Original image
Mat mat_resized = Mat(960, 512, CV_8UC4);        // Original image


// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

void msgCallback_img(const sensor_msgs::Image::ConstPtr& Img);

// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
	// ----------------- //
	// -- ROS setting -- //
	// ----------------- //
	ros::init(argc, argv, "save_cam_node");
	ros::NodeHandle nh;
    ros::Subscriber sub_image = nh.subscribe("/usb_cam/image_raw", 10, msgCallback_img);

	ros::Rate loop_rate(1/LOOP_TIME);

	double count = 0;
	char c;
    int img_count = 0;

    while(ros::ok()){
	resize(mat_original, mat_resized, Size(516, 386), 0, 0, INTER_LINEAR);
        stringstream file;
        file << "image" << img_count << ".bmp";
        string filename = file.str();
        cv::imwrite(filename, mat_resized);
        img_count++;
        
        ros::spinOnce();
        loop_rate.sleep();
        count = count + LOOP_TIME;
    }

    return 0;
}

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //

void msgCallback_img(const sensor_msgs::Image::ConstPtr& Img){
    //cout << "!!!" << endl;
    (cv_bridge::toCvShare(Img, "bgra8")->image).copyTo(mat_original);
}
