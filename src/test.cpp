#include "genie_nano/cordef.h"
#include "GenApi/GenApi.h"					//!< GenApi lib definitions.
#include "genie_nano/gevapi.h"				//!< GEV lib definitions.
#include "genie_nano/SapX11Util.h"
//#include "genie_nano/X_Display_utils.h"
#include "genie_nano/FileUtil.h"
#include <sched.h>
#include <iostream>

#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>

#define LOOP_TIME   			0.1



using namespace cv;
using namespace std;

// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
	// ----------------- //
	// -- ROS setting -- //
	// ----------------- //
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;

	ros::Rate loop_rate(1/LOOP_TIME);
	double count = 0;
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		count = count + LOOP_TIME;
		cout << count << endl;
	}

	return 0;
}