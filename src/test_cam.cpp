#include "ros/ros.h"

#include <iostream>

#define LOOP_TIME   0.1

using namespace std;

// ---------------------- //
// -- Grobal Variables -- //
// ---------------------- //


// ----------------------- //
// -- General Functions -- //
// ----------------------- //


// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //


// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
    // ----------------- //
    // -- ROS setting -- //
    // ----------------- //
    ros::init(argc, argv, "test_cam");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1/LOOP_TIME);

    double count = 0;

	while (ros::ok()){
        // --------------------- //
        // -- Main Controller -- //
        // --------------------- //


        // ----------------------- //
        // -- ROS Topic Publish -- //
        // ----------------------- //

        ros::spinOnce();
        loop_rate.sleep();
		count = count + LOOP_TIME;
	}

	return 0;
}

// ----------------------- //
// -- General Functions -- //
// ----------------------- //



// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //
