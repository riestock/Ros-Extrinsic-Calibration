#ifndef CAM_STATE_PUBLISHER_NODE_CPP
#define CAM_STATE_PUBLISHER_NODE_CPP

#include <sstream>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "extrinsic_calibration/config.h"

/// just a main initialize an config object which loads the extrinsic transformation
/// based on the given information and then broadcast this transformation into the ts2 system
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cam_state_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	auto device_name = pnh.param<std::string>("device_name", "default_camDeviceName");
	auto child_frame = pnh.param<std::string>("child_frame", "camera_link");
	auto marker_frame = pnh.param<std::string>("marker_frame", "marker_0");

	if (device_name.empty())
	{
		ROS_ERROR("[CAM_STATE_PUBLISHER] Invalid camera device %s; specify device_name_:=x; device must match current namespace", device_name.c_str());
		return 1;
	}

	cam_state_publisher::Config config(device_name, child_frame, marker_frame);

	while (!config.loadFromJSONFile() && ros::ok())
	{
		ROS_ERROR("[CAM_STATE_PUBLISHER] Config - LoadFromFile failed!");
		ros::shutdown();
	}

	ROS_INFO("[CAM_STATE_PUBLISHER] Publishing state for device %s ", config.device_name_.c_str());

	config.loadFromJSONFile();

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	static_broadcaster.sendTransform(config.transform_);

	ros::spin();

	return 0;
}

#endif // CAM_STATE_PUBLISHER_NODE_CPP