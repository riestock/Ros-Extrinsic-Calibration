#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>

namespace cam_state_publisher {

/**
 * @brief Config class for handling extrinsic calibration data
 *
 * Used to save and load the config from a central file.
 *
 * The config file is in yaml format, located in FILE_PREFIX.
 * It contains a geometry_msgs::TransformStamped which connects
 * MARKER_FRAME_NAME with the camera_depth_optical_frame
 */

class Config {
public:
	static const std::string FILE_SUFFIX;
	static const std::string FOLDER_PREFIX;

protected:
	timespec lastFileModification;

public:
	geometry_msgs::TransformStamped transform_;

	std::string device_name_;

	Config(const std::string &camDeviceName_in = "camera", const std::string &childFrameSuffix_in =  "camera_link", const std::string &markerFrameSuffix_in = "target_link");

  bool saveToJSONFile();
  bool loadFromJSONFile();
	bool hasNewConfig();
};

}

#endif // CONFIG_HPP
