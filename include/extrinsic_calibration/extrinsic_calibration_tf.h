#ifndef PROJECT_EXTRINSIC_CALIBRATION_TF_H
#define PROJECT_EXTRINSIC_CALIBRATION_TF_H

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <list>
#include <math.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>


#include "extrinsic_calibration/config.h"



namespace extrinsic_calibration
{

class ExtrinsicCalibratorTf
{
private:
  int initialize();
  int readParameters();
  void initializePublishers();
  void initializeTf();

  void writeConfig(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Timer timer_;
  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  bool debug_;
  bool running_;
  double rate_;

  std::string device_name_;
  std::string parent_frame_;
  std::string child_frame_;

  cam_state_publisher::Config state_config_;

public:
  ExtrinsicCalibratorTf(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &private_nh_= ros::NodeHandle("~"));
  void stop();
  bool start();
};


}


#endif //PROJECT_EXTRINSIC_CALIBRATION_TF_H
