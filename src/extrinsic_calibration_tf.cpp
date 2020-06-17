#ifndef EXTRINSIC_CALIBRATION_TF_CPP
#define EXTRINSIC_CALIBRATION_TF_CPP

#include "extrinsic_calibration/extrinsic_calibration_tf.h"

namespace extrinsic_calibration
{

  /// this class is for receiving a transformtaion via tf2 system and uses the config class to
  /// save this transformtion to a file
  /// \param nh_ Nodehandle used for publisher and subscriber
  /// \param priv_nh_ Nodehandle used for parameter handling from roslaunch
  ExtrinsicCalibratorTf::ExtrinsicCalibratorTf(const ros::NodeHandle &nh_, const ros::NodeHandle &priv_nh_) : nh_(nh_),
                                                                                                              priv_nh_(priv_nh_),
                                                                                                              debug_(false),
                                                                                                              running_(false),
                                                                                                              rate_(1),
                                                                                                              device_name_(""),
                                                                                                              parent_frame_(""),
                                                                                                              child_frame_("")
  {
  }

  bool ExtrinsicCalibratorTf::start()
  {
    if (running_)
    {
      ROS_ERROR("[ExtrinsicCalibratorTf] is already running!");
      return false;
    }
    if (!initialize())
    {
      ROS_ERROR("[ExtrinsicCalibratorTf] Initialization failed!");
      return false;
    }
    running_ = true;

    return true;
  }

  void ExtrinsicCalibratorTf::stop()
  {
    if (!running_)
    {
      ROS_ERROR("[ExtrinsicCalibratorTf] is not running!");
      return;
    }
    running_ = false;

    nh_.shutdown();
    priv_nh_.shutdown();
  }

  /// calls all sub-initalize functions
  /// \return true if all sub-initalize functions were successful
  int ExtrinsicCalibratorTf::initialize()
  {
    if (!readParameters())
    {
      ROS_ERROR("[ExtrinsicCalibratorTf] Could not read parameters!");
    }

    // initialize cam_state_publisher
    state_config_ = cam_state_publisher::Config(device_name_, child_frame_, parent_frame_);

    initializePublishers();
    initializeTf();
    return true;
  }

  /// reads the parameter from roslaunch
  /// \return true if all parameters were successfully received
  int ExtrinsicCalibratorTf::readParameters()
  {
    return priv_nh_.getParam("debug", debug_) &&
           priv_nh_.getParam("rate", rate_) &&
           priv_nh_.getParam("device_name", device_name_) &&
           priv_nh_.getParam("target_frame", parent_frame_) &&
           priv_nh_.getParam("child_frame", child_frame_);
  }

  void ExtrinsicCalibratorTf::initializePublishers()
  {
    ROS_INFO("[FusionCloudTf] Initializing Publishers");
    timer_ = nh_.createTimer(ros::Duration(1.0), &extrinsic_calibration::ExtrinsicCalibratorTf::writeConfig, this);
  }

  void ExtrinsicCalibratorTf::initializeTf()
  {
    ROS_INFO("[ExtrinsicCalibratorTf] Initializing Tf");
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
  }

  /// timercallback which is called constantly and receives the specified transformtion
  /// from tfs and calls saveToJSONFile() from the config class
  void ExtrinsicCalibratorTf::writeConfig(const ros::TimerEvent &)
  {
    //  get the transformation from tf2
    geometry_msgs::TransformStamped TransformStamped;
    try
    {
      TransformStamped = tf2_->lookupTransform(parent_frame_,
                                               child_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("[ExtrinsicCalibratorTf] %s", ex.what());
      return;
    }

    //  write transformation to file
    state_config_.transform_ = TransformStamped;
    if (!state_config_.saveToJSONFile())
    {
      ROS_ERROR("[ExtrinsicCalibratorTf] Could not save config file!");
    }
    else
    {
      ROS_INFO_ONCE("[ExtrinsicCalibratorTf] Wrote extrinsic data to file!");
      ros::shutdown();
    }
  }

} // namespace extrinsic_calibration

int main(int argc, char **argv)
{

  ros::init(argc, argv, "extrinsic_calibrator", ros::init_options::AnonymousName);

  if (!ros::ok())
  {
    ROS_ERROR("ros::ok failed!");
    return -1;
  }

  extrinsic_calibration::ExtrinsicCalibratorTf calib;
  if (calib.start())
  {
    ros::spin();

    calib.stop();
  }

  ros::shutdown();
  return 0;
}

#endif // EXTRINSIC_CALIBRATION_TF_CPP
