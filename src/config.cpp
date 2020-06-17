#ifndef EXTRINSIC_CALIBRATION_CONFIG_CPP
#define EXTRINSIC_CALIBRATION_CONFIG_CPP

#include <sstream>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>

#include "extrinsic_calibration/json.hpp"
#include "extrinsic_calibration/config.h"

using namespace boost::filesystem;
using json = nlohmann::json;

namespace cam_state_publisher
{

  //const std::string Config::FOLDER_PREFIX = "/.ros/camera_info/";
  const std::string Config::FOLDER_PREFIX = "/opt/multisensor_ros/camera_info/";
  const std::string Config::FILE_SUFFIX = "_rgb_calibration_extrinsic";

  /// config handles the saving and loading of the extrinsic calibration files
  /// \param nh_ Nodehandle used for publisher and subscriber
  /// \param priv_nh_ Nodehandle used for parameter handling from roslaunch
  Config::Config(const std::string &camDeviceName_in, const std::string &child_frame, const std::string &header_frame)
  {
    transform_.child_frame_id = child_frame;
    transform_.header.frame_id = header_frame;
    device_name_ = camDeviceName_in;
  }

  /// helper function receives the home directory
  /// \return home directory
  std::string getHomeDir()
  {
    const char *homedir;
    if ((homedir = getenv("HOME")) == nullptr)
    {
      homedir = getpwuid(getuid())->pw_dir;
    }
    return std::string(homedir);
  }

  /// saves the member geometry_msgs::TransformStamped transform_ as json file into the specified path
  /// \return true if successfully
  bool Config::saveToJSONFile()
  {
    // Write to File
    std::string filename = FOLDER_PREFIX + device_name_ + FILE_SUFFIX + ".json";
    system((std::string("mkdir -p ") + FOLDER_PREFIX).c_str());
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::trunc);

    json msgs_obj;
    msgs_obj["header"]["seq"] = 0;
    msgs_obj["header"]["stamp"] = ros::Time::now().nsec;
    msgs_obj["header"]["frame_id"] = transform_.header.frame_id;
    msgs_obj["child_frame_id"] = transform_.child_frame_id;
    msgs_obj["Transform"]["vector3"]["x"] = transform_.transform.translation.x;
    msgs_obj["Transform"]["vector3"]["y"] = transform_.transform.translation.y;
    msgs_obj["Transform"]["vector3"]["z"] = transform_.transform.translation.z;
    msgs_obj["Transform"]["rotation"]["x"] = transform_.transform.rotation.x;
    msgs_obj["Transform"]["rotation"]["y"] = transform_.transform.rotation.y;
    msgs_obj["Transform"]["rotation"]["z"] = transform_.transform.rotation.z;
    msgs_obj["Transform"]["rotation"]["w"] = transform_.transform.rotation.w;

    ofs << std::setw((int)msgs_obj.size()) << msgs_obj << std::endl;
    ofs.close();

    return true;
  }

  /// loads the json file from the specified path and saves it as member geometry_msgs::TransformStamped transform_
  /// \return true if successfully
  bool Config::loadFromJSONFile()
  {
    std::string filename = FOLDER_PREFIX + device_name_ + FILE_SUFFIX + ".json";
    std::ifstream ifs(filename.c_str(), std::ios::in);

    if (!ifs.is_open())
    {
      ROS_ERROR("[Config] Could not open config file %s: %s", filename.c_str(), strerror(errno));
      return false;
    }

    json msgs_obj;
    ifs >> msgs_obj;

    // todo catch NULL
    transform_.header.seq = msgs_obj["header"]["seq"];
    transform_.header.stamp = ros::Time::now();
    transform_.header.frame_id = msgs_obj["header"]["frame_id"];
    transform_.child_frame_id = msgs_obj["child_frame_id"];
    transform_.transform.translation.x = msgs_obj["Transform"]["vector3"]["x"];
    transform_.transform.translation.y = msgs_obj["Transform"]["vector3"]["y"];
    transform_.transform.translation.z = msgs_obj["Transform"]["vector3"]["z"];
    transform_.transform.rotation.x = msgs_obj["Transform"]["rotation"]["x"];
    transform_.transform.rotation.y = msgs_obj["Transform"]["rotation"]["y"];
    transform_.transform.rotation.z = msgs_obj["Transform"]["rotation"]["z"];
    transform_.transform.rotation.w = msgs_obj["Transform"]["rotation"]["w"];

    struct stat st;
    stat(filename.c_str(), &st);
    memcpy(&lastFileModification, &st.st_mtim, sizeof(timespec));

    return true;
  }

  bool Config::hasNewConfig()
  {
    std::string filename = FOLDER_PREFIX + device_name_ + FILE_SUFFIX;
    struct stat st;
    stat(filename.c_str(), &st);
    return memcmp(&lastFileModification, &st.st_mtim, sizeof(timespec));
  }

} // namespace cam_state_publisher

#endif // EXTRINSIC_CALIBRATION_CONFIG_CPP