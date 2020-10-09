#include "Slam.h"

#include <string.h>
#include <yaml-cpp/yaml.h>  // For Node

// ############################################################################
// SLAM Code

SLAM::SLAM(std::string fileName) {
  /// File buffer for the config file
  struct stat fileBuffer;

  // Check to see if the config file exists
  if (stat (fileName.c_str(), &fileBuffer) != 0) {
    throw "Configuration file does not exist.";
  }
  // Parse the config file
  YAML::Node config = YAML::LoadFile(configFile);
  if (!config["buffer"]) {
    throw "Configuration file does not hold the buffer size.";
  }
  bufferSize = config["buffer"].as<int>();
  if (!config["framerate"]) {
    throw "Configuration file does not hold the framerate.";
  }
  requestedFPS = config["framerate"].as<int>();
  if (!config["tablename"]) {
    throw "Configuration file does not hold the tablename.";
  }
  tableName = config["tablename"].as<std::string>();

  checkCameraConfiguration();
}

void SLAM::checkCameraConfiguration() {
  FILE *fp;
  char var[23];
  std::string cameraString;
  std::smatch cameraMatch;
  std::regex cameraRegex("supported=(0|1) detected=(0|1)");

  fp = popen("/opt/vc/bin/vcgencmd get_camera", "r");
  fgets(var, sizeof(var), fp);
  pclose(fp);
  cameraString = std::string(var);
  if (
    std::regex_search(cameraString, cameraMatch, cameraRegex)
  ) {
      if (std::string(cameraMatch[1]) != "1")
        throw "Camera is not supported";
      if (std::string(cameraMatch[2]) != "1")
        throw "Camera is not connected";
  } else {
      throw "Unable to get camera information";
  }
}

