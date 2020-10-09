#include "Slam.h"

#include <filesystem>
#include <string.h>
#include <yaml-cpp/yaml.h>  // For Node
#include <sys/stat.h>       // For stat
#include <regex>            // For smatch, regex
#include <boost/program_options.hpp> // For lexical_cast


/// The image height
const int HEIGHT = 960;
/// The image width
const int WIDTH = 1280;

/**
 *   A helper function that returns the current time in milliseconds.
 * 
 *   @returns The current time in milliseconds.
 */
int64_t getTimeInMiliseconds() {
  return(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()
    ).count()
  );
}

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
  YAML::Node config = YAML::LoadFile(fileName);
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
  setPiName();
  // Dynamically allocate enough space for the time of each capture and whether
  // each frame was written to disk or not.
  captureTimes = new double [bufferSize];
  captureWritten = new bool [bufferSize];
  for (size_t i=0; i<bufferSize; i++) { captureWritten[i] = true; }
  // Dynamically allocate the MAT objects to store the captures into.
  cv::Mat image = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
  // Calculate the length of time between captures. Note, the time needs to be
  // given in milliseconds.
  timeBetweenFrames = (double)(1/(float)fps*1000);
  // Set the state of the recording to not running.
  recording = false;
  // Open the camera and set the proper formats. 
  Camera.set( cv::CAP_PROP_FORMAT, CV_8UC3 );
  if ( !Camera.open() ) { throw "Error opening the camera"; }
  // Set the directory name to be the hardware serial and the time of the
  // declaration.
  directory = serial + std::to_string(getTimeInMiliseconds());
  // If the directory used to store the images does not exist, create
  // the directory.
  if ( !std::filesystem::exists(directory) ) {
      std::filesystem::create_directory(directory);
  }
}

SLAM::~SLAM() {
  Camera.release();
}

void SLAM::setPiName() {
  std::string line;
  std::string cpuString;
  std::smatch cpuMatch;
  std::regex const cpuRegex(
    "Hardware\t+: ([A-Z0-9]+)\n"
    "Revision\t+: ([a-z0-9]+)\n"
    "Serial\t+: ([a-z0-9]+)\n"
    "Model\t+: (.+)\n"
  );
  // Read the cpuinfo file
  fileInput.open("/proc/cpuinfo");
  if (fileInput.is_open()) {
    while ( getline(fileInput, line) ) { cpuString.append(line + "\n"); }
    fileInput.close();
  }
  if (std::regex_search(cpuString, cpuMatch, cpuRegex)) {
      serial = cpuMatch[3];
  } else {
      throw "Could not get CPU information";
  }
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

