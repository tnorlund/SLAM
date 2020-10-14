#include "Slam.h"

#include <filesystem>
#include <string.h>
#include <yaml-cpp/yaml.h>  // For Node
#include <sys/stat.h>       // For stat
#include <regex>            // For smatch, regex
#include <thread>           // For thread

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
  if (!config["time"])
    throw "Configuration file does not hold the time.";
  recordLength = config["time"].as<int>() * 1000;

  checkCameraConfiguration();
  setPiName();
  // Dynamically allocate enough space for the time of each capture and whether
  // each frame was written to disk or not.
  captureTimes = new int64_t [bufferSize];
  captureWritten = new bool [bufferSize];
  for (size_t i=0; i<bufferSize; i++) { captureWritten[i] = true; }
  // Dynamically allocate the MAT objects to store the captures into.
  cv::Mat image = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
  for (size_t i=0; i<bufferSize; i++) { captures.push_back(image.clone()); }
  // Calculate the length of time between captures. Note, the time needs to be
  // given in milliseconds.
  timeBetweenFrames = (double)( 1 / (float)requestedFPS * 1000);
  // Set the state of the recording to not running.
  recording = false;
  // Open the camera and set the proper formats. 
  Camera.set( cv::CAP_PROP_FORMAT, CV_8UC3 );
  if ( !Camera.open() ) { throw "Error opening the camera"; }
  // Set the directory name to be the hardware serial and the time of the
  // declaration.
  directory = serial;
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

void SLAM::writeOddImages() {
  // Constantly write the image buffer to disk while recording.
  while (recording) {
    for (size_t i=1; i<bufferSize; i+=2) {
      // Write the image to disk when the capture has not already been written
      // to the disk and then set the buffer to have been written to the disk.
      if (!captureWritten[i]) {
        cv::imwrite(
          "./" + directory + "/" + std::to_string(captureTimes[i]) + ".png",
          captures[i]
        );
        captureWritten[i] = true;
      }
    }
  }
  // After the recording has finished, go through the buffer one last time to
  // write whichever images are still in the buffer.
  for (size_t i=1; i<bufferSize; i+=2) {
    if (!captureWritten[i]) {
      cv::imwrite(
        "./" + directory + "/" + std::to_string(captureTimes[i]) + ".png",
        captures[i]
      );
      captureWritten[i] = true;
    }
  }
}

void SLAM::writeEvenImages() {
  // Constantly write the image buffer to disk while recording.
  while (recording) {
    for (size_t i=0; i<bufferSize; i+=2) {
      // Write the image to disk when the capture has not already been written
      // to the disk and then set the buffer to have been written to the disk.
      if (!captureWritten[i]) {
        cv::imwrite(
          "./" + directory + "/" + std::to_string(captureTimes[i]) + ".png",
          captures[i]
        );
        captureWritten[i] = true;
      }
    }
  }
  // After the recording has finished, go through the buffer one last time to
  // write whichever images are still in the buffer.
  for (size_t i=0; i<bufferSize; i+=2) {
    if (!captureWritten[i]) {
      cv::imwrite(
        "./" + directory + "/" + std::to_string(captureTimes[i]) + ".png",
        captures[i]
      );
      captureWritten[i] = true;
    }
  }
}

void SLAM::captureImages() {
  /// The index in the buffer of where to save the capture time, the capture,
  // and whether the frame still needs to be written to the disk.
  int bufferIndex = 0;
  /// The times used during the test.
  int64_t timerBegin, timerEnd, lastFrame, thisFrame;

  // Set the time of the start of the recording.
  timerBegin = getTimeInMiliseconds();
  // Take the first capture to fill the first index of the buffer.
  lastFrame = getTimeInMiliseconds();
  Camera.grab();
  captureTimes[bufferIndex] = getTimeInMiliseconds();
  Camera.retrieve(captures[bufferIndex]);
  captureWritten[bufferIndex] = false;
  bufferIndex++;

  // After taking the first capture, constantly capture until the time has
  // expired.
  while (true) {
    thisFrame = getTimeInMiliseconds();
    timerEnd = getTimeInMiliseconds();
    // Only capture an image if the proper time between the frames has been
    // meet and the capture in the buffer has already been written to the disk.
    if (
      thisFrame - lastFrame >= timeBetweenFrames &&
      captureWritten[bufferIndex]
    ) {
      Camera.grab();
      captureTimes[bufferIndex] = getTimeInMiliseconds();
      Camera.retrieve(captures[bufferIndex]);
      captureWritten[bufferIndex] = false;
      bufferIndex++;
      // When the end of the buffer has been reached, reset the index back to
      // the start of the buffer.
      if (bufferIndex > bufferSize) bufferIndex = 0;
    }
    // When the recording time has expired, set the recording to "not 
    // recording" and break out of the loop.
    if (timerEnd - timerBegin >= recordLength) { recording = false; break; }
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

void SLAM::record() {
  /// The number of images stored to disk during the recording.
  int numImages = 0;

  // Set the start time of the recording and set the state of the recording to
  // be "recording".
  startTime = getTimeInMiliseconds();
  recording = true;

  // Separate the different processes into threads and run concurrently.
  std::thread cameraThread(&SLAM::captureImages, this);
  std::thread oddWriteThread(&SLAM::writeOddImages, this);
  std::thread evenWriteThread(&SLAM::writeEvenImages, this);
  gyroscope.writeToFile(recordLength, directory + "/poses.csv");

  // Once the recording is complete, wait for the processes to end.
  oddWriteThread.join(); evenWriteThread.join(); cameraThread.join();
}

void SLAM::record(std::string dir) {
  directory = dir;
  /// The number of images stored to disk during the recording.
  int numImages = 0;

  // Set the start time of the recording and set the state of the recording to
  // be "recording".
  startTime = getTimeInMiliseconds();
  recording = true;
  std::cout << "directory: " << directory << std::endl;
  std::cout << "splitting threads" << std::endl;

  // Separate the different processes into threads and run concurrently.
  std::thread cameraThread(&SLAM::captureImages, this);
  std::thread oddWriteThread(&SLAM::writeOddImages, this);
  std::thread evenWriteThread(&SLAM::writeEvenImages, this);
  gyroscope.writeToFile(recordLength, directory + "/poses.csv");
  std::cout << "running multiple processes" << std::endl;
  // Once the recording is complete, wait for the processes to end.
  oddWriteThread.join(); evenWriteThread.join(); cameraThread.join();
}



