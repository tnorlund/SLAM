
#ifndef __SLAM_INCLUDED__
#define __SLAM_INCLUDED__

#include <string>                 // For string
#include <exception>              // For exception class
#include <opencv2/opencv.hpp>     // For Mat
#include <raspicam/raspicam_cv.h> // For RaspiCam_Cv
#include <fstream>                // For ofstream, ifstream

#include "MPU6050.h"              // For MPU6050

class SLAM {
public:
  /**
   *   @param fileName The config file used.
   */
  SLAM(std::string fileName);

  /**
   *   Close file streams and camera.
   */
  ~SLAM();

  /**
   *   @brief Start a recording.
   */
  void record();

private:
  /**
   *   @brief Capture from the camera and store in the buffer.
   *
   *   Given a certian period of time, capture images through the camera and
   *   store the images and the capture times in this object's buffer.
   *
   *   @param timeLength The amount of time, in seconds, to capture from
   *     the camera for.
   */
  void captureImages();

  /**
   *   Write the odd numbered images stored in the buffer to the disk.
   */
  void writeOddImages();

  /**
   *   Write the even numbered images stored in the buffer to the disk.
   */
  void writeEvenImages();

  /**
   *   @brief Checks the Raspberry Pi for a connected camera.
   * 
   *   This function checks the Raspberry Pi for whether the system is camera-
   *   compatible and if a camera is connected.
   */
  void checkCameraConfiguration();

  /**
   *   Finds the CPU information.
   */
  void setPiName();

  /// The recorded times for each image capture.
  double* captureTimes = NULL;
  /// Whether the captured images have been written to disk or not.
  bool* captureWritten = NULL;
  /// The captured images.
  std::vector<cv::Mat> captures;
  /// The size of the buffer.
  int bufferSize;
  /// The requested frames per second.
  int requestedFPS;
  /// The length of the recording in miliseconds.
  int recordLength;
  /// The amount of times between image captures.
  double timeBetweenFrames;
  /// Whether to print to the console.
  bool verbose;
  /// Whether the recording is running or not.
  bool recording;
  /// Number of seconds to record frames for.
  /// The start and stop times of the recording.
  double startTime, endTime;
  /// The camera use to capture the images.
  raspicam::RaspiCam_Cv Camera;
  /// The directory used to write the recording to.
  std::string directory;
  /// The filestream used to write data.
  std::ofstream fileOutput; 
  /// The filestream used to read data.
  std::ifstream fileInput;
  /// The serial number of the Pi.
  std::string serial;
  /// The DynamoDB table name used to store the test.
  std::string tableName;
  /// The gyroscope object used to record the poses.
  MPU6050 gyroscope;

};

#endif