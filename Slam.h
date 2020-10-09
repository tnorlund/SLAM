
#ifndef __SLAM_INCLUDED__
#define __SLAM_INCLUDED__

#include <string>                 // For string
#include <exception>              // For exception class
#include <opencv2/opencv.hpp>     // For Mat
#include <raspicam/raspicam_cv.h> // For RaspiCam_Cv

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

private:
  /**
   *   Capture from the camera and store in the buffer.
   * 
   *   @param timeLength The amount of time, in seconds, to capture from
   *     the camera for.
   */
  void capture_images( int timeLength );

  /**
   *   Write the images stored in the buffer to the disk.
   */
  void write_images();

  /**
   *   Checks the Raspberry Pi for a connected camera.
   * 
   *   This function checks the Raspberry Pi for whether the system is camera-
   *   compatible and if a camera is connected.
   */
  void checkCameraConfiguration();

  /**
   *   Finds the CPU information and stores it in DynamoDB.
   */
  void setPiName();

protected:
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
  /// The amount of times between image captures.
  double timeBetweenFrames;
  /// Whether to print to the console.
  bool verbose;
  /// Whether the recording is running or not.
  bool testRunning;
  /// Number of seconds to record frames for.
  /// The start and stop times of the recording.
  double startTime, endTime;
  /// The camera use to capture the images.
  raspicam::RaspiCam_Cv Camera;
  // The file used to write the test results to.
  std::string fileName;
  std::ofstream fileOutput; 
  std::ifstream fileInput;
  /// The DynamoDB table used to write the recording to.
  std::string tableName;
  /// The CPU information
  std::string cpuHardware;
  std::string cpuRevision;
  std::string cpuSerial;
  std::string cpuModel;
  int cpuCores;

};

#endif