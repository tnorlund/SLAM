/* This measures how quickly the camera can take an image and write it to the
 * disk.
 *
 * TODO
 * - Contact raspicam github
 */
#include <iostream>
#include <thread>
#include <filesystem>
#include <mutex>
#include <algorithm>
#include <fstream>
#include <regex>
#include <boost/program_options.hpp>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam_still_cv.h>
#include <opencv2/opencv.hpp>

const std::string DIRECTORY = "images";
const int HEIGHT = 960;
const int WIDTH = 1280;

/**
 *   A test for the a Raspberry Pi Camera's framerate.
 */
class FrameRateTest {
    // The recorded times for each image capture.
    double* captureTimes = NULL;
    // Whether the captured images have been written to disk or not.
    bool* captureWritten = NULL;
    // The captured images.
    std::vector<cv::Mat> captures;
    // The size of the buffer.
    int bufferSize;
    // The requested frames per second.
    int requestedFPS;
    // The amount of times between image captures.
    double timeBetweenFrames;
    // Whether to print to the console.
    bool verbose;
    // Whether the test is currently running or not.
    bool testRunning;
    // The camera use to capture the images.
    raspicam::RaspiCam_Cv Camera;
    // The file used to write the test results to.
    std::string fileName;
    std::ofstream fileOutput; 
    std::ifstream fileInput;

    private:
        /**
         *   Capture from the camera and store in the buffer.
         * 
         *   @param timeLength The amount of time, in seconds, to capture from
         *     the camera for.
         */
        void capture_images( int timeLength ) {
            // Convert the time in seconds to be in miliseconds.
            timeLength = timeLength * 1000;
            double timerBegin, timerEnd, lastFrame, thisFrame;
            timerBegin = std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            // The index in the buffer of where to save the capture time, the 
            // capture, and whether the frame still needs to be written to the
            // disk.
            int bufferIndex = 0;
            // Take the first capture to fill the first index of the buffer.
            lastFrame = std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            Camera.grab();
            captureTimes[bufferIndex] = std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            Camera.retrieve(captures[bufferIndex]);
            captureWritten[bufferIndex] = false;
            bufferIndex += 1;
            // Now, continuously check to see if the length of time
            while (true) {
                thisFrame = std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();
                timerEnd = std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count();
                // If the previously stored frame has been written, and the 
                // time between frames has been meet, capture the image and
                // store it in the buffer.
                if (
                    thisFrame - lastFrame >= timeBetweenFrames 
                    && captureWritten[bufferIndex]
                ) {
                    Camera.grab();
                    lastFrame = thisFrame;
                    captureTimes[bufferIndex] = std::chrono::duration_cast<
                        std::chrono::milliseconds
                    >(
                        std::chrono::system_clock::now().time_since_epoch()
                    ).count();
                    Camera.retrieve(captures[bufferIndex]);
                    captureWritten[bufferIndex] = false;
                    bufferIndex += 1;
                    // If the end of the buffer has been reached, restart from
                    // the start of the buffer. 
                    if (bufferIndex > bufferSize)
                        bufferIndex = 0;
                }
                if (timerEnd - timerBegin >= timeLength) { break; }
            }
            testRunning = false;
        }

        /**
         *   Write the images stored in the buffer to the disk.
         */
        void write_images() {
            // While the test is running, continuously write the images 
            // captured to the disk.
            while (testRunning) {
                for (size_t i = 0; i < bufferSize; i++) {
                    // If the capture has not been written to the disk, write
                    // the image to the disk and save the written status of the
                    // capture.
                    if (!captureWritten[i]) {
                        cv::imwrite(
                            "./" + DIRECTORY + "/" 
                                + boost::lexical_cast<std::string>(captureTimes[i]) 
                                + ".png",
                            captures[i]
                        );
                        captureWritten[i] = true;
                    }
                }
            }
            // After the test is finished running, iterate over the buffer one
            // more time to ensure all the captures have been written to the
            // disk.
            for (size_t i = 0; i < bufferSize; i++) {
                if (!captureWritten[i]) {
                    cv::imwrite(
                        "./" + DIRECTORY + "/" 
                            + boost::lexical_cast<std::string>(captureTimes[i]) 
                            + ".png",
                        captures[i]
                    );
                    captureWritten[i] = true;
                }
            }
        }

        /**
         *   Checks the Raspberry Pi for a connected camera.
         * 
         *   This function checks the Raspberry Pi for whether the system is camera-
         *   compatible and if a camera is connected.
         */
        void checkCameraConfiguration() {
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

    public:
        /**
         *   Constructor for the framerate test.
         * 
         *   @param buffer The number of pictures to store in RAM.
         *   @param fps The number of frames to record per second.
         *   @param verbose Whether to print to the console.
         *   @param fileName The name of the file to save the results of the 
         *     test.
         */
        FrameRateTest( 
            int buffer, int fps, bool _verbose, std::string file_name 
        ) {
            // Check to see if a camera is connected.
            checkCameraConfiguration();
            // Store the size of the buffer.
            bufferSize = buffer;
            // Dynamically allocate enough space for the time of each capture 
            // and whether each frame was written to disk or not.
            captureTimes = new double [buffer];
            captureWritten = new bool [buffer];
            for (size_t i=0; i<buffer; i++) { 
                captureWritten[i] = true;
            }
            // Dynamically allocate the MAT objects to store the captures into.
            cv::Mat image = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
            for (size_t i=0; i<buffer; i++) { 
                captures.push_back(image.clone());
            }
            // Calculate the length of time between captures. Note, the time 
            // needs to be given in milliseconds.
            requestedFPS = fps;
            timeBetweenFrames = (double)(1/(float)fps*1000);
            // Set the state of the test to not be running.
            testRunning = false;
            verbose = _verbose;
            fileName = file_name;
            if ( fileName != "" ) { 
                if ( !std::filesystem::exists(fileName) ) {
                    fileOutput.open(fileName, std::ios::app); 
                    fileOutput << "Datetime,Runtime,FPS(actual),NumberFrames,"
                        << "Buffer,FPS(requested)\n";
                    fileOutput.close();
                }
            }
            Camera.set( cv::CAP_PROP_FORMAT, CV_8UC3 );
            if ( !Camera.open() ) { throw "Error opening the camera"; }
            if ( 2 > std::thread::hardware_concurrency() ) {
                throw std::invalid_argument(
                    "Not enough threads ( 2 > " + std::to_string(
                            std::thread::hardware_concurrency()
                        ) + " )"
                );
            }
            // If the directory used to store the images does not exist, create
            // the directory.
            if ( !std::filesystem::exists(DIRECTORY) ) {
                std::filesystem::create_directory(DIRECTORY);
            }
            // Otherwise, the directory exists and must be deleted and recreated. 
            else {
                std::filesystem::remove_all(DIRECTORY);
                std::filesystem::create_directory(DIRECTORY);
            }
        }

        /**
         *   Destructor for the framerate test.
         * 
         *   The destructor releases the camera from memory and removes all the
         *   images saved to disk.
         */
        ~FrameRateTest() {
            int directorySize = 0;
            Camera.release();
            std::filesystem::remove_all(DIRECTORY);
        }

        /**
         *   Run the test for the given period of time.
         * 
         *   @param timeLength The amount of time to record frames for.
         */
        void run( int timeLength ) {
            // The number of images written to disk.
            int numImages = 0;
            // The start and stop times of the test.
            double startTime, endTime;
            if ( verbose ) 
                std::cout << "Going to capture frames for " << timeLength << 
                    " seconds" << std::endl;
            // Calculate the start time and set the state of the test to be running.
            startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            testRunning = true;
            // The thread used to capture the images from the camera.
            std::thread cameraThread(&FrameRateTest::capture_images, this, timeLength);
            // The thread used to write the captures to the disk.
            std::thread writeThread(&FrameRateTest::write_images, this);
            // Join the threads after the test has been finished.
            cameraThread.join(); writeThread.join();
            // Calculate the end time of the test.
            endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
            // Count the number of images saved to the disk.
            for (const auto & entry :  std::filesystem::directory_iterator(DIRECTORY)) 
                numImages++;
            // If a file is given, write the results to the file.
            if ( fileName != "" ) { 
                fileInput.open(fileName);
                fileOutput.open(fileName, std::ios::app); 
                fileOutput << boost::lexical_cast<std::string>(std::chrono::duration_cast<
                        std::chrono::milliseconds
                    >(
                        std::chrono::system_clock::now().time_since_epoch()
                    ).count()) << "," << boost::lexical_cast<std::string>((endTime - startTime)/1000) 
                    << "," << (float)numImages/(float)timeLength << "," << numImages << "," 
                    << bufferSize << "," << requestedFPS << ",\n";
                fileInput.close();
                fileOutput.close();
            }
            // Print the results to the console.
            if (verbose)
                std::cout << "FPS: " << (float)numImages/(float)timeLength << " | Buffer: " 
                    << bufferSize << std::endl << std::endl;
        }
};


int main (int argc, char * argv[]) {
    int timeLength, buffer, fps;
    std::string fileName;
    bool verbose = false;
    boost::program_options::options_description description("Allowed options");
    description.add_options()
        ("help,h", "Help screen")
        ("fps,f", boost::program_options::value(&fps), 
            "The number of frames to capture per second.\nThis defaults to 10.")
        ("buffer,b", boost::program_options::value(&buffer), 
            "The size of the buffer used in the test.\nThis defaults to 100.")
        ("time,t", boost::program_options::value(&timeLength), 
            "The length of time to run the test.\nThis defaults to 15.")
        ("filename,d", boost::program_options::value(&fileName), 
            "The file name used to store the test results.")
        ("verbose,v", "Whether to print messages to screen");
    boost::program_options::variables_map vm;
    boost::program_options::store (
        boost::program_options::command_line_parser (argc, argv)
            .options(description).run (), 
        vm
    );
    boost::program_options::notify (vm);
    if (vm.count("help")) { std::cout << description << std::endl; return 1; }
    if (!vm.count("fps")) { fps = 10; }
    if (!vm.count("time")) { timeLength = 15; }
    if (!vm.count("buffer")) { buffer = 100; }
    if (!vm.count("filename")) { fileName = ""; }
    if (vm.count("verbose")) { verbose = true; }
    FrameRateTest test(buffer, fps, verbose, fileName);
    test.run(timeLength);
    return 0;
}