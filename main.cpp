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
/* These are the libraries required for AWS.
 *
 */
#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h> 
#include <aws/dynamodb/DynamoDBClient.h>
#include <aws/dynamodb/model/AttributeDefinition.h>
#include <aws/dynamodb/model/PutItemRequest.h>
#include <aws/dynamodb/model/GetItemRequest.h>
#include <aws/dynamodb/model/PutItemResult.h>


const std::string DIRECTORY = "images";
const int HEIGHT = 960;
const int WIDTH = 1280;

Aws::SDKOptions options;

/**
 *   A helper function that returns the current time in milliseconds.
 * 
 *   @returns The current time in milliseconds.
 */
int64_t getTimeInMiliseconds () {
    return(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
}

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
    // The start and stop times of the test.
    double startTime, endTime;
    // The camera use to capture the images.
    raspicam::RaspiCam_Cv Camera;
    // The file used to write the test results to.
    std::string fileName;
    std::ofstream fileOutput; 
    std::ifstream fileInput;
    // The DynamoDB table used to write the test results to.
    std::string tableName;
    // The CPU information
    std::string cpuHardware;
    std::string cpuRevision;
    std::string cpuSerial;
    std::string cpuModel;
    int cpuCores;

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

        /**
         *   Finds the CPU information and stores it in the DB
         */
        void setPiName() {
            std::string line;
            std::string cpuString;
            std::smatch cpuMatch;
            std::regex const cpuRegex(
                "Hardware\t+: ([A-Z0-9]+)\n"
                "Revision\t+: ([a-z0-9]+)\n"
                "Serial\t+: ([a-z0-9]+)\n"
                "Model\t+: (.+)\n"
            );
            int numCores = 0;
            std::regex coreRegex("processor\t+: [0-9]");
            // Read the cpuinfo file
            fileInput.open("/proc/cpuinfo");
            if (fileInput.is_open()) {
                while ( getline(fileInput, line) ) {
                    cpuString.append(line + "\n");
                }
                fileInput.close();
            }
            // Get the general CPU information
            if (std::regex_search(cpuString, cpuMatch, cpuRegex)) {
                cpuHardware = cpuMatch[1];
                cpuRevision = cpuMatch[2];
                cpuSerial = cpuMatch[3];
                cpuModel = cpuMatch[4];
            } else {
                std::runtime_error("Could not get CPU information");
            }
            // Get the CPU core information
            std::regex_iterator<std::string::iterator> rit ( 
                cpuString.begin(), cpuString.end(), coreRegex 
            );
            std::regex_iterator<std::string::iterator> rend;
            while (rit!=rend) { ++numCores; ++rit; }
            cpuCores = numCores;
        }

        /**
         *   Save the results of the test to a ".csv" file.
         * 
         *   @param numImages The number of images the test saved to disk
         *   @param timeLength The amount of time to record frames for.
         */
        void saveResultToFile( int numImages, int timeLength ) {
            fileInput.open(fileName);
            fileOutput.open(fileName, std::ios::app); 
            fileOutput << 
                boost::lexical_cast<std::string>(getTimeInMiliseconds()) 
                << "," << 
                boost::lexical_cast<std::string>((endTime - startTime)/1000)
                << "," << (float)numImages/(float)timeLength << "," 
                << numImages << "," << bufferSize << "," << requestedFPS 
                << ",\n";
            fileInput.close();
            fileOutput.close();
        }

        bool _isCPUInDynamoDB() {
            Aws::Client::ClientConfiguration clientConfig;
            Aws::DynamoDB::DynamoDBClient dynamoClient(clientConfig);
            Aws::DynamoDB::Model::GetItemRequest req;
            std::string s{"RaspberryPi"};
            Aws::String hwTableName(s.c_str(), s.size());
            Aws::String _cpuSerial(cpuSerial.c_str(), cpuSerial.size());
            req.SetTableName(hwTableName);
            Aws::DynamoDB::Model::AttributeValue hashKey;
            hashKey.SetS(_cpuSerial);
            req.AddKey("ID", hashKey);
            const Aws::DynamoDB::Model::GetItemOutcome& result =
                dynamoClient.GetItem(req);
            if (result.IsSuccess()) { 
                if ( result.GetResult().GetItem().size() < 1 ) {
                    return false; 
                } else { return true; }
            } 
            else { 
                std::cerr << "Unsuccessfully queried HW data" << std::endl;
                exit(0);
            }
            
        }

        void _saveCPUToDynamoDB() {
            Aws::Client::ClientConfiguration clientConfig;
            Aws::DynamoDB::DynamoDBClient dynamoClient(clientConfig);
            Aws::DynamoDB::Model::PutItemRequest pir;
            Aws::DynamoDB::Model::AttributeValue av;
            Aws::DynamoDB::Model::AttributeValue val;
            Aws::DynamoDB::Model::PutItemOutcome result;

            std::string s{"RaspberryPi"};
            Aws::String hwTableName(s.c_str(), s.size());
            pir.SetTableName(hwTableName);
            Aws::String _cpuHardware(cpuHardware.c_str(), cpuHardware.size());
            Aws::String _cpuRevision(cpuRevision.c_str(), cpuRevision.size());
            Aws::String _cpuSerial(cpuSerial.c_str(), cpuSerial.size());
            Aws::String _cpuModel(cpuModel.c_str(), cpuModel.size());
            av.SetS(_cpuSerial); pir.AddItem("ID", av);
            val.SetS(_cpuHardware); pir.AddItem("hardware", val);
            val.SetS(_cpuRevision); pir.AddItem("revision", val);
            val.SetS(_cpuModel); pir.AddItem("model", val);
            val.SetN(cpuCores); pir.AddItem("numCores", val);
            result = dynamoClient.PutItem(pir);
            if (!result.IsSuccess()) {
                std::cerr << "Unsuccessfully put item in DynamoDB: " 
                    << result.GetError().GetMessage() << std::endl;
                exit(0);
            }
        }

        /**
         *   Save the results of the test to DynamoDB.
         * 
         *   @param numImages The number of images the test saved to disk
         *   @param timeLength The amount of time to record frames for.
         */
        void saveResultToDynamoDB( int numImages, int timeLength ) {
            Aws::Client::ClientConfiguration clientConfig;
            Aws::DynamoDB::DynamoDBClient dynamoClient(clientConfig);
            Aws::DynamoDB::Model::PutItemRequest pir;
            Aws::DynamoDB::Model::AttributeValue av;
            Aws::DynamoDB::Model::AttributeValue val;
            Aws::DynamoDB::Model::PutItemOutcome result;
            // Caste strings
            std::string _dateTime = boost::lexical_cast<std::string>(
                getTimeInMiliseconds()
            );
            // Convert to AWS::String
            Aws::String dateTime(_dateTime.c_str(), _dateTime.size());
            _dateTime += cpuSerial;
            Aws::String resultID(_dateTime.c_str(), _dateTime.size());
            Aws::String table_name(tableName.c_str(), tableName.size());
            Aws::String _cpuSerial(cpuSerial.c_str(), cpuSerial.size());
            // Set HW table name
            if ( !this->_isCPUInDynamoDB() ) {
                this->_saveCPUToDynamoDB();
            }
            // Set result table name
            pir.SetTableName(table_name);
            // Set the key for the table row
            av.SetS(resultID); pir.AddItem("id", av);
            // Set the different values for the row
            val.SetS(dateTime); pir.AddItem("dateTime", val);
            val.SetS(_cpuSerial); pir.AddItem("cpuSerial", val);
            val.SetN((endTime - startTime)/1000); pir.AddItem("runTime", val);
            val.SetN(timeLength); pir.AddItem("timeRequested", val);
            val.SetN(requestedFPS); pir.AddItem("fpsRequested", val);
            val.SetN(numImages); pir.AddItem("numberFrames", val);
            val.SetN(bufferSize); pir.AddItem("buffer", val);
            val.SetN((float)numImages/(float)timeLength);
                pir.AddItem("fpsReal", val);
            result = dynamoClient.PutItem(pir);
            if (!result.IsSuccess())
            {
                std::cerr << "Unsuccessfully put item in DynamoDB: " 
                    << result.GetError().GetMessage() << std::endl;
                exit(0);
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
         *   @param tableName The name of the DynamoDB table to save the test
         *     results to.
         */
        FrameRateTest( 
            int buffer, int fps, bool _verbose, std::string file_name, 
            std::string table_name
        ) {
            // Check to see if a camera is connected.
            checkCameraConfiguration();
            // Set CPU Info
            setPiName();
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
            tableName = table_name;
            if ( tableName != "" ) {
                Aws::InitAPI(options);
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
            if ( verbose ) 
                std::cout << "Going to capture frames for " << timeLength << 
                    " seconds" << std::endl;
            // Calculate the start time and set the state of the test to be
            // running.
            startTime = getTimeInMiliseconds();
            testRunning = true;
            // The thread used to capture the images from the camera.
            std::thread cameraThread(
                &FrameRateTest::capture_images, this, timeLength
            );
            // The thread used to write the captures to the disk.
            std::thread writeThread(&FrameRateTest::write_images, this);
            // Join the threads after the test has been finished.
            cameraThread.join(); writeThread.join();
            // Calculate the end time of the test.
            endTime = getTimeInMiliseconds();
            // Count the number of images saved to the disk.
            for (
                const auto & entry : 
                    std::filesystem::directory_iterator(DIRECTORY)
            )
                numImages++;
            // If a file is given, write the results to the file.
            if ( fileName != "" ) 
                this->saveResultToFile( numImages, timeLength );
            // If a DynamoDB table name is given, send the data to the DB
            if ( tableName != "" ) 
                this->saveResultToDynamoDB( numImages, timeLength );
            // Print the results to the console.
            if (verbose)
                std::cout << "FPS: " << (float)numImages/(float)timeLength 
                    << " | Buffer: " << bufferSize << std::endl << std::endl;
        }
};

int main (int argc, char * argv[]) {
    int timeLength, buffer, fps;
    std::string fileName;
    std::string tableName;
    bool verbose = false;
    boost::program_options::options_description description("Allowed options");
    description.add_options()
        ("help,h", "Help screen")
        ("fps,f", boost::program_options::value(&fps), 
            "The number of frames to capture per second.\nThis defaults to "
            "10.")
        ("buffer,b", boost::program_options::value(&buffer), 
            "The size of the buffer used in the test.\nThis defaults to 100.")
        ("time,t", boost::program_options::value(&timeLength), 
            "The length of time to run the test.\nThis defaults to 15.")
        ("filename,n", boost::program_options::value(&fileName), 
            "The file name used to store the test results.")
        ("tablename,d", boost::program_options::value(&tableName),
            "The DynamoDB table name used to store the results.")
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
    if (!vm.count("tablename")) { tableName = ""; }
    if (vm.count("verbose")) { verbose = true; }
    FrameRateTest test(buffer, fps, verbose, fileName, tableName);
    test.run(timeLength);
    Aws::ShutdownAPI(options);
    return 0;
}