#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <regex>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include "MPU6050.h"
#include "Socket.h"
#include <boost/program_options.hpp>

/**
 *   The size of the sent packet.
 */
const int RCVBUFFERSIZE = 32;

/**
 *   Handle the parameters given in the command line.
 * 
 *   @param config The name of the config ".yaml"
 *   @returns whether the parameters passed are proper or not
 */
bool processCommandLine(int argc, char** argv,
                          std::string& config) {
  try {
    boost::program_options::options_description description("Allowed options");
    description.add_options()
      ("help,h", "Help screen")
      ("config,c", boost::program_options::value(&config)->required(), 
        "the server IP address");
    
    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::command_line_parser(argc, argv)
            .options(description).run (), 
        vm
    );

    if (vm.count("help")) {
      std::cout << description << "\n";
      return false;
    }

    boost::program_options::notify( vm );
  } catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  } catch(...) {
    std::cerr << "Unknown error!" << "\n";
    return false;
  }
  return true;
}

/**
 *   Reads the configuration file and returns the raw file data.
 * 
 *   @param fileName The name of the file to be read
 *   @return The raw data read from the file
 */
std::string getFileData(std::string fileName) {
  std::string fileData;
  std::string _line;
  std::ifstream fileStream;
  fileStream.open(fileName);
  if (fileStream.is_open()) {
    while ( getline (fileStream, _line) ) {
      fileData.append(_line + "\n");
    }
    fileStream.close();
  } else {
    std::cerr << "Unable to read " << fileName << std::endl;
    exit(1);
  }
  return fileData;
}

/**
 *   Handles the message being sent to the server.
 *   
 *   The TCP socket uses a buffer to stream data. In order to use the buffer,
 *   the string passed as a command line argumenet must be discretized as an
 *   array of characters.
 * 
 *   @param message The message being sent as a string.
 *   @param messageBuffer The array of characters used to store the string.
 *   @param messageLength The length of the message.
 */
void handleBuffer(
  std::string message, char *&messageBuffer, int &messageLength
) {
  messageBuffer = (char *)malloc((message.size() + 1) * sizeof(char));
  message.copy(messageBuffer, message.size() + 1);
  messageBuffer[message.size() + 1] = '\0';
  messageLength = message.length();
}



int main(int argc, char** argv) {
  /// The result of whether the arguments given by command line are good or 
  // not.
  bool result;
  /// The file name of the ".yaml" configuration file.
  std::string configFile;
  /// File buffer for the config file
  struct stat configbuffer;
  /// The character buffer used to store the message sent to the server per
  /// packet sent.
  char * messageBuffer;
  /// The character buffer used to store the entier
  char completeMessageBuffer[RCVBUFFERSIZE + 1];
  /// The length of the message being sent to the server.
  int messageLength; 
  /// The number of bytes received in the specific packet.
  int bytesReceived = 0;
  /// The total number of bytes received among all packets.
  int totalBytesReceived = 0;
  /// The configuration file's content.
  std::string configRaw;

  


  // Handle the arguments passed via command line
  result = processCommandLine(argc, argv, configFile);
  if (!result)
    return 1;
  
  // Check to see if the config file exists
  if (stat (configFile.c_str(), &configbuffer) != 0) {
    std::cerr << "Configuration file does not exist." << std::endl;
    return 1;
  }

  // Parse the config file
  YAML::Node config = YAML::LoadFile(configFile);

  // 
  if (!config["server"]) {
    std::cerr << "Configuration file does not hold the server IP."
      << std::endl;
    return 1;
  }

  if (!config["port"]) {
    std::cerr << "Configuration file does not hold the server port."
      << std::endl;
    return 1;
  }

  configRaw = "CONFIG\n" + getFileData(configFile);
  handleBuffer(configRaw, messageBuffer, messageLength);

  
  try {
    // Initialize a connection to the server
    TCPSocket sock(
      config["server"].as<std::string>(),
      config["port"].as<unsigned short>()
    );
    // Send the message to the server
    sock.send(messageBuffer, messageLength);
    // Receive the same message back. Since the message length is known, the 
    // buffer can be used.
    while (totalBytesReceived < messageLength) {
      // When the bytesRecieved is negative, the packet received is no longer 
      // a part of the buffer. At this point the packet is no longer what the
      // server is sending back.
      if ((bytesReceived = (sock.recv(completeMessageBuffer, RCVBUFFERSIZE))) <= 0) {
        std::cerr << "Unable to read";
        exit(1);
      }
      totalBytesReceived += bytesReceived;
      completeMessageBuffer[bytesReceived] = '\0';
      std::cout << completeMessageBuffer;
    }
    std::cout << std::endl; 
  }
  catch(SocketException &e) {
    std::cerr << "Errored sending packet: " << e.what() << std::endl;
    return 1;
  }
  

  
  return 0;
}