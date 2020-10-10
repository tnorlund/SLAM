#include <iostream>
#include <fstream>
#include <exception>
#include <string>
#include <regex>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>


#include "Socket.h"
#include "Slam.h"
#include <boost/program_options.hpp>

/// The size of the sent packet.
const int RCVBUFFERSIZE = 32;

/**
 *   @brief Handle the parameters given in the command line.
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
 *   @brief Reads the configuration file and returns the raw file data.
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
 *   @brief Handles the message being sent to the server.
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

/**
 *   @brief Sends the configuration file to the server.
 *
 *   @param server The IP of the server.
 *   @param port The port used to send the message.
 *   @param configFile The name of the configuration file used.
 */
void sendConfig(
  std::string server, unsigned short port, std::string configFile
) {
  /// The contents of the config file.
  std::string configRaw;
  /// The start message sent to the server.
  std::string startMessage;
  /// The character buffer used to store the message sent to the server per
  /// packet sent.
  char * messageBuffer;
  /// The length of the message being sent to the server.
  int messageLength; 
  /// The number of bytes received from each server packet.
  int bytesReceived;
  // The character buffer used to store the message received from the server.
  char receiveBuffer[RCVBUFFERSIZE + 1];

  try {
    TCPSocket sock( server, port );

    // The first message sent will always be the message type, followed by the
    // zero-padded length of the message.
    configRaw = getFileData(configFile);
    startMessage = "CONFIG " + std::string( 
        6 - std::to_string( configRaw.length() ).length(), '0' 
      )
      .append( std::to_string( configRaw.length() ) );
    handleBuffer(startMessage, messageBuffer, messageLength);
    sock.send(messageBuffer, messageLength);

    // After telling the server how long the config file will be, send the
    // config file.
    handleBuffer(configRaw, messageBuffer, messageLength);
    sock.send(messageBuffer, messageLength);

    // The server will send a DONE message once the file has been processed.
    bytesReceived = sock.recv(receiveBuffer, RCVBUFFERSIZE);
    if (
      std::string(receiveBuffer).substr(0, bytesReceived) != "DONE"
    ) {
      std::cerr << "Didn't get DONE message" << std::endl;
      exit(1);
    }
  } catch (SocketException &e) {
    std::cerr << "Errored sending the configuration: " << e.what()
      << std::endl;
  }
}



int main(int argc, char** argv) {
  /// The result of whether the arguments given by command line are good or 
  // not.
  bool clArgumentsGood;
  /// The file name of the ".yaml" configuration file.
  std::string configFile;
  /// File buffer for the config file
  struct stat configbuffer;
  /// The character buffer used to store the message sent to the server per
  /// packet sent.
  char * messageBuffer;
  // The character buffer used to store the message received from the server.
  char receiveBuffer[RCVBUFFERSIZE + 1];
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
  /// The start message sent to the server to start recording.
  std::string start = "START         ";
  /// The first message sent stating the type of message and it's length.
  std::string lengthMessage;

  // Handle the arguments passed via command line
  clArgumentsGood = processCommandLine(argc, argv, configFile);
  if (!clArgumentsGood) return 1;
  
  // Check to see if the config file exists
  if (stat (configFile.c_str(), &configbuffer) != 0)
    throw "Configuration file does not exist.";

  // Parse the config file
  YAML::Node config = YAML::LoadFile(configFile);

  // Ensure that the configuration file has the proper keys.
  if (!config["server"])
    throw "Configuration file does not hold the server IP.";
  if (!config["port"]) 
    throw "Configuration file does not hold the server port.";

  // Send the configuration file to the server.
  sendConfig(
    config["server"].as<std::string>(),
    config["port"].as<unsigned short>(),
    configFile
  );
  try {
    SLAM recording(configFile);
    TCPSocket sock(
      config["server"].as<std::string>(),
      config["port"].as<unsigned short>()
    );
    handleBuffer(start, messageBuffer, messageLength);
    sock.send(messageBuffer, messageLength);
    recording.record();
  } catch(SocketException &e) {
    std::cerr << "Errored while sending start message" << std::endl;
  }
  
  return 0;
}