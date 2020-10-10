#include <iostream>
#include <exception>
#include <string>
#include <fstream>
#include <sstream>
#include <regex>
#include <thread>
#include <filesystem>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include "MPU6050.h"
#include "Socket.h"
#include "Slam.h"
#include <boost/program_options.hpp>


/// The size of the received packet.
const int RCVBUFFERSIZE = 32;
/// The name of the configuration file.
const std::string CONFIG_FILENAME = "config.yaml";

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

/**
 *   Handle the parameters given in the command line.
 * 
 *   @param port The port number to listen on
 *   @returns whether the parameters passed are proper or not
 */
bool processCommandLine(int argc, char** argv,
                          std::string& port) {
  try {
    boost::program_options::options_description description("Allowed options");
    description.add_options()
      ("help,h", "Help screen")
      ("port,p", boost::program_options::value(&port)->required(), 
        "the port to listen on");
    
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
 *   @brief Determines the length of the following packets.
 *
 *   @param clientSocket The socket to the client
 *   @param totalLength The length of the following packets
 *   @param messageType The type of message sent from the client
 */
void handleFirstMessage(
  TCPSocket *clientSocket, int &totalLength, std::string &messageType
) {
  int messageLength;
  char startBuffer[13];
  std::string startMessage;
  int spaceIdx = 0;

  // Receive the start message.
  messageLength = clientSocket->recv(startBuffer, 13);
  startMessage = std::string(startBuffer).substr(0, messageLength);
  // Determine the message type by getting the first word of the message
  // received.
  for (int i=0; i<startMessage.length(); i++) {
    if (startMessage.at(i) == ' ') {
      spaceIdx = i; 
      break;
    }
  }
  messageType = startMessage.substr(0, spaceIdx);
  // If the message type is a config file, the length of the config file is
  // obtained.
  if (messageType == "CONFIG")
    totalLength = std::stoi(
      startMessage.substr(
        startMessage.length() - 6, startMessage.length()
      )
    );
  
}

void writeConfigFile(std::string fileContents) {
  /// File buffer for the config file.
  struct stat configbuffer;
  /// File stream to write config to file.
  std::ofstream fileOutput;

  // Delete the file if it exists
  if (stat (CONFIG_FILENAME.c_str(), &configbuffer) == 0)
    std::filesystem::remove(CONFIG_FILENAME);
    
  fileOutput.open(CONFIG_FILENAME, std::ios::app); 
  fileOutput << fileContents;
  fileOutput.close();
}

/**
 *   Handle the client by receiveing the packet and sending it back.
 * 
 *   @param clientSocket the open socket used to accept the packet
 */
void HandleTCPClient(TCPSocket *clientSocket, SLAM *recording) {
  char messageBuffer[RCVBUFFERSIZE];
  /// The character buffer used to store the message sent to the server per
  /// packet sent.
  char startBuffer[13];
  char * sendBuffer;
  int messageLength;
  std::string message;
  std::string line;
  std::string done = "DONE";
  std::string messageType;
  int totalLength;
  int totalBytesReceived = 0;
  int bytesReceived;
  // The character buffer used to store the message received from the server.
  char receiveBuffer[RCVBUFFERSIZE + 1];

  try {
    std::cout << clientSocket->getForeignAddress() << ":";
  } catch (SocketException &e) {
    std::cerr << "Unable to get foreing address" << std::endl;
  }

  try {
    std::cout << clientSocket->getForeignPort();
  } catch(SocketException &e) {
    std::cerr << "Unable to get foreign port" << std::endl;
  }
  std::cout << std::endl;

  // The first packet received determines the message type and the length of
  // the packets following.
  handleFirstMessage(clientSocket, totalLength, messageType);

  // While the client is sending the configuration file, append each packet
  // into message.
  if ( messageType == "CONFIG") {
    while(totalBytesReceived < totalLength) {
      // Error out if the bytesReceived is negative.
      if (
        (bytesReceived = (clientSocket->recv(receiveBuffer, RCVBUFFERSIZE))) <= 0
      ) {
        std::cerr << "Unable to read packet from client." << std::endl;
        exit(1);
      }
      totalBytesReceived += bytesReceived;
      message.append(
        std::string(receiveBuffer).substr(0, bytesReceived)
      );
    }
  }

  // Execute whatever commands are required based on the messageType.
  if (messageType == "START") { 
    std::cout << "received START" << std::endl;
    recording->record();
  }
  else if (messageType == "CONFIG") { 
    writeConfigFile(message);

    handleBuffer(done, sendBuffer, messageLength);
    clientSocket->send(sendBuffer, messageLength);
  }
  else { std::cerr << "Received bad command: " << line << std::endl; }
  // Luckily, the destructor of the socket closes everything.
}

void *ThreadMain(void *clientSocket, SLAM *recording) {
  HandleTCPClient((TCPSocket *) clientSocket, recording);
  delete (TCPSocket *) clientSocket;
  return NULL;
}

int main (int argc, char * argv[]) {
  bool result;
  std::string port;
  unsigned short serverPort;
  SLAM recording(CONFIG_FILENAME);

  // Handle the arguments passed via command line
  result = processCommandLine(argc, argv, port);
  if (!result)
    return 1;
  serverPort = std::stoi(port);

  try {
    TCPServerSocket serverSocket(serverPort);
    while (true) {
      // Memory must be allocated in order to accept the client's argument.
      TCPSocket *clientSocket = serverSocket.accept();
      std::thread thread(ThreadMain, (void *) clientSocket, &recording);
      thread.detach();
    }
  } catch(SocketException &e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
  return 0;
}