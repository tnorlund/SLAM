#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <regex>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include "MPU6050.h"
#include "Socket.h"
#include <boost/program_options.hpp>


/**
 *   The size of the received packet.
 */
const int RCVBUFFERSIZE = 32;

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
 *   Handle the client by receiveing the packet and sending it back.
 * 
 *   @param clientSocket the open socket used to accept the packet
 */
void HandleTCPClient(TCPSocket *clientSocket) {
  char messageBuffer[RCVBUFFERSIZE];
  int messageLength;
  std::string message;
  std::string firstLine;

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
  std::cout << " with thread " << std::this_thread::get_id() << std::endl;

  // While the client is sending the message, append each packet into message.
  while(
    (messageLength = clientSocket->recv(messageBuffer, RCVBUFFERSIZE)) > 0
  ) {
    message.append(
      std::string(messageBuffer).substr(0, messageLength)
    );
  }
  std::istringstream iss(some_string);
  std::cout << std::getline(iss, firstLine) << std::endl;
  // Luckily, the destructor of the socket closes everything.
}

void *ThreadMain(void *clientSocket) {
  HandleTCPClient((TCPSocket *) clientSocket);
  delete (TCPSocket *) clientSocket;
  return NULL;
}

int main (int argc, char * argv[]) {
  bool result;
  std::string port;
  unsigned short serverPort;

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
      std::thread thread(ThreadMain, (void *) clientSocket);
      thread.detach();
    }
  } catch(SocketException &e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }
  return 0;
}