#include <iostream>
#include <exception>
#include <string>
#include <regex>
#include <yaml-cpp/yaml.h>

#include "MPU6050.h"
#include "Socket.h"
#include <boost/program_options.hpp>

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
        "the server IP address")
    
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

inline bool exists_test3 (const std::string& name) {
  
  return (stat (name.c_str(), &configbuffer) == 0); 
}

int main(int argc, char** argv) {
  bool result;
  std::string config;

  // File buffer for the config file
  struct stat configbuffer;   

  // Handle the arguments passed via command line
  result = processCommandLine(argc, argv, config);
  if (!result)
    return 1;
  
  // Check to see if the config file exists
  if !(stat (name.c_str(), &configbuffer) == 0)
    return 1;
  
  YAML::Node config = YAML::LoadFile(config);
  return 0
}