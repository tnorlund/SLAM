#include <yaml-cpp/yaml.h>
#include <iostream>


int main (int argc, char * argv[]) {
  std::cout << "trying to read config file" << std::endl;
  YAML::Node config = YAML::LoadFile("config.yaml");
}