#include "MPU6050.h"

#include <string>        // For string
#include <iostream>      // For cout
#include <fstream>       // For ofstream
#include <filesystem>    // For exists
#include <algorithm>     // For chrono
#include <wiringPiI2C.h> // For wiringPiI2CReadReg8 and wiringPiI2CSetup

MPU6050::MPU6050() {
  fd = wiringPiI2CSetup(Device_Address);
  wiringPiI2CWriteReg8(fd, SMPLRT_DIV,  0x07);
  wiringPiI2CWriteReg8(fd, PWR_MGMT_1,  0x01);
  wiringPiI2CWriteReg8(fd, CONFIG,      0);
  wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);
  wiringPiI2CWriteReg8(fd, INT_ENABLE,  0x01);
  Ax = Ay = Az = Gx = Gy = Gz = 0;
}

MPU6050::~MPU6050() {}

void MPU6050::writeToConsole(int seconds) {
  time_t timer_begin, timer_end;
  bool time_done = false;
  std::cout << "Writing data to console..." << std::endl;
  time(&timer_begin);
  // The MPU6050 is now read from and the data is displayed to the console for
  // the given amount of time.
  while (!time_done) {
    this->readAllData();
    std::cout << "Gx=" << Gx << " °/s\t" << "Gy=" << Gy << " °/s\t" << "Gz=" <<
      Gz << " °/s\t" << "Ax=" << Ax << " g\t" << "Ay=" << Ay << " g\t" << "Az="
      << Az << " g" << std::endl;
    time(&timer_end);
    if (difftime(timer_end, timer_begin) >= seconds)
        time_done = true;
  }
}

void MPU6050::writeToFile(int seconds, std::string fileName) {
  time_t timer_begin, timer_end;
  bool time_done = false;
  // In order to write the recorded data to a file, the file's stream must be 
  // declared.
  std::ofstream file_output;
  // If the file does not exist, the header of the ".csv" file needs to be
  // written. If the file does exist, the output needs to be appended to the
  // file.
  if (!std::filesystem::exists(fileName)) {
    file_output.open(fileName, std::ios::out); 
    file_output << "Datetime,Gx,Gy,Gz,Ax,Ay,Az\n";
  } else { file_output.open(fileName, std::ios::app); }
  // With the file initialized, the MPU6050 can be read and the data can be 
  // written to the file. The sensor is constantly read from, the file is
  // written to, and the time is checked to ensure the sensor is only read 
  // from for the given amount of time.
  time(&timer_begin);
  while (!time_done) {
    this->readAllData();
    file_output << std::to_string(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count()
    ) << "," << Gx << "," << Gy << "," << Gz << "," << Ax << "," << Ay << ","
      << Az << ",\n";
    time(&timer_end);
    if (difftime(timer_end, timer_begin) >= seconds)
        time_done = true;
  }
  // After writing the data to the file for the given period of time,
  // the file is closed to cleanup the process.
  file_output.close();
}

short MPU6050::readRawData(int addr) {
  short high_byte, low_byte, value;
  high_byte = wiringPiI2CReadReg8(fd, addr);
  low_byte = wiringPiI2CReadReg8(fd, addr + 1);
  value = (high_byte << 8) | low_byte;
  return value;
}

void MPU6050::readAllData() {
  Acc_x = readRawData(ACCEL_XOUT_H);
  Acc_y = readRawData(ACCEL_YOUT_H);
  Acc_z = readRawData(ACCEL_ZOUT_H);
  Gyro_x = readRawData(GYRO_XOUT_H);
  Gyro_y = readRawData(GYRO_YOUT_H);
  Gyro_z = readRawData(GYRO_ZOUT_H);
  Ax = Acc_x / 16384.0; Ay = Acc_y / 16384.0; Az = Acc_z / 16384.0;
  Gx = Gyro_x / 131; Gy = Gyro_y / 131; Gz = Gyro_z / 131;
}