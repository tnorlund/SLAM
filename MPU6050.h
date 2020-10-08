#ifndef __MPU6050_INCLUDED__
#define __MPU6050_INCLUDED__

#include <string> // For string

#define Device_Address 0x68	/* Device Address/Identifier for MPU6050 */

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

/**
 *   Class representing the MPU6050 sensor.
 * 
 *   This class can read the sensor after initializing and seting up the 
 *   sensor.
 */
class MPU6050 {
public:
  /**
   *   Create, initialize, and allocate space for the sensor.
   * 
   *   In order to initializes a connection to the MPU6050, the devices
   *   filehandle must be found. This handle is then used to write to
   *   the required registers.
   */
  MPU6050();

  /**
   *   Close and deallocate space for the sensor.
   */
  ~MPU6050();

  /**
   *   Read from the sensor and write the data to the screen.
   * 
   *   @param seconds the amount of time to read from the sensor
   */
  void writeToConsole(int seconds);

  /**
   *   Read from the sensor and write the data to a file.
   * 
   *   @param seconds the amount of time to read from the sensor
   *   @param fileName the name of the file to write to
   */
  void writeToFile(int seconds, std::string fileName);
private:
  /**
   *   Reads and saves the data from the sensor.
   */
  short readRawData(int addr);

  /**
   *   Reads all the datas from the sensor.
   */
  void readAllData();

  /**
   *   The file handle of the MPU6050.
   * 
   *   This handle is used to communicate with the MPU6050 by using the 
   *   different registers found on the sensor. The handle is set when the 
   *   sensor is initialized.
   */
  int fd;

  /**
   *   The raw acceleration data read from the sensor.
   */
  float Acc_x, Acc_y, Acc_z;

  /**
   *   The raw gyroscope data read from the sensor.
   */
  float Gyro_x, Gyro_y, Gyro_z;

  /**
   *   The acceleration data in degrees per second.
   * 
   *   The sensor is uncalibrated to start and reads the acceleration data as
   *   raw bytes. The acceleration data must be scaled to be read as degrees
   *   per second.
   */
  float Ax, Ay, Az;

  /**
   *   The gyroscope data in Newtons.
   * 
   *   The sensor is uncalibrated to start and reads the gyroscope data as raw
   *   bytes. The gyroscope's data must be scaled to be read in Newtons with
   *   respect to Earth's gravity.
   */
  float Gx, Gy, Gz;
};

#endif