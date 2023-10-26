/**
 * @file rover_deserialise.cpp
 * @author Richard Loong (richardloongcj@gmail.com)
 * @brief A ROS Node to receive sensor data over serial and publish them to ROS topics.
 * @version 0.1
 * @date 2023-09-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <bitset>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "rover_interfaces/msg/sensor_data.hpp"

class DeserialPublisher : public rclcpp::Node
{
    public:
        DeserialPublisher()
        : Node("deserial_publisher")
        {
            this->declare_parameter("sensor_data_topic_name", "/sensor_data");

            publisher_ =
                this->create_publisher<rover_interfaces::msg::SensorData>(
                    this->get_parameter("sensor_data_topic_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
        }

        void publishSensorData(rover_interfaces::msg::SensorData sensorData)
        {
            publisher_->publish(sensorData);
        }
    
    private:
        rclcpp::Publisher<rover_interfaces::msg::SensorData>::SharedPtr publisher_;
};

void serialCallback(const char* buffer, rover_interfaces::msg::SensorData &sensorData)
{
    sensorData.ext_temp0 = (uint16_t) buffer[1]  << 8  | (uint16_t) buffer[2];
    sensorData.int_temp0 = (uint16_t) buffer[3]  << 8  | (uint16_t) buffer[4];
    sensorData.int_hum0  = (uint16_t) buffer[5]  << 8  | (uint16_t) buffer[6];
    sensorData.int_temp1 = (uint16_t) buffer[7]  << 8  | (uint16_t) buffer[8];
    sensorData.int_hum1  = (uint16_t) buffer[9]  << 8  | (uint16_t) buffer[10];
    sensorData.int_temp2 = (uint16_t) buffer[11] << 8  | (uint16_t) buffer[12];
    sensorData.int_hum2  = (uint16_t) buffer[13] << 8  | (uint16_t) buffer[14];
    sensorData.int_pres0 = (uint16_t) buffer[15] << 8  | (uint16_t) buffer[16];
    sensorData.int_alt   = (int16_t)  buffer[17] << 8  | (int16_t)  buffer[18];
    sensorData.accel_x   = (int32_t)  buffer[19] << 24 | (int32_t)  buffer[20] << 16 | (int32_t)  buffer[21] << 8 | (int32_t)  buffer[22];
    sensorData.accel_y   = (int32_t)  buffer[23] << 24 | (int32_t)  buffer[24] << 16 | (int32_t)  buffer[25] << 8 | (int32_t)  buffer[26];
    sensorData.accel_z   = (int32_t)  buffer[27] << 24 | (int32_t)  buffer[28] << 16 | (int32_t)  buffer[29] << 8 | (int32_t)  buffer[30];
    sensorData.gyro_x    = (int32_t)  buffer[31] << 24 | (int32_t)  buffer[32] << 16 | (int32_t)  buffer[33] << 8 | (int32_t)  buffer[34];
    sensorData.gyro_y    = (int32_t)  buffer[35] << 24 | (int32_t)  buffer[36] << 16 | (int32_t)  buffer[37] << 8 | (int32_t)  buffer[38];
    sensorData.gyro_z    = (int32_t)  buffer[39] << 24 | (int32_t)  buffer[40] << 16 | (int32_t)  buffer[41] << 8 | (int32_t)  buffer[42];
    sensorData.mag_x     = (int32_t)  buffer[43] << 24 | (int32_t)  buffer[44] << 16 | (int32_t)  buffer[45] << 8 | (int32_t)  buffer[46];
    sensorData.mag_y     = (int32_t)  buffer[47] << 24 | (int32_t)  buffer[48] << 16 | (int32_t)  buffer[49] << 8 | (int32_t)  buffer[50];
    sensorData.mag_z     = (int32_t)  buffer[51] << 24 | (int32_t)  buffer[52] << 16 | (int32_t)  buffer[53] << 8 | (int32_t)  buffer[54];
    sensorData.int_temp3 = (uint16_t) buffer[55] << 8  | (uint16_t) buffer[56];
}

void timer(std::function<void(void)> func, unsigned int interval)
{
  std::thread([func, interval]()
  { 
    while (true)
    {
      func();
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
  }).detach();
}

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    rover_interfaces::msg::SensorData sensorData;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeserialPublisher>();

    std::ifstream serialPort("/dev/ttyESPsensors");
    if (!serialPort) {
        std::cerr << "Error: Cannot open the specified serial port." << std::endl;
        return 1;
    }
    std::cout << "Serial port opened" << std::endl;

    std::thread listenerThread([&]()
    {
        std::string line;
        while (true)
        {
            std::getline(serialPort, line);
            if (!serialPort) {
                std::cerr << "Error reading from the serial port." << std::endl;
                break;
            }
            std::cout << "Received: " << line << std::endl;
            serialCallback(line.c_str(), sensorData);
        }
    });
    listenerThread.detach();

    std::thread timerThread([&]()
    {
        while (true)
        {
            node->publishSensorData(sensorData);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
    timerThread.detach();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
