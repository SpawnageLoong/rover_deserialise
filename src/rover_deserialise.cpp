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
    std::string str(buffer);
    std::cout << "Received serial data" << std::endl;
    std::cout << str << std::endl;
    std::cout << "Received serial data" << std::endl;
    std::cout << buffer << std::endl;
    std::cout << "Buffer[0]: " << buffer[0] << std::endl;
    char a = buffer[0];
    std::cout << "a: " << a << std::endl;
    sensorData.ext_temp0 = (uint16_t) str[0]  << 8  | (uint16_t) str[0];
    std::cout << "buffer[0]: " << std::bitset<8>(str[0]) << std::endl;
    std::cout << "buffer[1]: " << std::bitset<8>(str[1]) << std::endl;
    std::cout << "buffer[0]: " << (int) str[0] << std::endl;
    std::cout << "buffer[1]: " << (int) str[1] << std::endl;
    std::cout << "External temperature 0 binary: " << std::bitset<16>(sensorData.ext_temp0) << std::endl;
    std::cout << "Char encoding: " << str[0] << std::endl;
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
    char buffer[256];

    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeserialPublisher>();

    std::ifstream serialPort("/dev/pts/6");
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