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

        setExtTemp0(uint16_t value)
        {
            ext_temp0 = value;
        }
    
    private:
        auto message = rover_interfaces::msg::SensorData();

        rclcpp::Publisher<rover_interfaces::msg::SensorData>::SharedPtr publisher_;
};

class SerialPort {
    public:
        SerialPort(const char* portName, speed_t baudRate)
        {
            fd_ = open(portName, O_RDWR);
            if (fd_ == -1)
            {
                std::cout << "Error opening serial port" << std::endl;
                return;
            }

            struct termios tty;
            if (tcgetattr(fd_, &tty) != 0)
            {
                std::cout << "Error getting serial port attributes" << std::endl;
                return;
            }

            cfsetospeed(&tty, baudRate);
            cfsetispeed(&tty, baudRate);

            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CREAD;
            tty.c_cflag |= CLOCAL;

            if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            {
                std::cout << "Error setting serial port attributes" << std::endl;
                return;
            }
        }

        ~SerialPort()
        {
            if (isOpen())
            {
                close(fd_);
            }
        }

        bool isOpen()
        {
            return fd_ != -1;
        }

        bool WriteData(const char* data, size_t length)
        {
            if (!isOpen())
            {
                return false;
            }
            ssize_t bytesWritten = write(fd_, data, length);
            return bytesWritten == static_cast<ssize_t>(length);
        }

        size_t readData(char* buffer, size_t maxLength)
        {
            if (!isOpen())
            {
                return -1;
            }
            ssize_t bytesRead = read(fd_, buffer, maxLength);
            return bytesRead;
        }

    private:
        int fd_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeserialPublisher>());

    SerialPort serialPort("/dev/pts/5", B115200);
    std::cout << "Serial port opened" << std::endl;
    
    rclcpp::shutdown();
    return 0;
}