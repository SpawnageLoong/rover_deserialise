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
#include <atomic>
#include <signal.h>
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
            stopListening();
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

        void startListening(std::function<void(const char*, rover_interfaces::msg::SensorData&)> callback, rover_interfaces::msg::SensorData &sensorData)
        {
            if (!isOpen() || listening_) {
                return;
            }

            listening_ = true;
            listenerThread_ = std::thread([this, callback, &sensorData]() {
                char buffer[256];
                while (listening_) {
                    ssize_t bytesRead = read(fd_, buffer, sizeof(buffer));
                    if (bytesRead > 0) {
                        callback(buffer, sensorData);
                    }
                }
            });
        }

        void stopListening()
        {
            listening_ = false;
            if (listenerThread_.joinable()) {
                listenerThread_.join();
            }
        }

    private:
        int fd_;
        std::thread listenerThread_;
        std::atomic<bool> listening_{false};
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

    SerialPort serialPort("/dev/pts/6", B115200);
    std::cout << "Serial port opened" << std::endl;
    //serialPort.startListening(serialCallback, sensorData);

    std::thread listenerThread([&]()
    {
        while (true)
        {
            char delim_buffer;
            serialPort.readData(&delim_buffer, (size_t) 1);
            if (delim_buffer != 0x0A)
            {
                continue;
            }
            serialPort.readData(&delim_buffer, (size_t) 1);
            if (delim_buffer != 0x24)
            {
                continue;
            }
            serialPort.readData(buffer, (size_t) 56);
            serialCallback(buffer, sensorData);
        }
    });

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

    //serialPort.stopListening();
    rclcpp::shutdown();
    return 0;
}