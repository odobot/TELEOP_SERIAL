#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <random>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

using namespace std::chrono_literals;
int serialPort;

class TeleopSTMNOde : public rclcpp::Node
{
public:
    int openSerialPort(const char *portname)
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        int serial_port = open(portname, O_RDWR);

        if (serial_port < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening  %s\n", portname);
            // printf("Error opening  %s\n", portname, strerror(errno));
            return -1;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }
        return serial_port;
    }
    bool configureSerialPort(int serial_port, int baudrate)
    {
        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port, &tty) != 0)
        {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s\n", strerror(errno));

            return false;
        }

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag |= OCRNL;
        //  tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        //  tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
        tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;   // read doesn't block

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s\n", strerror(errno));
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully configured board ");
        return true;
    }
    void closeSerialPort(int serial_port)
    {
        RCLCPP_INFO(this->get_logger(), "Closing Serial Port....");
        close(serial_port);
    }
    TeleopSTMNOde() : Node("TeleopSTMNOde") // Create a node with name stated
    {
        const char *portname = "/dev/ttyUSB0";
        serialPort = openSerialPort(portname);
        if (!configureSerialPort(serialPort, B115200))
        {
            RCLCPP_INFO(this->get_logger(), "Closing Serial Port....");
            // closeSerialPort(serialPort);
        }

        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TeleopSTMNOde::sub_callBack, this, std::placeholders::_1));
    }

private:
    void sub_callBack(const geometry_msgs::msg::Twist::SharedPtr data)
    {
        RCLCPP_INFO(this->get_logger(), "Callback triggered.");
        RCLCPP_INFO(this->get_logger(), "Linear:[%f,%f,%f]",
                    data->linear.x, data->linear.y, data->linear.z);
        if ((data->linear.x) > 0.0) // Moving Forward
        {
            /* 1st 3 ->  speed for Motor A
               2nd 3 ->  speed for Motor B
               7th bit  -> Motor A direction
               8th bit  -> Motor B direction
               */
            int speed = 170170;
            // std::string vel = "120120";
            std::string vel = std::to_string(speed);
            std::string data_to_send = vel;
            RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Forward", data_to_send.c_str());
            write(serialPort, data_to_send.c_str(), data_to_send.length());
        }
        else if ((data->linear.x) < 0.0) // Moving Backward
        {
            int speed = 270270;
            // std::string vel = "12012011";
            std::string vel = std::to_string(speed);
            std::string data_to_send = vel;
            RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Backward ", data_to_send.c_str());
            write(serialPort, data_to_send.c_str(), data_to_send.length());
        }
        else if ((data->angular.z) < 0.0) // Moving to Right
        {
            int speed = 170270;
            // std::string vel = "12012001";
            std::string vel = std::to_string(speed);
            std::string data_to_send = vel;
            RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Right ", data_to_send.c_str());
            write(serialPort, data_to_send.c_str(), data_to_send.length());
        }
        else if ((data->angular.z) > 0.0) // Moving to Left
        {
            int speed = 270170;
            // std::string vel = "12012010";
            std::string vel = std::to_string(speed);
            std::string data_to_send = vel;
            RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Left", data_to_send.c_str());
            write(serialPort, data_to_send.c_str(), data_to_send.length());
        }

        else if ((data->linear.x) == 0.0 && (data->angular.z) == 0.0)
        {
            // int speed = 000000;
            std::string vel = "000000";
            std::string data_to_send = vel;
            RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s", data_to_send.c_str());
            write(serialPort, data_to_send.c_str(), data_to_send.length());
        }

        unsigned char read_buf[80];
        // memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serialPort, &read_buf, sizeof(read_buf) - 1);

        if (num_bytes > 0)
        {

            read_buf[num_bytes] = 0;
            RCLCPP_INFO(this->get_logger(), "Read %d: \"\n\"%s", num_bytes, read_buf);

            for (int i = 0; i < 81; i++)
            {
                read_buf[i] = 0;
            }
        }
        // std:: string msg = std::string(read_buf, num_bytes);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                      // initialize ROS 2 communication
    auto node = std::make_shared<TeleopSTMNOde>(); // creating a shared ponter to the node
    rclcpp::spin(node);                            // loop as node is executed
    rclcpp::shutdown();                            // shutdown the communication, stop spinning
    return 0;
}