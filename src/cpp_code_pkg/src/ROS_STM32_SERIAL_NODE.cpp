#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

class SerialNode : public rclcpp::Node
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
                               // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
                               // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
        tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;    // read doesn't block

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

    SerialNode() : Node("Serial_node") // Create a node with name stated
    {

        const char *portname = "/dev/ttyUSB0";
        serialPort = openSerialPort(portname);

        if (!configureSerialPort(serialPort, B115200))
        {
            RCLCPP_INFO(this->get_logger(), "Closing Serial Port....");
            // closeSerialPort(serialPort);
        }

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SerialNode::timer_callback, this));
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "serial_write", 10, std::bind(&SerialNode::write_callback, this, std::placeholders::_1));

        // const char *message = "Hello World";
    }

private:
    void timer_callback()
    {
        // Keyboard code
        char buf[100];
        int n = read(serial_port, buf, sizeof(buf));

        if (n > 0)
        {
            std::string result;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(100000, 999999);
            int random_number = dis(gen);
            std::string random_number_str = std::to_string(random_number);

            auto message = std_msgs::msg::String();
            message.data = result;
            std::string data_to_send = random_number_str;
            RCLCPP_INFO(this->get_logger(), "Message Sent: %s", data_to_send.c_str());

            write(serialPort, data_to_send.c_str(), data_to_send.length());
            if (serialPort > 0)
            {
                // publisher_
            }

            // publisher_->publish(message);
        }
    }

    int serial_port;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                   // initialize ROS 2 communication
    auto node = std::make_shared<SerialNode>(); // creating a shared ponter to the node
    rclcpp::spin(node);                         // loop as node is executed
    rclcpp::shutdown();                         // shutdown the communication, stop spinning
    return 0;
}