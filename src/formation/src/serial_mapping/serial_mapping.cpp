#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <iostream>
#include <thread>
#include <memory>


class SerialDevice
{
public:
    explicit SerialDevice(const char* port, int baudrate)
    {
        _pollfd.fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_pollfd.fd < 0)
        {
            throw std::runtime_error("Failed to open serial port: " + std::string(strerror(errno)));
        }
    
        int speed;
        switch (baudrate)
        {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            case 1000000: speed = B1000000; break;
            case 1500000: speed = B1500000; break;
            case 2000000: speed = B2000000; break;
            case 2500000: speed = B2500000; break;
            case 3000000: speed = B3000000; break;
            default: throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
        }

        int ret;
        struct termios options;
        tcgetattr(_pollfd.fd, &options);
        /* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

        //
        // Input flags - Turn off input processing
        //
        // convert break to null byte, no CR to NL translation,
        // no NL to CR translation, don't mark parity errors or breaks
        // no input parity check, don't strip high bit off,
        // no XON/XOFF software flow control
        //
        options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
        //
        // Output flags - Turn off output processing
        //
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        //
        // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
        //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
        options.c_oflag = 0;

        //
        // No line processing
        //
        // echo off, echo newline off, canonical mode off,
        // extended input processing off, signal chars off
        //
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

        /* no parity, one stop bit, disable flow control */
        options.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

        /* set baudrate */
        if ((ret = cfsetispeed(&options, speed)) < 0)
        {
            throw std::runtime_error("Failed to set input baudrate: " + std::string(strerror(errno)));
        }
        if ((ret = cfsetospeed(&options, speed)) < 0)
        {
            throw std::runtime_error("Failed to set output baudrate: " + std::string(strerror(errno)));
        }
        if ((ret = tcsetattr(_pollfd.fd, TCSANOW, &options)) < 0)
        {
            throw std::runtime_error("Failed to set terminal attributes: " + std::string(strerror(errno)));
        }
        
        _pollfd.events = POLLIN;
        std::cout << "Serial [" << port <<"] opened." << std::endl;
    }

    ~SerialDevice() 
    {
        close(_pollfd.fd);
    }

    int write(const char* buf, int len)
    {
        int ret = ::write(_pollfd.fd, buf, len);
        if (ret < 0)
        {
            std::cout << "Failed to write to serial port:" << ret << std::endl;
        }
        return ret;
    }

    int read(char* buf, int len)
    {
        int ret = ::poll(&_pollfd, 1, 1000);
        if (ret > 0)
        {
            ret = ::read(_pollfd.fd, buf, len);
            if (ret < 0)
            {
                std::cout << "Failed to read from serial port:" << ret << std::endl;
            }
        }
        return ret;
    }

private:
    struct pollfd _pollfd;
};

class UdpDevice
{
public:
    explicit UdpDevice(const char* ip, int port)
    {
        _pollfd.fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (_pollfd.fd < 0)
        {
            throw std::runtime_error("Failed to create UDP socket: " + std::string(strerror(errno)));
        }

        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;

        if (getaddrinfo(ip, std::to_string(port).c_str(), &hints, &result) == 0)
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (connect(_pollfd.fd, ptr->ai_addr, ptr->ai_addrlen) == 0)
                {
                    _pollfd.events = POLLIN;
                    break;
                }
            }
            freeaddrinfo(result);
        }
        else
        {
            throw std::runtime_error("Failed to resolve UDP address: " + std::string(strerror(errno)));
        }
        std::cout << "UDP [" << ip << ":" << port << "] opened." << std::endl;
    }

    ~UdpDevice() 
    {
        close(_pollfd.fd);
    }

    int write(const char* buf, int len)
    {
        int ret = send(_pollfd.fd, buf, len, 0);
        if (ret < 0)
        {
            std::cout << "Failed to write to UDP socket:" << strerror(errno) << std::endl;
        }
        return ret;
    }

    int read(char* buf, int len)
    {
        int ret = poll(&_pollfd, 1, 1000);
        if (ret > 0)
        {
            ret = recv(_pollfd.fd, buf, len, 0);
            if (ret < 0)
            {
                std::cout << "Failed to read from UDP socket:" << strerror(errno) << std::endl;
            }
        }
        return ret;
    }
private:
    struct pollfd _pollfd;
};

class SerialMapping 
{
public:
    explicit SerialMapping(const char* serial_port, int serial_baudrate, const char* udp_ip, int udp_port): 
        _serial(std::make_shared<SerialDevice>(serial_port, serial_baudrate)),
        _udp(std::make_shared<UdpDevice>(udp_ip, udp_port))
    {
    }

    void listen_serial()
    {
        std::cout << "Listening serial" << std::endl;
        char buf[1024];
        while (true)
        {
            int len = _serial->read(buf, sizeof(buf));
            if (len > 0)
            {
                // std::cout << "Received " << len << " bytes from serial" << std::endl;
                _udp->write(buf, len);
            }
        }
    }

    void listen_udp()
    {
        std::cout << "Listening udp" << std::endl;
        char buf[1024];
        while (true)
        {
            int len = _udp->read(buf, sizeof(buf));
            if (len > 0)
            {
                _serial->write(buf, len);
            }
        }
    }

    void run()
    {
        std::thread serial_thread(&SerialMapping::listen_serial, this);
        std::thread udp_thread(&SerialMapping::listen_udp, this);
        
        serial_thread.join();
        udp_thread.join();
    }

private:
    std::shared_ptr<SerialDevice> _serial;
    std::shared_ptr<UdpDevice> _udp;
};


int main(int argc, const char** argv) {
    (void)argc;
    (void)argv;

    SerialMapping mapping("/dev/serial/by-id/usb-CUAV_PX4_CUAV_X7Pro_0-if00", 2000000, "localhost", 8888);
    mapping.run();
    return 0;
}