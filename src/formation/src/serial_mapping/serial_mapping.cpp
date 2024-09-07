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
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <filesystem>

using namespace std::chrono_literals;

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

        if (open_mavlinkv2() < 0)
        {
            throw std::runtime_error("Failed to open mavlinkv2");
        }
    }

    ~SerialDevice()
    {
        close(_pollfd.fd);
    }

    int open_mavlinkv2()
    {
        const char mavlinkv2[] = {(char)0xfd, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        int retry = 10;
        while (retry > 0)
        {
            int nwrite = SerialDevice::write(mavlinkv2, sizeof(mavlinkv2));
            if (nwrite == sizeof(mavlinkv2))
            {
                break;
            }
            retry--;
            usleep(100 * 1000);
        }
        if (retry == 0)
        {
            return -1;
        }

        retry = 100;
        int wait_bytes = 100;
        while (wait_bytes > 0 && retry > 0)
        {
            char buf[128];
            int nread = SerialDevice::read(buf, sizeof(buf));
            if (nread > 0)
            {
                wait_bytes -= nread;
            }
            retry--;
            usleep(20 * 1000);
        }

        return retry > 0 ? 0 : -1;
    }

    int write(const char* buf, int len)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int ret = ::write(_pollfd.fd, buf, len);
        if (ret < 0)
        {
            if (errno == ENODEV || errno == EIO)
            {
                throw std::runtime_error("Serial port disconnected");
            }
            std::cout << "Failed to write to serial port:" << strerror(errno) << std::endl;
        }
        return ret;
    }

    int read(char* buf, int len)
    {
        int ret = ::poll(&_pollfd, 1, 1000);
        if (ret > 0)
        {
            if (_pollfd.revents & POLLHUP)
            {
                throw std::runtime_error("Serial port disconnected");
            }
            else if (_pollfd.revents & POLLIN)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                ret = ::read(_pollfd.fd, buf, len);
                if (ret < 0)
                {
                    std::cout << "Failed to read from serial port:" << strerror(errno) << std::endl;
                }
            }
        }
        return ret;
    }

private:
    struct pollfd _pollfd;
    std::mutex _mutex;
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
        std::lock_guard<std::mutex> lock(_mutex);
        int ret = send(_pollfd.fd, buf, len, 0);
        if (ret < 0)
        {
            if (errno == ECONNREFUSED)
            {
                throw std::runtime_error("UDP socket disconnected");
            }
            std::cout << "Failed to write to UDP socket:" << strerror(errno) << std::endl;
        }
        return ret;
    }

    int read(char* buf, int len)
    {
        int ret = poll(&_pollfd, 1, 1000);
        if (ret > 0)
        {
            if (_pollfd.revents & POLLERR)
            {
                throw std::runtime_error("UDP socket disconnected");
            }
            else if (_pollfd.revents & POLLIN)
            {
                std::lock_guard<std::mutex> lock(_mutex);
                ret = recv(_pollfd.fd, buf, len, 0);
                if (ret < 0)
                {
                    std::cout << "Failed to read from UDP socket:" << strerror(errno) << std::endl;
                }
            }
        }
        return ret;
    }
private:
    struct pollfd _pollfd;
    std::mutex _mutex;
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
        try
        {
            std::cout << "Listening serial" << std::endl;
            char buf[4096];
            while (!_should_exit)
            {
                int len = _serial->read(buf, sizeof(buf));
                if (len > 0)
                {
                    _udp->write(buf, len);
                }
            }
        }
        catch(const std::exception& e)
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _ept_queue.push(std::current_exception());
            _cv.notify_one();
        }
    }

    void listen_udp()
    {
        try
        {
            std::cout << "Listening udp" << std::endl;
            char buf[4096];
            while (!_should_exit)
            {
                int len = _udp->read(buf, sizeof(buf));
                if (len > 0)
                {
                    _serial->write(buf, len);
                }
            }
        }
        catch(const std::exception& e)
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _ept_queue.push(std::current_exception());
            _cv.notify_one();
        }
    }

    void run()
    {
        std::thread serial_thread(&SerialMapping::listen_serial, this);
        std::thread udp_thread(&SerialMapping::listen_udp, this);

        {
            std::unique_lock<std::mutex> lock(_mutex);
            _cv.wait(lock, [this] { return !_ept_queue.empty(); });
        }
        _should_exit = true;
        serial_thread.join();
        udp_thread.join();
        std::rethrow_exception(_ept_queue.front());
    }

private:
    std::shared_ptr<SerialDevice> _serial;
    std::shared_ptr<UdpDevice> _udp;
    std::atomic<bool> _should_exit {false};
    std::mutex _mutex;
    std::condition_variable _cv;
    std::queue<std::exception_ptr> _ept_queue;
};


int main(int argc, const char** argv)
{
    std::string dst_ip = "localhost";
    std::string dst_port = "14550";
    if (argc > 1) {
        dst_ip = argv[1];
    }

    if (argc > 2) {
        dst_port = argv[2];
    }

    const std::string serial_dir = "/dev/serial/by-id/";
    const std::string serial_prefix = "usb-CUAV_PX4";
    std::string serial_filename;

    retry:
    try {
        bool found = false;
        for (const auto & entry : std::filesystem::directory_iterator(serial_dir))
        {
            serial_filename = entry.path().string();
            if (serial_filename.find(serial_prefix) != std::string::npos)
            {
                std::cout << "Found matching device: " << serial_filename << std::endl;
                found = true;
                break;
            }
        }

        if (!found)
        {
            throw std::runtime_error("No matching serial device found");
        }

        SerialMapping mapping(serial_filename.c_str(), 2000000, dst_ip.c_str(), std::stoi(dst_port));
        mapping.run();

    } catch (const std::exception& e) {
        std::cerr << "runtime error: " << e.what() << std::endl;

        std::this_thread::sleep_for(2s);
        goto retry;
    }

    return 0;
}
