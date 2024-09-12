#include "mavlink_types.h"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <boost/asio.hpp>

#include <mavlink.h>
#include <rclcpp/rclcpp.hpp>
#include <form_msgs/msg/uav_command.hpp>
#include <form_msgs/msg/uav_status.hpp>

using namespace std::chrono_literals;
using namespace form_msgs::msg;
using boost::asio::ip::tcp;

class MavlinkSession : public std::enable_shared_from_this<MavlinkSession>
{
public:
    explicit MavlinkSession(boost::asio::io_context &io_context, std::shared_ptr<tcp::socket> socket, std::function<void(const mavlink_message_t &)> depack_handler, std::function<int(uint8_t *, int)> pack_handler = nullptr)
        : _socket(socket), endpoint_name(socket->remote_endpoint().address().to_string()), _timer(io_context), _depack_handler(depack_handler), _pack_handler(pack_handler)
    {
    }

    ~MavlinkSession()
    {
        std::cout << "Connection to " << endpoint_name << " closed" << std::endl;
    }

    void start()
    {
        do_read();

        if (_pack_handler)
            start_timer();
    }

private:
    void do_read()
    {
        auto self(shared_from_this());
        _socket->async_read_some(boost::asio::buffer(_data, max_length), [this, self](boost::system::error_code ec, std::size_t length) {
            if (!ec)
            {
                std::cout << "Received " << length << " bytes from " << endpoint_name << std::endl;
                for (size_t i = 0; i < length; i++)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_0, _data[i], &_mavlink_msg, &_mavlink_status))
                    {
                        _depack_handler(_mavlink_msg);
                    }
                }
                do_read();
            }
        });
    }

    void start_timer()
    {
        auto self(shared_from_this());
        _timer.expires_after(100ms);
        _timer.async_wait([this, self](const boost::system::error_code &ec) {
            if (!ec)
            {
                if (!_pack_handler) return;

                uint8_t buf[max_length];
                int len;
                if ((len = _pack_handler(buf, max_length)) > 0)
                {
                    boost::system::error_code error;
                    _socket->write_some(boost::asio::buffer(buf, len), error);
                    if (error)
                    {
                        std::cerr << "Error sending data to " << endpoint_name << ": " << error.message() << std::endl;
                        return;
                    }
                }

                start_timer();
            }
        });
    }

    std::shared_ptr<tcp::socket> _socket;
    const std::string endpoint_name;
    boost::asio::steady_timer _timer;
    enum { max_length = 1024 };
    uint8_t _data[max_length];
    std::function<void(const mavlink_message_t &)> _depack_handler;
    std::function<int(uint8_t *, int)> _pack_handler;

    // Mavlink parser
    mavlink_message_t _mavlink_msg;
    mavlink_status_t _mavlink_status;
};


class GcsTransfer : public rclcpp::Node
{
public:
    explicit GcsTransfer(int uav_num);
    ~GcsTransfer();

private:
    // Part boost::asio
    void start_tcp_server();
    void handle_accept();
    boost::asio::io_context _io_context;
    tcp::acceptor _acceptor;
    std::thread _io_thread;
    std::mutex _mutex;

    // Part ros2
    int _uav_num;
    std::vector<std::vector<char>> _uav_status_buf;

    // Subscriber
    std::vector<rclcpp::Subscription<UavStatus>::SharedPtr> _uav_status_sub;

    // Publisher
    std::vector<rclcpp::Publisher<UavCommand>::SharedPtr> _uav_command_pub;
    void handle_mavlink_message(const mavlink_message_t &msg);
    int collect_mavlink_message(uint8_t *buf, int len);
};

GcsTransfer::GcsTransfer(int uav_num) :
    Node("GcsTransfer"), _acceptor(_io_context), _uav_num(uav_num)
{
    const char *topic_ns = "/px4_";

    _uav_status_buf.resize(_uav_num);
    _uav_status_sub.resize(_uav_num);
    _uav_command_pub.resize(_uav_num);
    for (int i = 0; i < _uav_num; i++)
    {
        _uav_status_sub[i] = this->create_subscription<UavStatus>(
            topic_ns + std::to_string(i + 1) + "/fmu/out/uav_status", 10,
            [this, i](const UavStatus::SharedPtr sp) {
                uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {};
                mavlink_message_t msg;
                {
                    mavlink_uav_status_t status;
                    status.timestamp = sp->timestamp;
                    status.stage = sp->stage;
                    static_assert(sizeof(status.sta_msg) == sizeof(sp->sta_msg), "Size of sta_msg does not match");
                    std::memcpy(status.sta_msg, sp->sta_msg.data(), sp->sta_msg.size());
                    mavlink_msg_uav_status_encode(i + 1, 0, &msg, &status);
                }
                int size = mavlink_msg_to_send_buffer(buf, &msg);
                if (size > 0)
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _uav_status_buf[i].assign(buf, buf + size);
                }
            });

        _uav_command_pub[i] = this->create_publisher<UavCommand>(
            topic_ns + std::to_string(i + 1) + "/fmu/in/uav_command", 10);
    }

    RCLCPP_INFO(get_logger(), "GCS transfer node for %d UAVs started.", _uav_num);

    start_tcp_server();
}

GcsTransfer::~GcsTransfer()
{
    _io_context.stop();
    _io_thread.join();
}

void GcsTransfer::start_tcp_server()
{
    try
    {
        tcp::endpoint endpoint(tcp::v4(), 14440);
        _acceptor.open(endpoint.protocol());
        _acceptor.set_option(tcp::acceptor::reuse_address(true));
        _acceptor.bind(endpoint);
        _acceptor.listen();

        // Start the io_context in a separate thread
        _io_thread = std::thread([this]() {
            RCLCPP_INFO(get_logger(), "TCP server started on port 14440");;
            _io_context.run();
        });

        // Accept incoming connections
        handle_accept();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Error starting TCP server: %s", e.what());
    }
}

void GcsTransfer::handle_accept()
{
    auto socket = std::make_shared<tcp::socket>(_io_context);
    _acceptor.async_accept(*socket, [this, socket](const boost::system::error_code &ec) {
        if (!ec)
        {
            RCLCPP_INFO(get_logger(), "New connection from %s", socket->remote_endpoint().address().to_string().c_str());

            auto session = std::make_shared<MavlinkSession>(_io_context, socket,
                std::bind(&GcsTransfer::handle_mavlink_message, this, std::placeholders::_1),
                std::bind(&GcsTransfer::collect_mavlink_message, this, std::placeholders::_1, std::placeholders::_2));

            session->start();
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error accepting connection: %s", ec.message().c_str());
        }
        handle_accept();
    });
}

// !!!important!!! Pay attention to thread safety.
void GcsTransfer::handle_mavlink_message(const mavlink_message_t &msg)
{
    if (msg.sysid < 1 || msg.sysid > _uav_num)
    {
        RCLCPP_WARN(get_logger(), "Received UAV_COMMAND message with invalid sysid %d", msg.sysid);
        return;
    }

    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_UAV_COMMAND:
        {
            mavlink_uav_command_t cmd;
            mavlink_msg_uav_command_decode(&msg, &cmd);

            auto uav_command = UavCommand();
            uav_command.timestamp = cmd.timestamp;
            uav_command.param1 = cmd.param1;
            uav_command.param2 = cmd.param2;
            uav_command.param3 = cmd.param3;
            uav_command.command = cmd.command;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _uav_command_pub[msg.sysid - 1]->publish(uav_command);
            }
            RCLCPP_INFO(get_logger(), "Received UAV_COMMAND message from UAV %d, cmd:%d", msg.sysid, cmd.command);
            break;
        }
    default:
        RCLCPP_WARN(get_logger(), "Received unknown message with ID %d", msg.msgid);
        break;
    }
}

// !!!important!!! Pay attention to thread safety.
int GcsTransfer::collect_mavlink_message(uint8_t *buf, int len)
{
    int initial_len = len;

    for (int i = 0; i < _uav_num; i++)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int size = _uav_status_buf[i].size();
        len -= size;
        if (len <= 0) {
            RCLCPP_WARN(get_logger(), "Buffer overflow.");
            break;
        }
        std::memcpy(buf, _uav_status_buf[i].data(), size);
        buf += size;
    }
    return initial_len - len;
}

int main(int argc, char * argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: GcsTransfer <uav_num>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GcsTransfer>(std::stoi(argv[1])));
    rclcpp::shutdown();
    return 0;
}
