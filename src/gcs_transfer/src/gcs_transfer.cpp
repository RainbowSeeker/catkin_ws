#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <form_msgs/msg/uav_command.hpp>
#include <mavlink.h>

#include <thread>
#include <boost/asio.hpp>


using namespace std::chrono_literals;
using namespace form_msgs::msg;

using boost::asio::ip::tcp;

class gcs_transfer : public rclcpp::Node
{
public:
    explicit gcs_transfer(int uav_num);
    ~gcs_transfer();

private:

    // Part boost::asio
    void start_tcp_server();
    void handle_accept();
    void handle_read(std::shared_ptr<tcp::socket> socket);
    boost::asio::io_context _io_context;
    tcp::acceptor _acceptor;
    std::thread _io_thread;

    // Part ros2
    int _uav_num;

    // Publisher
    rclcpp::Publisher<UavCommand>::SharedPtr *_uav_command_pub;

    // Part mavlink
    void handle_mavlink_message(const mavlink_message_t &msg);
    mavlink_status_t _mavlink_status;
    mavlink_message_t _mavlink_msg;
};

gcs_transfer::gcs_transfer(int uav_num) : 
    Node("gcs_transfer"), _acceptor(_io_context), _uav_num(uav_num)
{
    const char *topic_ns = "/px4_";
    
    _uav_command_pub = new rclcpp::Publisher<UavCommand>::SharedPtr[_uav_num];
    
    for (int i = 0; i < _uav_num; i++)
    {
        _uav_command_pub[i] = this->create_publisher<UavCommand>(
            topic_ns + std::to_string(i) + "/fmu/in/uav_command", 10);
    }
    
    RCLCPP_INFO(get_logger(), "GCS transfer node for %d UAVs started.", _uav_num);
    
    start_tcp_server();
}

gcs_transfer::~gcs_transfer()
{
    _io_context.stop();
    _io_thread.join();
    delete[] _uav_command_pub;
}

void gcs_transfer::start_tcp_server()
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

void gcs_transfer::handle_accept()
{
    auto socket = std::make_shared<tcp::socket>(_io_context);
    _acceptor.async_accept(*socket, [this, socket](const boost::system::error_code &ec) {
        if (!ec)
        {
            RCLCPP_INFO(get_logger(), "New connection from %s", socket->remote_endpoint().address().to_string().c_str());
            
            handle_read(socket);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error accepting connection: %s", ec.message().c_str());
        }
        handle_accept();
    });
}

void gcs_transfer::handle_read(std::shared_ptr<tcp::socket> socket)
{
    auto buffer = std::make_shared<std::vector<uint8_t>>(1024);
    socket->async_read_some(boost::asio::buffer(*buffer), [this, socket, buffer](const boost::system::error_code &ec, size_t bytes_transferred) {
        if (!ec)
        {
            RCLCPP_INFO(get_logger(), "Received %lu bytes", bytes_transferred);
            for (size_t i = 0; i < bytes_transferred; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, (*buffer)[i], &_mavlink_msg, &_mavlink_status))
                {
                    handle_mavlink_message(_mavlink_msg);
                }
            }
            handle_read(socket);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error reading from socket: %s", ec.message().c_str());
        }
    });
}

void gcs_transfer::handle_mavlink_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_UAV_COMMAND:
        {
            int sysid = msg.sysid; // from 1
            if (sysid < 1 || sysid > _uav_num)
            {
                RCLCPP_WARN(get_logger(), "Received UAV_COMMAND message with invalid sysid %d", sysid);
            }
            else
            {
                mavlink_uav_command_t cmd;
                mavlink_msg_uav_command_decode(&msg, &cmd);

                auto uav_command = UavCommand();
                uav_command.timestamp = cmd.timestamp;
                uav_command.param1 = cmd.param1;
                uav_command.param2 = cmd.param2;
                uav_command.param3 = cmd.param3;
                uav_command.command = cmd.command;
                _uav_command_pub[sysid - 1]->publish(uav_command);
            }
            break;
        }
    default:
        RCLCPP_WARN(get_logger(), "Received unknown message with ID %d", msg.msgid);
        break;
    }
}

int main(int argc, char * argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: gcs_transfer <uav_num>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gcs_transfer>(std::stoi(argv[1])));
    rclcpp::shutdown();
    return 0;
}