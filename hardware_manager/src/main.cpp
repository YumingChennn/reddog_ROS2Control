#include <rclcpp/rclcpp.hpp>
#include "xdainterface.h"
#include <mavros_msgs/msg/rtcm.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>

using std::chrono::milliseconds;

Journaller *gJournal = 0;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With SingleThreadedExecutor, all callbacks will be called from within this thread (the main thread in this case).
    rclcpp::executors::SingleThreadedExecutor exec;

    // Create a node called "xsens_driver"
    auto node = std::make_shared<rclcpp::Node>("xsens_driver");
    // Add the node to the executor
    exec.add_node(node);

    // Declare the XdaInterface with the node
    auto xdaInterface = std::make_shared<XdaInterface>(node);
    RCLCPP_INFO(node->get_logger(), "XdaInterface has been initialized");

    if (!xdaInterface->connectDevice()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect device");
        return -1;
    }

    while (rclcpp::ok())
    {
        auto data = xdaInterface->spinForReddog(milliseconds(10));
        if (data.empty()) {
            // RCLCPP_WARN(node->get_logger(), "No data received, retrying...");
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "IMU Data: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6],
            data[7], data[8], data[9]
        );
        exec.spin_some();
    }

    // Reset the xdaInterface pointer to ensure it is destroyed before calling rclcpp::shutdown()
    xdaInterface.reset();

    rclcpp::shutdown();

    return 0;
}
