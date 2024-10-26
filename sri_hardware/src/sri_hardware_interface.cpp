// Copyright 2024 Zixing Jiang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "sri_hardware/sri_hardware_interface.hpp"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <chrono>
#include <cmath>
#include <hardware_interface/sensor_interface.hpp>
#include <iomanip>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sri_hardware {

SRIForceTorqueSensorHardwareInterface::SRIForceTorqueSensorHardwareInterface()
    : hw_sensor_io_context_(), hw_sensor_socket_(hw_sensor_io_context_) {}

hardware_interface::CallbackReturn
SRIForceTorqueSensorHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {

    if (hardware_interface::SensorInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_sensor_ip_ = info_.hardware_parameters["sensor_ip"];
    hw_sensor_port_ = info_.hardware_parameters["sensor_port"];

    RCLCPP_INFO(get_logger(), "Initializing ...");
    RCLCPP_INFO(get_logger(), "Reading params from URDF ...");
    RCLCPP_INFO(get_logger(), "Get sensor IP: %s", hw_sensor_ip_.c_str());
    RCLCPP_INFO(get_logger(), "Get sensor port: %s", hw_sensor_port_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SRIForceTorqueSensorHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &) {
    try {
        boost::asio::ip::tcp::resolver resolver(hw_sensor_io_context_);
        boost::asio::ip::tcp::resolver::results_type endpoints =
            resolver.resolve(hw_sensor_ip_, hw_sensor_port_);
        boost::asio::connect(hw_sensor_socket_, endpoints);
        return hardware_interface::CallbackReturn::SUCCESS;
    } catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
SRIForceTorqueSensorHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &) {
    try {
        if (hw_sensor_socket_.is_open()) {
            hw_sensor_socket_.close();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    } catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SRIForceTorqueSensorHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    try {
        // Send command to the socket
        boost::asio::write(hw_sensor_socket_,
                           boost::asio::buffer(hw_sensor_command_));

        int buffer_size_ = 8192;
        std::vector<char> data(buffer_size_);

        size_t len = hw_sensor_socket_.read_some(boost::asio::buffer(data));
        if (len == 0) {
            return hardware_interface::return_type::ERROR;
        }

        std::array<float, 3> force = {*reinterpret_cast<float *>(&data[6]),
                                      *reinterpret_cast<float *>(&data[10]),
                                      *reinterpret_cast<float *>(&data[14])};

        std::array<float, 3> torque = {*reinterpret_cast<float *>(&data[18]),
                                       *reinterpret_cast<float *>(&data[22]),
                                       *reinterpret_cast<float *>(&data[26])};

        for (const auto &[name, descr] : sensor_state_interfaces_) {
            if (name.find("force.x") != std::string::npos) {
                set_state(name, double(force[0]));
            } else if (name.find("force.y") != std::string::npos) {
                set_state(name, double(force[1]));
            } else if (name.find("force.z") != std::string::npos) {
                set_state(name, double(force[2]));
            } else if (name.find("torque.x") != std::string::npos) {
                set_state(name, double(torque[0]));
            } else if (name.find("torque.y") != std::string::npos) {
                set_state(name, double(torque[1]));
            } else if (name.find("torque.z") != std::string::npos) {
                set_state(name, double(torque[2]));
            }
        }

        return hardware_interface::return_type::OK;

    } catch (std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

} // namespace sri_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sri_hardware::SRIForceTorqueSensorHardwareInterface,
                       hardware_interface::SensorInterface)
