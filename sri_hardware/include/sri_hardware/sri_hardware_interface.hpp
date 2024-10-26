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

#ifndef SRI_HARDWARE__EXTERNAL_RRBOT_FORCE_TORQUE_SENSOR_HPP_
#define SRI_HARDWARE__EXTERNAL_RRBOT_FORCE_TORQUE_SENSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace sri_hardware {
class SRIForceTorqueSensorHardwareInterface
    : public hardware_interface::SensorInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SRIForceTorqueSensorHardwareInterface)

    SRIForceTorqueSensorHardwareInterface(); // added constructor

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:

    std::string hw_sensor_ip_;
    std::string hw_sensor_port_;
    const std::string hw_sensor_command_ = "AT+GOD\r\n";
    boost::asio::io_context hw_sensor_io_context_;
    boost::asio::ip::tcp::socket hw_sensor_socket_;

};

} // namespace sri_hardware

#endif // sri_hardware__EXTERNAL_RRBOT_FORCE_TORQUE_SENSOR_HPP_
