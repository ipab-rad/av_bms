#ifndef AV_BMS__BMS_NODE_HPP_
#define AV_BMS__BMS_NODE_HPP_
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;
using can_msgs::msg::Frame;
using sensor_msgs::msg::BatteryState;
using std::placeholders::_1;

namespace bms
{

const auto BATTERY_STATUS = 0x19F21450;
const auto DC_DETAILED_STATUS = 0x19F21250;
const auto VREGS = 0x1CEFFF50;
const auto POWERFINN_CONTROL = 0x650;
const auto UNKNOWN_1 = 0x18EEFF50;
const auto UNKNOWN_2 = 0x5D0;
const auto UNKNOWN_3 = 0x75;
/**
 * @class BMS_node
 * @brief Converts relevant part of can bus data into BMS sensor msgs
 *
 */
class BMS_node : public rclcpp::Node
{
public:
  explicit BMS_node(const rclcpp::NodeOptions & options) : Node("bms_node", options)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/can_bus_2/can_rx");

    subscription_ = this->create_subscription<Frame>(
      input_topic_, 10, std::bind(&BMS_node::bms_CB, this, _1));

    output_topic_ = this->declare_parameter("output_topic", "bms_output");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&BMS_node::timer_CB, this));
    // Define publisher for converted bms msgs
    publisher_ = this->create_publisher<BatteryState>(output_topic_, 10);
  }

private:
  void bms_CB(const Frame::UniquePtr & can_msg)
  {
    parseCanMessage(can_msg);
  }

  void timer_CB()
  {
    auto bms_status_msg = BatteryState();

    bms_status_msg.header.stamp = this->get_clock()->now();
    bms_status_msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    bms_status_msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    bms_status_msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    bms_status_msg.voltage = voltage_;
    bms_status_msg.current = current_;
    bms_status_msg.temperature = highest_temperature_celsius_;
    bms_status_msg.percentage = state_of_charge_ / 100.0;

    publisher_->publish(bms_status_msg);
  }

  void parseCanMessage(const Frame::UniquePtr & msg)
  {
    if (msg->id == BATTERY_STATUS)
    {
      // The "battery status" message comes in three variants; that is, there are three different
      // possible message contents in messages with the same ID. They can be distinguished by their
      // zeroth byte.
      if (msg->data[0] == 0)  // Battery instance 0
      {
        // Voltage
        const int16_t raw_voltage = twoBytesToInt16(msg->data[1], msg->data[2]);
        const double calculated_voltage = 0.01 * static_cast<double>(raw_voltage);
        voltage_ = calculated_voltage;

        // Current
        const int16_t raw_current = twoBytesToInt16(msg->data[3], msg->data[4]);
        const double calculated_current = 0.1 * static_cast<double>(raw_current);
        current_ = calculated_current;

        // Temperature
        const uint16_t raw_high_temperature = twoBytesToUInt16(msg->data[5], msg->data[6]);
        const double calculated_highest_temperature =
          0.01 * static_cast<double>(raw_high_temperature) - 273.15;
        highest_temperature_celsius_ = calculated_highest_temperature;
      }
    } else if (msg->id == bms::DC_DETAILED_STATUS) {
      // The "detailed status" message comes in two halves; that is, there are two different
      // possible message contents in messages with the same ID. It's easiest to identify the
      // second message by the three separate 0xFF bytes so we catch that first.
      if (msg->data[2] == 0xFF && msg->data[6] == 0xFF && msg->data[7] == 0xFF)
      {
        // We do not currently extract any information from this message, but its presence
        // is part of the specification and its absence could indicate something wrong. We
        // therefore track when we last received it.
      } else if (msg->data[1] == 0x0B) {
        // Now we handle the first "half".
        state_of_charge_ = msg->data[5];

        RCLCPP_INFO_ONCE(this->get_logger(),
            "Battery state of charge: %d ", msg->data[5]);
      }
    } else if (
      msg->id != VREGS && msg->id != POWERFINN_CONTROL &&
      msg->id != UNKNOWN_1 && msg->id != UNKNOWN_2 && msg->id != UNKNOWN_3) {
      // VREGS and POWERFINN_CONTROL are CAN messages we expect to find on the same CAN bus but
      // which are not relevant for our application. If we spot any other messages, we throw a
      // warning to the terminal. Note that there's no reason to think this would impact the
      // performance of this node - it could simply be an indication of some change in hardware
      // configuration that might be worth knowing about.
      auto clk = rclcpp::Clock();
      RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), clk, 10,
        "Unexpected CAN frame in bagging area. ID = " << msg->id);
    }
  }

  int16_t twoBytesToInt16(const uint8_t firstbyte, const uint8_t secondbyte) const
  {
    return (static_cast<int16_t>(secondbyte) << 8) | firstbyte;
  }

  uint16_t twoBytesToUInt16(const uint8_t firstbyte, const uint8_t secondbyte) const
  {
    return (static_cast<uint16_t>(secondbyte) << 8) | firstbyte;
  }

  /// Topic strings
  std::string input_topic_, output_topic_;

  /// Battery member variables
  double voltage_, current_, highest_temperature_celsius_, state_of_charge_;

  /// Subscription to CANBUS BMS msg
  rclcpp::Subscription<Frame>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  /// Publisher for ROS2 Battery State msg
  rclcpp::Publisher<BatteryState>::SharedPtr publisher_;
};
}  // namespace bms

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bms::BMS_node)

#endif  // AV_BMS__BMS_NODE_HPP_
