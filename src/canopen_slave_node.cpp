#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

void CANOpenSlaveNode::send_TPDO(uint32_t data) {
  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "send_TPDO: writing 0x%X to 0x4001:0", data);
  (*this)[0x4001][0] = data;
  this->TpdoEvent(0);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {
  uint32_t val = (*this)[idx][subidx];
  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "OnWrite: received 0x%X on idx: 0x%X:%X", val, idx, subidx);
  if (idx == 0x4000 && subidx == 0) {
    ros2_bridge_node_->RPDO_callback(val);
  }
}
