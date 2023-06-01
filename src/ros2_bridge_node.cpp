#include "ros2_bridge_node.h"
#include "canopen_slave_node.h"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_configure(const rclcpp_lifecycle::State &) {
  active.store(false);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_activate(const rclcpp_lifecycle::State &) {
  get_parameter("canopen_node_id", canopen_node_id_);
  RCLCPP_INFO(this->get_logger(), "canopen_node_id: %i", canopen_node_id_);

  get_parameter("canopen_node_config", canopen_node_config_);
  RCLCPP_INFO(this->get_logger(), "canopen_node_config: %s", canopen_node_config_.c_str());

  get_parameter("can_interface_name", can_interface_name_);
  RCLCPP_INFO(this->get_logger(), "can_interface_name: %s", can_interface_name_.c_str());

  active.store(true);
  int_pub->on_activate();
  t = std::thread(std::bind(&ROS2BridgeNode::run_canopen_slave_node, this));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_deactivate(const rclcpp_lifecycle::State &) {
  active.store(false);
  t.join();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_cleanup(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_shutdown(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ROS2BridgeNode::topic_callback(const std_msgs::msg::UInt32::SharedPtr msg) const {
  if (canopen_slave_node != nullptr) {
    canopen_slave_node->send_TPDO(msg->data);
  }
}

void ROS2BridgeNode::run_canopen_slave_node()	{
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(can_interface_name_.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  canopen_slave_node = std::make_shared<CANOpenSlaveNode>(timer, chan, canopen_node_config_, "", canopen_node_id_, this);
  canopen_slave_node->Reset();

  while(active.load()) {
    loop.run_one_for(10ms);
  }
  ctx.shutdown();
}

void ROS2BridgeNode::RPDO_callback(uint32_t value) {
  std_msgs::msg::UInt32 msg;
  msg.data = value;
  int_pub->publish(msg);

  RCLCPP_INFO(this->get_logger(), "RPDO_callback: 0x%X", value);
}
