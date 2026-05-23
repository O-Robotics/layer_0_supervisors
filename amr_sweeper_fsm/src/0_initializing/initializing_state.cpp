#include "0_initializing/initializing_state.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fsm_layer_0::states::initializing
{

InitializingNode::InitializingNode(const rclcpp::NodeOptions & options)
: fsm_layer_0::StateNodeBase("initializing_state", options)
{
  // No additional construction-time work.
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializingNode::on_configure(const rclcpp_lifecycle::State & state)
{
  // Delegate parameter handling and setup to the common base:
  // - Reads `bringup.commands` (processes to start when ACTIVE)
  // - Reads `faults.rosout_triggers` (fault detection rules)
  // - Sets up any common publishers/subscriptions owned by StateNodeBase
  const auto base_ret = fsm_layer_0::StateNodeBase::on_configure(state);
  if (base_ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return base_ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializingNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // Start any bringup processes declared in parameters (handled by StateNodeBase).
  return fsm_layer_0::StateNodeBase::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializingNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // No state-specific resources to release; delegate to base class.
  return fsm_layer_0::StateNodeBase::on_deactivate(state);
}

}  // namespace fsm_layer_0::states::initializing


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<fsm_layer_0::states::initializing::InitializingNode>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}