#include "4_fault/fault_state.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<fsm_layer_0::states::fault::FaultNode>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  node->stop_managed_processes_for_exit();
  rclcpp::shutdown();
  return 0;
}
