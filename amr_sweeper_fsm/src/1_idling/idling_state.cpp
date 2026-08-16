#include "1_idling/idling_state.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<fsm_layer_0::states::idling::IdlingNode>(options);
  rclcpp::on_shutdown(
    [weak_node = std::weak_ptr<fsm_layer_0::states::idling::IdlingNode>(node)]() {
      if (const auto locked = weak_node.lock()) {
        locked->stop_managed_processes_for_exit();
      }
    },
    node->get_node_base_interface()->get_context());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  node->stop_managed_processes_for_exit();
  rclcpp::shutdown();
  return 0;
}
