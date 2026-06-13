#pragma once

#include "_supervisor/state_node_base.hpp"

namespace fsm_layer_0::states::running {

class RunningNode : public fsm_layer_0::StateNodeBase
{
public:
  explicit RunningNode(const rclcpp::NodeOptions & options)
  : fsm_layer_0::StateNodeBase("RUNNING_node", options) {}
};

}  // namespace fsm_layer_0::states::running
