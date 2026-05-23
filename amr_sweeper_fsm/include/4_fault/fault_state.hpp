#pragma once

#include "_supervisor/state_node_base.hpp"

namespace fsm_layer_0::states::fault {

class FaultNode : public fsm_layer_0::StateNodeBase
{
public:
  explicit FaultNode(const rclcpp::NodeOptions & options)
  : fsm_layer_0::StateNodeBase("fault_state", options) {}
};

}  // namespace fsm_layer_0::states::fault
