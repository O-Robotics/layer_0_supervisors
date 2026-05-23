#pragma once

#include "_supervisor/state_node_base.hpp"

namespace fsm_layer_0::states::idling {

class IdlingNode : public fsm_layer_0::StateNodeBase
{
public:
  explicit IdlingNode(const rclcpp::NodeOptions & options)
  : fsm_layer_0::StateNodeBase("idling_state", options) {}
};

}  // namespace fsm_layer_0::states::idling
