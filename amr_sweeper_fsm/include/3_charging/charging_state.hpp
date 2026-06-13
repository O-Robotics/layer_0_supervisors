#pragma once

#include "_supervisor/state_node_base.hpp"

namespace fsm_layer_0::states::charging {

class ChargingNode : public fsm_layer_0::StateNodeBase
{
public:
  explicit ChargingNode(const rclcpp::NodeOptions & options)
  : fsm_layer_0::StateNodeBase("CHARGING_node", options) {}
};

}  // namespace fsm_layer_0::states::charging
