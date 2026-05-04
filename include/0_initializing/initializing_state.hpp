#pragma once

#include "_supervisor/state_node_base.hpp"

#include <cstdint>
#include <string>

#include "amr_sweeper_layer_0_fsm/srv/request_state.hpp"

namespace fsm_layer_0::states::initializing
{

/**
 * \brief INITIALIZING state node.
 *
 * This node represents the "INITIALIZING" state in the FSM layer.
 *
 * After this cleanup, the node intentionally performs no diagnostic/system
 * validation. Any such checks should live in dedicated components (e.g.
 * diagnostics aggregators, health monitors) and feed failures into the FSM
 * via faults/rosout triggers or explicit state requests.
 *
 * Runtime behavior:
 *  - When configured/activated, StateNodeBase may start processes listed in the
 *    parameter `bringup.commands` (see StateNodeBase docs).
 *  - Upon activation success, this node immediately requests a transition to
 *    IDLING (state id 1).
 */
class InitializingNode : public fsm_layer_0::StateNodeBase
{
public:
  /// Create the INITIALIZING state node.
  explicit InitializingNode(const rclcpp::NodeOptions & options);

protected:
  /// Lifecycle: declare/read parameters and create the supervisor service client.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /// Lifecycle: request transition to IDLING immediately after activation.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  /// Lifecycle: currently no additional teardown beyond StateNodeBase.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * \brief Request a supervisor state transition.
   *
   * The supervisor exposes a `request_state` service in the node's namespace.
   * This helper wraps the call and logs success/failure.
   */
  bool request_state(int8_t target_state, const std::string & reason);

  // ---- Parameters ----
  /// Priority used when requesting state transitions (higher wins).
  uint8_t request_priority_{250};

  /// Timeout for waiting for the request_state service (seconds).
  double request_service_timeout_sec_{5.0};

  /// Timeout for waiting for the request_state response (seconds).
  double request_response_timeout_sec_{5.0};

  // ---- Runtime ----
  rclcpp::Client<amr_sweeper_layer_0_fsm::srv::RequestState>::SharedPtr request_state_client_;
  bool transition_requested_{false};
};

}  // namespace fsm_layer_0::states::initializing
