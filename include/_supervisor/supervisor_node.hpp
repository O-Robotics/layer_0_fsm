#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amr_sweeper_layer_0_fsm/msg/fsm_state.hpp"
#include "amr_sweeper_layer_0_fsm/msg/fsm_status.hpp"
#include "amr_sweeper_layer_0_fsm/srv/request_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fsm_layer_0
{

/**
 * @brief High-level FSM supervisor.
 *
 * The supervisor is a regular rclcpp::Node that orchestrates per-state
 * LifecycleNodes via lifecycle service calls (ChangeState/GetState). It also
 * exposes a RequestState service that external components can use to request
 * a new high-level state.
 *
 * Publishing is intentionally decoupled from the internal tick rate:
 *  - tick_timer_ drives lifecycle switching and polling responsiveness
 *  - publish_rules_ define independent timers for periodic publications
 *
 * Parameters (relevant here):
 *  - service_wait_ms: max wait for lifecycle services to become ready
 *  - op_timeout_ms: operation timeout (enter/switch) in milliseconds
 *  - publish.rules: list of rule strings, e.g.
 *      "topic=fsm_state;type=amr_sweeper_layer_0_fsm/msg/FSMState;period_ms=1000;source=state"
 *      "topic=fsm_status;type=amr_sweeper_layer_0_fsm/msg/FSMStatus;period_ms=1000;source=status"
 */
class SupervisorNode : public rclcpp::Node
{
public:
  SupervisorNode();

private:
  // ===========================================================================
  // FSM state model
  // ===========================================================================

  enum FSMState : int8_t { INITIALIZING = 0, IDLING = 1, RUNNING = 2, CHARGING = 3, FAULT = 4 };

  static std::string state_name(FSMState s);

  // ===========================================================================
  // Lifecycle orchestration
  // ===========================================================================

  struct StateEndpoints
  {
    std::string node_name;  // lifecycle node name
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state;
  };

  /**
   * @brief Fine-grained lifecycle transition phases used by the non-blocking
   * operation engine.
   *
   * The supervisor performs lifecycle transitions asynchronously. Each phase
   * indicates which lifecycle request is *next*.
   */
public:

  enum class OpPhase
  {
    IDLE,
    CUR_GET,
    CUR_DEACTIVATE,
    CUR_CLEANUP,
    TGT_CONFIGURE,
    TGT_ACTIVATE
  };

private:

  // Async lifecycle helpers. They schedule one request and invoke the callback
  // when a response arrives (or immediately with ok=false if not ready).
  void request_get_state(
    StateEndpoints & ep,
    std::function<void(bool ok, uint8_t lifecycle_id, const std::string & err)> cb);

  void request_change_state(
    StateEndpoints & ep,
    uint8_t transition_id,
    std::function<void(bool ok, const std::string & err)> cb);

  void init_clients();

  // Engine step: decide operations, drive one transition, and poll lifecycle label.
  void tick();

  // Drives the current operation (enter/switch) by issuing one lifecycle request.
  void drive();

  void start_enter_state(FSMState s);
  void start_switch_to(FSMState s);

  // ===========================================================================
  // Request handling
  // ===========================================================================

  void on_request_state(
    const std::shared_ptr<amr_sweeper_layer_0_fsm::srv::RequestState::Request> req,
    std::shared_ptr<amr_sweeper_layer_0_fsm::srv::RequestState::Response> resp);

  // ===========================================================================
  // Status publishing (independent of tick rate)
  // ===========================================================================

  struct StatusSnapshot
  {
    FSMState current_state{INITIALIZING};
    std::string active_lifecycle_label{"unknown"};
    uint16_t current_profile{0};
    uint16_t transitioning_to_profile{0};
    std::string transition_status{"STABLE"};
    uint8_t last_priority{0};
    rclcpp::Time last_priority_time;
    std::string last_requester;
    std::string last_message;
  };

  enum class PublishMsgType { FSM_STATE, FSM_STATUS };

  struct PublishRule
  {
    std::string raw;

    std::string topic;
    std::string type;    // "amr_sweeper_layer_0_fsm/msg/FSMState" or "amr_sweeper_layer_0_fsm/msg/FSMStatus"
    std::string source;  // "state" or "status"
    uint32_t period_ms{1000};  // publish period in milliseconds

    PublishMsgType msg_type{PublishMsgType::FSM_STATE};

    rclcpp::Publisher<amr_sweeper_layer_0_fsm::msg::FSMState>::SharedPtr state_pub;
    rclcpp::Publisher<amr_sweeper_layer_0_fsm::msg::FSMStatus>::SharedPtr status_pub;

    rclcpp::TimerBase::SharedPtr timer;
  };

  void init_publish_rules();
  bool parse_publish_rule(const std::string & rule_str, PublishRule & out, std::string & error) const;

  StatusSnapshot snapshot_status() const;

  void publish_from_rule(const PublishRule & rule, const StatusSnapshot & snap);
  amr_sweeper_layer_0_fsm::msg::FSMState build_state_payload(const StatusSnapshot & snap) const;
  amr_sweeper_layer_0_fsm::msg::FSMStatus build_status_payload(const StatusSnapshot & snap) const;

    // Priority "aging" helper (same policy as the original code: 1 step per second).
  uint8_t effective_last_priority() const;

  // ===========================================================================
  // Data
  // ===========================================================================

  mutable std::mutex mtx_;

  // Namespace of this node (includes leading '/'). Used to build fully-qualified
  // lifecycle service names for managed state nodes.
  std::string ns_{"/"};


  // Current and desired state.
  FSMState current_state_{INITIALIZING};
  FSMState desired_state_{INITIALIZING};

  // Current and desired profile id. Used for status publication and request bookkeeping.
  uint16_t current_profile_{0};
  uint16_t desired_profile_{0};

  // Desired lifecycle target for the *next* requested transition.
  // True  => enter/leave the target node ACTIVE (configure + activate).
  // False => stop after CONFIGURE, leaving the target node INACTIVE.
  bool desired_activate_{true};

  // Snapshots captured when an enter/switch operation begins.
  // These prevent in-flight RequestState updates from corrupting operation completion bookkeeping/logging.
  uint16_t op_target_profile_{0};
  bool op_target_activate_{true};


  // Request bookkeeping.
  uint8_t last_priority_{0};
  rclcpp::Time last_priority_time_;
  bool last_force_{false};
  std::string last_requester_{"Supervisor"};
  std::string last_message_;
  std::string last_reason_;
  std::string active_lifecycle_label_{"unknown"};

  // Transition reporting for FSMStatus publication.
  // transitioning_to_profile_ holds the profile id of the most recently accepted RequestState.
  // transition_status_ is a coarse label: STABLE / TRANSITIONING / FAILED.
  uint16_t transitioning_to_profile_{0};
  std::string transition_status_{"STABLE"};

  // Transition engine state.
  OpPhase op_phase_{OpPhase::IDLE};
  FSMState op_target_{INITIALIZING};
  bool op_inflight_{false};
  bool poll_inflight_{false};
  bool need_enter_current_{true};
  bool restart_requested_{false};

    // Dedicated node used for synchronous parameter service calls.
    rclcpp::Node::SharedPtr param_client_node_;

  uint8_t last_lifecycle_id_{0};

  // Per-state lifecycle service endpoints.
  std::map<FSMState, StateEndpoints> endpoints_;

  // Publishing rules (configured via publish.rules).
  std::vector<PublishRule> publish_rules_;

  // ROS interfaces.
  rclcpp::Service<amr_sweeper_layer_0_fsm::srv::RequestState>::SharedPtr request_srv_;

  // Engine tick timer.
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Parameters.
  int service_wait_ms_{500};
  int op_timeout_ms_{1500};
  int tick_period_ms_{100};
};

}  // namespace fsm_layer_0
