#pragma once

#include "amr_sweeper_layer_0_fsm/srv/request_state.hpp"

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "_supervisor/process_manager.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace fsm_layer_0
{

// Importance level for an external process defined by a profile.
enum class ProcessImportance { CRITICAL, DEGRADED, OPTIONAL };

struct ProfileProcess
{
  std::string name;
  std::string command;
  ProcessImportance importance{ProcessImportance::CRITICAL};
  int window_ms{0};

  // Restart policy (0 disables restarts)
  int max_restarts{0};
  int restart_delay_ms{0};

  std::vector<std::string> ready_topics;
  std::vector<std::string> ready_services;


  // Optional per-process error policy (declared under `errors:` in the profile YAML).
  // Empty action strings mean "use default/global behavior".
  struct ErrorPolicy
  {
    std::string on_readiness_fail;     // e.g., "FAULT"
    std::string on_unexpected_exit;    // e.g., "FAULT"
    int priority{-1};                  // -1 => use state faults.fault_priority
    bool force{false};
    bool has_force{false};             // distinguish false vs missing
    std::string lifecycle_target;      // "" => use state faults.fault_lifecycle_target
    int target_profile_id{-1};         // -1 => infer (FAULT->fault_transition_profile) or 0
  } errors;

  // Optional per-process shutdown policy (declared under `shutdown:` in the profile YAML).
  // Values <=0 mean "use defaults" (2s/2s/0.5s).
  struct ShutdownPolicy
  {
    int sigint_timeout_ms{0};
    int sigterm_timeout_ms{0};
    int sigkill_timeout_ms{0};
  } shutdown;
};

/**
 * @brief Base class for all FSM state nodes (ROS 2 LifecycleNode).
 *
 * This base class intentionally stays "thin" and focuses on cross-cutting concerns
 * that are common to *all* states:
 *
 * 1) **External process management**
 *    - Each state can define a list of bring-up commands (processes / launch files)
 *      via the parameter `processes` (typically loaded from state_parameters.yaml).
 *    - On activation: all commands are started (best-effort).
 *    - On deactivation/cleanup/shutdown/error: all commands are stopped (best-effort).
 *
 * 2) **ROSOUT monitoring + fault trigger rules**
 *    - Each state can define a set of rules in `faults.rosout_triggers`.
 *    - When a rule matches an incoming `/rosout` message, the base class calls
 *      `handle_fault_action(reason)`. Derived states decide how to act (e.g. request
 *      transition to FAULT via the supervisor service).
 *
 * Note:
 * - Legacy battery lifecycle management and localization sanity-check logic have been removed.
 *   The FSM layer should no longer own or manipulate those subsystems.
 */
class StateNodeBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit StateNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  // ----- Lifecycle callbacks (shared wiring for all states) -----

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Hook invoked when a ROSOUT trigger rule requests a fault.
   *
   * The base implementation only logs the reason. Derived classes should override
   * and request a FAULT transition (usually through the supervisor) if that is the
   * desired behavior.
   */
  virtual void handle_fault_action(const std::string & reason);

  // ----- Profile process error policies -----

  // If a profile process defines an `errors` policy, it can request a specific FSM transition
  // when readiness fails or the process exits unexpectedly.
  void handle_profile_error_policy_(
    const ProfileProcess & pp,
    const std::string & event,
    const std::string & reason);

  const ProfileProcess * find_profile_process_by_name_or_command_(const std::string & key) const;

  // ----- Bring-up readiness gating -----

  /**
   * @brief Ready requirements for this state.
   *
   * Read from parameters:
   *   - ready.nodes: ["foo_node", ...]              (graph-discovered nodes)
   *   - ready.lifecycle_nodes: ["/ns/bar", ...]     (must expose <node>/get_state)
   *   - ready.timeout_ms: 5000                      (0 disables gating)
   */
  struct ReadySpec
  {
    std::vector<std::string> nodes;
    struct LifecycleNodeRequirement
    {
      // Node name (absolute or relative; relative names are qualified into this node's namespace).
      std::string node;

      // Minimum acceptable lifecycle primary state id.
      // Examples:
      //   1 = UNCONFIGURED, 2 = INACTIVE, 3 = ACTIVE
      uint8_t min_state_id{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE};

      // Raw rule string (useful for logs/debug).
      std::string raw;
    };

    // Lifecycle readiness requirements parsed from the string array parameter `ready.lifecycle_nodes`.
    // Backwards compatible:
    //   - "/amr_sweeper/amr_sweeper_battery"  -> requires ACTIVE
    // New syntax:
    //   - "node=amr_sweeper_battery_node;level>=UNCONFIGURED"
    //   - "node=/amr_sweeper/amr_sweeper_battery;level>=INACTIVE"
    std::vector<LifecycleNodeRequirement> lifecycle_nodes;

    // Graph-discovered topics (names must match exactly; absolute or relative).
    std::vector<std::string> topics;

    // Graph-discovered services (names must match exactly; absolute or relative).
    std::vector<std::string> services;

    int timeout_ms{0};
  };

  /// Load (or declare) readiness parameters into readiness_.
  void load_readiness_spec();

  /// Wait until readiness requirements are met. Returns false with a reason on timeout.
  bool wait_for_readiness(std::string & why_not);

  // Helper: true if node exists in ROS graph (suffix-match allowed for convenience).
  bool graph_has_node(const std::string & node_name);

  // Helper: true if topic exists in ROS graph.
  bool graph_has_topic(const std::string & topic_name);

  // Helper: true if service exists in ROS graph.
  bool graph_has_service(const std::string & service_name);

  // Helper: true if lifecycle node state is >= required minimum.
  bool lifecycle_node_meets_requirement(
    const ReadySpec::LifecycleNodeRequirement & req,
    std::string & why_not);

  // Parse a lifecycle readiness rule line.
  static ReadySpec::LifecycleNodeRequirement parse_lifecycle_requirement_line(const std::string & line);

  // Map lifecycle level strings to primary state ids.
  static uint8_t parse_lifecycle_level(const std::string & s);

  static bool ends_with(const std::string & str, const std::string & suf);
  std::string qualify_to_ns(const std::string & maybe_relative) const;

  // ----- ROSOUT monitoring -----

  void on_rosout(const rcl_interfaces::msg::Log::SharedPtr msg);
  
  void reset_rosout_latches();
  
  // What to do when a rosout line matches a trigger.
  // NOTE: "DEGRADED" maps to requesting the system "DEGRADED" state.
  enum class RosoutAction { FAULT, DEGRADED, WARN_ONLY };

  struct RosoutTrigger
  {
    // Optional node name filter. If provided, we allow suffix-match to tolerate namespaces.
    bool has_node{false};
    std::string node;

    // Minimum log level required for the rule to match (e.g., ERROR=40).
    int min_level{40};

    // Optional substring match applied to msg->msg.
    bool has_match{false};
    std::string match;

    // Action to take when rule matches.
    RosoutAction action{RosoutAction::WARN_ONLY};

    // Optional: importance of the process that declared this trigger (if loaded from profile YAML).
    bool has_source_importance{false};
    ProcessImportance source_importance{ProcessImportance::CRITICAL};

    // Original raw rule string (useful in logs/debug).
    std::string raw;

  };

  // ----- Helpers -----

  /**
   * @brief Resolve placeholders inside a command string.
   *
   * Currently supports:
   *   - `{ns}` -> node namespace without leading '/' (empty string for root namespace).
   */
  std::string resolve_placeholders(std::string cmd) const;

  /// Start all processes specified in processes_ (best-effort).
  void start_state_processes();

  /// Stop all processes specified in processes_ (best-effort).
  void stop_state_processes();

// ----- Profile-defined FSM transitions -----

// Derive a RequestState target_state from a profile id band.
// Returns empty string if the profile does not map to a known FSM-state.
static std::string target_state_from_profile_id(uint16_t profile_id);


private:
  std::vector<std::string> read_string_array_param_(const std::string & name) const;
  int64_t read_int64_param_(const std::string & name, int64_t default_value) const;
  // ----- Rosout trigger parsing helpers -----

  static int parse_level(const std::string & s);

  /// Parse a single trigger line: "node=x;match=y;level>=ERROR;action=FAULT"
  static RosoutTrigger parse_trigger_line(const std::string & line);

  // ----- Per-process monitoring / restart -----
  void start_process_monitoring_();
  void stop_process_monitoring_();
  void on_process_monitor_tick_();

  // ----- Members -----

  std::vector<RosoutTrigger> rosout_triggers_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  bool fault_requested_{false};

  rclcpp::Client<amr_sweeper_layer_0_fsm::srv::RequestState>::SharedPtr request_state_client_;

  std::string request_state_service_{"request_state"};
  uint8_t fault_priority_{250};
  bool fault_force_{true};
  std::string fault_lifecycle_target_{};

  // Per-state profile configuration.
  std::string profiles_file_{};
  uint16_t default_profile_id_{0};
  uint16_t active_profile_id_{0};

// Profile-defined transition behavior.
bool auto_transition_on_{false};
uint16_t auto_transition_profile_{100};
bool fault_transition_on_{true};
uint16_t fault_transition_profile_{400};


  // State-specific bring-up commands (processes/launch files).
  std::vector<std::string> processes_;

  // Profile-defined process metadata (importance + per-process readiness window).
  std::vector<ProfileProcess> profile_processes_;

  // Track which non-critical process readiness warnings have already been logged.
  std::set<std::string> warned_noncritical_readiness_;

  // State-specific readiness spec.
  ReadySpec readiness_;

  // Process manager used to start/stop external commands.
  ProcessManager procman_;


  struct MonitoredProcess
  {
    ProfileProcess spec;
    int restarts_done{0};
    bool optional_exhausted_logged{false};

    // Next time a restart attempt is allowed (wall time).
    rclcpp::Time next_restart_time;
    bool has_next_restart_time{false};
  };

  rclcpp::TimerBase::SharedPtr proc_monitor_timer_;
  std::vector<MonitoredProcess> monitored_processes_;

};

}  // namespace fsm_layer_0
