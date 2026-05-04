#include "_supervisor/supervisor_node.hpp"
#include <cmath>

#include <chrono>
#include <memory>
#include <algorithm>
#include <cctype>
#include <vector>

#include <mutex>
#include <unordered_map>

#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include "rclcpp/parameter_client.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace fsm_layer_0
{

static int8_t profile_to_state_id(uint16_t profile)
{
  // Profiles are grouped in bands:
  //   INITIALIZING: 000-099, IDLING: 100-199, RUNNING: 200-299,
  //   CHARGING: 300-399, FAULT: 400-499
  if (profile >= 400 && profile <= 499) return 4;  // FAULT
  if (profile >= 300 && profile <= 399) return 3;  // CHARGING
  if (profile >= 200 && profile <= 299) return 2;  // RUNNING
  if (profile >= 100 && profile <= 199) return 1;  // IDLING
  return 0;  // INITIALIZING
}

static std::string lifecycle_id_to_label(uint8_t id)
{
  switch (id) {
    case State::PRIMARY_STATE_UNKNOWN:
      return "Unknown";
    case State::PRIMARY_STATE_UNCONFIGURED:
      return "Unconfigured";
    case State::PRIMARY_STATE_INACTIVE:
      return "Inactive";
    case State::PRIMARY_STATE_ACTIVE:
      return "Active";
    case State::PRIMARY_STATE_FINALIZED:
      return "Finalized";
    default:
      break;
  }

  // Transition states (use ROS 2 LifecycleNode callback naming)
  switch (id) {
    case State::TRANSITION_STATE_CONFIGURING:
      return "onConfigure()";
    case State::TRANSITION_STATE_CLEANINGUP:
      return "onCleanup()";
    case State::TRANSITION_STATE_ACTIVATING:
      return "onActivate()";
    case State::TRANSITION_STATE_DEACTIVATING:
      return "onDeactivate()";
    case State::TRANSITION_STATE_SHUTTINGDOWN:
      return "onShutdown()";
    case State::TRANSITION_STATE_ERRORPROCESSING:
      return "onError()";
    default:
      return "Unknown";
  }
}

static std::string profiles_yaml_path_for_state_id(int8_t state_id, std::string & error)
{
  error.clear();

  std::string filename;
  switch (state_id) {
    case 0:
      filename = "initializing_profiles.yaml";
      break;
    case 1:
      filename = "idling_profiles.yaml";
      break;
    case 2:
      filename = "running_profiles.yaml";
      break;
    case 3:
      filename = "charging_profiles.yaml";
      break;
    case 4:
      filename = "fault_profiles.yaml";
      break;
    default:
      error = "unknown state_id=" + std::to_string(state_id);
      return "";
  }

  try {
    const auto share = ament_index_cpp::get_package_share_directory("amr_sweeper_layer_0_fsm");
    return share + "/config/profiles/" + filename;
  } catch (const std::exception & e) {
    error = std::string("get_package_share_directory(amr_sweeper_layer_0_fsm) failed: ") + e.what();
    return "";
  }
}

static bool load_profile_ids_from_yaml(const std::string & path, std::vector<uint16_t> & out, std::string & error)
{
  out.clear();
  error.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    error = std::string("failed to load YAML: ") + e.what();
    return false;
  }

  const auto profiles = root["profiles"];
  if (!profiles || !profiles.IsSequence()) {
    error = "YAML missing 'profiles' sequence";
    return false;
  }

  for (std::size_t i = 0; i < profiles.size(); ++i) {
    const auto entry = profiles[i];
    const auto prof = entry["profile"];
    if (!prof || !prof.IsMap()) {
      continue;
    }
    const auto id_node = prof["id"];
    if (!id_node) {
      continue;
    }
    try {
      const int id = id_node.as<int>();
      if (id >= 0 && id <= 65535) {
        out.push_back(static_cast<uint16_t>(id));
      }
    } catch (...) {
      // Skip malformed ids without failing the whole file.
      continue;
    }
  }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return true;
}

static bool profile_exists_in_state_yaml(int8_t state_id, uint16_t profile_id, std::string & reason)
{
  static std::mutex g_mtx;
  static std::unordered_map<int8_t, std::vector<uint16_t>> g_state_profiles;
  static std::unordered_map<int8_t, std::string> g_state_load_error;

  std::lock_guard<std::mutex> lk(g_mtx);

  auto it = g_state_profiles.find(state_id);
  if (it == g_state_profiles.end() && g_state_load_error.find(state_id) == g_state_load_error.end()) {
    std::string path_err;
    const auto path = profiles_yaml_path_for_state_id(state_id, path_err);
    if (!path_err.empty()) {
      g_state_load_error[state_id] = path_err;
    } else {
      std::vector<uint16_t> ids;
      std::string load_err;
      if (!load_profile_ids_from_yaml(path, ids, load_err)) {
        g_state_load_error[state_id] = "cannot read '" + path + "': " + load_err;
      } else {
        g_state_profiles[state_id] = std::move(ids);
      }
    }
    it = g_state_profiles.find(state_id);
  }

  const auto err_it = g_state_load_error.find(state_id);
  if (err_it != g_state_load_error.end()) {
    reason = "Rejected: could not validate profiles for state_id=" + std::to_string(state_id) + ": " + err_it->second;
    return false;
  }

  const auto & ids = it->second;
  const auto found = std::binary_search(ids.begin(), ids.end(), profile_id);
  if (found) {
    return true;
  }

  std::string avail;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) avail += ", ";
    avail += std::to_string(ids[i]);
  }
  if (avail.empty()) {
    avail = "(none)";
  }

  reason = "Rejected: profile " + std::to_string(profile_id) + " not found. Available profiles: " + avail;
  return false;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------



static void log_packages_with_prefix(const rclcpp::Logger & logger, const std::string & prefix)
{
  const auto pkgs = ament_index_cpp::get_packages_with_prefixes();  // map<pkg, install_prefix>

  std::vector<std::string> matches;
  matches.reserve(pkgs.size());

  for (const auto & kv : pkgs) {
    const auto & name = kv.first;
    if (name.rfind(prefix, 0) == 0) {  // starts_with(prefix)
      matches.push_back(name);
    }
  }

  std::sort(matches.begin(), matches.end());

  RCLCPP_INFO(logger, "Detected %zu '%s' packages :", matches.size(), prefix.c_str());
  for (const auto & pkg : matches) {
    RCLCPP_INFO(logger, "  - %s", pkg.c_str());
  }
}

SupervisorNode::SupervisorNode()
: rclcpp::Node("supervisor")
{
  ns_ = this->get_namespace();  // includes '/'

  // Dedicated helper node for synchronous parameter service calls.
  // IMPORTANT: do NOT add this node to an executor when using SyncParametersClient.
  // This process is launched with global remaps (e.g. "-r __node:=supervisor").
  // If we let those global arguments apply here, this helper node will be remapped
  // to the *same* name as the supervisor node, causing duplicate node names and
  // rosout publisher warnings. Disable global arguments for the helper node.
  rclcpp::NodeOptions helper_opts;
  helper_opts.use_global_arguments(false);
  param_client_node_ = std::make_shared<rclcpp::Node>("supervisor_param_client", ns_, helper_opts);

  last_priority_time_ = this->now();  // NEW: initialize decay reference time

  // Kept for parameter compatibility (non-blocking implementation does not wait on futures).
  service_wait_ms_ = this->declare_parameter<int>("service_wait_ms", service_wait_ms_);
  op_timeout_ms_ = this->declare_parameter<int>("op_timeout_ms", op_timeout_ms_);
  tick_period_ms_ = this->declare_parameter<int>("tick_period_ms", tick_period_ms_);

  // Startup profile selection.
  // NOTE: Launch may provide this parameter as a startup override.
  desired_profile_ = static_cast<uint16_t>(this->declare_parameter<int>("desired_profile", 1));

  // Derive the initial FSM state from the selected startup profile.
  current_state_ = static_cast<FSMState>(profile_to_state_id(desired_profile_));
  desired_state_ = current_state_;
  // Track the currently running/attempted profile from the beginning so that
  // any early error escalation logs report the correct state/profile context.
  current_profile_ = desired_profile_;
  transitioning_to_profile_ = current_profile_;

  log_packages_with_prefix(this->get_logger(), "amr_sweeper");

  // Configure decoupled publishing timers from parameters.
  init_publish_rules();

  request_srv_ = this->create_service<amr_sweeper_layer_0_fsm::srv::RequestState>(
    "request_state",
    std::bind(&SupervisorNode::on_request_state, this, std::placeholders::_1, std::placeholders::_2));

  init_clients();

  // On startup we "enter" the derived initial state (configure + activate) asynchronously.
  need_enter_current_ = true;
  desired_state_ = current_state_;
  last_message_ = "Startup";

  // Drive engine + poll label
  tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_period_ms_), std::bind(&SupervisorNode::tick, this));
}

void SupervisorNode::init_clients()
{
  endpoints_.clear();

  endpoints_[INITIALIZING].node_name = "initializing_state";
  endpoints_[IDLING].node_name = "idling_state";
  endpoints_[RUNNING].node_name = "running_state";
  endpoints_[CHARGING].node_name = "charging_state";
  endpoints_[FAULT].node_name = "fault_state";

  for (auto & kv : endpoints_) {
    auto & ep = kv.second;
    const std::string fq = ns_ + "/" + ep.node_name;
    ep.change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(fq + "/change_state");
    ep.get_state = this->create_client<lifecycle_msgs::srv::GetState>(fq + "/get_state");
  }
}

std::string SupervisorNode::state_name(FSMState s)
{
  switch (s) {
    case INITIALIZING:
      return "INITIALIZING";
    case IDLING:
      return "IDLING";
    case RUNNING:
      return "RUNNING";
    case CHARGING:
      return "CHARGING";
    case FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

void SupervisorNode::request_get_state(
  StateEndpoints & ep,
  std::function<void(bool ok, uint8_t lifecycle_id, const std::string & err)> cb)
{
  if (!ep.get_state) {
    cb(false, State::PRIMARY_STATE_UNKNOWN, "get_state client not created");
    return;
  }
  if (!ep.get_state->service_is_ready()) {
    cb(false, State::PRIMARY_STATE_UNKNOWN, "get_state service not available for " + ep.node_name);
    return;
  }

  auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  (void)ep.get_state->async_send_request(
    req,
    [cb](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        cb(true, resp->current_state.id, "");
      } catch (const std::exception & e) {
        cb(false, State::PRIMARY_STATE_UNKNOWN, std::string("get_state exception: ") + e.what());
      }
    });
}

void SupervisorNode::request_change_state(
  StateEndpoints & ep,
  uint8_t transition_id,
  std::function<void(bool ok, const std::string & err)> cb)
{
  if (!ep.change_state) {
    cb(false, "change_state client not created");
    return;
  }
  if (!ep.change_state->service_is_ready()) {
    cb(false, "change_state service not available for " + ep.node_name);
    return;
  }

  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = transition_id;

  (void)ep.change_state->async_send_request(
    req,
    [cb, transition_id, node_name = ep.node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        if (!resp->success) {
          cb(false, "change_state returned success=false for " + node_name +
                      " (transition=" + std::to_string(transition_id) + ")");
          return;
        }
        cb(true, "");
      } catch (const std::exception & e) {
        cb(false, std::string("change_state exception for ") + node_name + ": " + e.what());
      }
    });
}

void SupervisorNode::start_enter_state(FSMState target)
{
  // Enter = CONFIGURE + ACTIVATE target
  op_target_ = target;
  op_target_profile_ = desired_profile_;
  op_target_activate_ = true;
  desired_activate_ = true;

  // Clamp to valid profile band for target state
  switch (op_target_) {
    case FSMState::INITIALIZING:
      if (op_target_profile_ > 99) { op_target_profile_ = 0; }
      break;
    case FSMState::IDLING:
      if (op_target_profile_ < 100 || op_target_profile_ > 199) { op_target_profile_ = 100; }
      break;
    case FSMState::RUNNING:
      if (op_target_profile_ < 200 || op_target_profile_ > 299) { op_target_profile_ = 200; }
      break;
    case FSMState::CHARGING:
      if (op_target_profile_ < 300 || op_target_profile_ > 399) { op_target_profile_ = 300; }
      break;
    case FSMState::FAULT:
      if (op_target_profile_ < 400 || op_target_profile_ > 499) { op_target_profile_ = 400; }
      break;
    default:
      break;
  }

  op_phase_ = OpPhase::TGT_CONFIGURE;
  op_inflight_ = false;
}

void SupervisorNode::start_switch_to(FSMState target)
{
  // Switch = (best-effort exit current) then enter target
  op_target_ = target;
  op_target_profile_ = desired_profile_;
  op_target_activate_ = desired_activate_;
  desired_activate_ = true;

  // Clamp to valid profile band for target state
  switch (op_target_) {
    case FSMState::INITIALIZING:
      if (op_target_profile_ > 99) { op_target_profile_ = 0; }
      break;
    case FSMState::IDLING:
      if (op_target_profile_ < 100 || op_target_profile_ > 199) { op_target_profile_ = 100; }
      break;
    case FSMState::RUNNING:
      if (op_target_profile_ < 200 || op_target_profile_ > 299) { op_target_profile_ = 200; }
      break;
    case FSMState::CHARGING:
      if (op_target_profile_ < 300 || op_target_profile_ > 399) { op_target_profile_ = 300; }
      break;
    case FSMState::FAULT:
      if (op_target_profile_ < 400 || op_target_profile_ > 499) { op_target_profile_ = 400; }
      break;
    default:
      break;
  }

  op_phase_ = OpPhase::CUR_GET;
  op_inflight_ = false;

  if (last_requester_ == "Supervisor") {
    RCLCPP_INFO(
      this->get_logger(),
      "FSM switch scheduled: %s (%03u) -> %s (%03u)",
      state_name(current_state_).c_str(),
      current_profile_,
      state_name(target).c_str(),
      op_target_profile_);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "FSM switch scheduled: %s (%03u) -> %s (%03u) (requester=%s priority=%u force=%s reason=%s)",
      state_name(current_state_).c_str(),
      current_profile_,
      state_name(target).c_str(),
      op_target_profile_,
      last_requester_.c_str(),
      last_priority_,
      last_force_ ? "true" : "false",
      last_reason_.c_str());
  }

}

void SupervisorNode::drive()
{
  // Never block. Schedule at most one async request per tick.
  enum class ActionType
  {
    NONE,
    GET,
    CHANGE
  };

  ActionType action = ActionType::NONE;
  FSMState cur_state = current_state_;
  FSMState target_state = op_target_;
  OpPhase phase = OpPhase::IDLE;
 
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (op_phase_ == OpPhase::IDLE || op_inflight_) {
      return;
    }

    cur_state = current_state_;
    target_state = op_target_;
    phase = op_phase_;

    // Mark in-flight before scheduling.
    op_inflight_ = true;

    switch (phase) {
      case OpPhase::CUR_GET:
        action = ActionType::GET;
        break;
      case OpPhase::CUR_DEACTIVATE:
        action = ActionType::CHANGE;
        break;
      case OpPhase::CUR_CLEANUP:
        action = ActionType::CHANGE;
        break;
      case OpPhase::TGT_CONFIGURE:
        action = ActionType::CHANGE;
        break;
      case OpPhase::TGT_ACTIVATE:
        action = ActionType::CHANGE;
        break;
      case OpPhase::IDLE:
      default:
        action = ActionType::NONE;
        op_inflight_ = false;
        break;
    }
  }

  if (action == ActionType::NONE) {
    return;
  }

  auto & cur_ep = endpoints_[cur_state];
  auto & tgt_ep = endpoints_[target_state];

  switch (phase) {
    case OpPhase::CUR_GET: {
      request_get_state(
        cur_ep,
        [this](bool ok, uint8_t id, const std::string & err) {
          std::lock_guard<std::mutex> lk(mtx_);
          op_inflight_ = false;

          if (!ok) {
            // Can't query; proceed to target enter anyway.
            last_message_ = "Exit current: " + err;
            op_phase_ = OpPhase::TGT_CONFIGURE;
            return;
          }

          last_lifecycle_id_ = id;
          active_lifecycle_label_ = lifecycle_id_to_label(id);

          if (id == State::PRIMARY_STATE_ACTIVE) {
            op_phase_ = OpPhase::CUR_DEACTIVATE;
          } else if (id == State::PRIMARY_STATE_INACTIVE) {
            op_phase_ = OpPhase::CUR_CLEANUP;
          } else {
            // Unconfigured/Unknown/etc: nothing meaningful to do here.
            op_phase_ = OpPhase::TGT_CONFIGURE;
          }
        });
      return;
    }

    case OpPhase::CUR_DEACTIVATE: {
      request_change_state(
        cur_ep,
        Transition::TRANSITION_DEACTIVATE,
        [this](bool ok, const std::string & err) {
          std::lock_guard<std::mutex> lk(mtx_);
          op_inflight_ = false;

          if (!ok) {
            last_message_ = "DEACTIVATE failed: " + err;
            // Best-effort: continue to target.
            op_phase_ = OpPhase::TGT_CONFIGURE;
            return;
          }
          op_phase_ = OpPhase::CUR_CLEANUP;
        });
      return;
    }

    case OpPhase::CUR_CLEANUP: {
      request_change_state(
        cur_ep,
        Transition::TRANSITION_CLEANUP,
        [this](bool ok, const std::string & err) {
          std::lock_guard<std::mutex> lk(mtx_);
          op_inflight_ = false;

          if (!ok) {
            last_message_ = "CLEANUP failed: " + err;
            // Best-effort: continue to target.
          }
          op_phase_ = OpPhase::TGT_CONFIGURE;
        });
      return;
    }

    case OpPhase::TGT_CONFIGURE: {
      // Push the supervisor-selected profile ID into the target lifecycle node
      // before configure(), so the node can load the correct per-profile YAML.
      try {
        const std::string remote = ns_ + "/" + tgt_ep.node_name;
        rclcpp::SyncParametersClient pc(param_client_node_, remote);

        using namespace std::chrono_literals;
        if (pc.wait_for_service(500ms)) {
          const auto results = pc.set_parameters({
            rclcpp::Parameter("profiles.active_profile_id", static_cast<int>(op_target_profile_))
          });
          if (!results.empty() && !results[0].successful) {
            RCLCPP_WARN(this->get_logger(), "Failed to set profiles.active_profile_id on %s: %s",
              remote.c_str(), results[0].reason.c_str());
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Parameter service not available for %s (profiles.active_profile_id not set)", remote.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception while setting profiles.active_profile_id: %s", e.what());
      }

      request_change_state(
        tgt_ep,
        Transition::TRANSITION_CONFIGURE,
        [this](bool ok, const std::string & err) {
          std::lock_guard<std::mutex> lk(mtx_);
          op_inflight_ = false;

          if (!ok) {
            last_message_ = "CONFIGURE failed: " + err;
            transition_status_ = "FAILED";
            // Stay in this phase and retry on next tick.
            return;
          }
          if (!op_target_activate_) {
            // Transition complete (configured only; leave target INACTIVE).
            current_state_ = op_target_;

            // Guard against profile/state mismatches (e.g. an in-flight request updating desired_profile_
            // while we are still configuring the current state). Profiles are grouped in bands:
            //   INITIALIZING: 000-099, IDLING: 100-199, RUNNING: 200-299, CHARGING: 300-399, FAULT: 400-499
            uint16_t effective_profile = op_target_profile_;
            switch (current_state_) {
              case FSMState::INITIALIZING:
                if (effective_profile > 99) { effective_profile = 0; }
                break;
              case FSMState::IDLING:
                if (effective_profile < 100 || effective_profile > 199) { effective_profile = 100; }
                break;
              case FSMState::RUNNING:
                if (effective_profile < 200 || effective_profile > 299) { effective_profile = 200; }
                break;
              case FSMState::CHARGING:
                if (effective_profile < 300 || effective_profile > 399) { effective_profile = 300; }
                break;
              case FSMState::FAULT:
                if (effective_profile < 400 || effective_profile > 499) { effective_profile = 400; }
                break;
              default:
                break;
            }
            current_profile_ = effective_profile;

            last_lifecycle_id_ = State::PRIMARY_STATE_INACTIVE;
            active_lifecycle_label_ = lifecycle_id_to_label(last_lifecycle_id_);

            RCLCPP_INFO(
              this->get_logger(),
              "FSM state configured (inactive): %s (%03u)",
              state_name(current_state_).c_str(),
              current_profile_);

            need_enter_current_ = false;

            // Keep request bookkeeping for request-driven transitions.
            if (last_requester_ != "Supervisor") {
              last_message_ = "Configured (inactive)";
            }

            op_phase_ = OpPhase::IDLE;
            return;
          }

          op_phase_ = OpPhase::TGT_ACTIVATE;
        });
      return;
    }

    case OpPhase::TGT_ACTIVATE: {
      request_change_state(
        tgt_ep,
        Transition::TRANSITION_ACTIVATE,
        [this](bool ok, const std::string & err) {
          std::lock_guard<std::mutex> lk(mtx_);
          op_inflight_ = false;

          if (!ok) {
            last_message_ = "ACTIVATE failed: " + err;
            transition_status_ = "ACTIVATE: " + err;
            // Escalate to FAULT if the target state cannot activate.
            // This covers readiness timeouts implemented inside state nodes.
            if (op_target_ != FSMState::FAULT) {
              RCLCPP_ERROR(
                this->get_logger(),
                "State activation failed for %s (%s). Escalating to FAULT.",
                state_name(op_target_).c_str(),
                err.c_str());
              desired_state_ = FSMState::FAULT;
              // Schedule a switch to FAULT.
              start_switch_to(FSMState::FAULT);
              return;
            }

            // If FAULT itself failed to activate, remain in this phase and retry.
            return;
          }

// Transition complete
current_state_ = op_target_;

// Guard against profile/state mismatches (e.g. an in-flight request updating desired_profile_
// while we are still activating the current state). Profiles are grouped in bands:
//   INITIALIZING: 000-099, IDLING: 100-199, RUNNING: 200-299, CHARGING: 300-399, FAULT: 400-499
uint16_t effective_profile = op_target_profile_;
switch (current_state_) {
  case FSMState::INITIALIZING:
    if (effective_profile > 99) { effective_profile = 0; }
    break;
  case FSMState::IDLING:
    if (effective_profile < 100 || effective_profile > 199) { effective_profile = 100; }
    break;
  case FSMState::RUNNING:
    if (effective_profile < 200 || effective_profile > 299) { effective_profile = 200; }
    break;
  case FSMState::CHARGING:
    if (effective_profile < 300 || effective_profile > 399) { effective_profile = 300; }
    break;
  case FSMState::FAULT:
    if (effective_profile < 400 || effective_profile > 499) { effective_profile = 400; }
    break;
  default:
    break;
}
current_profile_ = effective_profile;

RCLCPP_INFO(
  this->get_logger(),
  "FSM state changed, now running: %s (%03u)",
  state_name(current_state_).c_str(),
  current_profile_);

          need_enter_current_ = false;

// INTERNAL TRANSITIONS:
// Do not stamp "Entered <state>" into request bookkeeping. Entry is already logged via
// "FSM state changed, now running: <state>". Bookkeeping fields are reserved for request-driven
// transitions (request_state service).
if (last_requester_ == "Supervisor") {
  last_requester_ = "Supervisor";
  last_priority_  = 0;
  last_force_     = false;
  last_reason_.clear();
  last_message_.clear();
}


          op_phase_ = OpPhase::IDLE;
        });
      return;
    }

    case OpPhase::IDLE:
    default:
      {
        std::lock_guard<std::mutex> lk(mtx_);
        op_inflight_ = false;
      }
      return;
  }
}

uint8_t SupervisorNode::effective_last_priority() const
{
  const rclcpp::Time now = this->now();
  const double elapsed_s = (now - last_priority_time_).seconds();
  if (elapsed_s <= 0.0) {
    return last_priority_;
  }

  // Decay: 1 priority unit per second
  const int decayed =
    static_cast<int>(last_priority_) - static_cast<int>(std::floor(elapsed_s));

  return static_cast<uint8_t>(std::max(0, decayed));
}

void SupervisorNode::on_request_state(
  const std::shared_ptr<amr_sweeper_layer_0_fsm::srv::RequestState::Request> req,
  std::shared_ptr<amr_sweeper_layer_0_fsm::srv::RequestState::Response> resp)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // -------------------------------------------------------------------------
  // Parse requested state string.
  // -------------------------------------------------------------------------
  FSMState target = FSMState::INITIALIZING;
  if (req->target_state == "INITIALIZING") target = FSMState::INITIALIZING;
  else if (req->target_state == "IDLING") target = FSMState::IDLING;
  else if (req->target_state == "RUNNING") target = FSMState::RUNNING;
  else if (req->target_state == "CHARGING") target = FSMState::CHARGING;
  else if (req->target_state == "FAULT") target = FSMState::FAULT;
  else {
    resp->accepted = false;
    resp->current_state = state_name(current_state_);
    resp->current_profile_id = current_profile_;
    resp->desired_state = state_name(desired_state_);
    resp->desired_profile_id = desired_profile_;
    resp->message = "Rejected: unknown target_state='" + req->target_state + "'";

    last_message_ = resp->message;
    last_reason_ = req->reason;
    return;
  }

  // -------------------------------------------------------------------------
  // Parse requested lifecycle target (optional).
  // -------------------------------------------------------------------------
  // Convention:
  //  - empty / "Active"   => configure + activate target node
  //  - "Inactive"         => configure only (leave target node INACTIVE)
  bool request_activate = true;
  if (!req->target_lifecycle.empty()) {
    std::string v = req->target_lifecycle;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (v == "inactive") {
      request_activate = false;
    } else if (v == "active") {
      request_activate = true;
    } else {
      resp->accepted = false;
      resp->current_state = state_name(current_state_);
      resp->current_profile_id = current_profile_;
      resp->desired_state = state_name(desired_state_);
      resp->desired_profile_id = desired_profile_;
      resp->message = "Rejected: unknown target_lifecycle='" + req->target_lifecycle + "' (use '', 'Active', or 'Inactive')";

      last_message_ = resp->message;
      last_reason_ = req->reason;
      return;
    }
  }

  const uint8_t eff = effective_last_priority();  // NEW: apply time-based decay

  // -------------------------------------------------------------------------
  // Validate requested profile.
  // -------------------------------------------------------------------------
  // 1) Reject if the profile band does not match the requested state.
  // 2) Reject if the specific profile id is not defined in the state's profile YAML.
  const int8_t target_state_id = static_cast<int8_t>(target);
  if (profile_to_state_id(req->target_profile_id) != target_state_id) {
    resp->accepted = false;
    resp->current_state = state_name(current_state_);
    resp->current_profile_id = current_profile_;
    resp->desired_state = state_name(desired_state_);
    resp->desired_profile_id = desired_profile_;
    resp->message = "Rejected: profile " + std::to_string(req->target_profile_id) +
      " is not valid for target_state='" + req->target_state + "'";

    last_message_ = resp->message;
    last_reason_ = req->reason;
    return;
  }

  {
    std::string reason;
    if (!profile_exists_in_state_yaml(target_state_id, req->target_profile_id, reason)) {
      resp->accepted = false;
      resp->current_state = state_name(current_state_);
      resp->current_profile_id = current_profile_;
      resp->desired_state = state_name(desired_state_);
      resp->desired_profile_id = desired_profile_;
      resp->message = reason;

      last_message_ = resp->message;
      last_reason_ = req->reason;
      return;
    }
  }

  // Reject lower-priority requests unless forced
  if (!req->force && req->priority < eff) {
    resp->accepted = false;
    resp->current_state = state_name(current_state_);
    resp->current_profile_id = current_profile_;
    resp->desired_state = state_name(desired_state_);
    resp->desired_profile_id = desired_profile_;
    resp->message = "Rejected: lower priority than last request (gate=" +
      std::to_string(eff) + ", age_s=" +
      std::to_string((this->now() - last_priority_time_).seconds()) + ")";

    last_message_ = resp->message;
    last_reason_ = req->reason;
    return;
  }

  // Accept and queue the request; transitions are driven by tick().
  desired_activate_ = request_activate;
  desired_state_ = target;
  desired_profile_ = req->target_profile_id;
  transitioning_to_profile_ = desired_profile_;
  last_priority_ = req->priority;
  last_priority_time_ = this->now();
  last_requester_ = req->requester;
  last_force_ = req->force;
  last_reason_ = req->reason;
  last_message_ = "Queued: " + state_name(target) + " (req=" + req->requester + ")";

  resp->accepted = true;
  resp->current_state = state_name(current_state_);
  resp->current_profile_id = current_profile_;
  resp->desired_state = state_name(desired_state_);
  resp->desired_profile_id = desired_profile_;
  resp->message = "Queued";
}


void SupervisorNode::init_publish_rules()
{
  // publish.rules is a list of semicolon-delimited key=value pairs.
  // We keep parsing deliberately strict: if a rule is malformed we log it and skip it.
  const auto rule_strings = this->declare_parameter<std::vector<std::string>>("publish.rules", std::vector<std::string>{});

  publish_rules_.clear();
  publish_rules_.reserve(rule_strings.size());

  if (rule_strings.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No publish.rules configured. No periodic publications will be produced by the supervisor.");
    return;
  }

  for (const auto & rule_str : rule_strings) {
    PublishRule rule;
    std::string err;
    if (!parse_publish_rule(rule_str, rule, err)) {
      RCLCPP_ERROR(this->get_logger(), "publish.rules entry rejected: '%s' (%s)", rule_str.c_str(), err.c_str());
      continue;
    }

    // Create publisher for this rule.
    if (rule.msg_type == PublishMsgType::FSM_STATE) {
      rule.state_pub = this->create_publisher<amr_sweeper_layer_0_fsm::msg::FSMState>(rule.topic, 10);
    } else {
      rule.status_pub = this->create_publisher<amr_sweeper_layer_0_fsm::msg::FSMStatus>(rule.topic, 10);
    }

    // Create an independent timer for this rule. Publishing is decoupled from tick().
    const auto period = std::chrono::milliseconds(rule.period_ms);
    rule.timer = this->create_wall_timer(period, [this, idx = publish_rules_.size()]() {
      // Snapshot under lock to avoid publishing torn state.
      const auto snap = snapshot_status();
      publish_from_rule(publish_rules_[idx], snap);
    });

    RCLCPP_INFO(
      this->get_logger(),
      "Publish rule enabled: topic='%s' type='%s' source='%s' period_ms=%u",
      rule.topic.c_str(), rule.type.c_str(), rule.source.c_str(), rule.period_ms);

    publish_rules_.push_back(std::move(rule));
  }

  if (publish_rules_.empty()) {
    RCLCPP_WARN(this->get_logger(), "All publish.rules entries were invalid; no periodic publications enabled.");
  }
}

bool SupervisorNode::parse_publish_rule(const std::string & rule_str, PublishRule & out, std::string & error) const
{
  out = PublishRule{};
  out.raw = rule_str;

  auto trim = [](std::string s) {
    const char * ws = " \t\r\n";
    s.erase(0, s.find_first_not_of(ws));
    s.erase(s.find_last_not_of(ws) + 1);
    return s;
  };

  auto split = [](const std::string & s, char delim) {
    std::vector<std::string> parts;
    std::string cur;
    for (char c : s) {
      if (c == delim) {
        parts.push_back(cur);
        cur.clear();
      } else {
        cur.push_back(c);
      }
    }
    parts.push_back(cur);
    return parts;
  };

  std::string topic, type, source, period_ms_str, hz_str;

  for (const auto & part_raw : split(rule_str, ';')) {
    const auto part = trim(part_raw);
    if (part.empty()) continue;

    const auto eq = part.find('=');
    if (eq == std::string::npos) {
      error = "expected key=value segment";
      return false;
    }

    const auto key = trim(part.substr(0, eq));
    const auto val = trim(part.substr(eq + 1));

    if (key == "topic") topic = val;
    else if (key == "type") type = val;
    else if (key == "source") source = val;
    else if (key == "period_ms") period_ms_str = val;
    else if (key == "hz") hz_str = val;  // legacy (deprecated)
    else {
      // Ignore unknown keys for forward-compatibility.
    }
  }

  if (topic.empty()) { error = "missing 'topic'"; return false; }
  if (type.empty()) { error = "missing 'type'"; return false; }
  if (source.empty()) { error = "missing 'source'"; return false; }

  // Prefer explicit period_ms. Fall back to legacy hz for backwards compatibility.
  if (!period_ms_str.empty()) {
    long v = -1;
    try {
      v = std::stol(period_ms_str);
    } catch (...) {
      error = "period_ms is not a valid integer";
      return false;
    }
    if (v <= 0) {
      error = "period_ms must be > 0";
      return false;
    }
    out.period_ms = static_cast<uint32_t>(v);
  } else if (!hz_str.empty()) {
    double hz = 0.0;
    try {
      hz = std::stod(hz_str);
    } catch (...) {
      error = "hz is not a valid number";
      return false;
    }
    if (!(hz > 0.0)) {
      error = "hz must be > 0";
      return false;
    }

    // Convert hz to an integer millisecond period. Clamp to at least 1 ms.
    const double ms_d = 1000.0 / hz;
    uint32_t ms = 1U;
    if (ms_d > 1.0) {
      ms = static_cast<uint32_t>(std::llround(ms_d));
      if (ms == 0U) ms = 1U;
    }
    out.period_ms = ms;
  } else {
    error = "missing 'period_ms' (or legacy 'hz')";
    return false;
  }

  out.topic = topic;
  out.type = type;
  out.source = source;

  // Message type + source validation.
  if (type == "amr_sweeper_layer_0_fsm/msg/FSMState") {
    out.msg_type = PublishMsgType::FSM_STATE;
    if (source != "state") {
      error = "FSMState only supports source=state";
      return false;
    }
  } else if (type == "amr_sweeper_layer_0_fsm/msg/FSMStatus") {
    out.msg_type = PublishMsgType::FSM_STATUS;
    if (source != "status") {
      error = "FSMStatus only supports source=status";
      return false;
    }
  } else {
    error = "unsupported type (supported: amr_sweeper_layer_0_fsm/msg/FSMState, amr_sweeper_layer_0_fsm/msg/FSMStatus)";
    return false;
  }

  return true;
}

SupervisorNode::StatusSnapshot SupervisorNode::snapshot_status() const
{
  std::lock_guard<std::mutex> lk(mtx_);

  StatusSnapshot snap;
  snap.current_state = current_state_;
  snap.active_lifecycle_label = active_lifecycle_label_;
  snap.current_profile = current_profile_;
  snap.transitioning_to_profile = transitioning_to_profile_;
  snap.transition_status = transition_status_;
  snap.last_priority = last_priority_;
  snap.last_priority_time = last_priority_time_;
  snap.last_requester = last_requester_;
  snap.last_message = last_message_;
  return snap;
}

void SupervisorNode::publish_from_rule(const PublishRule & rule, const StatusSnapshot & snap)
{
  if (rule.msg_type == PublishMsgType::FSM_STATE) {
    const auto msg = build_state_payload(snap);
    if (rule.state_pub) rule.state_pub->publish(msg);
    return;
  }

  if (rule.msg_type == PublishMsgType::FSM_STATUS) {
    auto st = build_status_payload(snap);
    if (rule.status_pub) rule.status_pub->publish(st);
    return;
  }
}

amr_sweeper_layer_0_fsm::msg::FSMState SupervisorNode::build_state_payload(const StatusSnapshot & snap) const
{
  amr_sweeper_layer_0_fsm::msg::FSMState st;
  st.stamp = this->now();
  st.current_state = state_name(snap.current_state);
  st.current_profile = snap.current_profile;
  return st;
}

amr_sweeper_layer_0_fsm::msg::FSMStatus SupervisorNode::build_status_payload(const StatusSnapshot & snap) const
{
  amr_sweeper_layer_0_fsm::msg::FSMStatus st;
  st.stamp = this->now();
  st.current_state = state_name(snap.current_state);
  st.current_lifecycle_state = snap.active_lifecycle_label;
  st.current_profile = snap.current_profile;

  st.transitioning_to_profile = snap.transitioning_to_profile;
  st.transition_status = snap.transition_status;

  st.last_requester = snap.last_requester;
  st.last_request_priority = snap.last_priority;
  st.effective_priority_gate = effective_last_priority();

  // Priority "age" is computed from the supervisor clock, not from tick().
  const double age = (this->now() - snap.last_priority_time).seconds();
  st.priority_age_sec = static_cast<float>(age < 0.0 ? 0.0 : age);

  st.last_message = snap.last_message;
  return st;
}


void SupervisorNode::tick()
{
  // Decide what operation to run (start state machine) under lock.
  {
    std::lock_guard<std::mutex> lk(mtx_);

    // -----------------------------------------------------------------------
    // Transition status bookkeeping for FSMStatus.
    // -----------------------------------------------------------------------
    if (op_phase_ == OpPhase::IDLE) {
      transition_status_ = "STABLE";
    } else {
      transition_status_ = "TRANSITIONING";
    }

    if (op_phase_ == OpPhase::IDLE) {
      if (need_enter_current_) {
        start_enter_state(current_state_);
      } else if (desired_state_ != current_state_) {
        start_switch_to(desired_state_);
      } else if (desired_profile_ != current_profile_) {
        current_profile_ = desired_profile_;
  transitioning_to_profile_ = current_profile_;
        RCLCPP_INFO(
          this->get_logger(),
          "FSM profile changed: now %s (%03u)",
          state_name(current_state_).c_str(),
          current_profile_);
        last_message_ = "Applied profile update";
      }
    }
  }

  // Schedule exactly one lifecycle action if needed.
  drive();

  // Non-blocking lifecycle label poll of the current state's node.
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (op_inflight_ || poll_inflight_) {
      return;
    }
    poll_inflight_ = true;
  }

  auto & ep = endpoints_[current_state_];
  request_get_state(
    ep,
    [this](bool ok, uint8_t id, const std::string & err) {
      std::lock_guard<std::mutex> lk(mtx_);
      poll_inflight_ = false;

      if (ok) {
        last_lifecycle_id_ = id;
        active_lifecycle_label_ = lifecycle_id_to_label(id);
      } else {
        // Keep previous label, but update message for visibility.
        last_message_ = err;
      }
    });
}

}  // namespace fsm_layer_0

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fsm_layer_0::SupervisorNode>();

  // MultiThreadedExecutor is still a good idea even though we are non-blocking:
  // - lifecycle responses and service callbacks can interleave cleanly
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
