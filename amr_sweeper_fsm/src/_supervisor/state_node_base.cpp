#include "_supervisor/state_node_base.hpp"
#include <algorithm>
#include <set>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <filesystem>

#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include <yaml-cpp/yaml.h>

namespace {
  std::atomic<uint64_t> g_probe_seq{0};
  constexpr char kAnsiGreen[] = "\033[32m";
  constexpr char kAnsiLightMagenta[] = "\033[95m";
  constexpr char kAnsiReset[] = "\033[0m";
  const std::vector<std::string> kRuntimeOverrideKeys{
    "missions_directory",
    "auto_build_on_start",
    "watch_for_updates",
    "trigger_running_on_work_window",
    "use_amr_sweeper_ros2_control",
    "use_amr_sweeper_battery",
    "use_amr_sweeper_system_info",
    "use_amr_sweeper_usb_cameras",
    "use_amr_sweeper_depth_camera",
    "use_amr_sweeper_imu",
    "use_amr_sweeper_gnss",
    "use_ntrip_client",
    "use_amr_sweeper_drive_controller",
    "use_amr_sweeper_tool_controller",
    "use_amr_sweeper_teleop",
    "use_amr_sweeper_sweeping_controller",
    "use_sweeping_mode",
    "use_amr_sweeper_attitude_controller",
    "use_amr_sweeper_collision_detector",
    "use_amr_sweeper_safety_controller",
    "use_joy_node",
    "joy_dev",
    "use_amr_sweeper_visual_odometry",
    "use_amr_sweeper_localization",
    "use_amr_sweeper_mapping",
    "use_amr_sweeper_navigation",
    "auto_start_mission",
  };

  const std::vector<std::string> kLayer1BringupOverrideKeys{
    "use_sim_time",
    "use_amr_sweeper_ros2_control",
    "use_amr_sweeper_battery",
    "use_amr_sweeper_system_info",
    "use_amr_sweeper_usb_cameras",
    "use_amr_sweeper_depth_camera",
    "use_amr_sweeper_imu",
    "use_amr_sweeper_gnss",
    "use_ntrip_client",
  };

  const std::vector<std::string> kLayer2BringupOverrideKeys{
    "use_sim_time",
    "use_amr_sweeper_drive_controller",
    "use_amr_sweeper_tool_controller",
    "use_amr_sweeper_teleop",
    "use_amr_sweeper_sweeping_controller",
    "use_sweeping_mode",
    "use_amr_sweeper_attitude_controller",
    "use_amr_sweeper_collision_detector",
    "use_amr_sweeper_safety_controller",
    "use_joy_node",
    "joy_dev",
  };

  const std::vector<std::string> kLayer3BringupOverrideKeys{
    "use_sim_time",
    "use_amr_sweeper_visual_odometry",
    "use_amr_sweeper_localization",
    "use_amr_sweeper_mapping",
    "use_amr_sweeper_navigation",
    "auto_start_mission",
  };

  const std::vector<std::string> kParserOverrideKeys{
    "use_sim_time",
    "missions_directory",
    "auto_build_on_start",
    "watch_for_updates",
  };

  const std::vector<std::string> kSchedulerOverrideKeys{
    "use_sim_time",
    "missions_directory",
    "trigger_running_on_work_window",
  };

  struct TriggerLine
  {
    std::string line;
    fsm_layer_0::ProcessImportance importance{fsm_layer_0::ProcessImportance::CRITICAL};
  };

  using LaunchArgMap = std::unordered_map<std::string, std::string>;

  std::string join_args_as_shell(const YAML::Node & args)
  {
    std::string out;
    if (!args || !args.IsSequence()) {
      return out;
    }
    for (size_t i = 0; i < args.size(); ++i) {
      const auto s = args[i].as<std::string>("");
      if (s.empty()) {
        continue;
      }
      if (!out.empty()) {
        out += " ";
      }
      out += s;
    }
    return out;
  }

  fsm_layer_0::ProcessImportance parse_importance(const std::string & raw)
  {
    std::string s = raw;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "optional") {
      return fsm_layer_0::ProcessImportance::OPTIONAL;
    }
    if (s == "degraded") {
      return fsm_layer_0::ProcessImportance::DEGRADED;
    }
    return fsm_layer_0::ProcessImportance::CRITICAL;
  }

  uint8_t parse_lifecycle_primary_state_label(const std::string & raw)
  {
    if (raw == "UNCONFIGURED" || raw == "UNCONFIGURE" || raw == "UNCONFIG") {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    }
    if (raw == "INACTIVE" || raw == "CONFIGURED") {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }
    if (raw == "ACTIVE") {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }
    if (raw == "FINALIZED" || raw == "FINAL") {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    }
    return lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  uint8_t parse_profile_required_lifecycle_level(const std::string & raw)
  {
    return parse_lifecycle_primary_state_label(raw);
  }

  bool is_layer_bringup_command(const std::string & command)
  {
    return
      command.find("ros2 launch amr_sweeper_layer_1_hardware_bringup ") != std::string::npos ||
      command.find("ros2 launch amr_sweeper_layer_2_controllers_bringup ") != std::string::npos ||
      command.find("ros2 launch amr_sweeper_layer_3_navigation_bringup ") != std::string::npos;
  }

  bool parse_bool_text(const std::string & value, bool & out)
  {
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);
    if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
      out = true;
      return true;
    }
    if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
      out = false;
      return true;
    }
    return false;
  }

  LaunchArgMap parse_launch_arguments_from_command(const std::string & command)
  {
    LaunchArgMap args;
    std::stringstream ss(command);
    std::string token;
    while (ss >> token) {
      const auto pos = token.find(":=");
      if (pos == std::string::npos || pos == 0) {
        continue;
      }
      args[token.substr(0, pos)] = token.substr(pos + 2);
    }
    return args;
  }

  bool launch_arg_enabled(
    const LaunchArgMap & args,
    const std::string & key,
    bool default_value)
  {
    const auto it = args.find(key);
    if (it == args.end()) {
      return default_value;
    }
    bool parsed = default_value;
    if (parse_bool_text(it->second, parsed)) {
      return parsed;
    }
    return default_value;
  }

  bool target_matches_pattern(const std::string & target, const std::string & pattern)
  {
    if (target == pattern) {
      return true;
    }
    if (target.size() >= pattern.size() &&
        target.compare(target.size() - pattern.size(), pattern.size(), pattern) == 0) {
      return true;
    }
    return target.find(pattern) != std::string::npos;
  }

  bool is_profile_requirement_enabled(
    const std::string & resolved_command,
    const std::string & kind,
    const std::string & target)
  {
    const LaunchArgMap args = parse_launch_arguments_from_command(resolved_command);

    if (resolved_command.find("ros2 launch amr_sweeper_layer_1_hardware_bringup ") != std::string::npos) {
      if (
        target_matches_pattern(target, "imu/data_raw") ||
        target_matches_pattern(target, "imu/data_acc_gyro") ||
        target_matches_pattern(target, "imu/data_heading") ||
        target_matches_pattern(target, "imu/imu_node"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_imu", true);
      }
      if (
        target_matches_pattern(target, "gnss/navsat") ||
        target_matches_pattern(target, "gnss/gnss_node"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_gnss", true);
      }
      if (target_matches_pattern(target, "gnss/ntrip_client_node")) {
        return launch_arg_enabled(args, "use_ntrip_client", true);
      }
      if (target_matches_pattern(target, "usb_cameras/")) {
        return launch_arg_enabled(args, "use_amr_sweeper_usb_cameras", true);
      }
      if (target_matches_pattern(target, "depth_camera/")) {
        return launch_arg_enabled(args, "use_amr_sweeper_depth_camera", true);
      }
      if (target_matches_pattern(target, "battery/")) {
        return launch_arg_enabled(args, "use_amr_sweeper_battery", true);
      }
      if (target_matches_pattern(target, "system_info/")) {
        return launch_arg_enabled(args, "use_amr_sweeper_system_info", true);
      }
      if (
        kind == "hardware" ||
        kind == "controller" ||
        target_matches_pattern(target, "joint_states"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_ros2_control", true);
      }
    }

    if (resolved_command.find("ros2 launch amr_sweeper_layer_2_controllers_bringup ") != std::string::npos) {
      if (
        target_matches_pattern(target, "drive_controller/odom") ||
        target_matches_pattern(target, "drive_controller") ||
        target_matches_pattern(target, "cmd_vel_drive"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_drive_controller", true);
      }
      if (
        target_matches_pattern(target, "tool_controller") ||
        target_matches_pattern(target, "cmd_vel_tools"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_tool_controller", true);
      }
      if (
        target_matches_pattern(target, "attitude_controller/roll_pitch") ||
        target_matches_pattern(target, "attitude_controller/joint_states") ||
        target_matches_pattern(target, "amr_sweeper_attitude_controller/enable_attitude_estimation"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_attitude_controller", true);
      }
      if (
        target_matches_pattern(target, "safety_controller/status") ||
        target_matches_pattern(target, "amr_sweeper_safety_controller/enable_controller"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_safety_controller", true);
      }
      if (
        target_matches_pattern(target, "collision_detector/impact_detected") ||
        target_matches_pattern(target, "amr_sweeper_collision_detector/enable_detector"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_collision_detector", true);
      }
    }

    if (resolved_command.find("ros2 launch amr_sweeper_layer_3_navigation_bringup ") != std::string::npos) {
      if (
        target_matches_pattern(target, "localization/odometry_fused") ||
        target_matches_pattern(target, "fusioncore"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_localization", true);
      }
      if (
        target_matches_pattern(target, "gaussian_node") ||
        target_matches_pattern(target, "mapping_node"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_mapping", true);
      }
      if (
        target_matches_pattern(target, "controller_server") ||
        target_matches_pattern(target, "planner_server") ||
        target_matches_pattern(target, "bt_navigator") ||
        target_matches_pattern(target, "waypoint_follower"))
      {
        return launch_arg_enabled(args, "use_amr_sweeper_navigation", true);
      }
    }

    return true;
  }

  std::string format_started_message(const std::string & command)
  {
    const std::string message = "Started: " + command;
    if (!is_layer_bringup_command(command)) {
      return message;
    }
    return std::string(kAnsiLightMagenta) + message + kAnsiReset;
  }

  std::string format_green_message(const std::string & message)
  {
    return std::string(kAnsiGreen) + message + kAnsiReset;
  }

  bool load_profile_processes(
    const std::string & yaml_path,
    uint16_t profile_id,
    std::vector<fsm_layer_0::ProfileProcess> & out_processes,
    std::string & err)
  {
    out_processes.clear();
    err.clear();

    YAML::Node root;
    try {
      root = YAML::LoadFile(yaml_path);
    } catch (const std::exception & e) {
      err = std::string("YAML::LoadFile failed: ") + e.what();
      return false;
    }

    const auto profiles = root["profiles"];
    if (!profiles || !profiles.IsSequence()) {
      err = "Missing or invalid top-level 'profiles' sequence";
      return false;
    }

    YAML::Node matched;
    for (const auto & item : profiles) {
      const auto prof = item["profile"];
      if (!prof || !prof.IsMap()) {
        continue;
      }
      const auto id = prof["id"].as<int>(-1);
      if (id < 0) {
        continue;
      }
      if (static_cast<uint16_t>(id) == profile_id) {
        matched = prof;
        break;
      }
    }

    if (!matched) {
      err = "No matching profile id=" + std::to_string(profile_id);
      return false;
    }

    const auto procs = matched["processes"];
    if (!procs) {
      // Treat missing as empty.
      return true;
    }
    if (!procs.IsSequence()) {
      err = "Profile 'processes' is not a sequence";
      return false;
    }

    // yaml-cpp range-for can yield YAML::detail::iterator_value; materialize nodes explicitly.
    for (auto pit = procs.begin(); pit != procs.end(); ++pit) {
      const YAML::Node proc = *pit;
      const auto startup = proc["startup"];
      if (!startup) {
        continue;
      }
      const auto exec = startup["exec"].as<std::string>("");
      if (exec.empty()) {
        continue;
      }
      const auto args = startup["args"];
      const auto joined = join_args_as_shell(args);

      std::string cmd = exec;
      if (!joined.empty()) {
        cmd += " ";
        cmd += joined;
      }
      fsm_layer_0::ProfileProcess pp;
      pp.name = proc["name"].as<std::string>("");
      pp.command = cmd;
      pp.importance = parse_importance(proc["importance"].as<std::string>("critical"));
      pp.window_ms = startup["window_ms"].as<int>(0);

      const auto restart = proc["restart"];
      if (restart && restart.IsMap()) {
        pp.max_restarts = restart["max_restarts"].as<int>(0);
        pp.restart_delay_ms = restart["restart_delay_ms"].as<int>(0);
      }

      // Optional: readiness requirements declared per-process under startup.ready.
      const auto ready = startup["ready"];
      if (ready && ready.IsSequence()) {
        for (size_t ri = 0; ri < ready.size(); ++ri) {
          const auto r = ready[ri];
          if (!r || !r.IsMap()) {
            continue;
          }
          const bool required = r["required"].as<bool>(true);
          if (!required) {
            continue;
          }
          const auto type = r["type"].as<std::string>("");
          const auto target = r["target"].as<std::string>("");
          if (type.empty() || target.empty()) {
            continue;
          }
          if (type == "node") {
            pp.ready_nodes.push_back(target);
          } else if (type == "topic") {
            pp.ready_topics.push_back(target);
          } else if (type == "service") {
            pp.ready_services.push_back(target);
          } else if (type == "controller") {
            pp.ready_active_controllers.push_back(target);
          } else if (type == "hardware") {
            fsm_layer_0::HardwareComponentRequirement req;
            req.name = target;
            req.raw = target;
            if (r["state"]) {
              req.required_state_id = parse_profile_required_lifecycle_level(
                r["state"].as<std::string>("active"));
            }
            pp.ready_hardware_components.push_back(req);
          } else if (type == "lifecycle") {
            fsm_layer_0::LifecycleNodeRequirement req;
            req.node = target;
            req.raw = target;
            req.min_state_id = r["state"] ?
              parse_profile_required_lifecycle_level(r["state"].as<std::string>("active")) :
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            pp.ready_lifecycle_nodes.push_back(req);
          }
        }
      }

      

      // Optional per-process error policy.
      const auto errors = proc["errors"];
      if (errors && errors.IsMap()) {
        pp.errors.on_readiness_fail = errors["on_readiness_fail"].as<std::string>("");
        pp.errors.on_unexpected_exit = errors["on_unexpected_exit"].as<std::string>("");
        pp.errors.priority = errors["priority"].as<int>(-1);
        const auto force_n = errors["force"];
        if (force_n) {
          pp.errors.force = force_n.as<bool>(false);
          pp.errors.has_force = true;
        }
        pp.errors.lifecycle_target = errors["lifecycle_target"].as<std::string>("");
        pp.errors.target_profile_id = errors["target_profile_id"].as<int>(-1);
      }

      // Optional per-process shutdown policy.
      const auto shutdown = proc["shutdown"];
      if (shutdown && shutdown.IsMap()) {
        pp.shutdown.sigint_timeout_ms = shutdown["sigint_timeout_ms"].as<int>(0);
        pp.shutdown.sigterm_timeout_ms = shutdown["sigterm_timeout_ms"].as<int>(0);
        pp.shutdown.sigkill_timeout_ms = shutdown["sigkill_timeout_ms"].as<int>(0);
      }
out_processes.push_back(pp);
    }

    return true;
  }

struct ProfileTransitions
{
  bool auto_transition_on{false};
  uint16_t auto_transition_profile{100};
  bool fault_transition_on{true};
  uint16_t fault_transition_profile{400};
};

bool load_profile_transitions(
  const std::string & yaml_path,
  uint16_t profile_id,
  ProfileTransitions & out,
  std::string & err)
{
  out = ProfileTransitions{};
  err.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    err = std::string("YAML::LoadFile failed: ") + e.what();
    return false;
  }

  const auto profiles = root["profiles"];
  if (!profiles || !profiles.IsSequence()) {
    err = "Missing or invalid top-level 'profiles' sequence";
    return false;
  }

  YAML::Node matched;
  for (const auto & item : profiles) {
    const auto prof = item["profile"];
    if (!prof || !prof.IsMap()) {
      continue;
    }
    const auto id = prof["id"].as<int>(-1);
    if (id < 0) {
      continue;
    }
    if (static_cast<uint16_t>(id) == profile_id) {
      matched = prof;
      break;
    }
  }

  if (!matched) {
    err = "No matching profile id=" + std::to_string(profile_id);
    return false;
  }

  const auto transitions = matched["transitions"];
  if (!transitions || !transitions.IsMap()) {
    // Missing transitions: keep defaults (backwards-compatible behavior).
    return true;
  }

  // auto
  if (transitions["auto_transition_on"]) {
    out.auto_transition_on = transitions["auto_transition_on"].as<bool>(out.auto_transition_on);
  }
  if (transitions["auto_transition_profile"]) {
    const int v =
      transitions["auto_transition_profile"].as<int>(static_cast<int>(out.auto_transition_profile));
    out.auto_transition_profile = static_cast<uint16_t>(std::clamp(v, 0, 65535));
  }

  // fault
  if (transitions["fault_transition_on"]) {
    out.fault_transition_on = transitions["fault_transition_on"].as<bool>(out.fault_transition_on);
  }
  if (transitions["fault_transition_profile"]) {
    const int v =
      transitions["fault_transition_profile"].as<int>(static_cast<int>(out.fault_transition_profile));
    out.fault_transition_profile = static_cast<uint16_t>(std::clamp(v, 0, 65535));
  }

  return true;
}


  bool load_profile_rosout_triggers(
    const std::string & file_path,
    uint16_t profile_id,
    std::vector<TriggerLine> & out_trigger_lines,
    std::string & out_error)
  {
    out_trigger_lines.clear();

    YAML::Node root;
    try {
      root = YAML::LoadFile(file_path);
    } catch (const std::exception & e) {
      out_error = std::string("Failed to load YAML file: ") + e.what();
      return false;
    }

    if (!root || !root["profiles"] || !root["profiles"].IsSequence()) {
      out_error = "YAML missing 'profiles' list";
      return false;
    }

    YAML::Node profile;
    // yaml-cpp's range-for iterator type can be YAML::detail::iterator_value
    // depending on the library version. Avoid implicit conversions by using
    // explicit iterators and materializing YAML::Node values.
    for (auto it = root["profiles"].begin(); it != root["profiles"].end(); ++it) {
      const YAML::Node entry = *it;
      if (!entry) {
        continue;
      }

      // Support both schema styles:
      //   - { id: <n>, processes: [...] }
      //   - { profile: { id: <n>, processes: [...] } }
      const YAML::Node p = entry["profile"] ? entry["profile"] : entry;

      if (p && p["id"] && p["id"].as<uint16_t>() == profile_id) {
        profile = p;
        break;
      }
    }
    if (!profile) {
      out_error = "Profile id not found in YAML";
      return false;
    }

    const YAML::Node processes = profile["processes"];
    if (!processes || !processes.IsSequence()) {
      // No processes => no triggers; treat as success.
      return true;
    }

    for (auto it = processes.begin(); it != processes.end(); ++it) {
      const YAML::Node proc = *it;
      if (!proc || !proc.IsMap()) {
        continue;
      }
      const YAML::Node triggers = proc["rosout_triggers"];
      if (!triggers || !triggers.IsSequence()) {
        continue;
      }
      for (auto tit = triggers.begin(); tit != triggers.end(); ++tit) {
        const YAML::Node t = *tit;
        if (!t || !t.IsScalar()) {
          continue;
        }
        TriggerLine tl;
        tl.line = t.as<std::string>();
        tl.importance = parse_importance(proc["importance"].as<std::string>("critical"));
        out_trigger_lines.push_back(tl);
}
    }

    return true;
  }
}

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace fsm_layer_0
{


std::string StateNodeBase::target_state_from_profile_id(uint16_t profile_id)
{
  if (profile_id <= 99) {
    return "INITIALIZING";
  }
  if (profile_id <= 199) {
    return "IDLING";
  }
  if (profile_id <= 299) {
    return "RUNNING";
  }
  if (profile_id <= 399) {
    return "CHARGING";
  }
  if (profile_id <= 499) {
    return "FAULT";
  }
  return "";
}

// ---------- Rosout trigger parsing helpers ----------

int StateNodeBase::parse_level(const std::string & s)
{
  // rcl_interfaces/msg/Log uses numeric levels compatible with rcutils.
  if (s == "DEBUG") return 10;
  if (s == "INFO")  return 20;
  if (s == "WARN")  return 30;
  if (s == "ERROR") return 40;
  if (s == "FATAL") return 50;

  // Conservative default: treat unknown strings as ERROR.
  return 40;
}

bool StateNodeBase::ends_with(const std::string & str, const std::string & suf)
{
  return str.size() >= suf.size() &&
         str.compare(str.size() - suf.size(), suf.size(), suf) == 0;
}

// ---------- Lifecycle readiness parsing helpers ----------

uint8_t StateNodeBase::parse_lifecycle_level(const std::string & s)
{
  return parse_lifecycle_primary_state_label(s);
}

LifecycleNodeRequirement
StateNodeBase::parse_lifecycle_requirement_line(const std::string & line)
{
  LifecycleNodeRequirement r;
  r.raw = line;

  // Backwards compatible: a plain node name implies ACTIVE.
  if (line.find('=') == std::string::npos && line.find("level>=") == std::string::npos) {
    r.node = line;
    r.min_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    return r;
  }

  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ';')) {
    if (token.rfind("level>=", 0) == 0) {
      r.min_state_id = parse_lifecycle_level(token.substr(std::string("level>=").size()));
      continue;
    }

    const auto eq = token.find('=');
    if (eq == std::string::npos) {
      continue;
    }

    const std::string key = token.substr(0, eq);
    const std::string val = token.substr(eq + 1);
    if (key == "node") {
      r.node = val;
    }
  }

  // If the user forgot node=, treat the whole line as a node.
  if (r.node.empty()) {
    r.node = line;
  }
  return r;
}

// Minimal rule syntax: "k=v;k=v;level>=ERROR"
StateNodeBase::RosoutTrigger StateNodeBase::parse_trigger_line(const std::string & line)
{
  RosoutTrigger t;
  t.raw = line;

  std::stringstream ss(line);
  std::string token;

  while (std::getline(ss, token, ';')) {
    // level>=X is special (no '=')
    if (token.rfind("level>=", 0) == 0) {
      t.min_level = parse_level(token.substr(std::string("level>=").size()));
      continue;
    }

    const auto eq = token.find('=');
    if (eq == std::string::npos) {
      continue;
    }

    const std::string key = token.substr(0, eq);
    const std::string val = token.substr(eq + 1);

    if (key == "node") {
      if (!val.empty()) {
        t.has_node = true;
        t.node = val;
      }
    } else if (key == "match") {
      if (!val.empty()) {
        t.has_match = true;
        t.match = val;
      }
    } else if (key == "action") {
      if (val == "FAULT") {
        t.action = RosoutAction::FAULT;
      } else if (val == "DEGRADED") {
        t.action = RosoutAction::DEGRADED;
      } else {
        // Anything not explicitly handled is treated as warn-only.
        t.action = RosoutAction::WARN_ONLY;
      }
    }
  }

  return t;
}

// ---------- Construction ----------

StateNodeBase::StateNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options)
{
  // Intentionally empty: lifecycle callbacks own all runtime setup/teardown.

  // The supervisor sets this parameter before lifecycle transitions.
  // Declare it up-front so external set_parameters calls succeed.
  this->declare_parameter<int>("profiles.active_profile_id", static_cast<int>(active_profile_id_));
  this->declare_parameter<std::string>("runtime.mission_execution_directory", "");
  this->declare_parameter<bool>("runtime.use_test", false);
  this->declare_parameter<std::string>("runtime.test_output_directory", "");
  for (const auto & key : kRuntimeOverrideKeys) {
    this->declare_parameter<std::string>("runtime." + key, "");
  }
}


// ---------- Lifecycle callbacks ----------

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_configure(const rclcpp_lifecycle::State &)
{
  // Reset per-activation log guards.
  warned_noncritical_readiness_.clear();

  // 1) Bring-up commands for this state.
  //
  // The YAML is expected to provide a list of shell commands, e.g.:
  //   processes:
  //     commands:
  //       - "ros2 launch pkg file.launch.py ns:={ns}"
  //
  // These are started on activation and stopped on deactivation/cleanup.

  auto declare_if_needed = [this](const std::string & name, auto default_value) {
    using T = decltype(default_value);
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
  };

  declare_if_needed("faults.request_state_service", request_state_service_);
  declare_if_needed("faults.fault_priority", static_cast<int>(fault_priority_));
  declare_if_needed("faults.fault_force", fault_force_);
  declare_if_needed("faults.fault_lifecycle_target", fault_lifecycle_target_);

  declare_if_needed("profiles.default_profile_id", static_cast<int>(default_profile_id_));
  declare_if_needed("profiles.active_profile_id", static_cast<int>(active_profile_id_));
  declare_if_needed("profiles.file", profiles_file_);
  declare_if_needed("runtime.mission_execution_directory", std::string(""));
  declare_if_needed("runtime.use_test", false);
  declare_if_needed("runtime.test_output_directory", std::string(""));
  for (const auto & key : kRuntimeOverrideKeys) {
    declare_if_needed("runtime." + key, std::string(""));
  }

  // Read (safe on every configure)
  this->get_parameter("faults.request_state_service", request_state_service_);

  int prio = static_cast<int>(fault_priority_);
  this->get_parameter("faults.fault_priority", prio);
  prio = std::clamp(prio, 0, 255);
  fault_priority_ = static_cast<uint8_t>(prio);

  this->get_parameter("faults.fault_force", fault_force_);
  this->get_parameter("faults.fault_lifecycle_target", fault_lifecycle_target_);

  // Profile selection (used to load per-profile process definitions from YAML)
  int def_id = static_cast<int>(default_profile_id_);
  this->get_parameter("profiles.default_profile_id", def_id);
  def_id = std::clamp(def_id, 0, 65535);
  default_profile_id_ = static_cast<uint16_t>(def_id);

  int act_id = static_cast<int>(active_profile_id_);
  this->get_parameter("profiles.active_profile_id", act_id);
  act_id = std::clamp(act_id, 0, 65535);
  active_profile_id_ = static_cast<uint16_t>(act_id);

  this->get_parameter("profiles.file", profiles_file_);

  // Default profile file path if not provided via params.
  // Expected naming convention: config/profiles/<state>_profiles.yaml
  if (profiles_file_.empty()) {
    std::string base = this->get_name();
    const std::string suffix = "_state";
    if (base.size() > suffix.size() && base.rfind(suffix) == (base.size() - suffix.size())) {
      base = base.substr(0, base.size() - suffix.size());
    }
    profiles_file_ = "config/profiles/" + base + "_profiles.yaml";
  }


  // Create client once (service name may change via params, so you may want to recreate if it changes)
  if (!request_state_client_) {
    request_state_client_ =
      this->create_client<amr_sweeper_fsm::srv::RequestState>(request_state_service_);
  }

  {
    // Prefer explicit `processes` parameter (backwards compatible). If not set or empty,
    // load processes from the selected profile in the profile YAML file.
    rclcpp::Parameter p;
    if (this->get_parameter("processes", p)) {
      processes_ = p.get_value<std::vector<std::string>>();
    } else {
      processes_ =
        this->declare_parameter<std::vector<std::string>>("processes", std::vector<std::string>{});
    }

    if (processes_.empty()) {
      // Choose profile id: active if nonzero, otherwise default.
      const uint16_t chosen_profile = (active_profile_id_ != 0) ? active_profile_id_ : default_profile_id_;

      // Resolve YAML path (allow relative paths under the package share dir).
      std::filesystem::path yaml_path = profiles_file_;
      if (yaml_path.is_relative()) {
        const auto share_dir = ament_index_cpp::get_package_share_directory("amr_sweeper_fsm");
        yaml_path = std::filesystem::path(share_dir) / yaml_path;
      }

      std::string err;
      if (!load_profile_processes(
            yaml_path.string(), chosen_profile, profile_processes_, err)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to load profile processes: file='%s' profile=%u err=%s",
          yaml_path.string().c_str(),
          chosen_profile,
          err.c_str());
      } else {
        processes_.clear();
        processes_.reserve(profile_processes_.size());
        for (auto & p : profile_processes_) {
          for (auto & t : p.ready_topics) {
            t = qualify_to_ns(t);
          }
          for (auto & s : p.ready_services) {
            s = qualify_to_ns(s);
          }
          processes_.push_back(p.command);
        }
        RCLCPP_INFO(
          get_logger(),
          "Loaded %zu processes from profile file '%s' (profile=%u)",
          processes_.size(),
          yaml_path.string().c_str(),
          chosen_profile);
      }
    }
  }

// 1b) Profile-defined FSM transitions.
//
// These are specified under `profile.transitions` in the per-state profile YAML.
// If the transitions map is missing, defaults preserve legacy behavior:
//   - auto_transition_on: false
//   - fault_transition_on: true (to FAULT profile 400)
if (!profiles_file_.empty()) {
  const uint16_t chosen_profile =
    (active_profile_id_ != 0) ? active_profile_id_ : default_profile_id_;

  std::filesystem::path yaml_path = profiles_file_;
  if (yaml_path.is_relative()) {
    const auto share_dir = ament_index_cpp::get_package_share_directory("amr_sweeper_fsm");
    yaml_path = std::filesystem::path(share_dir) / yaml_path;
  }

  ProfileTransitions tr;
  std::string err;
  if (!load_profile_transitions(yaml_path.string(), chosen_profile, tr, err)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to load profile transitions from file '%s' (profile=%u): %s",
      yaml_path.string().c_str(),
      chosen_profile,
      err.c_str());
  } else {
    auto_transition_on_ = tr.auto_transition_on;
    auto_transition_profile_ = tr.auto_transition_profile;
    fault_transition_on_ = tr.fault_transition_on;
    fault_transition_profile_ = tr.fault_transition_profile;

    std::ostringstream transitions_message;
    transitions_message
      << "Loaded profile " << std::setw(3) << std::setfill('0') << chosen_profile
      << " transitions: auto_on=" << (auto_transition_on_ ? "true" : "false")
      << " auto_profile=" << auto_transition_profile_
      << " fault_on=" << (fault_transition_on_ ? "true" : "false")
      << " fault_profile=" << fault_transition_profile_;

    const std::string colored_message = format_green_message(transitions_message.str());
    RCLCPP_INFO(
      get_logger(),
      "%s",
      colored_message.c_str());
  }
}


  // 2) ROSOUT fault triggers.
  //
  // Each line is parsed by parse_trigger_line(). Example:
  //   "node=battery_node;match=overcurrent;level>=ERROR;action=FAULT"
  {
    rclcpp::Parameter p;
    if (this->get_parameter("faults.rosout_triggers", p)) {
      const auto lines = p.get_value<std::vector<std::string>>();
      rosout_triggers_.clear();
      rosout_triggers_.reserve(lines.size());
      for (const auto & l : lines) {
        rosout_triggers_.push_back(parse_trigger_line(l));
      }
    } else {
      const auto lines =
        this->declare_parameter<std::vector<std::string>>(
          "faults.rosout_triggers", std::vector<std::string>{});

      rosout_triggers_.clear();
      rosout_triggers_.reserve(lines.size());
      for (const auto & l : lines) {
        rosout_triggers_.push_back(parse_trigger_line(l));
      }
    }

	    // Merge per-process `rosout_triggers` from the selected profile YAML (additive).
    //
    // This is intentionally additive: parameter-defined triggers are always kept,
    // and profile triggers are appended (deduplicated by their raw string).
    if (!profiles_file_.empty()) {
      std::filesystem::path yaml_path = profiles_file_;
      if (yaml_path.is_relative()) {
        const auto share_dir = ament_index_cpp::get_package_share_directory("amr_sweeper_fsm");
        yaml_path = std::filesystem::path(share_dir) / yaml_path;
      }

      const uint16_t chosen_profile =
        (active_profile_id_ > 0) ? active_profile_id_ : default_profile_id_;

      std::vector<TriggerLine> trigger_lines;
      std::string err;
      if (load_profile_rosout_triggers(yaml_path.string(), chosen_profile, trigger_lines, err)) {

        std::set<std::string> seen;
        for (const auto & t : rosout_triggers_) {
          seen.insert(t.raw);
        }

        size_t appended = 0;
        for (const auto & tl : trigger_lines) {
          if (seen.insert(tl.line).second) {
            auto trig = parse_trigger_line(tl.line);
            trig.has_source_importance = true;
            trig.source_importance = tl.importance;
            rosout_triggers_.push_back(trig);
            ++appended;
          }
        }

        if (appended > 0) {
          RCLCPP_INFO(
            get_logger(),
            "Loaded %zu rosout triggers from profile file '%s' (profile=%u); total=%zu",
            appended,
            yaml_path.string().c_str(),
            chosen_profile,
            rosout_triggers_.size());
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Failed to load rosout triggers from profile file '%s' (profile=%u): %s",
          yaml_path.string().c_str(),
          chosen_profile,
          err.c_str());
      }
    }

    // Log all registered triggers explicitly so it's obvious what's active.
    for (const auto & t : rosout_triggers_) {
      RCLCPP_INFO(get_logger(), "rosout trigger enabled: %s", t.raw.c_str());
    }

    // Enable /rosout monitoring starting in CONFIGURING and continuing into INACTIVE/ACTIVE.
    if (!rosout_sub_) {
      rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        rclcpp::QoS(50),
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) { this->on_rosout(msg); });
    }

    // Reset guard preventing repeated FAULT requests
    fault_requested_ = false;

  }

  // 3) Bring-up readiness gating.
  load_readiness_spec();

  // Profile-defined readiness requirements are used both during staged startup and final gating.


  RCLCPP_INFO(
    get_logger(),
    "Configured state node. processes=%zu, rosout_triggers=%zu",
    processes_.size(),
    rosout_triggers_.size());

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_activate(const rclcpp_lifecycle::State &)
{
  // Reset guard preventing repeated FAULT requests
  fault_requested_ = false;

  // Ensure /rosout subscription exists (it should be created in on_configure()).
  if (!rosout_sub_) {
    rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout",
      rclcpp::QoS(50),
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) { this->on_rosout(msg); });
  }

  // Start external processes/launch files associated with this state.
  std::string process_start_why;
  if (!start_state_processes(process_start_why)) {
    RCLCPP_ERROR(get_logger(), "Process startup failed: %s", process_start_why.c_str());

    if (!profile_processes_.empty()) {
      const std::string prefix = "profile process '";
      const std::string mid = "' not ready:";
      const auto p0 = process_start_why.find(prefix);
      const auto p1 = process_start_why.find(mid);
      if (p0 != std::string::npos && p1 != std::string::npos && p1 > (p0 + prefix.size())) {
        const std::string key =
          process_start_why.substr(p0 + prefix.size(), p1 - (p0 + prefix.size()));
        const auto * pp = find_profile_process_by_name_or_command_(key);
        if (pp) {
          handle_profile_error_policy_(*pp, "readiness_fail", process_start_why);
        }
      }
    }

    stop_state_processes();
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  start_process_monitoring_();

  // Optionally block until configured readiness requirements are satisfied.
  // This ensures the supervisor's ACTIVATE call only returns once the state
  // bring-up is actually present in the ROS graph.
  std::string why;
  if (!wait_for_readiness(why)) {
    RCLCPP_ERROR(get_logger(), "Ready timeout: %s", why.c_str());

    // If the readiness failure corresponds to a profile process, honor its `errors` policy.
    // This is best-effort; the activation will still fail to keep supervisor logic consistent.
    if (!profile_processes_.empty()) {
      const std::string prefix = "profile process '";
      const std::string mid = "' not ready:";
      const auto p0 = why.find(prefix);
      const auto p1 = why.find(mid);
      if (p0 != std::string::npos && p1 != std::string::npos && p1 > (p0 + prefix.size())) {
        const std::string key = why.substr(p0 + prefix.size(), p1 - (p0 + prefix.size()));
        const auto * pp = find_profile_process_by_name_or_command_(key);
        if (pp) {
          handle_profile_error_policy_(*pp, "readiness_fail", why);
        }
      }
    }

    // Best-effort cleanup of anything we started.
    stop_state_processes();
    // Mark activation as failed (supervisor will also observe failure).
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }


// Optional: request an automatic FSM transition once this state is ACTIVE.
if (auto_transition_on_) {
  const std::string target_state = target_state_from_profile_id(auto_transition_profile_);
  if (target_state.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "auto_transition_on is true but auto_transition_profile=%u is out of known profile bands; ignoring",
      auto_transition_profile_);
  } else if (!request_state_client_) {
    RCLCPP_WARN(get_logger(), "auto_transition_on is true but no RequestState client exists; ignoring");
  } else if (!request_state_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(),
      "auto_transition_on is true but supervisor RequestState service '%s' is not ready; ignoring",
      request_state_service_.c_str());
  } else {
    auto req = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
    req->target_state = target_state;
    req->target_lifecycle.clear();
    req->target_profile_id = auto_transition_profile_;

    // Requester identity: "<namespace>/<node_name>" (same convention as fault requests).
    const std::string ns = this->get_namespace();
    const std::string name = this->get_name();
    if (ns.empty() || ns == "/") {
      req->requester = "/" + name;
    } else if (ns.back() == '/') {
      req->requester = ns + name;
    } else {
      req->requester = ns + "/" + name;
    }

    req->priority = 250;
    req->force = true;
    req->reason = "auto_transition";

    (void)request_state_client_->async_send_request(req);
  }
}

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Keep /rosout monitoring enabled in DEACTIVATING and in the resulting INACTIVE state.
  stop_process_monitoring_();

  // Stop external processes for the state (best-effort).
  stop_state_processes();

  // Reset guard preventing repeated FAULT requests
  fault_requested_ = false;

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Cleanup should leave the node in a re-configurable (UNCONFIGURED) state.
  // Keep /rosout monitoring during CLEANUP transition itself, then disable before returning.
  stop_process_monitoring_();
  stop_state_processes();
  rosout_sub_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_shutdown(const rclcpp_lifecycle::State &)
{
  // no triggers in ShuttingDown/Finalized
  rosout_sub_.reset();     
  stop_process_monitoring_();
  stop_state_processes();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
StateNodeBase::on_error(const rclcpp_lifecycle::State &)
{
  // no triggers in ErrorProcessing
  rosout_sub_.reset();     
  stop_process_monitoring_();
  stop_state_processes();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ---------- Ready gating ----------

std::string StateNodeBase::qualify_to_ns(const std::string & maybe_relative) const
{
  if (maybe_relative.empty()) {
    return maybe_relative;
  }
  if (maybe_relative.front() == '/') {
    return maybe_relative;  // already fully qualified
  }

  // Qualify relative names into this node's namespace.
  const std::string ns = this->get_namespace();
  if (ns.empty() || ns == "/") {
    return std::string("/") + maybe_relative;
  }
  return ns + "/" + maybe_relative;
}

void StateNodeBase::load_readiness_spec()
{
  // Note: In ROS 2 parameter YAML, an *empty* sequence like "nodes: []" has no element to infer type from.
  // rclcpp may then present the parameter as NOT_SET even though it exists in the YAML.
  // Treat NOT_SET as "empty list / default" for readiness fields.

auto read_string_array = [this](const std::string & name) -> std::vector<std::string> {
  // If the parameter hasn't been declared yet, declaring it will try to apply YAML overrides.
  // Unfortunately, an empty list "[]" can arrive as PARAMETER_NOT_SET, which causes declare_parameter
  // to throw. We therefore inspect overrides first and ignore NOT_SET overrides.
  const auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  const auto it = overrides.find(name);

  const bool has_override = (it != overrides.end());
  const bool override_not_set = has_override &&
    (it->second.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET);

  // Declare with an empty default. If the override is NOT_SET, ignore it to avoid exceptions.
  if (!this->has_parameter(name)) {
    this->declare_parameter<std::vector<std::string>>(
      name, std::vector<std::string>{},
      rcl_interfaces::msg::ParameterDescriptor{}, override_not_set);
  }

  rclcpp::Parameter p;
  if (!this->get_parameter(name, p)) {
    return {};
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return p.as_string_array();
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return {};
  }

  RCLCPP_WARN(this->get_logger(),
    "Parameter '%s' has unexpected type (%d); treating as empty list",
    name.c_str(), static_cast<int>(p.get_type()));
  return {};
};

auto read_int = [this](const std::string & name, int default_value) -> int {
  const auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  const auto it = overrides.find(name);

  const bool has_override = (it != overrides.end());
  const bool override_not_set = has_override &&
    (it->second.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET);

  if (!this->has_parameter(name)) {
    this->declare_parameter<int>(
      name, default_value,
      rcl_interfaces::msg::ParameterDescriptor{}, override_not_set);
  }

  rclcpp::Parameter p;
  if (!this->get_parameter(name, p)) {
    return default_value;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return p.as_int();
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return default_value;
  }

  RCLCPP_WARN(this->get_logger(),
    "Parameter '%s' has unexpected type (%d); using default (%d)",
    name.c_str(), static_cast<int>(p.get_type()), default_value);
  return default_value;
};

  readiness_.nodes = read_string_array("ready.nodes");

  // Parse lifecycle readiness rules.
  {
    const auto lines = read_string_array("ready.lifecycle_nodes");
    readiness_.lifecycle_nodes.clear();
    readiness_.lifecycle_nodes.reserve(lines.size());
    for (const auto & l : lines) {
      if (l.empty()) {
        continue;
      }
      readiness_.lifecycle_nodes.push_back(parse_lifecycle_requirement_line(l));
    }
  }

  readiness_.topics = read_string_array("ready.topics");
  readiness_.services = read_string_array("ready.services");

  // Qualify relative topic/service names into this node's namespace.
  for (auto & t : readiness_.topics) {
    t = qualify_to_ns(t);
  }
  for (auto & s : readiness_.services) {
    s = qualify_to_ns(s);
  }

  readiness_.timeout_ms = read_int("ready.timeout_ms", 0);
  readiness_.controller_query_timeout_ms = read_int("ready.controller_query_timeout_ms", 5000);
  readiness_.lifecycle_query_timeout_ms = read_int("ready.lifecycle_query_timeout_ms", 200);
}

bool StateNodeBase::graph_has_node(const std::string & node_name)
{
  const auto graph = this->get_node_graph_interface();
  if (!graph) {
    return false;
  }

  const auto names_and_ns = graph->get_node_names_and_namespaces();
  for (const auto & nn : names_and_ns) {
    const std::string fq = (nn.second == "/") ? ("/" + nn.first) : (nn.second + "/" + nn.first);
    if (fq == node_name) {
      return true;
    }
    // Allow suffix match so users can write short names in YAML.
    if (ends_with(fq, node_name) || ends_with(nn.first, node_name)) {
      return true;
    }
  }

  return false;
}


bool StateNodeBase::graph_has_topic(const std::string & topic_name)
{
  const auto graph = this->get_node_graph_interface();
  if (!graph) {
    return false;
  }

  const auto names_and_types = graph->get_topic_names_and_types();
  for (const auto & nt : names_and_types) {
    if (nt.first == topic_name) {
      return true;
    }
  }
  return false;
}

bool StateNodeBase::graph_has_service(const std::string & service_name)
{
  const auto graph = this->get_node_graph_interface();
  if (!graph) {
    return false;
  }

  const auto names_and_types = graph->get_service_names_and_types();
  for (const auto & nt : names_and_types) {
    if (nt.first == service_name) {
      return true;
    }
  }
  return false;
}

bool StateNodeBase::controller_is_active(const std::string & controller_name, std::string & why_not)
{
  const std::string srv = qualify_to_ns("controller_manager/list_controllers");

  // Make probe node name unique to avoid rosout publisher collisions if checks repeat/overlap.
  const uint64_t id = g_probe_seq.fetch_add(1, std::memory_order_relaxed);
  const std::string probe_name = "fsm_cprobe_" + std::to_string(id);
  auto probe = std::make_shared<rclcpp::Node>(
    probe_name,
    rclcpp::NodeOptions()
    .context(this->get_node_base_interface()->get_context())
    .enable_rosout(false));

  auto client = probe->create_client<controller_manager_msgs::srv::ListControllers>(srv);
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    why_not = "controller manager service not available: '" + srv + "'";
    return false;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = client->async_send_request(req);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(probe);
  const auto rc = exec.spin_until_future_complete(
    future,
    std::chrono::milliseconds(readiness_.controller_query_timeout_ms));
  exec.remove_node(probe);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    why_not = "controller manager did not answer list_controllers for '" + controller_name + "'";
    return false;
  }

  const auto resp = future.get();
  for (const auto & controller : resp->controller) {
    if (controller.name == controller_name) {
      if (controller.state == "active") {
        why_not.clear();
        return true;
      }
      why_not =
        "controller '" + controller_name + "' state is '" + controller.state + "' (expected 'active')";
      return false;
    }
  }

  why_not = "controller '" + controller_name + "' not listed by controller_manager";
  return false;
}

bool StateNodeBase::hardware_component_meets_requirement(
  const HardwareComponentRequirement & req,
  std::string & why_not)
{
  const std::string srv = qualify_to_ns("controller_manager/list_hardware_components");

  const uint64_t id = g_probe_seq.fetch_add(1, std::memory_order_relaxed);
  const std::string probe_name = "fsm_hprobe_" + std::to_string(id);
  auto probe = std::make_shared<rclcpp::Node>(
    probe_name,
    rclcpp::NodeOptions()
    .context(this->get_node_base_interface()->get_context())
    .enable_rosout(false));

  auto client = probe->create_client<controller_manager_msgs::srv::ListHardwareComponents>(srv);
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    why_not = "controller manager hardware service not available: '" + srv + "'";
    return false;
  }

  auto req_msg = std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>();
  auto future = client->async_send_request(req_msg);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(probe);
  const auto rc = exec.spin_until_future_complete(
    future,
    std::chrono::milliseconds(readiness_.controller_query_timeout_ms));
  exec.remove_node(probe);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    why_not = "controller manager did not answer list_hardware_components for '" + req.name + "'";
    return false;
  }

  const auto resp = future.get();
  for (const auto & component : resp->component) {
    if (component.name != req.name) {
      continue;
    }
    if (component.state.id == req.required_state_id) {
      why_not.clear();
      return true;
    }
    why_not =
      "hardware component '" + req.name + "' state is '" + component.state.label +
      "' (expected id " + std::to_string(req.required_state_id) + ")";
    return false;
  }

  why_not = "hardware component '" + req.name + "' not listed by controller_manager";
  return false;
}


bool StateNodeBase::lifecycle_node_meets_requirement(
  const LifecycleNodeRequirement & req,
  std::string & why_not)
{
  const std::string node_fq = qualify_to_ns(req.node);
  const std::string srv = node_fq + "/get_state";

  // Make probe node name unique to avoid rosout publisher collisions if checks repeat/overlap.
  const uint64_t id = g_probe_seq.fetch_add(1, std::memory_order_relaxed);
  const std::string probe_name = "fsm_rprobe_" + std::to_string(id);
  auto probe = std::make_shared<rclcpp::Node>(
    probe_name,
    rclcpp::NodeOptions()
    .context(this->get_node_base_interface()->get_context())
    .enable_rosout(false));

  auto client = probe->create_client<lifecycle_msgs::srv::GetState>(srv);

  // 1) Service must exist.
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    why_not = "lifecycle service not available: '" + srv + "'";
    return false;
  }

  // 2) Service must report a state >= required minimum.
  auto req_msg = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = client->async_send_request(req_msg);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(probe);

  const auto rc = exec.spin_until_future_complete(
    future,
    std::chrono::milliseconds(readiness_.lifecycle_query_timeout_ms));
  exec.remove_node(probe);

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    why_not = "lifecycle get_state timed out: '" + node_fq + "'";
    return false;
  }

  const auto resp = future.get();
  if (!resp) {
    why_not = "lifecycle get_state returned null: '" + node_fq + "'";
    return false;
  }

  const uint8_t cur = resp->current_state.id;
  if (cur < req.min_state_id) {
    why_not =
      "not in required lifecycle state: '" + node_fq + "' "
      "(state=" + std::to_string(cur) + " (" + resp->current_state.label + ")" +
      ", required>=" + std::to_string(req.min_state_id) + ")";
    return false;
  }

  return true;
}



bool StateNodeBase::wait_for_readiness(std::string & why_not)
{
  const bool has_global_reqs =
    !(readiness_.nodes.empty() &&
      readiness_.topics.empty() &&
      readiness_.services.empty() &&
      readiness_.lifecycle_nodes.empty());

  const bool global_gating_enabled = (has_global_reqs && readiness_.timeout_ms > 0);

  bool has_profile_critical_reqs = false;
  for (const auto & pp : profile_processes_) {
    if (pp.importance == ProcessImportance::CRITICAL &&
        pp.window_ms > 0 &&
        (!pp.ready_nodes.empty() ||
        !pp.ready_topics.empty() ||
        !pp.ready_services.empty() ||
        !pp.ready_active_controllers.empty() ||
        !pp.ready_hardware_components.empty() ||
        !pp.ready_lifecycle_nodes.empty())) {
      has_profile_critical_reqs = true;
      break;
    }
  }

  // Disabled if neither global nor profile-based critical readiness requirements are active.
  if (!global_gating_enabled && !has_profile_critical_reqs) {
    return true;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto global_deadline = start + std::chrono::milliseconds(readiness_.timeout_ms);

  std::vector<std::chrono::steady_clock::time_point> proc_deadlines;
  proc_deadlines.reserve(profile_processes_.size());
  for (const auto & pp : profile_processes_) {
    proc_deadlines.push_back(start + std::chrono::milliseconds(pp.window_ms));
  }

  rclcpp::WallRate rate(10.0);

  while (rclcpp::ok()) {
    bool waiting = false;
    const auto now = std::chrono::steady_clock::now();

    // 1) Global readiness spec (parameter-driven)
    if (global_gating_enabled) {
      for (const auto & n : readiness_.nodes) {
        if (!graph_has_node(n)) {
          why_not = "node not discovered: '" + n + "'";
          if (now >= global_deadline) {
            why_not =
              "timed out after " + std::to_string(readiness_.timeout_ms) +
              " ms waiting for " + why_not;
            return false;
          }
          waiting = true;
        }
      }

      for (const auto & t : readiness_.topics) {
        if (!graph_has_topic(t)) {
          why_not = "topic not discovered: '" + t + "'";
          if (now >= global_deadline) {
            why_not =
              "timed out after " + std::to_string(readiness_.timeout_ms) +
              " ms waiting for " + why_not;
            return false;
          }
          waiting = true;
        }
      }

      for (const auto & s : readiness_.services) {
        if (!graph_has_service(s)) {
          why_not = "service not discovered: '" + s + "'";
          if (now >= global_deadline) {
            why_not =
              "timed out after " + std::to_string(readiness_.timeout_ms) +
              " ms waiting for " + why_not;
            return false;
          }
          waiting = true;
        }
      }

      for (const auto & lreq : readiness_.lifecycle_nodes) {
        std::string lwhy;
        if (!lifecycle_node_meets_requirement(lreq, lwhy)) {
          why_not = lwhy;
          if (now >= global_deadline) {
            why_not =
              "timed out after " + std::to_string(readiness_.timeout_ms) +
              " ms waiting for " + why_not;
            return false;
          }
          waiting = true;
        }
      }
    }

    // 2) Profile-defined readiness requirements (startup.ready) with per-process windows.
    for (size_t i = 0; i < profile_processes_.size(); ++i) {
      const auto & pp = profile_processes_[i];
      const auto proc_deadline = proc_deadlines[i];
      const auto resolved_command = resolve_placeholders(pp.command);

      auto missing_reason_for = [&](const std::string & what, const std::string & target) {
        const std::string pname = pp.name.empty() ? pp.command : pp.name;
        return "profile process '" + pname + "' not ready: missing " + what + " '" + target + "'";
      };

      bool proc_waiting = false;
      for (const auto & n : pp.ready_nodes) {
        if (!is_profile_requirement_enabled(resolved_command, "node", n)) {
          continue;
        }
        if (!graph_has_node(qualify_to_ns(n))) {
          proc_waiting = true;
          why_not = missing_reason_for("node", qualify_to_ns(n));
          break;
        }
      }

      if (!proc_waiting) {
        for (const auto & t : pp.ready_topics) {
          if (!is_profile_requirement_enabled(resolved_command, "topic", t)) {
            continue;
          }
          if (!graph_has_topic(t)) {
            proc_waiting = true;
            why_not = missing_reason_for("topic", t);
            break;
          }
        }
      }
      if (!proc_waiting) {
        for (const auto & s : pp.ready_services) {
          if (!is_profile_requirement_enabled(resolved_command, "service", s)) {
            continue;
          }
          if (!graph_has_service(s)) {
            proc_waiting = true;
            why_not = missing_reason_for("service", s);
            break;
          }
        }
      }

      if (!proc_waiting) {
        for (const auto & lreq : pp.ready_lifecycle_nodes) {
          if (!is_profile_requirement_enabled(resolved_command, "lifecycle", lreq.node)) {
            continue;
          }
          std::string lwhy;
          if (!lifecycle_node_meets_requirement(lreq, lwhy)) {
            proc_waiting = true;
            const std::string pname = pp.name.empty() ? pp.command : pp.name;
            why_not = "profile process '" + pname + "' not ready: " + lwhy;
            break;
          }
        }
      }

      if (!proc_waiting) {
        for (const auto & hreq : pp.ready_hardware_components) {
          if (!is_profile_requirement_enabled(resolved_command, "hardware", hreq.name)) {
            continue;
          }
          std::string hwhy;
          if (!hardware_component_meets_requirement(hreq, hwhy)) {
            proc_waiting = true;
            const std::string pname = pp.name.empty() ? pp.command : pp.name;
            why_not = "profile process '" + pname + "' not ready: " + hwhy;
            break;
          }
        }
      }

      if (!proc_waiting) {
        for (const auto & c : pp.ready_active_controllers) {
          if (!is_profile_requirement_enabled(resolved_command, "controller", c)) {
            continue;
          }
          std::string cwhy;
          if (!controller_is_active(c, cwhy)) {
            proc_waiting = true;
            const std::string pname = pp.name.empty() ? pp.command : pp.name;
            why_not = "profile process '" + pname + "' not ready: " + cwhy;
            break;
          }
        }
      }

      if (!proc_waiting) {
        continue;
      }

      if (pp.importance == ProcessImportance::CRITICAL) {
        if (pp.window_ms > 0 && now >= proc_deadline) {
          const auto failures = collect_profile_process_readiness_failures_(pp);
          if (!failures.empty()) {
            const std::string pname = pp.name.empty() ? pp.command : pp.name;
            std::string failure_list;
            for (size_t fi = 0; fi < failures.size(); ++fi) {
              if (fi > 0) {
                failure_list += "; ";
              }
              failure_list += failures[fi];
            }
            why_not =
              "timed out after " + std::to_string(pp.window_ms) + " ms waiting for profile process '" +
              pname + "' not ready: " + failure_list;
          } else {
            why_not = "timed out after " + std::to_string(pp.window_ms) + " ms waiting for " + why_not;
          }
          return false;
        }
        waiting = true;
      } else {
        // DEGRADED/OPTIONAL: do not block activation; log once when window elapses.
        if (pp.window_ms > 0 && now >= proc_deadline) {
          const std::string key = pp.name.empty() ? pp.command : pp.name;
          if (warned_noncritical_readiness_.insert(key).second) {
            RCLCPP_WARN(get_logger(), "%s", why_not.c_str());
          }
        }
      }
    }

    if (!waiting) {
      why_not.clear();
      return true;
    }

    rate.sleep();
  }

  why_not = "rclcpp shutdown";
  return false;
}


// ---------- Profile process error policies ----------

const fsm_layer_0::ProfileProcess * fsm_layer_0::StateNodeBase::find_profile_process_by_name_or_command_(
  const std::string & key) const
{
  for (const auto & pp : profile_processes_) {
    if (!pp.name.empty() && pp.name == key) {
      return &pp;
    }
    // Fall back to command (unresolved placeholder) match.
    if (pp.command == key) {
      return &pp;
    }
    // Also allow matching the resolved command for convenience.
    if (resolve_placeholders(pp.command) == key) {
      return &pp;
    }
  }
  return nullptr;
}

void fsm_layer_0::StateNodeBase::handle_profile_error_policy_(
  const ProfileProcess & pp,
  const std::string & event,
  const std::string & reason)
{
  if (fault_requested_) {
    return;
  }

  std::string action;
  if (event == "readiness_fail") {
    action = pp.errors.on_readiness_fail;
  } else if (event == "unexpected_exit") {
    action = pp.errors.on_unexpected_exit;
  }

  // If no per-process action is configured, fall back to the global fault behavior.
  if (action.empty()) {
    fault_requested_ = true;
    handle_fault_action(reason);
    return;
  }

  // Validate RequestState client exists.
  if (!request_state_client_) {
    RCLCPP_ERROR(get_logger(), "No RequestState client; cannot request transition for %s", event.c_str());
    return;
  }
  if (!request_state_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_ERROR(
      get_logger(),
      "Supervisor RequestState service '%s' not available; cannot request transition for %s",
      request_state_service_.c_str(),
      event.c_str());
    return;
  }

  // Determine target profile id.
  uint16_t target_profile_id = 0;
  if (pp.errors.target_profile_id >= 0) {
    target_profile_id = static_cast<uint16_t>(std::clamp(pp.errors.target_profile_id, 0, 65535));
  } else if (action == "FAULT") {
    // Default: FAULT action uses the profile-defined FAULT transition profile for this state/profile.
    target_profile_id = fault_transition_profile_;
  }

  // Priority and force defaults follow the existing state-level fault parameters.
  int prio = pp.errors.priority;
  if (prio < 0) {
    prio = static_cast<int>(fault_priority_);
  }
  prio = std::clamp(prio, 0, 255);

  const bool force = pp.errors.has_force ? pp.errors.force : fault_force_;
  const std::string lifecycle_target = !pp.errors.lifecycle_target.empty()
    ? pp.errors.lifecycle_target
    : fault_lifecycle_target_;

  auto req = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  req->target_state = action;
  req->target_lifecycle = lifecycle_target;
  req->target_profile_id = target_profile_id;

  // Requester identity: "<namespace>/<node_name>"
  const std::string ns = this->get_namespace();
  const std::string name = this->get_name();
  if (ns.empty() || ns == "/") {
    req->requester = "/" + name;
  } else if (ns.back() == '/') {
    req->requester = ns + name;
  } else {
    req->requester = ns + "/" + name;
  }

  req->priority = static_cast<uint8_t>(prio);
  req->force = force;
  req->reason = event + ": " + reason;

  fault_requested_ = true;
  (void)request_state_client_->async_send_request(req);
}

// ---------- Fault handling ----------

void StateNodeBase::handle_fault_action(const std::string & reason)
{
  RCLCPP_ERROR(this->get_logger(), "Fault requested: %s", reason.c_str());

  // If fault transitions are disabled for this profile, stop here.
  if (!fault_transition_on_) {
    RCLCPP_WARN(get_logger(), "fault_transition_on is false for this profile; not requesting a state change");
    return;
  }

  // Logic block: validate RequestState client exists.
  if (!request_state_client_) {
    RCLCPP_ERROR(get_logger(), "No RequestState client; cannot request FAULT");
    return;
  }

  // Logic block: validate supervisor service is available.
  if (!request_state_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_ERROR(
      get_logger(),
      "Supervisor RequestState service '%s' not available; cannot request FAULT",
      request_state_service_.c_str());
    return;
  }

  auto req = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();

  // Request a transition based on the profile's `transitions.fault_*` settings.
  const uint16_t target_profile = fault_transition_profile_;
  const std::string target_state = target_state_from_profile_id(target_profile);

  if (target_state.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "fault_transition_profile=%u is out of known profile bands; falling back to FAULT(400)",
      target_profile);
    req->target_state = "FAULT";
    req->target_lifecycle = fault_lifecycle_target_;
    req->target_profile_id = 400;
  } else {
    req->target_state = target_state;
    req->target_lifecycle = fault_lifecycle_target_;
    req->target_profile_id = target_profile;
  }

  // Logic block: build requester identity as "<namespace>/<node_name>".
  const std::string ns = this->get_namespace();
  const std::string name = this->get_name();
  std::string requester;
  if (ns.empty() || ns == "/") {
    requester = "/" + name;
  } else if (ns.back() == '/') {
    requester = ns + name;
  } else {
    requester = ns + "/" + name;
  }
  req->requester = requester;

  // Logic block: apply fault request metadata.
  req->priority = fault_priority_;
  req->force = fault_force_;
  req->reason = reason;

  (void)request_state_client_->async_send_request(req);
}


void StateNodeBase::on_rosout(const rcl_interfaces::msg::Log::SharedPtr msg)
{
  if (!msg || fault_requested_) {
    return;
  }

  // Ignore rosout triggers outside ACTIVE so shutdown/configure chatter from
  // child processes cannot escalate a state that is already leaving service.
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // Evaluate rules in order; first match wins.
  for (auto & t : rosout_triggers_) {
    // 1) Level gate.
    if (static_cast<int>(msg->level) < t.min_level) {
      continue;
    }

    // 2) Node name gate (optional).
    //
    // We allow suffix-match because logger names often include namespaces, e.g.:
    //   /amr_sweeper/battery_monitor
    // while the YAML may just use:
    //   battery_monitor
    if (t.has_node) {
      if (!(msg->name == t.node || ends_with(msg->name, t.node))) {
        continue;
      }
    }

    // 3) String match gate (optional).
    if (t.has_match) {
      if (msg->msg.find(t.match) == std::string::npos) {
        continue;
      }
    }

    // --- Rule matched ---
    if (t.action == RosoutAction::WARN_ONLY) {
      RCLCPP_WARN(
        get_logger(),
        "rosout WARN trigger matched: rule='%s' src='%s' msg='%s'",
        t.raw.c_str(),
        msg->name.c_str(),
        msg->msg.c_str());

      continue;
    }

    if (t.action == RosoutAction::DEGRADED) {
      RCLCPP_WARN(
        get_logger(),
        "rosout DEGRADED trigger matched: rule='%s' src='%s' msg='%s'",
        t.raw.c_str(),
        msg->name.c_str(),
        msg->msg.c_str());

      if (t.has_source_importance && t.source_importance == ProcessImportance::CRITICAL) {
        // Critical processes are not allowed to run with DEGRADED triggers.
        fault_requested_ = true;
        const std::string reason =
          "rosout trigger (critical->FAULT): " + t.raw + " | " + msg->name + ": " + msg->msg;
        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
        handle_fault_action(reason);
        return;
      }

      fault_requested_ = true;

      // Service is defined in amr_sweeper_fsm (see generated request_state.hpp)
      auto req = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
      req->target_state = "IDLING";
      req->target_lifecycle.clear();
      req->target_profile_id = 101;  // Default IDLING profile
      req->priority = 250;
      req->force = true;
      req->requester = "rosout_trigger";
      req->reason = "Rosout trigger matched: " + t.match;

      if (!request_state_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "request_state service not available; cannot request DEGRADED");
        continue;
      }

      (void)request_state_client_->async_send_request(req);
      continue;
    }

    // action == FAULT
    fault_requested_ = true;

    const std::string reason =
      "rosout trigger: " + t.raw + " | " + msg->name + ": " + msg->msg;

    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());

    // Delegate to derived state (e.g., call supervisor service to request FAULT).
    handle_fault_action(reason);
    return;
  }
}


// ---------- Per-process monitoring / restart ----------

void StateNodeBase::start_process_monitoring_()
{
  stop_process_monitoring_();

  monitored_processes_.clear();
  monitored_processes_.reserve(profile_processes_.empty() ? processes_.size() : profile_processes_.size());

  if (!profile_processes_.empty()) {
    for (const auto & pp : profile_processes_) {
      MonitoredProcess mp;
      mp.spec = pp;
      monitored_processes_.push_back(mp);
    }
  } else {
    // Backwards compatible: no profile metadata available. Treat all as CRITICAL with no restart policy.
    for (const auto & raw : processes_) {
      MonitoredProcess mp;
      mp.spec.command = raw;
      mp.spec.importance = ProcessImportance::CRITICAL;
      mp.spec.max_restarts = 0;
      mp.spec.restart_delay_ms = 0;
      monitored_processes_.push_back(mp);
    }
  }

  proc_monitor_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() { this->on_process_monitor_tick_(); });
}

void StateNodeBase::stop_process_monitoring_()
{
  proc_monitor_timer_.reset();
  monitored_processes_.clear();
}

void StateNodeBase::on_process_monitor_tick_()
{
  // Timer exists only while ACTIVE, but be defensive.
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  const rclcpp::Time now = this->now();

  for (auto & mp : monitored_processes_) {
    const std::string cmd = resolve_placeholders(mp.spec.command);

    if (procman_.is_running(cmd)) {
      continue;
    }

    // Process is not running -> unexpected exit.
    const bool can_restart = (mp.spec.max_restarts > 0) && (mp.restarts_done < mp.spec.max_restarts);

    if (!can_restart) {
      // No restarts configured or exhausted.
      const std::string reason =
        "Process exited unexpectedly: '" + (mp.spec.name.empty() ? cmd : mp.spec.name) + "' (cmd=" + cmd + ")";
      if (mp.spec.importance == ProcessImportance::OPTIONAL) {
        if (!mp.optional_exhausted_logged) {
          mp.optional_exhausted_logged = true;
          RCLCPP_WARN(get_logger(), "%s (optional; ignoring)", reason.c_str());
        }
        continue;
      }

      // CRITICAL and DEGRADED: honor per-process error policy (defaults to fault behavior).
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
      handle_profile_error_policy_(mp.spec, "unexpected_exit", reason);
      return;
    }

    // Restart with delay.
    if (!mp.has_next_restart_time) {
      mp.has_next_restart_time = true;
      mp.next_restart_time = now + rclcpp::Duration(std::chrono::milliseconds(mp.spec.restart_delay_ms));
      continue;
    }
    if (now < mp.next_restart_time) {
      continue;
    }

    std::string err;
    if (procman_.start(cmd, err)) {
      mp.restarts_done += 1;
      mp.has_next_restart_time = false;
      RCLCPP_WARN(
        get_logger(),
        "Restarted process '%s' (attempt %d/%d): %s",
        mp.spec.name.empty() ? cmd.c_str() : mp.spec.name.c_str(),
        mp.restarts_done,
        mp.spec.max_restarts,
        cmd.c_str());
    } else {
      // Failed to restart: schedule next attempt after delay.
      mp.next_restart_time = now + rclcpp::Duration(std::chrono::milliseconds(mp.spec.restart_delay_ms));
      RCLCPP_WARN(
        get_logger(),
        "Failed to restart process '%s': %s",
        mp.spec.name.empty() ? cmd.c_str() : mp.spec.name.c_str(),
        err.c_str());
    }
  }
}

// ---------- Process management ----------

std::string StateNodeBase::resolve_placeholders(std::string cmd) const
{
  auto append_runtime_overrides = [this, &cmd](const std::vector<std::string> & keys) {
    for (const auto & key : keys) {
      std::string value;
      this->get_parameter("runtime." + key, value);
      if (value.empty()) {
        continue;
      }
      cmd += " " + key + ":=" + value;
    }
  };
  auto replace_launch_argument_placeholder =
    [&cmd](const std::string & argument_name, const std::string & placeholder, const std::string & value) {
      if (value.empty()) {
        const std::string argument_token = argument_name + ":=" + placeholder;
        size_t argument_pos = 0;
        while ((argument_pos = cmd.find(argument_token, argument_pos)) != std::string::npos) {
          const bool has_leading_space = argument_pos > 0 && cmd.at(argument_pos - 1U) == ' ';
          const size_t erase_pos = has_leading_space ? argument_pos - 1U : argument_pos;
          const size_t erase_len = argument_token.size() + (has_leading_space ? 1U : 0U);
          cmd.erase(erase_pos, erase_len);
          argument_pos = erase_pos;
        }
        return;
      }

      size_t placeholder_pos = 0;
      while ((placeholder_pos = cmd.find(placeholder, placeholder_pos)) != std::string::npos) {
        cmd.replace(placeholder_pos, placeholder.size(), value);
        placeholder_pos += value.size();
      }
    };

  // `{ns}` resolves to namespace without leading slash. Root namespace -> "".
  std::string ns = this->get_namespace();
  if (!ns.empty() && ns.front() == '/') {
    ns.erase(0, 1);
  }

  const std::string key = "{ns}";
  size_t pos = 0;
  while ((pos = cmd.find(key, pos)) != std::string::npos) {
    cmd.replace(pos, key.size(), ns);
    pos += ns.size();
  }

  std::string mission_execution_directory;
  this->get_parameter("runtime.mission_execution_directory", mission_execution_directory);
  const std::string mission_key = "{mission_execution_directory}";
  replace_launch_argument_placeholder(
    "mission_execution_directory",
    mission_key,
    mission_execution_directory);

  bool use_test = false;
  this->get_parameter("runtime.use_test", use_test);
  const std::string use_test_value = use_test ? "true" : "false";
  const std::string use_test_key = "{use_test}";
  pos = 0;
  while ((pos = cmd.find(use_test_key, pos)) != std::string::npos) {
    cmd.replace(pos, use_test_key.size(), use_test_value);
    pos += use_test_value.size();
  }

  std::string test_output_directory;
  this->get_parameter("runtime.test_output_directory", test_output_directory);
  const std::string test_output_directory_key = "{test_output_directory}";
  pos = 0;
  while ((pos = cmd.find(test_output_directory_key, pos)) != std::string::npos) {
    cmd.replace(pos, test_output_directory_key.size(), test_output_directory);
    pos += test_output_directory.size();
  }

  if (cmd.find("ros2 launch amr_sweeper_layer_1_hardware_bringup ") != std::string::npos) {
    append_runtime_overrides(kLayer1BringupOverrideKeys);
  } else if (cmd.find("ros2 launch amr_sweeper_layer_2_controllers_bringup ") != std::string::npos) {
    append_runtime_overrides(kLayer2BringupOverrideKeys);
  } else if (cmd.find("ros2 launch amr_sweeper_layer_3_navigation_bringup ") != std::string::npos) {
    append_runtime_overrides(kLayer3BringupOverrideKeys);
  } else if (cmd.find("ros2 launch amr_sweeper_vda5050_parser ") != std::string::npos) {
    append_runtime_overrides(kParserOverrideKeys);
  } else if (cmd.find("ros2 launch amr_sweeper_scheduler ") != std::string::npos) {
    append_runtime_overrides(kSchedulerOverrideKeys);
  }

  return cmd;
}

bool StateNodeBase::profile_process_readiness_satisfied_(
  const ProfileProcess & pp,
  std::string & why_not)
{
  const auto resolved_command = resolve_placeholders(pp.command);
  auto missing_reason_for = [&](const std::string & what, const std::string & target) {
    const std::string pname = pp.name.empty() ? pp.command : pp.name;
    return "profile process '" + pname + "' not ready: missing " + what + " '" + target + "'";
  };

  for (const auto & n : pp.ready_nodes) {
    if (!is_profile_requirement_enabled(resolved_command, "node", n)) {
      continue;
    }
    const auto fq = qualify_to_ns(n);
    if (!graph_has_node(fq)) {
      why_not = missing_reason_for("node", fq);
      return false;
    }
  }

  for (const auto & t : pp.ready_topics) {
    if (!is_profile_requirement_enabled(resolved_command, "topic", t)) {
      continue;
    }
    if (!graph_has_topic(t)) {
      why_not = missing_reason_for("topic", t);
      return false;
    }
  }

  for (const auto & s : pp.ready_services) {
    if (!is_profile_requirement_enabled(resolved_command, "service", s)) {
      continue;
    }
    if (!graph_has_service(s)) {
      why_not = missing_reason_for("service", s);
      return false;
    }
  }

  for (const auto & lreq : pp.ready_lifecycle_nodes) {
    if (!is_profile_requirement_enabled(resolved_command, "lifecycle", lreq.node)) {
      continue;
    }
    if (!lifecycle_node_meets_requirement(lreq, why_not)) {
      const std::string pname = pp.name.empty() ? pp.command : pp.name;
      why_not = "profile process '" + pname + "' not ready: " + why_not;
      return false;
    }
  }

  for (const auto & hreq : pp.ready_hardware_components) {
    if (!is_profile_requirement_enabled(resolved_command, "hardware", hreq.name)) {
      continue;
    }
    if (!hardware_component_meets_requirement(hreq, why_not)) {
      const std::string pname = pp.name.empty() ? pp.command : pp.name;
      why_not = "profile process '" + pname + "' not ready: " + why_not;
      return false;
    }
  }

  for (const auto & c : pp.ready_active_controllers) {
    if (!is_profile_requirement_enabled(resolved_command, "controller", c)) {
      continue;
    }
    if (!controller_is_active(c, why_not)) {
      const std::string pname = pp.name.empty() ? pp.command : pp.name;
      why_not = "profile process '" + pname + "' not ready: " + why_not;
      return false;
    }
  }

  why_not.clear();
  return true;
}

std::vector<std::string> StateNodeBase::collect_profile_process_readiness_failures_(
  const ProfileProcess & pp)
{
  const auto resolved_command = resolve_placeholders(pp.command);
  std::vector<std::string> failures;
  failures.reserve(
    pp.ready_nodes.size() +
    pp.ready_topics.size() +
    pp.ready_services.size() +
    pp.ready_lifecycle_nodes.size() +
    pp.ready_hardware_components.size() +
    pp.ready_active_controllers.size());

  for (const auto & n : pp.ready_nodes) {
    if (!is_profile_requirement_enabled(resolved_command, "node", n)) {
      continue;
    }
    const auto fq = qualify_to_ns(n);
    if (!graph_has_node(fq)) {
      failures.push_back("missing node '" + fq + "'");
    }
  }

  for (const auto & t : pp.ready_topics) {
    if (!is_profile_requirement_enabled(resolved_command, "topic", t)) {
      continue;
    }
    if (!graph_has_topic(t)) {
      failures.push_back("missing topic '" + t + "'");
    }
  }

  for (const auto & s : pp.ready_services) {
    if (!is_profile_requirement_enabled(resolved_command, "service", s)) {
      continue;
    }
    if (!graph_has_service(s)) {
      failures.push_back("missing service '" + s + "'");
    }
  }

  for (const auto & lreq : pp.ready_lifecycle_nodes) {
    if (!is_profile_requirement_enabled(resolved_command, "lifecycle", lreq.node)) {
      continue;
    }
    std::string lwhy;
    if (!lifecycle_node_meets_requirement(lreq, lwhy)) {
      failures.push_back(lwhy);
    }
  }

  for (const auto & hreq : pp.ready_hardware_components) {
    if (!is_profile_requirement_enabled(resolved_command, "hardware", hreq.name)) {
      continue;
    }
    std::string hwhy;
    if (!hardware_component_meets_requirement(hreq, hwhy)) {
      failures.push_back(hwhy);
    }
  }

  for (const auto & c : pp.ready_active_controllers) {
    if (!is_profile_requirement_enabled(resolved_command, "controller", c)) {
      continue;
    }
    std::string cwhy;
    if (!controller_is_active(c, cwhy)) {
      failures.push_back(cwhy);
    }
  }

  return failures;
}

bool StateNodeBase::wait_for_profile_process_readiness_(
  const ProfileProcess & pp,
  std::string & why_not)
{
  if (
    pp.ready_nodes.empty() &&
    pp.ready_topics.empty() &&
    pp.ready_services.empty() &&
    pp.ready_lifecycle_nodes.empty() &&
    pp.ready_hardware_components.empty() &&
    pp.ready_active_controllers.empty())
  {
    why_not.clear();
    return true;
  }

  if (pp.window_ms <= 0) {
    why_not.clear();
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(pp.window_ms);
  rclcpp::WallRate rate(10.0);

  while (rclcpp::ok()) {
    if (profile_process_readiness_satisfied_(pp, why_not)) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      const std::string pname = pp.name.empty() ? pp.command : pp.name;
      const auto failures = collect_profile_process_readiness_failures_(pp);
      if (!failures.empty()) {
        std::string failure_list;
        for (size_t i = 0; i < failures.size(); ++i) {
          if (i > 0) {
            failure_list += "; ";
          }
          failure_list += failures[i];
        }
        why_not =
          "timed out after " + std::to_string(pp.window_ms) + " ms waiting for profile process '" +
          pname + "' not ready: " + failure_list;
      } else if (why_not.empty()) {
        why_not =
          "profile process '" + pname + "' not ready: timed out after " +
          std::to_string(pp.window_ms) + " ms";
      } else {
        why_not =
          "timed out after " + std::to_string(pp.window_ms) + " ms waiting for " + why_not;
      }
      return false;
    }
    rate.sleep();
  }

  why_not = "rclcpp shutdown";
  return false;
}

bool StateNodeBase::start_state_processes(std::string & why_not)
{
  why_not.clear();

  // Prefer profile metadata when available so startup can be gated in process order.
  if (!profile_processes_.empty()) {
    for (const auto & pp : profile_processes_) {
      const auto cmd = resolve_placeholders(pp.command);
      const std::string pname = pp.name.empty() ? cmd : pp.name;
      const bool has_process_readiness =
        !pp.ready_nodes.empty() ||
        !pp.ready_topics.empty() ||
        !pp.ready_services.empty() ||
        !pp.ready_lifecycle_nodes.empty() ||
        !pp.ready_hardware_components.empty() ||
        !pp.ready_active_controllers.empty();
      std::string err;

      if (!procman_.start(cmd, err)) {
        RCLCPP_WARN(get_logger(), "Failed to start command: '%s' (%s)", cmd.c_str(), err.c_str());
        if (pp.importance == ProcessImportance::CRITICAL) {
          why_not = "profile process '" + pname + "' failed to start: " + err;
          return false;
        }
        continue;
      }

      const auto started_message = format_started_message(cmd);
      RCLCPP_INFO(get_logger(), "%s", started_message.c_str());

      if (pp.importance != ProcessImportance::CRITICAL) {
        continue;
      }

      if (!wait_for_profile_process_readiness_(pp, why_not)) {
        return false;
      }

      if (has_process_readiness) {
        RCLCPP_INFO(
          get_logger(),
          "Critical profile process ready: %s",
          pname.c_str());
      }
      why_not.clear();
    }
    return true;
  }

  // Backwards compatible: no profile metadata, just start each command best-effort.
  for (const auto & raw : processes_) {
    const auto cmd = resolve_placeholders(raw);
    std::string err;

    if (!procman_.start(cmd, err)) {
      RCLCPP_WARN(get_logger(), "Failed to start command: '%s' (%s)", cmd.c_str(), err.c_str());
    } else {
      const auto started_message = format_started_message(cmd);
      RCLCPP_INFO(get_logger(), "%s", started_message.c_str());
    }
  }

  return true;
}

void StateNodeBase::stop_state_processes()
{
  // Stop is best-effort and intentionally ignores errors (common during teardown).
  // Prefer per-profile process specs so we can honor per-process shutdown policies.
  // Stop in reverse bring-up order so higher-level dependents exit before providers.
  if (!profile_processes_.empty()) {
    RCLCPP_INFO(get_logger(), "FSM state transition stopping proccesses");
    for (auto it = profile_processes_.rbegin(); it != profile_processes_.rend(); ++it) {
      const auto & pp = *it;
      const auto cmd = resolve_placeholders(pp.command);
      fsm_layer_0::ProcessManager::StopPolicy pol;

      if (pp.shutdown.sigint_timeout_ms > 0) {
        pol.sigint_timeout = std::chrono::milliseconds(pp.shutdown.sigint_timeout_ms);
      }
      if (pp.shutdown.sigterm_timeout_ms > 0) {
        pol.sigterm_timeout = std::chrono::milliseconds(pp.shutdown.sigterm_timeout_ms);
      }
      if (pp.shutdown.sigkill_timeout_ms > 0) {
        pol.sigkill_timeout = std::chrono::milliseconds(pp.shutdown.sigkill_timeout_ms);
      }

      std::string err;
      (void)procman_.stop(cmd, err, pol);
    }
    return;
  }

  // Backwards compatible: no profile metadata. Preserve the same reverse-order shutdown.
  RCLCPP_INFO(get_logger(), "FSM state transition stopping proccesses");
  for (auto it = processes_.rbegin(); it != processes_.rend(); ++it) {
    const auto cmd = resolve_placeholders(*it);
    std::string err;
    (void)procman_.stop(cmd, err);
  }
}


}  // namespace fsm_layer_0

// --- Ready parameter helpers ---
std::vector<std::string> fsm_layer_0::StateNodeBase::read_string_array_param_(const std::string & name) const
{
  // Note: an empty YAML list "[]" may be loaded by rclcpp as PARAMETER_NOT_SET.
  // Avoid calling as_string_array() unless the type is actually STRING_ARRAY.
  rclcpp::Parameter p;
  if (!this->get_parameter(name, p)) {
    return {};
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return p.as_string_array();
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return {};
  }
  // If the user misconfigures type, throw with a clear message.
  throw rclcpp::exceptions::InvalidParameterValueException(
    "Parameter '" + name + "' must be a string array (e.g. ['a','b'] or [])");
}

int64_t fsm_layer_0::StateNodeBase::read_int64_param_(const std::string & name, int64_t default_value) const
{
  rclcpp::Parameter p;
  if (!this->get_parameter(name, p)) {
    return default_value;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return p.as_int();
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return default_value;
  }
  throw rclcpp::exceptions::InvalidParameterValueException(
    "Parameter '" + name + "' must be an integer (ms), e.g. 0 or 5000");
}
