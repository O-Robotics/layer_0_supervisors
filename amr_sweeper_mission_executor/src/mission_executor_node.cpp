#include "mission_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <cmath>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace amr_sweeper_mission_executor
{

namespace
{

constexpr char kManualMappingExecutionMode[] = "manual_mapping";
constexpr char kTeleoperationExecutionMode[] = "teleoperation";
constexpr char kFollowWaypointsExecutionMode[] = "follow_waypoints";
constexpr char kBuiltinManualMappingMissionType[] = "builtin_manual_mapping";
constexpr char kBuiltinLocalPatternMissionType[] = "builtin_local_pattern";
constexpr char kBuiltinTeleopMissionType[] = "builtin_teleop";
constexpr char kDefaultMissionsPackageName[] = "amr_sweeper_default_missions";
constexpr char kRuntimeStatusStarted[] = "STARTED";
constexpr char kRuntimeStatusCompleted[] = "COMPLETED";
constexpr char kRuntimeStatusAborted[] = "ABORTED";
constexpr char kSafetyScheduleType[] = "SAFETY";
constexpr char kTeleopInactivityEndReason[] = "teleop mission auto-ended after 5 minutes without motion";
constexpr char kManualMappingInactivityEndReason[] =
  "manual mapping mission auto-ended after 5 minutes without motion";
constexpr char kScheduledMissionType[] = "vda5050_scheduled_mission";

std::string toLower(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

nlohmann::json loadJsonDocument(const std::filesystem::path & path)
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission file: " + path.string());
  }

  nlohmann::json document;
  input_stream >> document;
  return document;
}

std::string defaultIfEmpty(const std::string & value, const std::string & fallback)
{
  return value.empty() ? fallback : value;
}

double computePathLengthMeters(const nlohmann::json & route_document)
{
  double total_length_meters = 0.0;
  if (!route_document.contains("features") || !route_document.at("features").is_array()) {
    return total_length_meters;
  }

  for (const auto & feature : route_document.at("features")) {
    if (!feature.contains("geometry") || !feature.at("geometry").is_object()) {
      continue;
    }
    const auto & geometry = feature.at("geometry");
    if (!geometry.contains("type") || geometry.at("type") != "LineString" ||
      !geometry.contains("coordinates") || !geometry.at("coordinates").is_array())
    {
      continue;
    }

    const auto & coordinates = geometry.at("coordinates");
    for (std::size_t index = 1; index < coordinates.size(); ++index) {
      const auto & previous = coordinates.at(index - 1U);
      const auto & current = coordinates.at(index);
      if (!previous.is_array() || !current.is_array() || previous.size() < 2U || current.size() < 2U) {
        continue;
      }
      const double dx = current.at(0).get<double>() - previous.at(0).get<double>();
      const double dy = current.at(1).get<double>() - previous.at(1).get<double>();
      total_length_meters += std::hypot(dx, dy);
    }
  }

  return total_length_meters;
}

std::chrono::system_clock::time_point parseUtcTimestamp(const std::string & value)
{
  std::tm time_info{};
  std::istringstream stream(value);
  stream >> std::get_time(&time_info, "%Y%m%dT%H%M%SZ");
  if (stream.fail()) {
    throw std::runtime_error("Failed to parse UTC timestamp: " + value);
  }
#if defined(_WIN32)
  const std::time_t as_time_t = _mkgmtime(&time_info);
#else
  const std::time_t as_time_t = timegm(&time_info);
#endif
  return std::chrono::system_clock::from_time_t(as_time_t);
}

std::string sanitizeUidToken(std::string value)
{
  std::replace_if(
    value.begin(),
    value.end(),
    [](const char character) {
      return !(std::isalnum(static_cast<unsigned char>(character)) || character == '-' || character == '_');
    },
    '_');
  return value;
}

std::filesystem::path discoverNewestSchedulePath(const std::filesystem::path & missions_directory)
{
  std::optional<std::filesystem::path> selected_path;
  std::optional<std::filesystem::file_time_type> selected_stamp;
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return {};
  }

  for (const auto & entry : std::filesystem::directory_iterator(missions_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto filename = entry.path().filename().string();
    if (filename.rfind("schedule_", 0) != 0 || entry.path().extension() != ".ics") {
      continue;
    }
    const auto stamp = std::filesystem::last_write_time(entry.path());
    if (!selected_stamp || stamp > *selected_stamp) {
      selected_stamp = stamp;
      selected_path = entry.path();
    }
  }

  return selected_path.value_or(std::filesystem::path{});
}

std::string discoverScheduleTimezone(const std::string & schedule_text)
{
  {
    constexpr char calendar_timezone_prefix[] = "X-WR-TIMEZONE:";
    const auto position = schedule_text.find(calendar_timezone_prefix);
    if (position != std::string::npos) {
      const auto start = position + std::char_traits<char>::length(calendar_timezone_prefix);
      const auto end = schedule_text.find('\n', start);
      return schedule_text.substr(start, end - start);
    }
  }

  constexpr char dtstart_timezone_prefix[] = "DTSTART;TZID=";
  const auto dtstart_position = schedule_text.find(dtstart_timezone_prefix);
  if (dtstart_position != std::string::npos) {
    const auto start = dtstart_position + std::char_traits<char>::length(dtstart_timezone_prefix);
    const auto end = schedule_text.find(':', start);
    if (end != std::string::npos) {
      return schedule_text.substr(start, end - start);
    }
  }

  return "UTC";
}

}  // namespace

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_executor_node", options)
{
  missions_directory_ = declare_parameter<std::string>("missions_directory", "src/missions_from_db");
  missions_log_directory_ = declare_parameter<std::string>(
    "missions_log_directory",
    "src/missions_log");
  manual_missions_directory_ = declare_parameter<std::string>("manual_missions_directory", "");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  active_costmap_output_basename_ = declare_parameter<std::string>(
    "active_costmap_output_basename",
    "global_costmap");
  active_route_output_basename_ = declare_parameter<std::string>(
    "active_route_output_basename",
    "active_mission_path");
  active_execution_pointer_filename_ = declare_parameter<std::string>(
    "active_execution_pointer_filename",
    "active_execution.json");
  schedule_ics_path_ = declare_parameter<std::string>("schedule_ics_path", "");
  robot_id_ = declare_parameter<std::string>("robot_id", "RBT-01");
  safety_stop_topic_ = declare_parameter<std::string>("safety_stop_topic", "safety_msgs/stop");
  teleop_odometry_topic_ = declare_parameter<std::string>("teleop_odometry_topic", "diff_cont/odom");
  manual_mapping_odometry_topic_ = declare_parameter<std::string>(
    "manual_mapping_odometry_topic",
    "odometry/fused");
  mission_parser_node_name_ = declare_parameter<std::string>(
    "mission_parser_node_name",
    "vda5050_parser_node");
  mission_parser_build_service_ = declare_parameter<std::string>(
    "mission_parser_build_service",
    "build_current_mission");
  fsm_request_service_ = declare_parameter<std::string>("fsm_request_service", "request_state");
  manual_mission_inactivity_timeout_seconds_ = declare_parameter<double>(
    "manual_mission_inactivity_timeout_seconds",
    300.0);
  manual_mission_min_linear_speed_mps_ = declare_parameter<double>(
    "manual_mission_min_linear_speed_mps",
    0.01);
  manual_mission_min_angular_speed_rps_ = declare_parameter<double>(
    "manual_mission_min_angular_speed_rps",
    0.01);
  teleop_path_sample_distance_m_ = declare_parameter<double>("teleop_path_sample_distance_m", 0.1);
  idling_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("idling_profile_id", 101));
  scheduled_running_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("scheduled_running_profile_id", 201));
  manual_mapping_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_mapping_profile_id", 202));
  manual_routed_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_routed_profile_id", 203));
  manual_teleop_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_teleop_profile_id", 204));
  default_activation_priority_ = static_cast<std::uint8_t>(
    declare_parameter<int>("default_activation_priority", 200));

  client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mission_parser_parameter_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, mission_parser_node_name_);
  mission_parser_build_client_ = create_client<std_srvs::srv::Trigger>(
    mission_parser_build_service_,
    rclcpp::ServicesQoS(),
    client_callback_group_);
  fsm_request_client_ = create_client<amr_sweeper_fsm::srv::RequestState>(
    fsm_request_service_,
    rclcpp::ServicesQoS(),
    client_callback_group_);

  list_executable_missions_service_ = create_service<srv::ListExecutableMissions>(
    "list_executable_missions",
    std::bind(
      &MissionExecutorNode::handleListExecutableMissions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  list_manual_missions_service_ = create_service<srv::ListManualMissions>(
    "list_manual_missions",
    std::bind(
      &MissionExecutorNode::handleListManualMissions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  upload_vda5050_mission_service_ = create_service<srv::UploadVda5050Mission>(
    "upload_vda5050_mission",
    std::bind(
      &MissionExecutorNode::handleUploadVda5050Mission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  prepare_manual_mission_service_ = create_service<srv::PrepareManualMission>(
    "prepare_manual_mission",
    std::bind(
      &MissionExecutorNode::handlePrepareManualMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  execute_mission_service_ = create_service<srv::ExecuteMission>(
    "execute_mission",
    std::bind(
      &MissionExecutorNode::handleExecuteMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  end_mission_service_ = create_service<srv::EndMission>(
    "end_mission",
    std::bind(
      &MissionExecutorNode::handleEndMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  safety_stop_subscription_ = create_subscription<amr_sweeper_safety_msgs::msg::SafetyStop>(
    safety_stop_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleSafetyStop, this, std::placeholders::_1));
  teleop_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    teleop_odometry_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleManualMissionOdometry, this, std::placeholders::_1));
  manual_mapping_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    manual_mapping_odometry_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleManualMissionOdometry, this, std::placeholders::_1));
  manual_mission_watchdog_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&MissionExecutorNode::checkManualMissionInactivity, this));

  RCLCPP_INFO(
    get_logger(),
    "Mission executor watching %s and manual templates %s with profile mapping scheduled=%u manual_mapping=%u routed=%u teleop=%u teleop odometry %s and manual mapping odometry %s",
    missions_directory_.c_str(),
    resolveManualMissionsDirectory().string().c_str(),
    scheduled_running_profile_id_,
    manual_mapping_profile_id_,
    manual_routed_profile_id_,
    manual_teleop_profile_id_,
    teleop_odometry_topic_.c_str(),
    manual_mapping_odometry_topic_.c_str());
}

void MissionExecutorNode::handleListExecutableMissions(
  const std::shared_ptr<srv::ListExecutableMissions::Request>,
  std::shared_ptr<srv::ListExecutableMissions::Response> response)
{
  const auto missions = discoverManualMissions();
  response->success = true;
  response->message = "Executable missions listed";
  for (const auto & mission : missions) {
    response->mission_ids.push_back(mission.mission_id);
    response->mission_types.push_back(mission.mission_type);
    response->execution_modes.push_back(mission.execution_mode);
    response->running_profile_ids.push_back(mission.running_profile_id);
    response->is_manual.push_back(mission.is_manual);
    response->artifacts_ready.push_back(mission.artifacts_ready);
  }
}

void MissionExecutorNode::handleListManualMissions(
  const std::shared_ptr<srv::ListManualMissions::Request>,
  std::shared_ptr<srv::ListManualMissions::Response> response)
{
  const auto missions = discoverManualMissions();
  response->success = true;
  response->message = "Manual missions listed";
  for (const auto & mission : missions) {
    if (!mission.is_manual) {
      continue;
    }
    response->mission_ids.push_back(mission.mission_id);
    response->mission_types.push_back(mission.mission_type);
    response->execution_modes.push_back(mission.execution_mode);
    response->running_profile_ids.push_back(mission.running_profile_id);
  }
}

void MissionExecutorNode::handleUploadVda5050Mission(
  const std::shared_ptr<srv::UploadVda5050Mission::Request> request,
  std::shared_ptr<srv::UploadVda5050Mission::Response> response)
{
  if (request->mission_json.empty()) {
    response->success = false;
    response->message = "mission_json is required";
    return;
  }

  try {
    auto mission_document = nlohmann::json::parse(request->mission_json);
    if (!mission_document.is_object()) {
      throw std::runtime_error("mission_json must describe a JSON object");
    }

    const std::string mission_id = deriveMissionId(mission_document, request->mission_id);
    const auto missions_root = resolveMissionsFromDbDirectory();
    const auto mission_folder = missions_root / mission_id;
    const auto mission_file = mission_folder / (mission_id + mission_file_extension_);

    if ((std::filesystem::exists(mission_folder) || std::filesystem::exists(mission_file)) &&
      !request->overwrite_existing)
    {
      response->success = false;
      response->message = "Mission already exists for mission_id=" + mission_id;
      return;
    }

    std::filesystem::create_directories(mission_folder);

    if (mission_document.contains("mission_type") &&
      mission_document.at("mission_type").is_string() &&
      toLower(mission_document.at("mission_type").get<std::string>()) != kScheduledMissionType)
    {
      throw std::runtime_error("upload_vda5050_mission only accepts autonomous VDA5050 missions");
    }

    mission_document["mission_type"] = kScheduledMissionType;

    std::ofstream mission_stream(mission_file, std::ios::trunc);
    if (!mission_stream.is_open()) {
      throw std::runtime_error("Failed to write mission file: " + mission_file.string());
    }
    mission_stream << std::setw(2) << mission_document << '\n';

    // Clear stale generated artifacts so the parser rebuilds from the new VDA5050 payload on execution.
    std::filesystem::remove(mission_folder / (mission_id + "_costmap.yaml"));
    std::filesystem::remove(mission_folder / (mission_id + "_costmap.pgm"));
    std::filesystem::remove(mission_folder / (mission_id + "_path.geojson"));

    const auto mission = classifyMissionFile(mission_file);
    if (!mission) {
      throw std::runtime_error("Stored mission could not be classified");
    }

    response->success = true;
    response->message = "VDA5050 mission uploaded";
    response->mission_id = mission->mission_id;
    response->mission_file = mission->mission_path;
    response->mission_folder = mission_folder.string();
    response->mission_type = mission->mission_type;
    response->running_profile_id = mission->running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handlePrepareManualMission(
  const std::shared_ptr<srv::PrepareManualMission::Request> request,
  std::shared_ptr<srv::PrepareManualMission::Response> response)
{
  const auto mission = findManualMission(request->mission_id);
  if (!mission || !mission->is_manual) {
    response->success = false;
    response->message = "Manual mission not found for mission_id=" + request->mission_id;
    return;
  }

  try {
    if (!ensureMissionArtifactsReady(*mission)) {
      response->success = false;
      response->message = "Mission artifacts are not ready for mission_id=" + mission->mission_id;
      return;
    }
    const PreparedMissionContext context = prepareMissionArtifacts(*mission, "", "");
    response->success = true;
    response->message = "Manual mission prepared";
    response->mission_execution_directory = context.mission_execution_directory;
    response->execution_context_file = context.execution_context_file;
    response->running_profile_id = context.running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleExecuteMission(
  const std::shared_ptr<srv::ExecuteMission::Request> request,
  std::shared_ptr<srv::ExecuteMission::Response> response)
{
  const auto mission = findManualMission(request->mission_id);
  if (!mission) {
    response->success = false;
    response->message = "Mission not found for mission_id=" + request->mission_id;
    return;
  }

  try {
    const auto resolved_mission = *mission;
    PreparedMissionContext context;
    if (!request->mission_execution_directory.empty()) {
      context.mission_execution_directory = request->mission_execution_directory;
      context.execution_context_file =
        (std::filesystem::path(request->mission_execution_directory) / "execution_context.json").string();
      context.running_profile_id = resolved_mission.running_profile_id;
    } else {
      if (!ensureMissionArtifactsReady(resolved_mission)) {
        response->success = false;
        response->message = "Mission artifacts are not ready for mission_id=" + resolved_mission.mission_id;
        return;
      }
      context = prepareMissionArtifacts(
        resolved_mission,
        request->mission_window_start,
        request->mission_window_end);
    }
    std::string message;
    if (!requestRunningState(context, *request, message)) {
      response->success = false;
      response->message = message;
      response->mission_execution_directory = context.mission_execution_directory;
      response->execution_context_file = context.execution_context_file;
      response->running_profile_id = context.running_profile_id;
      return;
    }
    recordMissionExecutionStart(resolved_mission, context, *request);
    auto context_document = loadJsonDocument(context.execution_context_file);
    refreshActiveMissionState(context_document);

    response->success = true;
    response->message = message;
    response->mission_execution_directory = context.mission_execution_directory;
    response->execution_context_file = context.execution_context_file;
    response->running_profile_id = context.running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleEndMission(
  const std::shared_ptr<srv::EndMission::Request> request,
  std::shared_ptr<srv::EndMission::Response> response)
{
  try {
    auto context_document = resolveExecutionContext(request->mission_id);
    const std::string mission_run_directory = context_document ?
      context_document->value("mission_run_directory", std::string{}) : std::string{};
    const std::string execution_context_file = context_document ?
      context_document->value("execution_context_file", std::string{}) : std::string{};
    std::string finalization_message;
    if (!finalizeMissionExecution(*request, finalization_message, std::move(context_document))) {
      response->success = false;
      response->message = finalization_message;
      return;
    }

    response->success = true;
    response->message = finalization_message;
    response->mission_execution_directory = mission_run_directory;
    response->execution_context_file = execution_context_file;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleSafetyStop(
  const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr message)
{
  if (!message) {
    return;
  }
  try {
    recordSafetyEvent(*message, resolveExecutionContext(""));
  } catch (const std::exception & exception) {
    RCLCPP_WARN(get_logger(), "Failed to record safety stop event: %s", exception.what());
  }
}

void MissionExecutorNode::handleManualMissionOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::string actual_path_file;
  bool tracked_mission_active = false;
  bool should_write_path = false;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    tracked_mission_active = active_mission_running_ && active_mission_uses_inactivity_watchdog_;
    if (!tracked_mission_active) {
      return;
    }
    should_write_path = active_mission_is_teleop_;
    actual_path_file = active_actual_path_file_;
  }

  const auto & position = message->pose.pose.position;
  const double linear_speed = std::hypot(
    message->twist.twist.linear.x,
    message->twist.twist.linear.y);
  const double angular_speed = std::abs(message->twist.twist.angular.z);
  const bool moving =
    linear_speed >= manual_mission_min_linear_speed_mps_ ||
    angular_speed >= manual_mission_min_angular_speed_rps_;

  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (!active_mission_running_ || !active_mission_uses_inactivity_watchdog_) {
      return;
    }

    if (active_mission_is_teleop_ && teleop_traveled_path_points_.empty()) {
      geometry_msgs::msg::Point point;
      point.x = position.x;
      point.y = position.y;
      point.z = position.z;
      teleop_traveled_path_points_.push_back(point);
    } else if (active_mission_is_teleop_) {
      const auto & last_point = teleop_traveled_path_points_.back();
      const double dx = position.x - last_point.x;
      const double dy = position.y - last_point.y;
      if ((dx * dx + dy * dy) >= (teleop_path_sample_distance_m_ * teleop_path_sample_distance_m_)) {
        geometry_msgs::msg::Point point;
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
        teleop_traveled_path_points_.push_back(point);
      }
    }

    if (moving) {
      last_manual_mission_motion_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
        now() : rclcpp::Time(message->header.stamp);
    }
  }

  if (should_write_path && !actual_path_file.empty()) {
    nlohmann::json actual_path_document{
      {"type", "FeatureCollection"},
      {"features", nlohmann::json::array({
        {
          {"type", "Feature"},
          {"properties", {{"name", "actual_path"}, {"coordinate_frame", "odom"}}},
          {"geometry", {{"type", "LineString"}, {"coordinates", nlohmann::json::array()}}}
        }
      })}
    };

    nlohmann::json coordinates = nlohmann::json::array();
    {
      std::lock_guard<std::mutex> lock(active_mission_mutex_);
      for (const auto & point : teleop_traveled_path_points_) {
        coordinates.push_back({point.x, point.y});
      }
    }
    actual_path_document["features"][0]["geometry"]["coordinates"] = coordinates;

    std::ofstream output_stream(actual_path_file, std::ios::trunc);
    if (output_stream.is_open()) {
      output_stream << std::setw(2) << actual_path_document << '\n';
    }
  }
}

void MissionExecutorNode::checkManualMissionInactivity()
{
  srv::EndMission::Request request;
  std::string end_reason;
  std::string requester;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (!active_mission_running_ || !active_mission_uses_inactivity_watchdog_) {
      return;
    }
    if ((now() - last_manual_mission_motion_time_).seconds() < manual_mission_inactivity_timeout_seconds_) {
      return;
    }
    request.mission_id = active_mission_id_;
    if (active_mission_is_manual_mapping_) {
      end_reason = kManualMappingInactivityEndReason;
      requester = "manual_mapping_inactivity_watchdog";
    } else {
      end_reason = kTeleopInactivityEndReason;
      requester = "teleop_inactivity_watchdog";
    }
  }

  request.reason = end_reason;
  request.outcome = "completed";
  request.requester = requester;
  request.priority = default_activation_priority_;
  request.force = false;
  request.request_idling = true;

  std::string message;
  if (!finalizeMissionExecution(request, message)) {
    RCLCPP_WARN(get_logger(), "Failed to auto-end inactive manual mission: %s", message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

std::vector<ManualMissionInfo> MissionExecutorNode::discoverManualMissions() const
{
  std::vector<ManualMissionInfo> missions;
  std::set<std::string> seen_ids;
  auto maybe_add = [this, &missions, &seen_ids](const std::filesystem::path & candidate_path) {
      try {
      const auto mission = classifyMissionFile(candidate_path);
      if (!mission || !seen_ids.insert(mission->mission_id).second) {
        return;
      }
      missions.push_back(*mission);
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(),
          "Skipping mission candidate %s: %s",
          candidate_path.string().c_str(),
          exception.what());
      }
    };

  const auto scan_directory = [&maybe_add](const std::filesystem::path & directory) {
      if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
        return;
      }
      for (const auto & entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
          maybe_add(entry.path());
          continue;
        }
        if (!entry.is_directory()) {
          continue;
        }
        const std::filesystem::path canonical_nested_path =
          entry.path() / (entry.path().filename().string() + ".json");
        if (std::filesystem::exists(canonical_nested_path)) {
          maybe_add(canonical_nested_path);
        }
      }
    };

  scan_directory(resolveManualMissionsDirectory());
  scan_directory(resolveMissionsFromDbDirectory());

  std::sort(
    missions.begin(),
    missions.end(),
    [](const ManualMissionInfo & left, const ManualMissionInfo & right) {
      return left.mission_id < right.mission_id;
    });
  return missions;
}

std::optional<ManualMissionInfo> MissionExecutorNode::findManualMission(
  const std::string & mission_id) const
{
  const auto missions = discoverManualMissions();
  const auto it = std::find_if(
    missions.begin(),
    missions.end(),
    [&mission_id](const ManualMissionInfo & mission) {return mission.mission_id == mission_id;});
  if (it == missions.end()) {
    return std::nullopt;
  }
  return *it;
}

std::string MissionExecutorNode::sanitizeMissionId(const std::string & mission_id)
{
  std::string sanitized;
  sanitized.reserve(mission_id.size());
  for (const unsigned char character : mission_id) {
    if (std::isalnum(character) || character == '-' || character == '_') {
      sanitized.push_back(static_cast<char>(character));
      continue;
    }
    if (character == ' ' || character == '.' || character == '/') {
      sanitized.push_back('_');
    }
  }

  if (sanitized.empty()) {
    throw std::runtime_error("Mission id must contain at least one alphanumeric character");
  }
  return sanitized;
}

std::string MissionExecutorNode::deriveMissionId(
  const nlohmann::json & document,
  const std::string & requested_mission_id)
{
  if (!requested_mission_id.empty()) {
    return sanitizeMissionId(requested_mission_id);
  }

  if (document.contains("orderId") && document.at("orderId").is_string()) {
    return sanitizeMissionId(document.at("orderId").get<std::string>());
  }

  if (document.contains("name") && document.at("name").is_string()) {
    return sanitizeMissionId(document.at("name").get<std::string>());
  }

  throw std::runtime_error("Unable to derive mission_id from mission_json; provide mission_id explicitly");
}

std::filesystem::path MissionExecutorNode::resolvePath(const std::string & configured_path) const
{
  const std::filesystem::path configured(configured_path);
  if (configured.is_absolute()) {
    return configured;
  }

  const std::filesystem::path workspace_relative = std::filesystem::current_path() / configured;
  if (std::filesystem::exists(workspace_relative)) {
    return workspace_relative;
  }
  return configured;
}

std::filesystem::path MissionExecutorNode::resolveMissionsFromDbDirectory() const
{
  return resolvePath(missions_directory_);
}

std::filesystem::path MissionExecutorNode::resolveMissionsLogDirectory() const
{
  return resolvePath(missions_log_directory_);
}

std::filesystem::path MissionExecutorNode::resolveManualMissionsDirectory() const
{
  if (!manual_missions_directory_.empty()) {
    return resolvePath(manual_missions_directory_);
  }

  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory(kDefaultMissionsPackageName)) / "missions";
}

std::filesystem::path MissionExecutorNode::missionFolderPath(
  const std::filesystem::path & mission_path) const
{
  return mission_path.parent_path();
}

std::string MissionExecutorNode::missionStemForPath(const std::filesystem::path & mission_path) const
{
  if (mission_path.has_parent_path() && mission_path.parent_path() != resolveMissionsFromDbDirectory()) {
    return mission_path.parent_path().filename().string();
  }
  return mission_path.stem().string();
}

std::string MissionExecutorNode::missionCostmapBasename(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_costmap";
}

std::string MissionExecutorNode::missionRouteBasename(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_path";
}

std::filesystem::path MissionExecutorNode::missionHistoryDirectory(const ManualMissionInfo & mission) const
{
  return resolveMissionsLogDirectory() / mission.mission_id;
}

std::optional<ManualMissionInfo> MissionExecutorNode::classifyMissionFile(
  const std::filesystem::path & mission_path) const
{
  if (!std::filesystem::is_regular_file(mission_path) || mission_path.extension() != mission_file_extension_) {
    return std::nullopt;
  }

  const nlohmann::json document = loadJsonDocument(mission_path);
  ManualMissionInfo mission;
  mission.mission_id = missionStemForPath(mission_path);
  mission.mission_path = mission_path.string();
  mission.mission_type =
    document.contains("mission_type") && document.at("mission_type").is_string() ?
    document.at("mission_type").get<std::string>() :
    kScheduledMissionType;
  mission.execution_mode = kFollowWaypointsExecutionMode;

  if (document.contains("execution_mode") && document.at("execution_mode").is_string()) {
    mission.execution_mode = toLower(document.at("execution_mode").get<std::string>());
  } else if (toLower(mission.mission_type) == kBuiltinManualMappingMissionType) {
    mission.execution_mode = kManualMappingExecutionMode;
  } else if (toLower(mission.mission_type) == kBuiltinTeleopMissionType) {
    mission.execution_mode = kTeleoperationExecutionMode;
  }

  const std::string lowered_mission_type = toLower(mission.mission_type);
  if (mission.execution_mode == kManualMappingExecutionMode) {
    mission.running_profile_id = manual_mapping_profile_id_;
  } else if (mission.execution_mode == kTeleoperationExecutionMode) {
    mission.running_profile_id = manual_teleop_profile_id_;
  } else if (lowered_mission_type == kBuiltinLocalPatternMissionType) {
    mission.running_profile_id = manual_routed_profile_id_;
  } else {
    mission.running_profile_id = scheduled_running_profile_id_;
  }
  mission.is_manual = lowered_mission_type != kScheduledMissionType;
  mission.artifacts_ready = missionArtifactsReady(mission);
  return mission;
}

PreparedMissionContext MissionExecutorNode::prepareMissionArtifacts(
  const ManualMissionInfo & mission,
  const std::string & mission_window_start,
  const std::string & mission_window_end) const
{
  namespace fs = std::filesystem;
  const std::filesystem::path mission_file(mission.mission_path);
  const std::filesystem::path source_mission_folder = missionFolderPath(mission_file);
  const std::filesystem::path mission_costmap_yaml =
    source_mission_folder / (missionCostmapBasename(mission_file) + ".yaml");
  const std::filesystem::path mission_costmap_image =
    source_mission_folder / (missionCostmapBasename(mission_file) + ".pgm");
  const std::filesystem::path mission_route =
    source_mission_folder / (missionRouteBasename(mission_file) + ".geojson");

  if (!fs::exists(mission_file) ||
    !fs::exists(mission_costmap_yaml) ||
    !fs::exists(mission_costmap_image) ||
    !fs::exists(mission_route))
  {
    throw std::runtime_error("Manual mission artifacts are incomplete for mission_id=" + mission.mission_id);
  }

  const auto run_start_time = std::chrono::system_clock::now();
  const std::string run_timestamp = formatUtcTimestamp(run_start_time);
  const fs::path mission_history_directory = missionHistoryDirectory(mission);
  const fs::path mission_run_directory = mission_history_directory / run_timestamp;
  fs::create_directories(mission_run_directory);

  const fs::path history_mission_file = mission_history_directory / (mission.mission_id + mission_file_extension_);
  const fs::path history_costmap_yaml =
    mission_history_directory / (mission.mission_id + "_costmap.yaml");
  const fs::path history_costmap_image =
    mission_history_directory / (mission.mission_id + "_costmap.pgm");
  const fs::path history_route = mission_history_directory / (mission.mission_id + "_path.geojson");
  fs::create_directories(mission_history_directory);
  if (mission_file != history_mission_file) {
    fs::copy_file(mission_file, history_mission_file, fs::copy_options::overwrite_existing);
  }
  if (mission_costmap_yaml != history_costmap_yaml) {
    fs::copy_file(mission_costmap_yaml, history_costmap_yaml, fs::copy_options::overwrite_existing);
  }
  if (mission_costmap_image != history_costmap_image) {
    fs::copy_file(mission_costmap_image, history_costmap_image, fs::copy_options::overwrite_existing);
  }
  if (mission_route != history_route) {
    fs::copy_file(mission_route, history_route, fs::copy_options::overwrite_existing);
  }

  const fs::path run_mission_file = mission_run_directory / history_mission_file.filename();
  const fs::path run_costmap_yaml = mission_run_directory / history_costmap_yaml.filename();
  const fs::path run_costmap_image = mission_run_directory / history_costmap_image.filename();
  const fs::path run_route = mission_run_directory / history_route.filename();
  const fs::path actual_path_file = mission_run_directory / "actual_path.geojson";
  const fs::path gaussian_output_directory = mission_run_directory / "gaussian";
  const fs::path captured_images_directory = mission_run_directory / "captured_images";
  const fs::path collected_artifacts_directory = mission_run_directory / "artifacts";

  fs::copy_file(history_mission_file, run_mission_file, fs::copy_options::overwrite_existing);
  fs::copy_file(history_costmap_yaml, run_costmap_yaml, fs::copy_options::overwrite_existing);
  fs::copy_file(history_costmap_image, run_costmap_image, fs::copy_options::overwrite_existing);
  fs::copy_file(history_route, run_route, fs::copy_options::overwrite_existing);
  fs::create_directories(gaussian_output_directory);
  fs::create_directories(captured_images_directory);
  fs::create_directories(collected_artifacts_directory);

  {
    nlohmann::json actual_path_document{
      {"type", "FeatureCollection"},
      {"features", nlohmann::json::array({
        {
          {"type", "Feature"},
          {"properties", {{"name", "actual_path"}, {"coordinate_frame", "odom"}}},
          {"geometry", {{"type", "LineString"}, {"coordinates", nlohmann::json::array()}}}
        }
      })}
    };
    std::ofstream actual_path_stream(actual_path_file);
    if (!actual_path_stream.is_open()) {
      throw std::runtime_error("Failed to create actual path artifact for mission_id=" + mission.mission_id);
    }
    actual_path_stream << std::setw(2) << actual_path_document << '\n';
  }

  const nlohmann::json context{
    {"mission_id", mission.mission_id},
    {"mission_type", mission.mission_type},
    {"execution_mode", mission.execution_mode},
    {"mission_file", run_mission_file.string()},
    {"mission_folder", mission_history_directory.string()},
    {"mission_route_file", run_route.string()},
    {"mission_costmap_yaml", run_costmap_yaml.string()},
    {"mission_run_directory", mission_run_directory.string()},
    {"mission_window_start", mission_window_start},
    {"mission_window_end", mission_window_end},
    {"run_started_at", run_timestamp},
    {"source_mission_file", mission_file.string()},
    {"actual_path_file", actual_path_file.string()},
    {"gaussian_output_directory", gaussian_output_directory.string()},
    {"captured_images_directory", captured_images_directory.string()},
    {"collected_artifacts_directory", collected_artifacts_directory.string()},
    {"schedule_log_path", ensureScheduleLogPath(resolveScheduleSourcePath()).string()}};

  const fs::path execution_context_file = mission_run_directory / "execution_context.json";
  std::ofstream context_stream(execution_context_file);
  if (!context_stream.is_open()) {
    throw std::runtime_error("Failed to write manual mission execution context");
  }
  context_stream << std::setw(2) << context << '\n';

  const nlohmann::json execution_pointer{
    {"mission_id", mission.mission_id},
    {"mission_folder", mission_history_directory.string()},
    {"mission_run_directory", mission_run_directory.string()},
    {"execution_context_file", execution_context_file.string()},
    {"mission_window_start", mission_window_start},
    {"mission_window_end", mission_window_end}};
  const fs::path missions_log_directory = resolveMissionsLogDirectory();
  std::ofstream pointer_stream(
    missions_log_directory / active_execution_pointer_filename_,
    std::ios::trunc);
  if (!pointer_stream.is_open()) {
    throw std::runtime_error("Failed to write active manual mission execution pointer");
  }
  pointer_stream << std::setw(2) << execution_pointer << '\n';

  PreparedMissionContext prepared;
  prepared.mission_execution_directory = mission_run_directory.string();
  prepared.execution_context_file = execution_context_file.string();
  prepared.running_profile_id = mission.running_profile_id;
  return prepared;
}

std::filesystem::path MissionExecutorNode::activeExecutionPointerPath() const
{
  return resolveMissionsLogDirectory() / active_execution_pointer_filename_;
}

std::optional<nlohmann::json> MissionExecutorNode::loadActiveExecutionPointer() const
{
  const auto pointer_path = activeExecutionPointerPath();
  if (!std::filesystem::exists(pointer_path)) {
    return std::nullopt;
  }
  return loadJsonDocument(pointer_path);
}

std::optional<nlohmann::json> MissionExecutorNode::resolveExecutionContext(
  const std::string & mission_id) const
{
  const auto pointer_document = loadActiveExecutionPointer();
  if (!pointer_document) {
    return std::nullopt;
  }

  if (!mission_id.empty() &&
    pointer_document->contains("mission_id") &&
    pointer_document->at("mission_id").is_string() &&
    pointer_document->at("mission_id").get<std::string>() != mission_id)
  {
    return std::nullopt;
  }

  std::filesystem::path context_path;
  if (pointer_document->contains("execution_context_file") &&
    pointer_document->at("execution_context_file").is_string())
  {
    context_path = pointer_document->at("execution_context_file").get<std::string>();
  } else if (pointer_document->contains("mission_run_directory") &&
    pointer_document->at("mission_run_directory").is_string())
  {
    context_path = std::filesystem::path(
      pointer_document->at("mission_run_directory").get<std::string>()) / "execution_context.json";
  }

  if (context_path.empty() || !std::filesystem::exists(context_path)) {
    return std::nullopt;
  }

  auto context_document = loadJsonDocument(context_path);
  context_document["execution_context_file"] = context_path.string();
  return context_document;
}

bool MissionExecutorNode::requestIdlingState(
  const srv::EndMission::Request & request,
  std::string & message) const
{
  if (!fsm_request_client_->wait_for_service(std::chrono::seconds(5))) {
    message = "FSM request_state service is unavailable for end_mission";
    return false;
  }

  auto fsm_request = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  fsm_request->target_state = "IDLING";
  fsm_request->target_lifecycle = "Active";
  fsm_request->target_profile_id = idling_profile_id_;
  fsm_request->requester = defaultIfEmpty(request.requester, "mission_executor");
  fsm_request->priority = request.priority == 0U ? default_activation_priority_ : request.priority;
  fsm_request->force = request.force;
  fsm_request->reason = defaultIfEmpty(request.reason, "mission ended");
  fsm_request->mission_execution_directory = "";

  auto future = fsm_request_client_->async_send_request(fsm_request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    message = "Timed out waiting for FSM IDLING request response";
    return false;
  }

  const auto response = future.get();
  if (!response->accepted) {
    message = response->message;
    return false;
  }

  message = "Mission execution finalized and FSM requested IDLING";
  return true;
}

bool MissionExecutorNode::finalizeMissionExecution(
  const srv::EndMission::Request & request,
  std::string & message,
  std::optional<nlohmann::json> context_document)
{
  if (!context_document) {
    context_document = resolveExecutionContext(request.mission_id);
  }
  if (!context_document) {
    message = "No active mission execution context found";
    return false;
  }

  recordMissionExecutionEnd(*context_document, request);
  clearActiveMissionState();

  if (!requestIdlingState(request, message)) {
    return false;
  }

  return true;
}

void MissionExecutorNode::refreshActiveMissionState(const nlohmann::json & context_document)
{
  std::lock_guard<std::mutex> lock(active_mission_mutex_);
  active_mission_running_ = true;
  active_mission_id_ = context_document.value("mission_id", std::string{});
  active_execution_mode_ = context_document.value("execution_mode", std::string{});
  active_mission_is_teleop_ = toLower(active_execution_mode_) == kTeleoperationExecutionMode;
  active_mission_is_manual_mapping_ = toLower(active_execution_mode_) == kManualMappingExecutionMode;
  active_mission_uses_inactivity_watchdog_ = active_mission_is_teleop_ || active_mission_is_manual_mapping_;
  active_actual_path_file_ = context_document.value("actual_path_file", std::string{});
  teleop_traveled_path_points_.clear();
  last_manual_mission_motion_time_ = now();
}

void MissionExecutorNode::clearActiveMissionState()
{
  std::lock_guard<std::mutex> lock(active_mission_mutex_);
  active_mission_running_ = false;
  active_mission_is_teleop_ = false;
  active_mission_is_manual_mapping_ = false;
  active_mission_uses_inactivity_watchdog_ = false;
  active_mission_id_.clear();
  active_execution_mode_.clear();
  active_actual_path_file_.clear();
  teleop_traveled_path_points_.clear();
  last_manual_mission_motion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void MissionExecutorNode::recordMissionExecutionStart(
  const ManualMissionInfo & mission,
  const PreparedMissionContext & context,
  const srv::ExecuteMission::Request & request) const
{
  auto context_document = loadJsonDocument(context.execution_context_file);
  std::string schedule_path_string;
  if (context_document.contains("schedule_log_path") && context_document.at("schedule_log_path").is_string()) {
    schedule_path_string = context_document.at("schedule_log_path").get<std::string>();
  }
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();
  const std::string timezone = discoverScheduleTimezone(schedule_text);
  const auto now = std::chrono::system_clock::now();
  const std::string actual_start_utc = formatUtcTimestamp(now);
  const std::string actual_start_local = formatLocalTimestamp(now);
  std::string event_uid;

  if (!request.mission_window_start.empty()) {
    const std::string mission_tag = "X-MISSION-ID:" + mission.mission_id;
    const std::string start_tag = request.mission_window_start;
    const auto mission_position = schedule_text.find(mission_tag);
    if (mission_position != std::string::npos) {
      const auto event_begin = schedule_text.rfind("BEGIN:VEVENT", mission_position);
      const auto event_end = schedule_text.find("END:VEVENT", mission_position);
      if (event_begin != std::string::npos && event_end != std::string::npos &&
        schedule_text.find(start_tag, event_begin) != std::string::npos &&
        schedule_text.find(start_tag, event_begin) < event_end)
      {
        const auto uid_position = schedule_text.find("UID:", event_begin);
        if (uid_position != std::string::npos && uid_position < event_end) {
          const auto uid_end = schedule_text.find('\n', uid_position);
          event_uid = schedule_text.substr(uid_position + 4, uid_end - (uid_position + 4));
        }
        const std::string runtime_line = "X-ACTUAL-START-UTC:" + actual_start_utc + "\n";
        if (schedule_text.find(runtime_line, event_begin) == std::string::npos ||
          schedule_text.find(runtime_line, event_begin) > event_end)
        {
          schedule_text.insert(event_end, runtime_line);
        }
        const std::string status_line = std::string("X-RUNTIME-STATUS:") + kRuntimeStatusStarted + "\n";
        if (schedule_text.find(status_line, event_begin) == std::string::npos ||
          schedule_text.find(status_line, event_begin) > event_end)
        {
          schedule_text.insert(event_end, status_line);
        }
      }
    }
  } else {
    event_uid = "manual-" + sanitizeUidToken(mission.mission_id) + "-" + sanitizeUidToken(actual_start_utc);
    std::ostringstream event_stream;
    event_stream
      << "BEGIN:VEVENT\n"
      << "UID:" << event_uid << "\n"
      << "DTSTART;TZID=" << timezone << ":" << actual_start_local << "\n"
      << "DURATION:PT0S\n"
      << "SUMMARY:Manual mission execution " << mission.mission_id << "\n"
      << "X-ROBOT-ID:" << robot_id_ << "\n"
      << "X-SCHEDULE-TYPE:WORK\n"
      << "X-MISSION-ID:" << mission.mission_id << "\n"
      << "X-ACTUAL-START-UTC:" << actual_start_utc << "\n"
      << "X-RUNTIME-STATUS:" << kRuntimeStatusStarted << "\n"
      << "END:VEVENT\n";

    const auto calendar_end = schedule_text.rfind("END:VCALENDAR");
    if (calendar_end == std::string::npos) {
      return;
    }
    schedule_text.insert(calendar_end, event_stream.str());
  }

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;

  context_document["schedule_event_uid"] = event_uid;
  context_document["actual_start_utc"] = actual_start_utc;
  context_document["runtime_status"] = kRuntimeStatusStarted;
  std::ofstream context_stream(context.execution_context_file, std::ios::trunc);
  if (!context_stream.is_open()) {
    return;
  }
  context_stream << std::setw(2) << context_document << '\n';
}

void MissionExecutorNode::recordMissionExecutionEnd(
  nlohmann::json & context_document,
  const srv::EndMission::Request & request) const
{
  const auto now = std::chrono::system_clock::now();
  const std::string actual_end_utc = formatUtcTimestamp(now);
  const std::string normalized_outcome = defaultIfEmpty(request.outcome, "completed");
  const std::string runtime_status =
    toLower(normalized_outcome) == "completed" ? kRuntimeStatusCompleted : kRuntimeStatusAborted;

  double actual_duration_seconds = 0.0;
  if (context_document.contains("run_started_at") && context_document.at("run_started_at").is_string()) {
    const auto started = parseUtcTimestamp(context_document.at("run_started_at").get<std::string>());
    actual_duration_seconds = std::chrono::duration<double>(now - started).count();
  }

  double actual_path_length_meters = 0.0;
  if (context_document.contains("actual_path_file") && context_document.at("actual_path_file").is_string()) {
    const std::filesystem::path actual_path_file = context_document.at("actual_path_file").get<std::string>();
    if (std::filesystem::exists(actual_path_file)) {
      actual_path_length_meters = computePathLengthMeters(loadJsonDocument(actual_path_file));
    }
  }

  context_document["actual_end_utc"] = actual_end_utc;
  context_document["runtime_status"] = runtime_status;
  context_document["mission_outcome"] = normalized_outcome;
  context_document["end_reason"] = request.reason;
  context_document["actual_duration_seconds"] = actual_duration_seconds;
  context_document["actual_path_length_meters"] = actual_path_length_meters;

  const std::filesystem::path context_path(
    context_document.value("execution_context_file", std::string{}));
  if (!context_path.empty()) {
    std::ofstream context_stream(context_path, std::ios::trunc);
    if (!context_stream.is_open()) {
      throw std::runtime_error("Failed to update execution context during end_mission");
    }
    context_stream << std::setw(2) << context_document << '\n';
  }

  auto pointer_document = loadActiveExecutionPointer();
  if (pointer_document) {
    (*pointer_document)["actual_end_utc"] = actual_end_utc;
    (*pointer_document)["runtime_status"] = runtime_status;
    (*pointer_document)["mission_outcome"] = normalized_outcome;
    (*pointer_document)["end_reason"] = request.reason;
    (*pointer_document)["active"] = false;
    std::ofstream pointer_stream(activeExecutionPointerPath(), std::ios::trunc);
    if (pointer_stream.is_open()) {
      pointer_stream << std::setw(2) << *pointer_document << '\n';
    }
  }

  std::string schedule_path_string =
    context_document.value("schedule_log_path", std::string{});
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();

  const std::string event_uid = context_document.value("schedule_event_uid", std::string{});
  const std::string mission_id = context_document.value("mission_id", std::string{});
  const std::string mission_window_start = context_document.value("mission_window_start", std::string{});
  const auto event_anchor = !event_uid.empty() ? schedule_text.find("UID:" + event_uid) : std::string::npos;
  std::size_t event_begin = std::string::npos;
  std::size_t event_end = std::string::npos;
  if (event_anchor != std::string::npos) {
    event_begin = schedule_text.rfind("BEGIN:VEVENT", event_anchor);
    event_end = schedule_text.find("END:VEVENT", event_anchor);
  } else if (!mission_id.empty()) {
    const auto mission_anchor = schedule_text.find("X-MISSION-ID:" + mission_id);
    if (mission_anchor != std::string::npos) {
      event_begin = schedule_text.rfind("BEGIN:VEVENT", mission_anchor);
      event_end = schedule_text.find("END:VEVENT", mission_anchor);
      if (event_begin != std::string::npos && !mission_window_start.empty()) {
        const auto start_anchor = schedule_text.find(mission_window_start, event_begin);
        if (start_anchor == std::string::npos || start_anchor > event_end) {
          event_begin = std::string::npos;
          event_end = std::string::npos;
        }
      }
    }
  }

  if (event_begin == std::string::npos || event_end == std::string::npos) {
    return;
  }

  const auto insert_or_replace_line = [&schedule_text, event_begin, event_end](
      const std::string & prefix, const std::string & line) {
      const auto position = schedule_text.find(prefix, event_begin);
      if (position != std::string::npos && position < event_end) {
        const auto line_end = schedule_text.find('\n', position);
        schedule_text.replace(position, (line_end == std::string::npos ? event_end : line_end + 1) - position, line);
      } else {
        schedule_text.insert(event_end, line);
      }
    };

  insert_or_replace_line("X-ACTUAL-END-UTC:", "X-ACTUAL-END-UTC:" + actual_end_utc + "\n");
  insert_or_replace_line(
    "X-ACTUAL-DURATION-SECONDS:",
    "X-ACTUAL-DURATION-SECONDS:" + std::to_string(static_cast<long long>(actual_duration_seconds)) + "\n");
  insert_or_replace_line(
    "X-ACTUAL-PATH-LENGTH-METERS:",
    "X-ACTUAL-PATH-LENGTH-METERS:" + std::to_string(actual_path_length_meters) + "\n");
  insert_or_replace_line("X-RUNTIME-STATUS:", "X-RUNTIME-STATUS:" + runtime_status + "\n");
  insert_or_replace_line("X-END-REASON:", "X-END-REASON:" + request.reason + "\n");

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;
}

void MissionExecutorNode::recordSafetyEvent(
  const amr_sweeper_safety_msgs::msg::SafetyStop & event,
  const std::optional<nlohmann::json> & context_document) const
{
  std::string schedule_path_string;
  std::string related_mission_id;
  std::string mission_run_directory;
  if (context_document) {
    schedule_path_string = context_document->value("schedule_log_path", std::string{});
    related_mission_id = context_document->value("mission_id", std::string{});
    mission_run_directory = context_document->value("mission_run_directory", std::string{});
  }
  if (schedule_path_string.empty()) {
    const auto schedule_path = ensureScheduleLogPath(resolveScheduleSourcePath());
    schedule_path_string = schedule_path.string();
  }
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();
  const std::string timezone = discoverScheduleTimezone(schedule_text);

  rclcpp::Time event_time(event.stamp);
  const auto time_point = std::chrono::system_clock::time_point(std::chrono::nanoseconds(event_time.nanoseconds()));
  const std::string event_utc = formatUtcTimestamp(time_point);
  const std::string event_local = formatLocalTimestamp(time_point);

  std::ostringstream event_stream;
  event_stream
    << "BEGIN:VEVENT\n"
    << "UID:safety-" << sanitizeUidToken(event.sender) << "-" << sanitizeUidToken(event_utc) << "\n"
    << "DTSTART;TZID=" << timezone << ":" << event_local << "\n"
    << "DURATION:PT0S\n"
    << "SUMMARY:Safety stop " << event.sender << "\n"
    << "X-ROBOT-ID:" << robot_id_ << "\n"
    << "X-SCHEDULE-TYPE:" << kSafetyScheduleType << "\n"
    << "X-SAFETY-SENDER:" << event.sender << "\n"
    << "X-SAFETY-REASON:" << event.reason << "\n"
    << "X-ACTUAL-START-UTC:" << event_utc << "\n";
  if (!related_mission_id.empty()) {
    event_stream << "X-MISSION-ID:" << related_mission_id << "\n";
  }
  if (!mission_run_directory.empty()) {
    event_stream << "X-MISSION-RUN-DIRECTORY:" << mission_run_directory << "\n";
  }
  event_stream << "END:VEVENT\n";

  const auto calendar_end = schedule_text.rfind("END:VCALENDAR");
  if (calendar_end == std::string::npos) {
    return;
  }
  schedule_text.insert(calendar_end, event_stream.str());

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;
}

bool MissionExecutorNode::missionArtifactsReady(const ManualMissionInfo & mission) const
{
  const std::filesystem::path mission_file(mission.mission_path);
  const std::filesystem::path mission_folder = missionFolderPath(mission_file);
  return std::filesystem::exists(mission_file) &&
         std::filesystem::exists(mission_folder / (missionCostmapBasename(mission_file) + ".yaml")) &&
         std::filesystem::exists(mission_folder / (missionCostmapBasename(mission_file) + ".pgm")) &&
         std::filesystem::exists(mission_folder / (missionRouteBasename(mission_file) + ".geojson"));
}

bool MissionExecutorNode::ensureMissionArtifactsReady(const ManualMissionInfo & mission)
{
  if (missionArtifactsReady(mission)) {
    return true;
  }
  if (toLower(mission.mission_type) != kScheduledMissionType) {
    return false;
  }
  if (!mission_parser_parameter_client_->service_is_ready() ||
    !mission_parser_build_client_->service_is_ready())
  {
    RCLCPP_WARN(
      get_logger(),
      "VDA5050 mission artifacts are missing for %s but the mission builder is unavailable.",
      mission.mission_id.c_str());
    return false;
  }

  auto parameter_future = mission_parser_parameter_client_->set_parameters(
    {rclcpp::Parameter("mission_path", mission.mission_path)});
  if (parameter_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(),
      "Timed out setting mission_path on the VDA5050 mission builder for %s",
      mission.mission_id.c_str());
    return false;
  }

  for (const auto & result : parameter_future.get()) {
    if (!result.successful) {
      RCLCPP_WARN(
        get_logger(),
        "VDA5050 mission builder rejected mission_path for %s: %s",
        mission.mission_id.c_str(),
        result.reason.c_str());
      return false;
    }
  }

  auto build_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto build_future = mission_parser_build_client_->async_send_request(build_request);
  if (build_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(),
      "Timed out waiting for the VDA5050 mission builder while preparing %s",
      mission.mission_id.c_str());
    return false;
  }

  const auto build_response = build_future.get();
  if (!build_response->success) {
    RCLCPP_WARN(
      get_logger(),
      "VDA5050 mission builder failed for %s: %s",
      mission.mission_id.c_str(),
      build_response->message.c_str());
    return false;
  }

  return missionArtifactsReady(mission);
}

bool MissionExecutorNode::requestRunningState(
  const PreparedMissionContext & context,
  const srv::ExecuteMission::Request & request,
  std::string & message) const
{
  if (!fsm_request_client_->wait_for_service(std::chrono::seconds(5))) {
    message = "FSM request_state service is unavailable";
    return false;
  }

  auto fsm_request = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  fsm_request->target_state = "RUNNING";
  fsm_request->target_lifecycle = "Active";
  fsm_request->target_profile_id = context.running_profile_id;
  fsm_request->requester = defaultIfEmpty(request.requester, "mission_executor");
  fsm_request->priority = request.priority == 0U ? default_activation_priority_ : request.priority;
  fsm_request->force = request.force;
  fsm_request->reason = defaultIfEmpty(request.reason, "manual mission activation");
  fsm_request->mission_execution_directory = context.mission_execution_directory;

  auto future = fsm_request_client_->async_send_request(fsm_request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    message = "Timed out waiting for FSM RUNNING request response";
    return false;
  }

  const auto response = future.get();
  if (!response->accepted) {
    message = response->message;
    return false;
  }

  std::ostringstream stream;
  stream
    << "Requested FSM RUNNING profile " << context.running_profile_id
    << " for mission_id=" << request.mission_id;
  if (!response->message.empty()) {
    stream << " (" << response->message << ")";
  }
  message = stream.str();
  return true;
}

std::string MissionExecutorNode::formatUtcTimestamp(
  const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  std::tm time_info{};
#if defined(_WIN32)
  gmtime_s(&time_info, &as_time_t);
#else
  gmtime_r(&as_time_t, &time_info);
#endif
  std::ostringstream stream;
  stream << std::put_time(&time_info, "%Y%m%dT%H%M%SZ");
  return stream.str();
}

std::string MissionExecutorNode::formatLocalTimestamp(
  const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  std::tm time_info{};
#if defined(_WIN32)
  localtime_s(&time_info, &as_time_t);
#else
  localtime_r(&as_time_t, &time_info);
#endif
  std::ostringstream stream;
  stream << std::put_time(&time_info, "%Y%m%dT%H%M%S");
  return stream.str();
}

std::filesystem::path MissionExecutorNode::resolveScheduleSourcePath() const
{
  if (!schedule_ics_path_.empty()) {
    return resolvePath(schedule_ics_path_);
  }
  return discoverNewestSchedulePath(resolveMissionsFromDbDirectory());
}

std::filesystem::path MissionExecutorNode::ensureScheduleLogPath(
  const std::filesystem::path & schedule_source_path) const
{
  if (schedule_source_path.empty()) {
    return {};
  }

  const std::filesystem::path missions_log_directory = resolveMissionsLogDirectory();
  std::filesystem::create_directories(missions_log_directory);
  const std::filesystem::path schedule_log_path =
    missions_log_directory / schedule_source_path.filename();
  if (!std::filesystem::exists(schedule_log_path) && std::filesystem::exists(schedule_source_path)) {
    std::filesystem::copy_file(
      schedule_source_path,
      schedule_log_path,
      std::filesystem::copy_options::overwrite_existing);
  }
  return schedule_log_path;
}

}  // namespace amr_sweeper_mission_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_mission_executor::MissionExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
