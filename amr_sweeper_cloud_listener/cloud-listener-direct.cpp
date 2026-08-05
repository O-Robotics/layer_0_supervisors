#include "cloud-listener.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <vector>

#undef WORKING
#undef WARNING
#undef ERROR
#define WORKING 271
#define WARNING 371
#define ERROR "471"

namespace cloud_listener
{

namespace
{

constexpr const char * kCommandMethodName = "command";
constexpr const char * kLastCommandOutputFileName = "last_command.json";

constexpr const char * kClearActivePackageAction = "CLEAR_ACTIVE_PACKAGE";
constexpr const char * kStartSingleMissionAction = "START_SINGLE_MISSION";
constexpr const char * kPauseAction = "PAUSE";
constexpr const char * kResumeAction = "RESUME";
constexpr const char * kStopAction = "STOP";

constexpr const char * kPreservedMessageFileName = "message.json";
constexpr const char * kPreservedLastCommandFileName = "last_command.json";
constexpr const char * kPreservedScheduleFileName = "schedule_20260000T000000Z.ics";

constexpr const char * kExecuteMissionServiceName = "execute_mission";
constexpr const char * kExecuteMissionServiceType =
  "amr_sweeper_mission_executor/srv/ExecuteMission";
constexpr const char * kEndMissionServiceName = "end_mission";
constexpr const char * kEndMissionServiceType =
  "amr_sweeper_mission_executor/srv/EndMission";
constexpr const char * kClearSafetyStopServiceName =
  "amr_sweeper_safety_controller/clear_safety_stop";
constexpr const char * kClearSafetyStopServiceType = "std_srvs/srv/Trigger";
constexpr const char * kSafetyStopTopicName = "safety_msgs/stop";
constexpr const char * kSafetyStopTopicType = "amr_sweeper_safety_msgs/msg/SafetyStop";

struct PackageCommand
{
  std::string action;
  std::optional<std::string> mission_id;
  std::optional<std::string> mission_execution_directory;
  std::optional<std::string> mission_window_start;
  std::optional<std::string> mission_window_end;
  std::optional<std::string> requester;
  std::optional<std::string> sender;
  std::optional<std::string> reason;
  std::optional<std::string> outcome;
  std::optional<std::int64_t> priority;
  std::optional<bool> force;
  std::optional<bool> record_rosbag;
  std::optional<bool> request_idling;
};

std::optional<std::string> optional_json_string_field(
  const std::string & json_object,
  const char * key)
{
  if (extract_top_level_json_value(json_object, key).has_value()) {
    return require_json_string_field(json_object, key);
  }

  return std::nullopt;
}

std::optional<std::int64_t> optional_json_integer_field(
  const std::string & json_object,
  const char * key)
{
  if (!extract_top_level_json_value(json_object, key).has_value()) {
    return std::nullopt;
  }

  return require_json_integer_field(json_object, key);
}

std::optional<bool> optional_json_bool_field(
  const std::string & json_object,
  const char * key)
{
  const std::optional<std::string> raw_value = extract_top_level_json_value(json_object, key);

  if (!raw_value.has_value()) {
    return std::nullopt;
  }

  if (*raw_value == "true") {
    return true;
  }

  if (*raw_value == "false") {
    return false;
  }

  throw std::runtime_error(std::string(ERROR) + " payload." + key + " MUST BE BOOLEAN");
}

struct DirectMethodRequester
{
  std::string user_id;
  std::string client;
  std::string session_id;

  static DirectMethodRequester from_json(const std::string & json_object)
  {
    DirectMethodRequester requester;
    requester.user_id = require_json_string_field(json_object, "user_id");
    requester.client = require_json_string_field(json_object, "client");
    requester.session_id = require_json_string_field(json_object, "session_id");
    return requester;
  }
};

struct DirectMethodPayload
{
  std::string expected_package_id;
  std::int64_t expected_desired_version {};
  std::string reason;
  std::optional<std::string> mission_id;
  std::optional<std::string> mission_execution_directory;
  std::optional<std::string> mission_window_start;
  std::optional<std::string> mission_window_end;
  std::optional<std::string> requester;
  std::optional<std::string> sender;
  std::optional<std::string> outcome;
  std::optional<bool> force;
  std::optional<bool> record_rosbag;
  std::optional<bool> request_idling;

  static DirectMethodPayload from_json(const std::string & json_object)
  {
    DirectMethodPayload payload;
    payload.expected_package_id = require_json_string_field(json_object, "expected_package_id");
    payload.expected_desired_version =
      require_json_integer_field(json_object, "expected_desired_version");
    payload.reason = require_json_string_field(json_object, "reason");
    payload.mission_id = optional_json_string_field(json_object, "mission_id");
    payload.mission_execution_directory =
      optional_json_string_field(json_object, "mission_execution_directory");
    payload.mission_window_start =
      optional_json_string_field(json_object, "mission_window_start");
    payload.mission_window_end =
      optional_json_string_field(json_object, "mission_window_end");
    payload.requester = optional_json_string_field(json_object, "requester");
    payload.sender = optional_json_string_field(json_object, "sender");
    payload.outcome = optional_json_string_field(json_object, "outcome");
    payload.force = optional_json_bool_field(json_object, "force");
    payload.record_rosbag = optional_json_bool_field(json_object, "record_rosbag");
    payload.request_idling = optional_json_bool_field(json_object, "request_idling");
    return payload;
  }
};

struct DirectMethodCommand
{
  std::string schema;
  std::string cmd;
  std::string cmd_id;
  std::string robot_id;
  std::string issued_at;
  std::int64_t ttl_ms {};
  std::int64_t priority {};
  DirectMethodRequester requested_by;
  DirectMethodPayload payload;

  static DirectMethodCommand from_json(const std::string & json_object)
  {
    DirectMethodCommand command;
    command.schema = require_json_string_field(json_object, "schema");
    command.cmd = require_json_string_field(json_object, "cmd");
    command.cmd_id = require_json_string_field(json_object, "cmd_id");
    command.robot_id = require_json_string_field(json_object, "robot_id");
    command.issued_at = require_json_string_field(json_object, "issued_at");
    command.ttl_ms = require_json_integer_field(json_object, "ttl_ms");
    command.priority = require_json_integer_field(json_object, "priority");
    command.requested_by =
      DirectMethodRequester::from_json(require_json_object_field(json_object, "requested_by"));
    command.payload =
      DirectMethodPayload::from_json(require_json_object_field(json_object, "payload"));
    return command;
  }

  static DirectMethodCommand load_from_file(const std::filesystem::path & path)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      throw std::runtime_error(
        std::string(ERROR) + " FAILED TO OPEN DIRECT METHOD FILE: " + path.string());
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    try {
      return from_json(buffer.str());
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string(ERROR) + " FAILED TO PARSE DIRECT METHOD FILE '" +
        path.string() + "'\n" + ex.what());
    }
  }

  static std::optional<DirectMethodCommand> try_load_from_file_if_exists(
    const std::filesystem::path & path)
  {
    std::error_code exists_error;
    const bool exists = std::filesystem::exists(path, exists_error);
    if (exists_error) {
      throw std::runtime_error(
        std::string(ERROR) + " FAILED TO INSPECT DIRECT METHOD FILE: " + path.string());
    }

    if (!exists) {
      return std::nullopt;
    }

    return load_from_file(path);
  }
};

bool is_database_file_candidate_for_removal(const std::filesystem::path & path)
{
  const std::string extension = path.extension().string();
  return extension == ".ics" || extension == ".json";
}

bool is_database_file_preserved(const std::filesystem::path & path)
{
  const std::string file_name = path.filename().string();
  return file_name == kPreservedMessageFileName ||
         file_name == kPreservedLastCommandFileName ||
         file_name == kPreservedScheduleFileName;
}

void clear_active_package_database_files();

std::filesystem::path last_command_output_path()
{
  return manifest_output_path_from_home().parent_path() / kLastCommandOutputFileName;
}

std::string yaml_escape_double_quoted_string(const std::string & value)
{
  std::ostringstream escaped;

  for (unsigned char ch : value) {
    switch (ch) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        escaped << static_cast<char>(ch);
        break;
    }
  }

  return escaped.str();
}

std::string shell_single_quote(const std::string & value)
{
  std::string quoted = "'";

  for (char ch : value) {
    if (ch == '\'') {
      quoted += "'\"'\"'";
    } else {
      quoted.push_back(ch);
    }
  }

  quoted += "'";
  return quoted;
}

std::string yaml_string_field(const std::string & key, const std::string & value)
{
  return key + ": \"" + yaml_escape_double_quoted_string(value) + "\"";
}

std::string yaml_integer_field(const std::string & key, std::int64_t value)
{
  return key + ": " + std::to_string(value);
}

std::string yaml_bool_field(const std::string & key, bool value)
{
  return key + ": " + std::string(value ? "true" : "false");
}

std::string yaml_object_field(const std::string & key, const std::string & value)
{
  return key + ": " + value;
}

std::string join_yaml_object_fields(const std::vector<std::string> & fields)
{
  std::ostringstream output;
  output << "{";

  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) {
      output << ", ";
    }
    output << fields[i];
  }

  output << "}";
  return output.str();
}

bool run_shell_command(
  const std::string & command_line,
  std::string * output,
  std::string * error_message)
{
  const std::string command_with_stderr = command_line + " 2>&1";
  FILE * pipe = ::popen(command_with_stderr.c_str(), "r");
  if (pipe == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to start shell command";
    }
    return false;
  }

  std::string command_output;
  char buffer[512];
  while (::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    command_output.append(buffer);
  }

  const int close_status = ::pclose(pipe);
  if (output != nullptr) {
    *output = command_output;
  }

  if (close_status == -1) {
    if (error_message != nullptr) {
      *error_message = "failed to collect shell command exit status";
    }
    return false;
  }

  if (WIFEXITED(close_status) && WEXITSTATUS(close_status) == 0) {
    return true;
  }

  if (error_message != nullptr) {
    std::ostringstream formatted;
    formatted << "shell command failed";
    if (WIFEXITED(close_status)) {
      formatted << " with exit code " << WEXITSTATUS(close_status);
    }
    if (!command_output.empty()) {
      formatted << ": " << trim(command_output);
    }
    *error_message = formatted.str();
  }
  return false;
}

bool ros2_output_indicates_boolean_success(
  const std::string & command_output,
  const std::string & field_name,
  std::string * error_message)
{
  const std::vector<std::string> true_patterns = {
    field_name + "=True",
    field_name + "=true",
    field_name + ": true",
    field_name + ": True"
  };
  const std::vector<std::string> false_patterns = {
    field_name + "=False",
    field_name + "=false",
    field_name + ": false",
    field_name + ": False"
  };

  for (const std::string & pattern : true_patterns) {
    if (command_output.find(pattern) != std::string::npos) {
      return true;
    }
  }

  for (const std::string & pattern : false_patterns) {
    if (command_output.find(pattern) != std::string::npos) {
      if (error_message != nullptr) {
        *error_message = trim(command_output);
      }
      return false;
    }
  }

  if (error_message != nullptr) {
    *error_message =
      "unable to confirm ROS2 service result from output: " + trim(command_output);
  }
  return false;
}

bool call_ros2_service(
  const std::string & service_name,
  const std::string & service_type,
  const std::string & request_payload,
  const char * success_field_name,
  std::string * command_output,
  std::string * error_message)
{
  const std::string command_line =
    "timeout 20s ros2 service call " +
    service_name + " " + service_type + " " + shell_single_quote(request_payload);

  if (!run_shell_command(command_line, command_output, error_message)) {
    return false;
  }

  if (success_field_name != nullptr &&
      !ros2_output_indicates_boolean_success(*command_output, success_field_name, error_message)) {
    return false;
  }

  return true;
}

bool publish_ros2_message_once(
  const std::string & topic_name,
  const std::string & topic_type,
  const std::string & message_payload,
  std::string * command_output,
  std::string * error_message)
{
  const std::string command_line =
    "timeout 10s ros2 topic pub --once "
    "--qos-reliability reliable --qos-durability transient_local " +
    topic_name + " " + topic_type + " " + shell_single_quote(message_payload);

  return run_shell_command(command_line, command_output, error_message);
}

std::string current_ros_time_yaml_object()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);

  return "{sec: " + std::to_string(seconds.count()) +
         ", nanosec: " + std::to_string(nanoseconds.count()) + "}";
}

std::string direct_method_requester_identity(const DirectMethodRequester & requester)
{
  return requester.user_id + "@" + requester.client + "/" + requester.session_id;
}

std::string normalize_direct_method_command_name(const std::string & command_name)
{
  if (command_name == "clearActivePackage") {
    return kClearActivePackageAction;
  }

  if (command_name == "startSingleMission") {
    return kStartSingleMissionAction;
  }

  if (command_name == "pause") {
    return kPauseAction;
  }

  if (command_name == "resume") {
    return kResumeAction;
  }

  if (command_name == "stop") {
    return kStopAction;
  }

  return command_name;
}

PackageCommand build_package_command_from_direct_method(const DirectMethodCommand & direct_command)
{
  PackageCommand package_command;
  package_command.action = normalize_direct_method_command_name(direct_command.cmd);
  package_command.mission_id = direct_command.payload.mission_id;
  package_command.mission_execution_directory =
    direct_command.payload.mission_execution_directory;
  package_command.mission_window_start = direct_command.payload.mission_window_start;
  package_command.mission_window_end = direct_command.payload.mission_window_end;
  package_command.requester = direct_command.payload.requester.has_value()
    ? direct_command.payload.requester
    : std::optional<std::string>(direct_method_requester_identity(direct_command.requested_by));
  package_command.sender = direct_command.payload.sender.has_value()
    ? direct_command.payload.sender
    : std::optional<std::string>(direct_command.requested_by.client);
  package_command.reason = direct_command.payload.reason;
  package_command.outcome = direct_command.payload.outcome;
  package_command.priority = direct_command.priority;
  package_command.force = direct_command.payload.force;
  package_command.record_rosbag = direct_command.payload.record_rosbag;
  package_command.request_idling = direct_command.payload.request_idling;
  return package_command;
}

std::string build_execute_mission_request(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(yaml_string_field("mission_id", *command.mission_id));
  fields.push_back(yaml_string_field(
    "mission_execution_directory",
    command.mission_execution_directory.value_or("")));
  fields.push_back(yaml_string_field(
    "mission_window_start",
    command.mission_window_start.value_or("")));
  fields.push_back(yaml_string_field(
    "mission_window_end",
    command.mission_window_end.value_or("")));
  fields.push_back(yaml_string_field(
    "requester",
    command.requester.value_or("cloud-listener-direct")));
  fields.push_back(yaml_integer_field("priority", command.priority.value_or(200)));
  fields.push_back(yaml_bool_field("force", command.force.value_or(false)));
  fields.push_back(yaml_bool_field("record_rosbag", command.record_rosbag.value_or(false)));
  fields.push_back(yaml_string_field(
    "reason",
    command.reason.value_or("startSingleMission requested from Azure IoT Hub direct method")));
  return join_yaml_object_fields(fields);
}

std::string build_end_mission_request(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(yaml_string_field("mission_id", command.mission_id.value_or("")));
  fields.push_back(yaml_string_field(
    "reason",
    command.reason.value_or("stop requested from Azure IoT Hub direct method")));
  fields.push_back(yaml_string_field(
    "outcome",
    command.outcome.value_or("aborted")));
  fields.push_back(yaml_string_field(
    "requester",
    command.requester.value_or("cloud-listener-direct")));
  fields.push_back(yaml_integer_field("priority", command.priority.value_or(200)));
  fields.push_back(yaml_bool_field("force", command.force.value_or(false)));
  fields.push_back(yaml_bool_field("request_idling", command.request_idling.value_or(true)));
  return join_yaml_object_fields(fields);
}

std::string build_safety_stop_message(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(yaml_object_field("stamp", current_ros_time_yaml_object()));
  fields.push_back(yaml_string_field("sender", command.sender.value_or(
    command.requester.value_or("cloud-listener-direct"))));
  fields.push_back(yaml_string_field(
    "reason",
    command.reason.value_or("pause requested from Azure IoT Hub direct method")));
  return join_yaml_object_fields(fields);
}

void send_start_single_mission_command(const PackageCommand & command)
{
  if (!command.mission_id.has_value() || command.mission_id->empty()) {
    throw std::runtime_error(
      std::string(ERROR) + " startSingleMission REQUIRES payload.mission_id");
  }

  std::string command_output;
  std::string error_message;
  if (!call_ros2_service(
        kExecuteMissionServiceName,
        kExecuteMissionServiceType,
        build_execute_mission_request(command),
        "success",
        &command_output,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " START MISSION SERVICE SENT: "
            << *command.mission_id << std::endl;
  if (!command_output.empty()) {
    std::cout << WORKING << " ROS2 RESPONSE: " << trim(command_output) << std::endl;
  }
}

void send_stop_command(const PackageCommand & command)
{
  std::string command_output;
  std::string error_message;
  if (!call_ros2_service(
        kEndMissionServiceName,
        kEndMissionServiceType,
        build_end_mission_request(command),
        "success",
        &command_output,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " STOP MISSION SERVICE SENT" << std::endl;
  if (!command_output.empty()) {
    std::cout << WORKING << " ROS2 RESPONSE: " << trim(command_output) << std::endl;
  }
}

void send_pause_command(const PackageCommand & command)
{
  std::string command_output;
  std::string error_message;
  if (!publish_ros2_message_once(
        kSafetyStopTopicName,
        kSafetyStopTopicType,
        build_safety_stop_message(command),
        &command_output,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " SAFETY STOP MESSAGE SENT FOR PAUSE" << std::endl;
  if (!command_output.empty()) {
    std::cout << WORKING << " ROS2 OUTPUT: " << trim(command_output) << std::endl;
  }
}

void send_resume_command()
{
  std::string command_output;
  std::string error_message;
  if (!call_ros2_service(
        kClearSafetyStopServiceName,
        kClearSafetyStopServiceType,
        "{}",
        "success",
        &command_output,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " CLEAR SAFETY STOP SERVICE SENT FOR RESUME" << std::endl;
  if (!command_output.empty()) {
    std::cout << WORKING << " ROS2 RESPONSE: " << trim(command_output) << std::endl;
  }
}

bool handle_package_command(const PackageCommand & command)
{
  if (command.action == kClearActivePackageAction) {
    clear_active_package_database_files();
    return true;
  }

  if (command.action == kStartSingleMissionAction) {
    send_start_single_mission_command(command);
    return true;
  }

  if (command.action == kStopAction) {
    send_stop_command(command);
    return true;
  }

  if (command.action == kPauseAction) {
    send_pause_command(command);
    return true;
  }

  if (command.action == kResumeAction) {
    send_resume_command();
    return true;
  }

  std::cout << WARNING << " UNSUPPORTED DIRECT COMMAND: "
            << command.action << std::endl;
  return false;
}

void clear_active_package_database_files()
{
  const std::filesystem::path database_directory = manifest_output_path_from_home().parent_path();

  std::error_code exists_error;
  const bool database_exists = std::filesystem::exists(database_directory, exists_error);
  if (exists_error) {
    throw std::runtime_error(
      std::string(ERROR) + " FAILED TO INSPECT database DIRECTORY: " + database_directory.string());
  }

  if (!database_exists) {
    std::cout << WARNING << " DATABASE DIRECTORY DOES NOT EXIST: "
              << database_directory.string() << std::endl;
    return;
  }

  std::error_code directory_error;
  const bool is_directory = std::filesystem::is_directory(database_directory, directory_error);
  if (directory_error || !is_directory) {
    throw std::runtime_error(
      std::string(ERROR) + " DATABASE PATH IS NOT A DIRECTORY: " + database_directory.string());
  }

  for (const std::filesystem::directory_entry & entry :
       std::filesystem::directory_iterator(database_directory)) {
    std::error_code type_error;
    if (!entry.is_regular_file(type_error)) {
      if (type_error) {
        std::cerr << WARNING << " FAILED TO INSPECT DIRECTORY ENTRY: "
                  << entry.path().string() << std::endl;
      }
      continue;
    }

    const std::filesystem::path file_path = entry.path();
    if (!is_database_file_candidate_for_removal(file_path) ||
        is_database_file_preserved(file_path)) {
      continue;
    }

    std::error_code remove_error;
    if (!std::filesystem::remove(file_path, remove_error)) {
      if (remove_error) {
        throw std::runtime_error(
          std::string(ERROR) + " FAILED TO REMOVE FILE: " + file_path.string());
      }
      continue;
    }

    std::cout << WORKING << " REMOVING FILE: " << file_path.string() << std::endl;
  }
}

void set_method_response(
  const std::string & response_body,
  unsigned char ** response,
  std::size_t * response_size)
{
  if (response == nullptr || response_size == nullptr) {
    return;
  }

  *response = nullptr;
  *response_size = 0;

  const std::size_t size = response_body.size();
  unsigned char * buffer = static_cast<unsigned char *>(std::malloc(size));
  if (buffer == nullptr) {
    throw std::runtime_error(std::string(ERROR) + " FAILED TO ALLOCATE DIRECT METHOD RESPONSE");
  }

  if (size > 0) {
    std::memcpy(buffer, response_body.data(), size);
  }

  *response = buffer;
  *response_size = size;
}

}  // namespace

int on_direct_method_invoked(
  const char * method_name,
  const unsigned char * payload,
  std::size_t size,
  unsigned char ** response,
  std::size_t * response_size,
  void * user_context)
{
  (void)user_context;

  try {
    const std::string invoked_method = method_name == nullptr ? "" : method_name;
    if (invoked_method != kCommandMethodName) {
      set_method_response("{\"status\":\"unsupported method\"}", response, response_size);
      std::cout << WARNING << " UNSUPPORTED DIRECT METHOD: " << invoked_method << std::endl;
      return 404;
    }

    const std::filesystem::path output_path = last_command_output_path();
    const std::optional<DirectMethodCommand> previous_command =
      DirectMethodCommand::try_load_from_file_if_exists(output_path);
    const std::string command_payload = payload_to_string(payload, size);
    save_payload_to_path(output_path, command_payload);

    std::cout << WORKING << " DIRECT METHOD RECEIVED: " << invoked_method << std::endl;
    std::cout << WORKING << " SAVING FILE: " << output_path.string() << std::endl;

    const DirectMethodCommand parsed_command = DirectMethodCommand::load_from_file(output_path);
    std::cout << WORKING << " DIRECT COMMAND LOADED: " << parsed_command.cmd
              << " (" << parsed_command.cmd_id << ")" << std::endl;

    if (previous_command.has_value() && previous_command->cmd_id == parsed_command.cmd_id) {
      std::cout << WARNING << " RECEIVED DUPLICATE COMMAND" << std::endl;
      set_method_response("{\"accepted\":true}", response, response_size);
      return 200;
    }

    const PackageCommand package_command =
      build_package_command_from_direct_method(parsed_command);
    std::cout << WORKING << " RECEIVED COMMAND: " << package_command.action << std::endl;

    if (!handle_package_command(package_command)) {
      set_method_response("{\"accepted\":false}", response, response_size);
      return 404;
    }

    set_method_response("{\"accepted\":true}", response, response_size);
    return 200;
  } catch (const std::exception & ex) {
    try {
      set_method_response("{\"status\":\"failed\"}", response, response_size);
    } catch (const std::exception &) {
    }

    std::cerr << ERROR << " FAILED TO PROCESS DIRECT METHOD" << std::endl << ex.what() << std::endl;
    return 500;
  }
}

}  // namespace cloud_listener
