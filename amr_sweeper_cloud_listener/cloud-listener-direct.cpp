#include "cloud-listener.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

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
constexpr const char * kLastResponseOutputFileName = "last_response.json";

constexpr const char * kClearActivePackageAction = "CLEAR_ACTIVE_PACKAGE";
constexpr const char * kStartSingleMissionAction = "START_SINGLE_MISSION";
constexpr const char * kPauseAction = "PAUSE";
constexpr const char * kResumeAction = "RESUME";
constexpr const char * kStopAction = "STOP";

constexpr const char * kPreservedMessageFileName = "message.json";
constexpr const char * kPreservedLastCommandFileName = "last_command.json";
constexpr const char * kPreservedLastResponseFileName = "last_response.json";
constexpr const char * kPreservedScheduleFileName = "schedule_20260000T000000Z.ics";

constexpr const char * kInterfaceBackendSocketPathEnv = "INTERFACE_BACKEND_SOCKET_PATH";
constexpr const char * kDefaultInterfaceBackendSocketPath =
  "/tmp/amr_sweeper_interface_backend.sock";
constexpr long kInterfaceBackendWriteTimeoutMs = 5000L;
constexpr long kInterfaceBackendReadTimeoutMs = 20000L;
constexpr std::size_t kMaxInterfaceBackendResponseBytes = 1024 * 1024;

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

struct DirectMethodResponse
{
  int status {};
  std::string payload;

  static DirectMethodResponse from_json(const std::string & json_object)
  {
    DirectMethodResponse response;
    response.status = static_cast<int>(require_json_integer_field(json_object, "status"));

    const std::optional<std::string> payload_value =
      extract_top_level_json_value(json_object, "payload");
    if (!payload_value.has_value()) {
      throw std::runtime_error(std::string(ERROR) + " JSON OBJECT DOES NOT INCLUDE REQUIRED KEY 'payload'");
    }

    response.payload = *payload_value;
    return response;
  }

  static std::string to_json(int status, const std::string & payload)
  {
    std::ostringstream output;
    output << "{\n"
           << "    \"status\": " << status << ",\n"
           << "    \"payload\": " << payload << "\n"
           << "}";
    return output.str();
  }

  static DirectMethodResponse load_from_file(const std::filesystem::path & path)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      throw std::runtime_error(
        std::string(ERROR) + " FAILED TO OPEN DIRECT METHOD RESPONSE FILE: " + path.string());
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    try {
      return from_json(buffer.str());
    } catch (const std::exception & ex) {
      throw std::runtime_error(
        std::string(ERROR) + " FAILED TO PARSE DIRECT METHOD RESPONSE FILE '" +
        path.string() + "'\n" + ex.what());
    }
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
         file_name == kPreservedLastResponseFileName ||
         file_name == kPreservedScheduleFileName;
}

void clear_active_package_database_files();

std::filesystem::path last_command_output_path()
{
  return manifest_output_path_from_home().parent_path() / kLastCommandOutputFileName;
}

std::filesystem::path last_response_output_path()
{
  return manifest_output_path_from_home().parent_path() / kLastResponseOutputFileName;
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

std::string interface_backend_socket_path()
{
  const char * configured_path = std::getenv(kInterfaceBackendSocketPathEnv);
  return configured_path == nullptr || std::string(configured_path).empty()
    ? kDefaultInterfaceBackendSocketPath
    : configured_path;
}

struct ScopedSocket
{
  explicit ScopedSocket(int socket_fd)
  : fd(socket_fd)
  {
  }

  ~ScopedSocket()
  {
    if (fd >= 0) {
      ::close(fd);
    }
  }

  ScopedSocket(const ScopedSocket &) = delete;
  ScopedSocket & operator=(const ScopedSocket &) = delete;

  int fd;
};

bool set_socket_timeout(
  int socket_fd,
  int option_name,
  long timeout_ms,
  std::string * error_message)
{
  timeval timeout {};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
  if (::setsockopt(
        socket_fd,
        SOL_SOCKET,
        option_name,
        &timeout,
        static_cast<socklen_t>(sizeof(timeout))) != 0) {
    if (error_message != nullptr) {
      *error_message = std::string("failed to configure socket timeout: ") +
        std::strerror(errno);
    }
    return false;
  }
  return true;
}

bool send_all_on_socket(
  int socket_fd,
  const std::string & message,
  std::string * error_message)
{
  std::size_t bytes_sent = 0;
  while (bytes_sent < message.size()) {
    const ssize_t result = ::send(
      socket_fd,
      message.data() + bytes_sent,
      message.size() - bytes_sent,
      0);
    if (result > 0) {
      bytes_sent += static_cast<std::size_t>(result);
      continue;
    }

    if (result < 0 && errno == EINTR) {
      continue;
    }

    if (error_message != nullptr) {
      *error_message = result == 0
        ? "failed to write backend request: socket closed"
        : std::string("failed to write backend request: ") + std::strerror(errno);
    }
    return false;
  }
  return true;
}

bool read_backend_response_line(
  int socket_fd,
  std::string * response_body,
  std::string * error_message)
{
  std::string buffer;
  char chunk[512];

  while (buffer.size() < kMaxInterfaceBackendResponseBytes) {
    const ssize_t result = ::recv(socket_fd, chunk, sizeof(chunk), 0);
    if (result == 0) {
      break;
    }

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      if (error_message != nullptr) {
        *error_message = std::string("failed to read backend response: ") +
          std::strerror(errno);
      }
      return false;
    }

    buffer.append(chunk, static_cast<std::size_t>(result));
    const std::size_t newline_pos = buffer.find('\n');
    if (newline_pos != std::string::npos) {
      const std::string line = trim(buffer.substr(0, newline_pos));
      if (response_body != nullptr) {
        *response_body = line;
      }
      if (line.empty()) {
        if (error_message != nullptr) {
          *error_message = "backend returned an empty response";
        }
        return false;
      }
      return true;
    }
  }

  if (buffer.size() >= kMaxInterfaceBackendResponseBytes) {
    if (error_message != nullptr) {
      *error_message = "backend response exceeded maximum size";
    }
    return false;
  }

  const std::string line = trim(buffer);
  if (response_body != nullptr) {
    *response_body = line;
  }
  if (line.empty()) {
    if (error_message != nullptr) {
      *error_message = "backend closed the socket without a response";
    }
    return false;
  }
  return true;
}

std::string json_string_field(const std::string & key, const std::string & value)
{
  return "\"" + escape_json_string(key) + "\":\"" + escape_json_string(value) + "\"";
}

std::string json_integer_field(const std::string & key, std::int64_t value)
{
  return "\"" + escape_json_string(key) + "\":" + std::to_string(value);
}

std::string json_bool_field(const std::string & key, bool value)
{
  return "\"" + escape_json_string(key) + "\":" + std::string(value ? "true" : "false");
}

std::string json_object_field(const std::string & key, const std::string & object_json)
{
  return "\"" + escape_json_string(key) + "\":" + object_json;
}

std::string join_json_object_fields(const std::vector<std::string> & fields)
{
  std::ostringstream output;
  output << "{";
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i > 0) {
      output << ",";
    }
    output << fields[i];
  }
  output << "}";
  return output.str();
}

bool backend_response_reports_success(const std::string & response_body)
{
  const std::optional<std::string> success_value =
    extract_top_level_json_value(response_body, "success");
  return success_value.has_value() && *success_value == "true";
}

std::string backend_response_error_detail(const std::string & response_body)
{
  const std::optional<std::string> error_value =
    extract_top_level_json_value(response_body, "error");
  if (!error_value.has_value()) {
    return trim(response_body);
  }

  const std::optional<std::string> error_string = parse_json_string_literal(*error_value);
  return error_string.has_value() ? *error_string : trim(*error_value);
}

bool exchange_interface_backend_message(
  const std::string & request_body,
  std::string * response_body,
  std::string * error_message)
{
  const std::string socket_path = interface_backend_socket_path();
  if (socket_path.empty()) {
    if (error_message != nullptr) {
      *error_message = "interface backend socket path is empty";
    }
    return false;
  }

  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  if (socket_path.size() >= sizeof(address.sun_path)) {
    if (error_message != nullptr) {
      *error_message = "interface backend socket path is too long: " + socket_path;
    }
    return false;
  }
  std::memcpy(address.sun_path, socket_path.c_str(), socket_path.size() + 1);

  const int socket_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_fd < 0) {
    if (error_message != nullptr) {
      *error_message = std::string("failed to create backend socket: ") +
        std::strerror(errno);
    }
    return false;
  }
  ScopedSocket socket_guard(socket_fd);

  if (!set_socket_timeout(socket_fd, SO_SNDTIMEO, kInterfaceBackendWriteTimeoutMs, error_message) ||
      !set_socket_timeout(socket_fd, SO_RCVTIMEO, kInterfaceBackendReadTimeoutMs, error_message)) {
    return false;
  }

  if (::connect(
        socket_fd,
        reinterpret_cast<const sockaddr *>(&address),
        static_cast<socklen_t>(sizeof(address))) != 0) {
    if (error_message != nullptr) {
      *error_message = std::string("failed to connect to backend socket '") +
        socket_path + "': " + std::strerror(errno);
    }
    return false;
  }

  // Raw UDS framing: one compact JSON request, one compact JSON response.
  if (!send_all_on_socket(socket_fd, request_body + "\n", error_message)) {
    return false;
  }
  if (::shutdown(socket_fd, SHUT_WR) != 0) {
    if (error_message != nullptr) {
      *error_message = std::string("failed to finalize backend request: ") +
        std::strerror(errno);
    }
    return false;
  }

  return read_backend_response_line(socket_fd, response_body, error_message);
}

bool send_backend_command(
  const std::string & action,
  const std::string & payload_json,
  std::string * response_body,
  std::string * error_message)
{
  std::vector<std::string> request_fields;
  request_fields.push_back(json_string_field("action", action));
  request_fields.push_back(json_object_field("payload", payload_json));

  std::string local_response_body;
  if (!exchange_interface_backend_message(
        join_json_object_fields(request_fields),
        &local_response_body,
        error_message)) {
    return false;
  }

  if (response_body != nullptr) {
    *response_body = local_response_body;
  }
  if (!backend_response_reports_success(local_response_body)) {
    if (error_message != nullptr) {
      *error_message = "backend rejected request: " +
        backend_response_error_detail(local_response_body);
    }
    return false;
  }
  return true;
}

std::string build_start_single_mission_payload_json(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(json_string_field("mission_id", *command.mission_id));
  fields.push_back(json_string_field(
    "mission_execution_directory",
    command.mission_execution_directory.value_or("")));
  fields.push_back(json_string_field(
    "mission_window_start",
    command.mission_window_start.value_or("")));
  fields.push_back(json_string_field(
    "mission_window_end",
    command.mission_window_end.value_or("")));
  fields.push_back(json_string_field(
    "requester",
    command.requester.value_or("cloud-listener-direct")));
  fields.push_back(json_integer_field("priority", command.priority.value_or(200)));
  fields.push_back(json_bool_field("force", command.force.value_or(false)));
  fields.push_back(json_bool_field("record_rosbag", command.record_rosbag.value_or(false)));
  fields.push_back(json_string_field(
    "reason",
    command.reason.value_or("startSingleMission requested from Azure IoT Hub direct method")));
  return join_json_object_fields(fields);
}

std::string build_stop_payload_json(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(json_string_field("mission_id", command.mission_id.value_or("")));
  fields.push_back(json_string_field(
    "reason",
    command.reason.value_or("stop requested from Azure IoT Hub direct method")));
  fields.push_back(json_string_field("outcome", command.outcome.value_or("aborted")));
  fields.push_back(json_string_field(
    "requester",
    command.requester.value_or("cloud-listener-direct")));
  fields.push_back(json_integer_field("priority", command.priority.value_or(200)));
  fields.push_back(json_bool_field("force", command.force.value_or(false)));
  fields.push_back(json_bool_field("request_idling", command.request_idling.value_or(true)));
  return join_json_object_fields(fields);
}

std::string build_pause_payload_json(const PackageCommand & command)
{
  std::vector<std::string> fields;
  fields.push_back(json_string_field("sender", command.sender.value_or(
    command.requester.value_or("cloud-listener-direct"))));
  fields.push_back(json_string_field(
    "reason",
    command.reason.value_or("pause requested from Azure IoT Hub direct method")));
  return join_json_object_fields(fields);
}

void send_start_single_mission_command(const PackageCommand & command)
{
  if (!command.mission_id.has_value() || command.mission_id->empty()) {
    throw std::runtime_error(
      std::string(ERROR) + " startSingleMission REQUIRES payload.mission_id");
  }

  std::string response_body;
  std::string error_message;
  if (!send_backend_command(
        kStartSingleMissionAction,
        build_start_single_mission_payload_json(command),
        &response_body,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " START MISSION BACKEND COMMAND SENT: "
            << *command.mission_id << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_stop_command(const PackageCommand & command)
{
  std::string response_body;
  std::string error_message;
  if (!send_backend_command(
        kStopAction,
        build_stop_payload_json(command),
        &response_body,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " STOP MISSION BACKEND COMMAND SENT" << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_pause_command(const PackageCommand & command)
{
  std::string response_body;
  std::string error_message;
  if (!send_backend_command(
        kPauseAction,
        build_pause_payload_json(command),
        &response_body,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " SAFETY STOP BACKEND COMMAND SENT FOR PAUSE" << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_resume_command()
{
  std::string response_body;
  std::string error_message;
  if (!send_backend_command(
        kResumeAction,
        "{}",
        &response_body,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " CLEAR SAFETY STOP BACKEND COMMAND SENT FOR RESUME" << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
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
  int status_code,
  const std::string & response_body,
  unsigned char ** response,
  std::size_t * response_size)
{
  save_payload_to_path(
    last_response_output_path(),
    DirectMethodResponse::to_json(status_code, response_body));

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
      set_method_response(400, "{\"status\":\"unsupported method\"}", response, response_size);
      std::cout << WARNING << " UNSUPPORTED DIRECT METHOD: " << invoked_method << std::endl;
      return 400;
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
      const DirectMethodResponse previous_response =
        DirectMethodResponse::load_from_file(last_response_output_path());
      set_method_response(
        previous_response.status,
        previous_response.payload,
        response,
        response_size);
      return previous_response.status;
    }

    const PackageCommand package_command =
      build_package_command_from_direct_method(parsed_command);
    std::cout << WORKING << " RECEIVED COMMAND: " << package_command.action << std::endl;

    if (!handle_package_command(package_command)) {
      set_method_response(400, "{\"accepted\":false}", response, response_size);
      return 400;
    }

    set_method_response(200, "{\"accepted\":true}", response, response_size);
    return 200;
  } catch (const std::exception & ex) {
    try {
      set_method_response(500, "{\"status\":\"failed\"}", response, response_size);
    } catch (const std::exception &) {
    }

    std::cerr << ERROR << " FAILED TO PROCESS DIRECT METHOD" << std::endl << ex.what() << std::endl;
    return 500;
  }
}

}  // namespace cloud_listener
