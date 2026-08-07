#include "cloud-listener.hpp"

#include <curl/curl.h>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
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

bool ensure_direct_curl_initialized(std::string * error_message)
{
  static const CURLcode init_result = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (init_result != CURLE_OK) {
    if (error_message != nullptr) {
      *error_message = std::string("curl_global_init failed: ") + curl_easy_strerror(init_result);
    }
    return false;
  }
  return true;
}

std::size_t write_http_response_to_string(
  char * ptr,
  std::size_t size,
  std::size_t nmemb,
  void * userdata)
{
  std::string * output = static_cast<std::string *>(userdata);
  if (output == nullptr) {
    return 0;
  }
  const std::size_t bytes = size * nmemb;
  output->append(ptr, bytes);
  return bytes;
}

std::string url_encode_path_component(const std::string & value)
{
  std::string error_message;
  if (!ensure_direct_curl_initialized(&error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    throw std::runtime_error(std::string(ERROR) + " curl_easy_init failed");
  }

  char * escaped = curl_easy_escape(curl, value.c_str(), static_cast<int>(value.size()));
  if (escaped == nullptr) {
    curl_easy_cleanup(curl);
    throw std::runtime_error(std::string(ERROR) + " curl_easy_escape failed");
  }

  std::string encoded(escaped);
  curl_free(escaped);
  curl_easy_cleanup(curl);
  return encoded;
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

bool post_interface_backend_json(
  const std::string & path,
  const std::string & request_body,
  std::string * response_body,
  long * response_code,
  std::string * error_message)
{
  if (!ensure_direct_curl_initialized(error_message)) {
    return false;
  }

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    if (error_message != nullptr) {
      *error_message = "curl_easy_init failed";
    }
    return false;
  }

  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "Accept: application/json");

  std::string local_response_body;
  const std::string socket_path = interface_backend_socket_path();
  const std::string url = "http://localhost" + path;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_UNIX_SOCKET_PATH, socket_path.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(request_body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_http_response_to_string);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &local_response_body);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 20000L);

  const CURLcode result = curl_easy_perform(curl);
  long local_response_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &local_response_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (response_body != nullptr) {
    *response_body = local_response_body;
  }
  if (response_code != nullptr) {
    *response_code = local_response_code;
  }

  if (result != CURLE_OK) {
    if (error_message != nullptr) {
      *error_message = std::string("backend request failed: ") + curl_easy_strerror(result);
    }
    return false;
  }

  if (local_response_code < 200 || local_response_code >= 300) {
    if (error_message != nullptr) {
      *error_message = "backend returned HTTP " + std::to_string(local_response_code) +
        ": " + trim(local_response_body);
    }
    return false;
  }

  if (!backend_response_reports_success(local_response_body)) {
    if (error_message != nullptr) {
      *error_message = "backend rejected request: " + trim(local_response_body);
    }
    return false;
  }

  return true;
}

std::string build_execute_mission_json(const PackageCommand & command)
{
  std::vector<std::string> fields;
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

std::string build_stop_mission_json(const PackageCommand & command)
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

std::string build_safety_stop_json(const PackageCommand & command)
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
  long response_code = 0;
  std::string error_message;
  const std::string path =
    "/api/v1/missions/" + url_encode_path_component(*command.mission_id) + "/execute";
  if (!post_interface_backend_json(
        path,
        build_execute_mission_json(command),
        &response_body,
        &response_code,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " START MISSION BACKEND API SENT: "
            << *command.mission_id << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_stop_command(const PackageCommand & command)
{
  std::string response_body;
  long response_code = 0;
  std::string error_message;
  if (!post_interface_backend_json(
        "/api/v1/mission/stop",
        build_stop_mission_json(command),
        &response_body,
        &response_code,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " STOP MISSION BACKEND API SENT" << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_pause_command(const PackageCommand & command)
{
  std::string response_body;
  long response_code = 0;
  std::string error_message;
  if (!post_interface_backend_json(
        "/api/v1/safety/stop",
        build_safety_stop_json(command),
        &response_body,
        &response_code,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " SAFETY STOP BACKEND API SENT FOR PAUSE" << std::endl;
  if (!response_body.empty()) {
    std::cout << WORKING << " BACKEND RESPONSE: " << trim(response_body) << std::endl;
  }
}

void send_resume_command()
{
  std::string response_body;
  long response_code = 0;
  std::string error_message;
  if (!post_interface_backend_json(
        "/api/v1/safety/clear",
        "{}",
        &response_body,
        &response_code,
        &error_message)) {
    throw std::runtime_error(std::string(ERROR) + " " + error_message);
  }

  std::cout << WORKING << " CLEAR SAFETY STOP BACKEND API SENT FOR RESUME" << std::endl;
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
