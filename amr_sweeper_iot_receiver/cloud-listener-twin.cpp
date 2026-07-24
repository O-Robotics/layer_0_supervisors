#include "cloud-listener.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#undef WORKING
#undef VALIDATING
#undef WARNING
#undef ERROR
#define WORKING 251
#define VALIDATING 253
#define WARNING 351
#define ERROR "451"

namespace cloud_listener
{

namespace
{

struct ManifestFileEntry
{
  std::string id;
  std::string kind;
  std::string file_name;
  std::string content_type;
  std::string format;
  std::int64_t version {};
  std::int64_t size_bytes {};
  std::string checksum;
  std::string download_url;

  static ManifestFileEntry from_json(const std::string & json_object)
  {
    ManifestFileEntry entry;
    entry.id = require_json_string_field(json_object, "id");
    entry.kind = require_json_string_field(json_object, "kind");
    entry.file_name = require_json_string_field(json_object, "fileName");
    entry.content_type = require_json_string_field(json_object, "contentType");
    entry.format = require_json_string_field(json_object, "format");
    entry.version = require_json_integer_field(json_object, "version");
    entry.size_bytes = require_json_integer_field(json_object, "sizeBytes");
    entry.checksum = require_json_string_field(json_object, "checksum");
    entry.download_url = require_json_string_field(json_object, "downloadUrl");
    return entry;
  }
};

struct ManifestScheduleMission
{
  std::string mission_id;
  std::string file_name;

  static ManifestScheduleMission from_json(const std::string & json_object)
  {
    ManifestScheduleMission mission;
    mission.mission_id = require_json_string_field(json_object, "missionId");
    mission.file_name = require_json_string_field(json_object, "fileName");
    return mission;
  }
};

struct ManifestSchedule
{
  std::string file_name;
  std::vector<ManifestScheduleMission> missions;

  static ManifestSchedule from_json(const std::string & json_object)
  {
    ManifestSchedule schedule;
    schedule.file_name = require_json_string_field(json_object, "fileName");

    const std::string missions_array = require_json_array_field(json_object, "missions");
    for (const std::string & mission_json : split_top_level_array_elements(missions_array)) {
      schedule.missions.push_back(ManifestScheduleMission::from_json(mission_json));
    }

    return schedule;
  }
};

struct Manifest
{
  std::string manifest_version;
  std::string package_id;
  std::string robot_id;
  std::string package_type;
  std::int64_t version {};
  std::string created_at;
  std::string checksum;
  std::vector<ManifestFileEntry> files;
  std::optional<ManifestSchedule> schedule;

  static Manifest from_json(const std::string & json_object)
  {
    const std::string manifest_json = trim(json_object);
    if (manifest_json.size() < 2 || manifest_json.front() != '{' || manifest_json.back() != '}') {
      throw std::runtime_error(std::string(ERROR) + "manifest content is not a JSON object");
    }

    Manifest manifest;
    manifest.manifest_version = require_json_string_field(manifest_json, "manifestVersion");
    manifest.package_id = require_json_string_field(manifest_json, "packageId");
    manifest.robot_id = require_json_string_field(manifest_json, "robotId");
    manifest.package_type = require_json_string_field(manifest_json, "packageType");
    manifest.version = require_json_integer_field(manifest_json, "version");
    manifest.created_at = require_json_string_field(manifest_json, "createdAt");
    manifest.checksum = require_json_string_field(manifest_json, "checksum");

    const std::string files_array = require_json_array_field(manifest_json, "files");
    for (const std::string & file_json : split_top_level_array_elements(files_array)) {
      manifest.files.push_back(ManifestFileEntry::from_json(file_json));
    }

    if (extract_top_level_json_value(manifest_json, "schedule").has_value()) {
      manifest.schedule = ManifestSchedule::from_json(require_json_object_field(manifest_json, "schedule"));
    }

    return manifest;
  }

  static Manifest load_from_file(const std::filesystem::path & path)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      throw std::runtime_error(std::string(ERROR) + "failed to open manifest file '" + path.string() + "'");
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    try {
      return from_json(buffer.str());
    } catch (const std::exception & ex) {
      throw std::runtime_error(std::string(ERROR) + "failed to parse manifest file '" + path.string() + "': " + ex.what());
    }
  }
};

struct ReportedStateContext
{
  ListenerContext * listener_context = nullptr;
  std::string package_id;
  std::int64_t version {};
  std::string status;
  std::string confirmation_key;
  std::string payload_json;
};

std::string reported_status_confirmation_key(
  const ActivePackageReportInfo & package_info,
  const std::string & status)
{
  return package_info.package_id + "|" + std::to_string(package_info.version) + "|" + status;
}

bool existing_file_matches_checksum(
  const std::filesystem::path & path,
  const std::string & expected_checksum,
  std::string * error_message = nullptr)
{
  std::error_code exists_error;
  const bool file_exists = std::filesystem::exists(path, exists_error);
  if (exists_error) {
    if (error_message != nullptr) {
      *error_message = "failed to inspect existing file: " + path.string();
    }
    return false;
  }

  if (!file_exists) {
    return false;
  }

  try {
    const std::string actual_checksum = calculate_sha256_checksum(path);
    return normalize_sha256_checksum(actual_checksum) == expected_checksum;
  } catch (const std::exception & ex) {
    if (error_message != nullptr) {
      *error_message = ex.what();
    }
    return false;
  }
}

bool download_and_verify_manifest_files(
  const Manifest & manifest,
  const std::filesystem::path & manifest_path,
  const ListenerContext & listener_context)
{
  if (manifest.files.empty()) {
    std::cout << ERROR << "MANIFEST HAS NO FILES TO DOWNLOAD" << std::endl;
    return false;
  }

  bool all_files_verified = true;

  for (const ManifestFileEntry & file : manifest.files) {
    try {
      const std::filesystem::path output_path =
        content_output_path_from_manifest(manifest_path, file.file_name);
      const std::string expected_checksum = normalize_sha256_checksum(file.checksum);
      std::string error_message;

      if (existing_file_matches_checksum(output_path, expected_checksum, &error_message)) {
        std::cout << WORKING << " REUSING FILE: " << output_path.string() << std::endl;
        std::cout << VALIDATING << " VERIFYING CHECKSUM: OK" << std::endl;
        continue;
      }

      if (!error_message.empty()) {
        std::cout << WARNING << " EXISTING FILE CHECK FAILED: " << file.file_name
                  << ": " << error_message << std::endl;
        error_message.clear();
      }

      if (!download_file(
            file.download_url,
            output_path.string(),
            listener_context.hostname,
            listener_context.robot_api_key,
            &error_message)) {
        std::cerr << ERROR << "FAILED TO SAVE MANIFEST FILE: " << file.file_name << ": "
                  << error_message << std::endl;
        all_files_verified = false;
        continue;
      }

      std::cout << WORKING << " SAVING FILE: " << output_path.string() << std::endl;

      const std::string actual_checksum = calculate_sha256_checksum(output_path);
      const bool checksum_matches = normalize_sha256_checksum(actual_checksum) == expected_checksum;

      std::cout << VALIDATING << " VERIFYING CHECKSUM: "
                << (checksum_matches ? "OK" : "MISMATCH") << std::endl;
      if (!checksum_matches) {
        all_files_verified = false;
        std::error_code remove_error;
        std::filesystem::remove(output_path, remove_error);
        if (remove_error) {
          std::cerr << WARNING << " FAILED TO REMOVE INVALID FILE: " << output_path.string() << std::endl;
        }
      }
    } catch (const std::exception & ex) {
      std::cerr << ERROR << "FAILED TO PROCESS FILE: " << file.file_name
                << ": " << ex.what() << std::endl;
      all_files_verified = false;
    }
  }

  return all_files_verified;
}

const char * connection_reason_to_string(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
  switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
      return "expired SAS token";
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
      return "device disabled";
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
      return "bad credential";
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
      return "retry expired";
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
      return "no network";
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
      return "communication error";
    case IOTHUB_CLIENT_CONNECTION_OK:
      return "ok";
    case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:
      return "no ping response";
    case IOTHUB_CLIENT_CONNECTION_QUOTA_EXCEEDED:
      return "quota exceeded";
    default:
      return "unknown";
  }
}

void on_reported_state_sent(int status_code, void * user_context)
{
  ReportedStateContext * callback_context = static_cast<ReportedStateContext *>(user_context);

  if (callback_context != nullptr) {
    if (status_code >= 200 && status_code < 300 && callback_context->listener_context != nullptr) {
      std::lock_guard<std::mutex> lock(callback_context->listener_context->state_mutex);
      callback_context->listener_context->last_confirmed_reported_status_key =
        callback_context->confirmation_key;
    }

    if (status_code >= 200 && status_code < 300) {
      std::cout << WORKING << " REPORTED activePackage STATUS CONFIRMED: "
                << callback_context->status << " (" << callback_context->package_id
                << " v" << callback_context->version << ")" << std::endl;
    } else {
      std::cerr << ERROR << " FAILED TO REPORT activePackage STATUS: "
                << callback_context->status << " (" << callback_context->package_id
                << " v" << callback_context->version << "), HTTP " << status_code << std::endl;
    }

    delete callback_context;
    return;
  }

  if (status_code < 200 || status_code >= 300) {
    std::cerr << ERROR << " FAILED TO REPORT activePackage STATUS, HTTP "
              << status_code << std::endl;
  }
}

bool is_reported_status_confirmed(
  ListenerContext & listener_context,
  const std::string & confirmation_key)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  return listener_context.last_confirmed_reported_status_key.has_value() &&
         *listener_context.last_confirmed_reported_status_key == confirmation_key;
}

}  // namespace

void flush_iothub_work_until_reported_status(
  ListenerContext & listener_context,
  const std::string & confirmation_key,
  std::chrono::milliseconds max_wait)
{
  if (listener_context.device_client_handle == nullptr) {
    return;
  }

  const auto deadline = std::chrono::steady_clock::now() + max_wait;
  while (g_running != 0 && std::chrono::steady_clock::now() < deadline) {
    if (is_reported_status_confirmed(listener_context, confirmation_key)) {
      return;
    }

    IoTHubDeviceClient_LL_DoWork(listener_context.device_client_handle);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  IoTHubDeviceClient_LL_DoWork(listener_context.device_client_handle);
}

bool send_reported_active_package_status(
  const ListenerContext & listener_context,
  const ActivePackageReportInfo & package_info,
  const char * status)
{
  try {
    if (listener_context.device_client_handle == nullptr) {
      std::cerr << ERROR << " DEVICE CLIENT HANDLE NOT INITIALIZED FOR REPORTED STATE" << std::endl;
      return false;
    }

    ReportedStateContext * callback_context = new ReportedStateContext;
    callback_context->listener_context = const_cast<ListenerContext *>(&listener_context);
    callback_context->package_id = package_info.package_id;
    callback_context->version = package_info.version;
    callback_context->status = status;
    callback_context->confirmation_key =
      reported_status_confirmation_key(package_info, status);
    callback_context->payload_json =
      "{"
        "\"activePackage\":{"
          "\"packageId\":\"" + escape_json_string(package_info.package_id) + "\","
          "\"version\":" + std::to_string(package_info.version) + ","
          "\"status\":\"" + escape_json_string(status) + "\","
          "\"updatedAt\":\"" + escape_json_string(current_timestamp_utc_iso8601()) + "\""
        "}"
      "}";

    const IOTHUB_CLIENT_RESULT result = IoTHubDeviceClient_LL_SendReportedState(
      listener_context.device_client_handle,
      reinterpret_cast<const unsigned char *>(callback_context->payload_json.c_str()),
      callback_context->payload_json.size(),
      on_reported_state_sent,
      callback_context);

    if (result != IOTHUB_CLIENT_OK) {
      delete callback_context;
      std::cerr << ERROR << " FAILED TO QUEUE activePackage STATUS '" << status
                << "' FOR " << package_info.package_id << " v" << package_info.version << std::endl;
      return false;
    }

    std::cout << WORKING << " REPORTING activePackage STATUS: "
              << status << std::endl;
    return true;
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO PREPARE activePackage STATUS '" << status
              << "' FOR " << package_info.package_id << " v" << package_info.version
              << ": " << ex.what() << std::endl;
    return false;
  }
}

void on_connection_status(
  IOTHUB_CLIENT_CONNECTION_STATUS status,
  IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
  void * user_context)
{
  ListenerContext * listener_context = static_cast<ListenerContext *>(user_context);
  if (listener_context != nullptr) {
    std::lock_guard<std::mutex> lock(listener_context->state_mutex);
    if (status != IOTHUB_CLIENT_CONNECTION_AUTHENTICATED &&
        !listener_context->startup_twin_completed) {
      listener_context->startup_twin_requested = false;
    }
  }

  if (status == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED) {
    std::cout << "IOT HUB DEVICE LISTENER v" << VERSION << " CONNECTED TO IOT HUB" << std::endl;
    std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
  } else {
    std::cerr << ERROR << " IOT HUB AUTHENTICATION FAILURE: "
              << connection_reason_to_string(reason) << std::endl;
  }
}

void process_device_twin_update(
  DEVICE_TWIN_UPDATE_STATE update_state,
  const std::string & twin_payload,
  ListenerContext & listener_context)
{
  try {
    const std::string * property_source = &twin_payload;
    const std::filesystem::path manifest_output_path = manifest_output_path_from_home();
    std::string error_message;
    const std::optional<std::string> desired_properties =
      extract_top_level_json_value(twin_payload, "desired");
    const bool payload_contains_full_twin =
      desired_properties.has_value() ||
      extract_top_level_json_value(twin_payload, "reported").has_value();

    if (update_state == DEVICE_TWIN_UPDATE_COMPLETE || payload_contains_full_twin) {
      mark_startup_twin_completed(listener_context);
    }

    std::cout << WORKING << " PROCESSING DEVICE TWIN UPDATE" << std::endl;

    if (desired_properties.has_value()) {
      property_source = &*desired_properties;
    } else if (update_state == DEVICE_TWIN_UPDATE_COMPLETE || payload_contains_full_twin) {
      std::cout << "TWIN:" << std::endl;
      std::cout << twin_payload << std::endl;
    }

    const std::optional<std::string> active_package =
      extract_top_level_json_value(*property_source, "activePackage");
    if (!active_package.has_value()) {
      std::cout << WARNING << " UPDATE HAS NO activePackage" << std::endl;
      std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
      return;
    }

    const std::string active_package_signature = trim(*active_package);
    if (listener_context.last_processed_active_package.has_value() &&
        *listener_context.last_processed_active_package == active_package_signature) {
      std::cout << WORKING << " DUPLICATE activePackage RECEIVED (SKIPPING)" << std::endl;
      std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
      return;
    }

    std::string package_report_error;
    const std::optional<ActivePackageReportInfo> package_report_info =
      try_parse_active_package_report_info(*active_package, &package_report_error);
    if (!package_report_info.has_value()) {
      std::cerr << WARNING << " activePackage HAS NO REPORTABLE packageId/version: "
                << package_report_error << std::endl;
    }

    const std::optional<std::string> raw_url =
      extract_top_level_json_value(*active_package, "manifestUrl");
    if (!raw_url.has_value()) {
      std::cout << ERROR << " RECEIVED activePackage HAS NO manifestUrl" << std::endl;
      std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
      return;
    }

    const std::optional<std::string> url = parse_json_string_literal(*raw_url);
    if (!url.has_value() || url->empty()) {
      std::cerr << ERROR << " UPDATE HAS INVALID manifestUrl: " << *raw_url << std::endl;
      std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
      return;
    }

    if (package_report_info.has_value()) {
      if (send_reported_active_package_status(listener_context, *package_report_info, "seen")) {
        flush_iothub_work_until_reported_status(
          listener_context,
          reported_status_confirmation_key(*package_report_info, "seen"),
          std::chrono::milliseconds(750));
      }
    }

    if (!download_file(
          *url,
          manifest_output_path.string(),
          listener_context.hostname,
          listener_context.robot_api_key,
          &error_message)) {
      std::cerr << "Failed to save '" << manifest_output_path.string() << "': " << error_message << std::endl;
      std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
      return;
    }

    std::cout << WORKING << " SAVING FILE: " << manifest_output_path.string() << std::endl;

    const Manifest manifest = Manifest::load_from_file(manifest_output_path);
    const bool download_completed =
      download_and_verify_manifest_files(manifest, manifest_output_path, listener_context);

    if (download_completed) {
      if (package_report_info.has_value()) {
        send_reported_active_package_status(listener_context, *package_report_info, "downloaded");
      }
      listener_context.last_processed_active_package = active_package_signature;
    } else {
      std::cerr << WARNING << " PACKAGE DOWNLOAD NOT COMPLETE; NOT REPORTING 'downloaded'" << std::endl;
    }
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO PROCESS DATA: " << ex.what() << std::endl;
  }
  std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
}

void on_device_twin_update(
  DEVICE_TWIN_UPDATE_STATE update_state,
  const unsigned char * payload,
  std::size_t size,
  void * user_context)
{
  try {
    ListenerContext * listener_context = static_cast<ListenerContext *>(user_context);
    if (listener_context == nullptr) {
      throw std::runtime_error(std::string(ERROR) + " LISTENER CONTEXT NOT INITIALIZED");
    }

    std::cout << WORKING << " DEVICE TWIN UPDATE RECEIVED" << std::endl;

    queue_twin_update(*listener_context, update_state, payload_to_string(payload, size));
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO QUEUE DEVICE TWIN DATA: " << ex.what() << std::endl;
  }
}

}  // namespace cloud_listener
