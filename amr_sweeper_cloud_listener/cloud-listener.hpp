#pragma once

#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#if __has_include(<azureiot/iothub.h>)
#include <azureiot/iothub.h>
#include <azureiot/iothub_device_client_ll.h>
#include <azureiot/iothubtransportmqtt.h>
#elif __has_include(<iothub.h>)
#include <iothub.h>
#include <iothub_device_client_ll.h>
#include <iothubtransportmqtt.h>
#else
#error "Azure IoT C SDK headers not found. Install azure-iot-sdk-c or set CPPFLAGS."
#endif

#define VERSION "0.1 pre-alfa"

#define IDLING 101
#define WORKING 201
#define ERROR "401"
#define BREAK 998
#define EXIT 999

namespace cloud_listener
{

inline constexpr const char * kEnvPath = "/storage/secrets/.env";
inline constexpr const char * kConnectionStringEnv = "IOTHUB_DEVICE_CONNECTION_STRING";
inline constexpr const char * kRobotApiKeyEnv = "ROBOT_API_KEY";
inline constexpr const char * kHomeEnv = "HOME";
inline constexpr const char * kMessageOutputFileName = "message.json";

extern volatile std::sig_atomic_t g_running;

struct PendingTwinUpdate
{
  DEVICE_TWIN_UPDATE_STATE update_state;
  std::string payload;
};

struct PendingMessage
{
  std::string payload;
};

struct ActivePackageReportInfo
{
  std::string package_id;
  std::int64_t version {};
};

struct ListenerContext
{
  std::string hostname;
  std::string robot_api_key;
  IOTHUB_DEVICE_CLIENT_LL_HANDLE device_client_handle = nullptr;
  std::mutex state_mutex;
  std::deque<PendingTwinUpdate> pending_twin_updates;
  std::deque<PendingMessage> pending_messages;
  std::optional<std::string> last_processed_active_package;
  std::optional<std::string> last_confirmed_reported_status_key;
  bool startup_twin_requested = false;
  bool startup_twin_completed = false;
};

void handle_signal(int);

std::string trim(const std::string & value);
bool starts_with(const std::string & value, const std::string & prefix);
std::string escape_json_string(const std::string & value);
std::optional<std::string> extract_top_level_json_value(const std::string & json, const std::string & key);
std::optional<std::string> parse_json_string_literal(const std::string & value);
std::string require_json_string_field(const std::string & json_object, const std::string & key);
std::int64_t require_json_integer_field(const std::string & json_object, const std::string & key);
std::string require_json_object_field(const std::string & json_object, const std::string & key);
std::string require_json_array_field(const std::string & json_object, const std::string & key);
std::string current_timestamp_utc_iso8601();
std::vector<std::string> split_top_level_array_elements(const std::string & json_array);
std::string normalize_sha256_checksum(const std::string & checksum);
std::string calculate_sha256_checksum(const std::filesystem::path & path);
std::optional<ActivePackageReportInfo> try_parse_active_package_report_info(
  const std::string & active_package_json,
  std::string * error_message = nullptr);
bool download_file(
  const std::string & url,
  const std::string & output_path,
  const std::string & hostname,
  const std::string & robot_api_key,
  std::string * error_message = nullptr);

std::string connection_string_from_environment();
std::string robot_api_key_from_environment();
std::string host_name_from_system();
std::filesystem::path manifest_output_path_from_home();
std::filesystem::path message_output_path();
std::filesystem::path content_output_path_from_manifest(
  const std::filesystem::path & manifest_path,
  const std::string & file_name);
void save_payload_to_path(
  const std::filesystem::path & output_path,
  const std::string & payload);
std::string payload_to_string(const unsigned char * payload, std::size_t size);

void queue_twin_update(
  ListenerContext & listener_context,
  DEVICE_TWIN_UPDATE_STATE update_state,
  std::string payload);
void queue_message(
  ListenerContext & listener_context,
  std::string payload);
std::optional<PendingTwinUpdate> take_pending_twin_update(ListenerContext & listener_context);
std::optional<PendingMessage> take_pending_message(ListenerContext & listener_context);

bool try_begin_startup_twin_request(ListenerContext & listener_context);
void mark_startup_twin_request_pending(ListenerContext & listener_context);
void mark_startup_twin_completed(ListenerContext & listener_context);

bool send_reported_active_package_status(
  const ListenerContext & listener_context,
  const ActivePackageReportInfo & package_info,
  const char * status);
void flush_iothub_work_until_reported_status(
  ListenerContext & listener_context,
  const std::string & confirmation_key,
  std::chrono::milliseconds max_wait);

void on_connection_status(
  IOTHUB_CLIENT_CONNECTION_STATUS status,
  IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
  void * user_context);
void on_device_twin_update(
  DEVICE_TWIN_UPDATE_STATE update_state,
  const unsigned char * payload,
  std::size_t size,
  void * user_context);
void process_device_twin_update(
  DEVICE_TWIN_UPDATE_STATE update_state,
  const std::string & twin_payload,
  ListenerContext & listener_context);

IOTHUBMESSAGE_DISPOSITION_RESULT on_message_received(
  IOTHUB_MESSAGE_HANDLE message,
  void * user_context);
void process_received_message(const PendingMessage & message);
int on_direct_method_invoked(
  const char * method_name,
  const unsigned char * payload,
  std::size_t size,
  unsigned char ** response,
  std::size_t * response_size,
  void * user_context);

class IoTHubRuntime
{
public:
  IoTHubRuntime();
  ~IoTHubRuntime();

  IoTHubRuntime(const IoTHubRuntime &) = delete;
  IoTHubRuntime & operator=(const IoTHubRuntime &) = delete;
};

class DeviceClient
{
public:
  explicit DeviceClient(const std::string & connection_string);
  ~DeviceClient();

  DeviceClient(const DeviceClient &) = delete;
  DeviceClient & operator=(const DeviceClient &) = delete;

  IOTHUB_DEVICE_CLIENT_LL_HANDLE get() const;

private:
  IOTHUB_DEVICE_CLIENT_LL_HANDLE handle_ {};
};

}  // namespace cloud_listener
