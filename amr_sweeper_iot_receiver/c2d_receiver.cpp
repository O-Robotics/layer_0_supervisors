#include <cerrno>
#include <chrono>
#include <csignal>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>

#if __has_include(<azureiot/iothub.h>)
#include <azureiot/iothub.h>
#include <azureiot/iothub_device_client.h>
#include <azureiot/iothub_message.h>
#include <azureiot/iothubtransportmqtt.h>
#elif __has_include(<iothub.h>)
#include <iothub.h>
#include <iothub_device_client.h>
#include <iothub_message.h>
#include <iothubtransportmqtt.h>
#else
#error "Azure IoT C SDK headers not found. Install azure-iot-sdk-c or set CPPFLAGS."
#endif

namespace
{
constexpr const char * kEnvPath = "/storage/secrets/.env";
constexpr const char * kConnectionStringEnv = "IOTHUB_DEVICE_CONNECTION_STRING";
constexpr const char * kSaveDirRoot = "/home/dev/rob_ws/missions/database";
constexpr const char * kScheduleFilename = "schedule.ics";
constexpr const char * kScheduleUpdatedStatus = "SCHEDULE_UPDATED";
constexpr const char * kInitScheduleReceivedStatus = "INIT_SCHEDULE_RECEIVED";
constexpr const char * kMissionReceivedStatus = "MISSION_RECEIVED";
constexpr const char * kMissionIdPattern =
  R"regex("mission_id"\s*[:=]\s*(?:"([^"]*)"|'([^']*)'|([^,\s}\]]+))regex";

volatile std::sig_atomic_t g_running = 1;

struct ReceiverContext
{
  IOTHUB_DEVICE_CLIENT_HANDLE device_client {};
};

void handle_signal(int)
{
  g_running = 0;
}

bool is_space(char ch)
{
  return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

std::string trim(const std::string & value)
{
  std::size_t begin = 0;
  while (begin < value.size() && is_space(value[begin])) {
    ++begin;
  }

  std::size_t end = value.size();
  while (end > begin && is_space(value[end - 1])) {
    --end;
  }

  return value.substr(begin, end - begin);
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::string unescape_double_quoted(const std::string & value)
{
  std::string result;
  result.reserve(value.size());

  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] != '\\' || i + 1 >= value.size()) {
      result.push_back(value[i]);
      continue;
    }

    const char escaped = value[++i];
    switch (escaped) {
      case 'n':
        result.push_back('\n');
        break;
      case 'r':
        result.push_back('\r');
        break;
      case 't':
        result.push_back('\t');
        break;
      default:
        result.push_back(escaped);
        break;
    }
  }

  return result;
}

std::string strip_unquoted_comment(const std::string & value)
{
  bool in_single_quotes = false;
  bool in_double_quotes = false;
  bool escaped = false;

  for (std::size_t i = 0; i < value.size(); ++i) {
    const char ch = value[i];

    if (escaped) {
      escaped = false;
      continue;
    }

    if (ch == '\\' && in_double_quotes) {
      escaped = true;
      continue;
    }

    if (ch == '\'' && !in_double_quotes) {
      in_single_quotes = !in_single_quotes;
      continue;
    }

    if (ch == '"' && !in_single_quotes) {
      in_double_quotes = !in_double_quotes;
      continue;
    }

    if (ch == '#' && !in_single_quotes && !in_double_quotes && (i == 0 || is_space(value[i - 1]))) {
      return trim(value.substr(0, i));
    }
  }

  return trim(value);
}

std::string parse_dotenv_value(const std::string & raw_value)
{
  std::string value = strip_unquoted_comment(raw_value);

  if (value.size() >= 2 && value.front() == '\'' && value.back() == '\'') {
    return value.substr(1, value.size() - 2);
  }

  if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
    return unescape_double_quoted(value.substr(1, value.size() - 2));
  }

  return value;
}

void load_dotenv(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    return;
  }

  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    line = trim(line);
    if (line.empty() || line.front() == '#') {
      continue;
    }

    if (starts_with(line, "export ")) {
      line = trim(line.substr(7));
    }

    const std::size_t equals_pos = line.find('=');
    if (equals_pos == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, equals_pos));
    if (key.empty() || std::getenv(key.c_str()) != nullptr) {
      continue;
    }

    const std::string value = parse_dotenv_value(line.substr(equals_pos + 1));
    if (::setenv(key.c_str(), value.c_str(), 0) != 0) {
      throw std::runtime_error("failed to set environment variable '" + key + "': " + std::strerror(errno));
    }
  }
}

std::string connection_string_from_environment()
{
  load_dotenv(kEnvPath);

  const char * connection_string = std::getenv(kConnectionStringEnv);
  if (connection_string == nullptr || connection_string[0] == '\0') {
    throw std::runtime_error(
      "IOTHUB_DEVICE_CONNECTION_STRING is not set. Provide it via /storage/secrets/.env or environment variable.");
  }

  return connection_string;
}

std::string message_payload(IOTHUB_MESSAGE_HANDLE message)
{
  if (IoTHubMessage_GetContentType(message) == IOTHUBMESSAGE_STRING) {
    const char * text = IoTHubMessage_GetString(message);
    if (text == nullptr) {
      throw std::runtime_error("received string C2D message without a readable payload");
    }
    return text;
  }

  const unsigned char * buffer = nullptr;
  std::size_t size = 0;
  if (IoTHubMessage_GetByteArray(message, &buffer, &size) == IOTHUB_MESSAGE_OK) {
    if (buffer == nullptr || size == 0) {
      return {};
    }
    return std::string(reinterpret_cast<const char *>(buffer), size);
  }

  const char * text = IoTHubMessage_GetString(message);
  if (text != nullptr) {
    return text;
  }

  throw std::runtime_error("received C2D message with unsupported payload type");
}

std::optional<std::string> extract_mission_id(const std::string & content)
{
  static const std::regex pattern(kMissionIdPattern);

  std::smatch match;
  if (!std::regex_search(content, match, pattern)) {
    return std::nullopt;
  }

  for (std::size_t i = 1; i < match.size(); ++i) {
    if (match[i].matched) {
      return trim(match[i].str());
    }
  }

  return std::string();
}

const char * confirmation_result_to_string(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  switch (result) {
    case IOTHUB_CLIENT_CONFIRMATION_OK:
      return "ok";
    case IOTHUB_CLIENT_CONFIRMATION_BECAUSE_DESTROY:
      return "client destroyed";
    case IOTHUB_CLIENT_CONFIRMATION_MESSAGE_TIMEOUT:
      return "message timeout";
    case IOTHUB_CLIENT_CONFIRMATION_ERROR:
      return "error";
    default:
      return "unknown";
  }
}

void on_status_message_confirmed(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void * user_context)
{
  const char * status_message = static_cast<const char *>(user_context);
  if (status_message == nullptr) {
    status_message = "UNKNOWN";
  }

  if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
    std::cout << "Reported status '" << status_message << "' to IoT Hub." << std::endl;
  } else {
    std::cerr << "Status report '" << status_message << "' was not acknowledged: "
              << confirmation_result_to_string(result) << std::endl;
  }
}

bool schedule_file_exists()
{
  return std::filesystem::exists(std::filesystem::path(kSaveDirRoot) / kScheduleFilename);
}

void send_status_message(IOTHUB_DEVICE_CLIENT_HANDLE device_client, const char * status_message)
{
  if (device_client == nullptr) {
    throw std::runtime_error("device client handle is not initialized");
  }

  IOTHUB_MESSAGE_HANDLE outbound_message = IoTHubMessage_CreateFromString(status_message);
  if (outbound_message == nullptr) {
    throw std::runtime_error("failed to create status message '" + std::string(status_message) + "'");
  }

  const IOTHUB_CLIENT_RESULT result = IoTHubDeviceClient_SendEventAsync(
    device_client,
    outbound_message,
    on_status_message_confirmed,
    const_cast<char *>(status_message));
  IoTHubMessage_Destroy(outbound_message);

  if (result != IOTHUB_CLIENT_OK) {
    throw std::runtime_error("failed to queue status message '" + std::string(status_message) + "'");
  }

  std::cout << "Queued status message '" << status_message << "' to IoT Hub." << std::endl;
}

void write_message_file(const std::string & filename, const std::string & content)
{
  const std::filesystem::path save_root(kSaveDirRoot);
  std::filesystem::create_directories(save_root);

  const std::filesystem::path filepath = save_root / filename;
  std::ofstream output(filepath, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open '" + filepath.string() + "' for writing");
  }

  output.write(content.data(), static_cast<std::streamsize>(content.size()));
  if (!output) {
    throw std::runtime_error("failed to write '" + filepath.string() + "'");
  }

  std::cout << "Data saved to '" << filepath.string() << "'" << std::endl;
}

IOTHUBMESSAGE_DISPOSITION_RESULT on_message_received(IOTHUB_MESSAGE_HANDLE message, void * user_context)
{
  try {
    const std::string content = message_payload(message);
    const std::optional<std::string> mission_id = extract_mission_id(content);
    const ReceiverContext * receiver_context = static_cast<const ReceiverContext *>(user_context);
    const char * status_message = nullptr;

    std::cout << "Received C2D message!" << std::endl;

    if (receiver_context == nullptr || receiver_context->device_client == nullptr) {
      throw std::runtime_error("receiver context is not initialized");
    }

    if (!mission_id.has_value()) {
      std::cout << "Message detected as schedule." << std::endl;
      status_message = schedule_file_exists() ? kScheduleUpdatedStatus : kInitScheduleReceivedStatus;
      write_message_file(kScheduleFilename, content);
    } else {
      std::cout << "Message detected as mission." << std::endl;
      status_message = kMissionReceivedStatus;
      write_message_file(*mission_id +".json", content);
    }

    send_status_message(receiver_context->device_client, status_message);

    return IOTHUBMESSAGE_ACCEPTED;
  } catch (const std::exception & ex) {
    std::cerr << "Handler error: " << ex.what() << std::endl;
    return IOTHUBMESSAGE_ABANDONED;
  }
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

void on_connection_status(
  IOTHUB_CLIENT_CONNECTION_STATUS status,
  IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
  void *)
{
  if (status == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED) {
    std::cout << "Connected to IoT Hub." << std::endl;
  } else {
    std::cerr << "IoT Hub connection is unauthenticated: "
              << connection_reason_to_string(reason) << std::endl;
  }
}

class IoTHubRuntime
{
public:
  IoTHubRuntime()
  {
    if (IoTHub_Init() != 0) {
      throw std::runtime_error("IoTHub_Init failed");
    }
  }

  ~IoTHubRuntime()
  {
    IoTHub_Deinit();
  }

  IoTHubRuntime(const IoTHubRuntime &) = delete;
  IoTHubRuntime & operator=(const IoTHubRuntime &) = delete;
};

class DeviceClient
{
public:
  explicit DeviceClient(const std::string & connection_string)
  : handle_(IoTHubDeviceClient_CreateFromConnectionString(connection_string.c_str(), MQTT_Protocol))
  {
    if (handle_ == nullptr) {
      throw std::runtime_error("failed to create IoT Hub device client from connection string");
    }
  }

  ~DeviceClient()
  {
    if (handle_ != nullptr) {
      IoTHubDeviceClient_Destroy(handle_);
    }
  }

  DeviceClient(const DeviceClient &) = delete;
  DeviceClient & operator=(const DeviceClient &) = delete;

  IOTHUB_DEVICE_CLIENT_HANDLE get() const
  {
    return handle_;
  }

private:
  IOTHUB_DEVICE_CLIENT_HANDLE handle_ {};
};
}  // namespace

int main()
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  try {
    const std::string connection_string = connection_string_from_environment();

    IoTHubRuntime runtime;
    DeviceClient device_client(connection_string);
    ReceiverContext receiver_context {device_client.get()};

    if (IoTHubDeviceClient_SetMessageCallback(device_client.get(), on_message_received, &receiver_context) != IOTHUB_CLIENT_OK) {
      throw std::runtime_error("failed to register C2D message callback");
    }

    if (IoTHubDeviceClient_SetConnectionStatusCallback(device_client.get(), on_connection_status, nullptr) != IOTHUB_CLIENT_OK) {
      std::cerr << "Warning: failed to register IoT Hub connection status callback" << std::endl;
    }

    while (g_running != 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Interrupted by user." << std::endl;
    std::cout << "Disconnected." << std::endl;
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
}
