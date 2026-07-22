#include "cloud-listener.hpp"

#include <array>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <curl/curl.h>
#include <openssl/evp.h>
#include <unistd.h>

namespace cloud_listener
{

volatile std::sig_atomic_t g_running = 1;

namespace
{

bool is_space(char ch)
{
  return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

std::size_t skip_spaces(const std::string & value, std::size_t position)
{
  while (position < value.size() && is_space(value[position])) {
    ++position;
  }

  return position;
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
      throw std::runtime_error(std::string(ERROR) + "failed to set environment variable '" + key + "': " + std::strerror(errno));
    }
  }
}

void ensure_dotenv_loaded()
{
  static const bool dotenv_loaded = []() {
    load_dotenv(kEnvPath);
    return true;
  }();

  (void)dotenv_loaded;
}

std::string required_environment_variable(const char * variable_name)
{
  ensure_dotenv_loaded();

  const char * value = std::getenv(variable_name);
  if (value == nullptr || value[0] == '\0') {
    throw std::runtime_error(
      std::string(variable_name) +
      " is not set. Provide it via /storage/secrets/.env or environment variable.");
  }

  return value;
}

std::size_t find_string_end(const std::string & json, std::size_t opening_quote)
{
  bool escaped = false;

  for (std::size_t i = opening_quote + 1; i < json.size(); ++i) {
    const char ch = json[i];

    if (escaped) {
      escaped = false;
      continue;
    }

    if (ch == '\\') {
      escaped = true;
      continue;
    }

    if (ch == '"') {
      return i;
    }
  }

  return std::string::npos;
}

std::size_t find_json_value_end(const std::string & json, std::size_t start)
{
  if (start >= json.size()) {
    return std::string::npos;
  }

  const char first = json[start];
  if (first == '"') {
    const std::size_t end = find_string_end(json, start);
    return end == std::string::npos ? end : end + 1;
  }

  if (first != '{' && first != '[') {
    std::size_t end = start;
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != ']') {
      ++end;
    }
    return end;
  }

  bool in_string = false;
  bool escaped = false;
  int object_depth = 0;
  int array_depth = 0;

  for (std::size_t i = start; i < json.size(); ++i) {
    const char ch = json[i];

    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
      continue;
    }

    switch (ch) {
      case '"':
        in_string = true;
        break;
      case '{':
        ++object_depth;
        break;
      case '}':
        --object_depth;
        break;
      case '[':
        ++array_depth;
        break;
      case ']':
        --array_depth;
        break;
      default:
        break;
    }

    if (object_depth == 0 && array_depth == 0) {
      return i + 1;
    }
  }

  return std::string::npos;
}

std::string require_json_value(const std::string & json_object, const std::string & key)
{
  const std::optional<std::string> value = extract_top_level_json_value(json_object, key);
  if (!value.has_value()) {
    throw std::runtime_error(std::string(ERROR) + "JSON object does not include required key '" + key + "'");
  }

  return *value;
}

std::string lowercase_ascii(std::string value)
{
  for (char & ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  return value;
}

size_t write_to_stream(void * contents, size_t size, size_t count, void * user_data)
{
  std::ofstream * output = static_cast<std::ofstream *>(user_data);
  const size_t bytes = size * count;
  output->write(static_cast<const char *>(contents), static_cast<std::streamsize>(bytes));
  return output->good() ? bytes : 0;
}

bool ensure_curl_initialized(std::string * error_message)
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

bool append_http_header(
  struct curl_slist ** headers,
  const std::string & header_name,
  const std::string & header_value,
  std::string * error_message)
{
  const std::string header_line = header_name + ": " + header_value;
  struct curl_slist * updated_headers = curl_slist_append(*headers, header_line.c_str());
  if (updated_headers == nullptr) {
    if (error_message != nullptr) {
      *error_message = "failed to append HTTP header '" + header_name + "'";
    }
    return false;
  }

  *headers = updated_headers;
  return true;
}

}  // namespace

void handle_signal(int)
{
  g_running = 0;
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

std::string escape_json_string(const std::string & value)
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
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
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
        if (ch < 0x20) {
          escaped << "\\u"
                  << std::hex
                  << std::setw(4)
                  << std::setfill('0')
                  << static_cast<unsigned int>(ch)
                  << std::dec
                  << std::setfill(' ');
        } else {
          escaped << static_cast<char>(ch);
        }
        break;
    }
  }

  return escaped.str();
}

std::optional<std::string> extract_top_level_json_value(const std::string & json, const std::string & key)
{
  int object_depth = 0;
  int array_depth = 0;

  for (std::size_t i = 0; i < json.size(); ++i) {
    const char ch = json[i];

    switch (ch) {
      case '"':
      {
        const std::size_t string_end = find_string_end(json, i);
        if (string_end == std::string::npos) {
          return std::nullopt;
        }

        if (object_depth == 1 && array_depth == 0 &&
            json.compare(i + 1, string_end - i - 1, key) == 0) {
          std::size_t value_pos = skip_spaces(json, string_end + 1);
          if (value_pos >= json.size() || json[value_pos] != ':') {
            i = string_end;
            continue;
          }

          value_pos = skip_spaces(json, value_pos + 1);
          const std::size_t value_end = find_json_value_end(json, value_pos);
          if (value_end == std::string::npos) {
            return std::nullopt;
          }

          return trim(json.substr(value_pos, value_end - value_pos));
        }

        i = string_end;
        break;
      }
      case '{':
        ++object_depth;
        break;
      case '}':
        --object_depth;
        break;
      case '[':
        ++array_depth;
        break;
      case ']':
        --array_depth;
        break;
      default:
        break;
    }
  }

  return std::nullopt;
}

std::optional<std::string> parse_json_string_literal(const std::string & value)
{
  if (value.size() < 2 || value.front() != '"' || value.back() != '"') {
    return std::nullopt;
  }

  return unescape_double_quoted(value.substr(1, value.size() - 2));
}

std::string require_json_string_field(const std::string & json_object, const std::string & key)
{
  const std::optional<std::string> value = parse_json_string_literal(require_json_value(json_object, key));
  if (!value.has_value()) {
    throw std::runtime_error(std::string(ERROR) + "JSON field '" + key + "' is not a string");
  }

  return *value;
}

std::int64_t require_json_integer_field(const std::string & json_object, const std::string & key)
{
  const std::string raw_value = trim(require_json_value(json_object, key));

  std::size_t consumed = 0;
  long long value = 0;
  try {
    value = std::stoll(raw_value, &consumed, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(ERROR) + "JSON field '" + key + "' is not an integer");
  }

  if (consumed != raw_value.size()) {
    throw std::runtime_error(std::string(ERROR) + "JSON field '" + key + "' is not an integer");
  }

  return static_cast<std::int64_t>(value);
}

std::string require_json_object_field(const std::string & json_object, const std::string & key)
{
  const std::string value = trim(require_json_value(json_object, key));
  if (value.size() < 2 || value.front() != '{' || value.back() != '}') {
    throw std::runtime_error(std::string(ERROR) + "JSON field '" + key + "' is not an object");
  }

  return value;
}

std::string require_json_array_field(const std::string & json_object, const std::string & key)
{
  const std::string value = trim(require_json_value(json_object, key));
  if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
    throw std::runtime_error(std::string(ERROR) + "JSON field '" + key + "' is not an array");
  }

  return value;
}

std::string current_timestamp_utc_iso8601()
{
  const std::time_t now = std::time(nullptr);
  std::tm time_components {};

#if defined(_WIN32)
  if (gmtime_s(&time_components, &now) != 0) {
    throw std::runtime_error(std::string(ERROR) + "failed to convert current time to UTC");
  }
#else
  if (gmtime_r(&now, &time_components) == nullptr) {
    throw std::runtime_error(std::string(ERROR) + "failed to convert current time to UTC");
  }
#endif

  std::ostringstream formatted;
  formatted << std::put_time(&time_components, "%Y-%m-%dT%H:%M:%SZ");
  return formatted.str();
}

std::vector<std::string> split_top_level_array_elements(const std::string & json_array)
{
  const std::string array_text = trim(json_array);
  if (array_text.size() < 2 || array_text.front() != '[' || array_text.back() != ']') {
    throw std::runtime_error(std::string(ERROR) + "value is not a JSON array");
  }

  std::vector<std::string> elements;
  std::size_t position = skip_spaces(array_text, 1);

  while (position < array_text.size() - 1) {
    if (array_text[position] == ']') {
      break;
    }

    const std::size_t value_end = find_json_value_end(array_text, position);
    if (value_end == std::string::npos || value_end > array_text.size()) {
      throw std::runtime_error(std::string(ERROR) + "failed to parse JSON array element");
    }

    elements.push_back(trim(array_text.substr(position, value_end - position)));
    position = skip_spaces(array_text, value_end);

    if (position >= array_text.size()) {
      throw std::runtime_error(std::string(ERROR) + " JSON array ended unexpectedly");
    }

    if (array_text[position] == ',') {
      position = skip_spaces(array_text, position + 1);
      continue;
    }

    if (array_text[position] == ']') {
      break;
    }

    throw std::runtime_error(std::string(ERROR) + "expected ',' or ']' while parsing JSON array");
  }

  return elements;
}

std::string normalize_sha256_checksum(const std::string & checksum)
{
  std::string normalized = lowercase_ascii(trim(checksum));
  if (starts_with(normalized, "sha256:")) {
    normalized = normalized.substr(7);
  }

  return normalized;
}

std::string calculate_sha256_checksum(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error(std::string(ERROR) + "FAILED TO OPEN FILE: " + path.string());
  }

  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr) {
    throw std::runtime_error(std::string(ERROR) + "EVP_MD_CTX_new failed");
  }

  auto cleanup_context = [&context]() {
    if (context != nullptr) {
      EVP_MD_CTX_free(context);
      context = nullptr;
    }
  };

  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    cleanup_context();
    throw std::runtime_error(std::string(ERROR) + "EVP_DigestInit_ex failed");
  }

  std::array<char, 8192> buffer {};
  while (input.good()) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize bytes_read = input.gcount();
    if (bytes_read > 0 &&
        EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(bytes_read)) != 1) {
      cleanup_context();
      throw std::runtime_error(std::string(ERROR) + "EVP_DigestUpdate failed");
    }
  }

  if (!input.eof()) {
    cleanup_context();
    throw std::runtime_error(std::string(ERROR) + "FAILED TO READ FILE: " + path.string());
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest {};
  unsigned int digest_length = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_length) != 1) {
    cleanup_context();
    throw std::runtime_error(std::string(ERROR) + "EVP_DigestFinal_ex failed");
  }

  cleanup_context();

  std::ostringstream hex_output;
  hex_output << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_length; ++i) {
    hex_output << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }

  return "sha256:" + hex_output.str();
}

std::optional<ActivePackageReportInfo> try_parse_active_package_report_info(
  const std::string & active_package_json,
  std::string * error_message)
{
  try {
    ActivePackageReportInfo package_info;
    package_info.package_id = require_json_string_field(active_package_json, "packageId");
    package_info.version = require_json_integer_field(active_package_json, "version");
    return package_info;
  } catch (const std::exception & ex) {
    if (error_message != nullptr) {
      *error_message = ex.what();
    }
    return std::nullopt;
  }
}

std::string connection_string_from_environment()
{
  return required_environment_variable(kConnectionStringEnv);
}

std::string robot_api_key_from_environment()
{
  return required_environment_variable(kRobotApiKeyEnv);
}

std::string host_name_from_system()
{
  std::array<char, 256> buffer {};
  if (::gethostname(buffer.data(), buffer.size()) != 0) {
    throw std::runtime_error(std::string(ERROR) + "failed to read hostname: " + std::string(std::strerror(errno)));
  }

  buffer.back() = '\0';
  if (buffer.front() == '\0') {
    throw std::runtime_error(std::string(ERROR) + "hostname is empty");
  }

  return buffer.data();
}

std::filesystem::path manifest_output_path_from_home()
{
  ensure_dotenv_loaded();

  const char * home = std::getenv(kHomeEnv);
  if (home == nullptr || home[0] == '\0') {
    throw std::runtime_error(std::string(ERROR) + "HOME is not set. Provide it via environment variable.");
  }

  return std::filesystem::path(home) / "rob_ws" / "missions" / "database" / "manifest.json";
}

std::filesystem::path message_output_path()
{
  return kMessageOutputFileName;
}

std::filesystem::path content_output_path_from_manifest(
  const std::filesystem::path & manifest_path,
  const std::string & file_name)
{
  return manifest_path.parent_path() / file_name;
}

std::string payload_to_string(const unsigned char * payload, std::size_t size)
{
  if (payload == nullptr || size == 0) {
    return {};
  }

  return std::string(reinterpret_cast<const char *>(payload), size);
}

void queue_twin_update(
  ListenerContext & listener_context,
  DEVICE_TWIN_UPDATE_STATE update_state,
  std::string payload)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  listener_context.pending_twin_updates.push_back(PendingTwinUpdate {update_state, std::move(payload)});
}

void queue_message(
  ListenerContext & listener_context,
  std::string payload)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  listener_context.pending_messages.push_back(PendingMessage {std::move(payload)});
}

std::optional<PendingTwinUpdate> take_pending_twin_update(ListenerContext & listener_context)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  if (listener_context.pending_twin_updates.empty()) {
    return std::nullopt;
  }

  PendingTwinUpdate next_update = std::move(listener_context.pending_twin_updates.front());
  listener_context.pending_twin_updates.pop_front();
  return next_update;
}

std::optional<PendingMessage> take_pending_message(ListenerContext & listener_context)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  if (listener_context.pending_messages.empty()) {
    return std::nullopt;
  }

  PendingMessage next_message = std::move(listener_context.pending_messages.front());
  listener_context.pending_messages.pop_front();
  return next_message;
}

bool try_begin_startup_twin_request(ListenerContext & listener_context)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  if (listener_context.startup_twin_requested ||
      listener_context.startup_twin_completed) {
    return false;
  }

  listener_context.startup_twin_requested = true;
  return true;
}

void mark_startup_twin_request_pending(ListenerContext & listener_context)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  if (!listener_context.startup_twin_completed) {
    listener_context.startup_twin_requested = false;
  }
}

void mark_startup_twin_completed(ListenerContext & listener_context)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  listener_context.startup_twin_requested = true;
  listener_context.startup_twin_completed = true;
}

bool download_file(
  const std::string & url,
  const std::string & output_path,
  const std::string & hostname,
  const std::string & robot_api_key,
  std::string * error_message)
{
  if (!ensure_curl_initialized(error_message)) {
    return false;
  }

  const std::filesystem::path output_path_object(output_path);
  const std::filesystem::path output_directory = output_path_object.parent_path();
  if (!output_directory.empty()) {
    std::error_code create_error;
    std::filesystem::create_directories(output_directory, create_error);
    if (create_error) {
      if (error_message != nullptr) {
        *error_message = "failed to create output directory: " + output_directory.string();
      }
      return false;
    }
  }

  std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    if (error_message != nullptr) {
      *error_message = "failed to open output file: " + output_path;
    }
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
  if (!append_http_header(&headers, "X-Device-Id", hostname, error_message) ||
      !append_http_header(&headers, "X-Robot-Api-Key", robot_api_key, error_message)) {
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return false;
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_stream);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &output);

  const CURLcode result = curl_easy_perform(curl);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  output.close();

  if (result != CURLE_OK) {
    std::remove(output_path.c_str());
    if (error_message != nullptr) {
      *error_message = std::string("download failed: ") + curl_easy_strerror(result);
    }
    return false;
  }

  return true;
}

IoTHubRuntime::IoTHubRuntime()
{
  if (IoTHub_Init() != 0) {
    throw std::runtime_error(std::string(ERROR) + " IOTHUB INITIALIZATION FAILED");
  }
}

IoTHubRuntime::~IoTHubRuntime()
{
  IoTHub_Deinit();
}

DeviceClient::DeviceClient(const std::string & connection_string)
  : handle_(IoTHubDeviceClient_LL_CreateFromConnectionString(connection_string.c_str(), MQTT_Protocol))
{
  if (handle_ == nullptr) {
    throw std::runtime_error(std::string(ERROR) + "failed to create IoT Hub device client from connection string");
  }
}

DeviceClient::~DeviceClient()
{
  if (handle_ != nullptr) {
    IoTHubDeviceClient_LL_Destroy(handle_);
  }
}

IOTHUB_DEVICE_CLIENT_LL_HANDLE DeviceClient::get() const
{
  return handle_;
}

}  // namespace cloud_listener

