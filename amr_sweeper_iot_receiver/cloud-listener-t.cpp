#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <cstdio>
#include <curl/curl.h>
#include <openssl/evp.h>
#include <unistd.h>

#define VERSION "0.1 pre-alfa"
#define IDLING 101
#define WORKING 201
#define VALIDATING 203
#define WARNING 301
#define ERROR "400"
#define BREAK 998
#define EXIT 999

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

namespace
{
constexpr const char * kEnvPath = "/storage/secrets/.env";
constexpr const char * kConnectionStringEnv = "IOTHUB_DEVICE_CONNECTION_STRING";
constexpr const char * kRobotApiKeyEnv = "ROBOT_API_KEY";
constexpr const char * kHomeEnv = "HOME";

volatile std::sig_atomic_t g_running = 1;

struct PendingTwinUpdate
{
  DEVICE_TWIN_UPDATE_STATE update_state;
  std::string payload;
};

struct ActivePackageReportInfo
{
  std::string package_id;
  std::int64_t version {};
};

struct ReportedStateContext
{
  struct ListenerContext * listener_context = nullptr;
  std::string package_id;
  std::int64_t version {};
  std::string status;
  std::string confirmation_key;
  std::string payload_json;
};

struct ListenerContext
{
  std::string hostname;
  std::string robot_api_key;
  IOTHUB_DEVICE_CLIENT_LL_HANDLE device_client_handle = nullptr;
  std::mutex state_mutex;
  std::deque<PendingTwinUpdate> pending_twin_updates;
  std::optional<std::string> last_processed_active_package;
  std::optional<std::string> last_confirmed_reported_status_key;
  bool startup_twin_requested = false;
  bool startup_twin_completed = false;
};

void handle_signal(int)
{
  g_running = 0;
}

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
    throw std::runtime_error("failed to read hostname: " + std::string(std::strerror(errno)));
  }

  buffer.back() = '\0';
  if (buffer.front() == '\0') {
    throw std::runtime_error("hostname is empty");
  }

  return buffer.data();
}

std::string manifest_output_path_from_home()
{
  const char * home = std::getenv(kHomeEnv);
  if (home == nullptr || home[0] == '\0') {
    throw std::runtime_error("HOME is not set. Provide it via environment variable.");
  }

  return (std::filesystem::path(home) / "rob_ws" / "missions" / "database" / "manifest.json").string();
}

std::filesystem::path content_output_path_from_manifest(
  const std::filesystem::path & manifest_path,
  const std::string & file_name)
{
  return manifest_path.parent_path() / file_name;
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

std::string require_json_value(const std::string & json_object, const std::string & key)
{
  const std::optional<std::string> value = extract_top_level_json_value(json_object, key);
  if (!value.has_value()) {
    throw std::runtime_error("JSON object does not include required key '" + key + "'");
  }

  return *value;
}

std::string require_json_string_field(const std::string & json_object, const std::string & key)
{
  const std::optional<std::string> value = parse_json_string_literal(require_json_value(json_object, key));
  if (!value.has_value()) {
    throw std::runtime_error("JSON field '" + key + "' is not a string");
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
    throw std::runtime_error("JSON field '" + key + "' is not an integer");
  }

  if (consumed != raw_value.size()) {
    throw std::runtime_error("JSON field '" + key + "' is not an integer");
  }

  return static_cast<std::int64_t>(value);
}

std::string require_json_object_field(const std::string & json_object, const std::string & key)
{
  const std::string value = trim(require_json_value(json_object, key));
  if (value.size() < 2 || value.front() != '{' || value.back() != '}') {
    throw std::runtime_error("JSON field '" + key + "' is not an object");
  }

  return value;
}

std::string require_json_array_field(const std::string & json_object, const std::string & key)
{
  const std::string value = trim(require_json_value(json_object, key));
  if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
    throw std::runtime_error("JSON field '" + key + "' is not an array");
  }

  return value;
}

std::string current_timestamp_utc_iso8601()
{
  const std::time_t now = std::time(nullptr);
  std::tm time_components {};

#if defined(_WIN32)
  if (gmtime_s(&time_components, &now) != 0) {
    throw std::runtime_error("failed to convert current time to UTC");
  }
#else
  if (gmtime_r(&now, &time_components) == nullptr) {
    throw std::runtime_error("failed to convert current time to UTC");
  }
#endif

  std::ostringstream formatted;
  formatted << std::put_time(&time_components, "%Y-%m-%dT%H:%M:%SZ");
  return formatted.str();
}

std::string reported_status_confirmation_key(
  const ActivePackageReportInfo & package_info,
  const std::string & status)
{
  return package_info.package_id + "|" + std::to_string(package_info.version) + "|" + status;
}

std::vector<std::string> split_top_level_array_elements(const std::string & json_array)
{
  const std::string array_text = trim(json_array);
  if (array_text.size() < 2 || array_text.front() != '[' || array_text.back() != ']') {
    throw std::runtime_error("value is not a JSON array");
  }

  std::vector<std::string> elements;
  std::size_t position = skip_spaces(array_text, 1);

  while (position < array_text.size() - 1) {
    if (array_text[position] == ']') {
      break;
    }

    const std::size_t value_end = find_json_value_end(array_text, position);
    if (value_end == std::string::npos || value_end > array_text.size()) {
      throw std::runtime_error("failed to parse JSON array element");
    }

    elements.push_back(trim(array_text.substr(position, value_end - position)));
    position = skip_spaces(array_text, value_end);

    if (position >= array_text.size()) {
      throw std::runtime_error(ERROR" JSON array ended unexpectedly");
    }

    if (array_text[position] == ',') {
      position = skip_spaces(array_text, position + 1);
      continue;
    }

    if (array_text[position] == ']') {
      break;
    }

    throw std::runtime_error("expected ',' or ']' while parsing JSON array");
  }

  return elements;
}

std::string lowercase_ascii(std::string value)
{
  for (char & ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  return value;
}

std::string normalize_sha256_checksum(const std::string & checksum)
{
  std::string normalized = lowercase_ascii(trim(checksum));
  if (starts_with(normalized, "sha256:")) {
    normalized = normalized.substr(7);
  }

  return normalized;
}

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
      throw std::runtime_error("manifest content is not a JSON object");
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
      throw std::runtime_error("failed to open manifest file '" + path.string() + "'");
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    try {
      return from_json(buffer.str());
    } catch (const std::exception & ex) {
      throw std::runtime_error("failed to parse manifest file '" + path.string() + "': " + ex.what());
    }
  }
};

bool download_file(
  const std::string & url,
  const std::string & output_path,
  const std::string & hostname,
  const std::string & robot_api_key,
  std::string * error_message);

std::string calculate_sha256_checksum(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("FAILED TO OPEN FILE: " + path.string());
  }

  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr) {
    throw std::runtime_error("EVP_MD_CTX_new failed");
  }

  auto cleanup_context = [&context]() {
    if (context != nullptr) {
      EVP_MD_CTX_free(context);
      context = nullptr;
    }
  };

  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    cleanup_context();
    throw std::runtime_error("EVP_DigestInit_ex failed");
  }

  std::array<char, 8192> buffer {};
  while (input.good()) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize bytes_read = input.gcount();
    if (bytes_read > 0 &&
        EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(bytes_read)) != 1) {
      cleanup_context();
      throw std::runtime_error("EVP_DigestUpdate failed");
    }
  }

  if (!input.eof()) {
    cleanup_context();
    throw std::runtime_error("FAILED TO READ FILE: " + path.string());
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest {};
  unsigned int digest_length = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_length) != 1) {
    cleanup_context();
    throw std::runtime_error("EVP_DigestFinal_ex failed");
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
  std::string * error_message = nullptr)
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

      std::cout << VALIDATING << " VERIFYING CHECKSUM: " << (checksum_matches ? "OK" : "MISMATCH") << std::endl;
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

std::string payload_to_string(const unsigned char * payload, std::size_t size)
{
  if (payload == nullptr || size == 0) {
    return {};
  }

  return std::string(reinterpret_cast<const char *>(payload), size);
}

const char * twin_update_state_to_string(DEVICE_TWIN_UPDATE_STATE state)
{
  switch (state) {
    case DEVICE_TWIN_UPDATE_COMPLETE:
      return "complete";
    case DEVICE_TWIN_UPDATE_PARTIAL:
      return "partial";
    default:
      return "unknown";
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

void queue_twin_update(
  ListenerContext & listener_context,
  DEVICE_TWIN_UPDATE_STATE update_state,
  std::string payload)
{
  std::lock_guard<std::mutex> lock(listener_context.state_mutex);
  listener_context.pending_twin_updates.push_back(PendingTwinUpdate {update_state, std::move(payload)});
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
    std::cout << "IOT HUB DEVICE TWIN LISTENER v" << VERSION << " CONNECTED TO IOT HUB" << std::endl;
    std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
  } else {
    std::cerr << ERROR << " IOT HUB AUTHENTICATION FAILURE: "
              << connection_reason_to_string(reason) << std::endl;
  }
}

size_t write_to_stream(void * contents, size_t size, size_t count, void * user_data)
{
  std::ofstream * output = static_cast<std::ofstream *>(user_data);
  const size_t bytes = size * count;
  output->write(static_cast<const char *>(contents), static_cast<std::streamsize>(bytes));
  return output->good() ? bytes : 0;
}

// curl_global_init only needs to succeed once for the whole process.
bool ensure_curl_initialized(std::string * error_message)
{
  // Function-local static keeps initialization lazy and one-time.
  static const CURLcode init_result = curl_global_init(CURL_GLOBAL_DEFAULT);
  // If global setup failed, every later transfer would fail as well.
  if (init_result != CURLE_OK) {
    // Fill the optional output parameter only when the caller asked for details.
    if (error_message != nullptr) {
      // curl_easy_strerror converts the libcurl code into readable text.
      *error_message = std::string("curl_global_init failed: ") + curl_easy_strerror(init_result);
    }
    return false;
  }

  // Global libcurl state is ready for use.
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

// Download the resource at "url" into "output_path".
bool download_file(
  const std::string & url,
  const std::string & output_path,
  const std::string & hostname,
  const std::string & robot_api_key,
  std::string * error_message = nullptr)
{
  // Stop early if libcurl could not initialize process-wide state.
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

  // Truncate any existing file so a new download starts from a clean slate.
  std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
  // File creation can fail because of permissions, missing directories, or disk issues.
  if (!output) {
    if (error_message != nullptr) {
      *error_message = "failed to open output file: " + output_path;
    }
    return false;
  }

  // Create one easy handle for this transfer.
  CURL * curl = curl_easy_init();
  // A null handle means libcurl could not allocate or initialize the request object.
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

  // Tell libcurl which resource to fetch.
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  // Follow HTTP 3xx redirects automatically.
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  // Treat HTTP error status codes such as 404 as transfer failures.
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
  // Send device identity and API key headers along with the request.
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  // Route incoming response bytes into our file-writing callback.
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_stream);
  // Pass the opened file stream into the callback as user data.
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &output);

  // Perform the request synchronously; this call blocks until the transfer ends.
  const CURLcode result = curl_easy_perform(curl);
  // The custom header list is only needed for this single transfer.
  curl_slist_free_all(headers);
  // Release the libcurl handle regardless of success or failure.
  curl_easy_cleanup(curl);
  // Flush and close the file before checking the final result.
  output.close();

  // Any non-OK status means the file on disk should not be trusted.
  if (result != CURLE_OK) {
    // Remove incomplete output so callers do not mistake it for a valid file.
    std::remove(output_path.c_str());
    if (error_message != nullptr) {
      // Return a human-readable description of the transfer failure.
      *error_message = std::string("download failed: ") + curl_easy_strerror(result);
    }
    return false;
  }

  // Success means the file was fully downloaded and kept on disk.
  return true;
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
      throw std::runtime_error(ERROR" LISTENER CONTEXT NOT INITIALIZED");
    }

    std::cout << WORKING << " DEVICE TWIN UPDATE RECEIVED" << std::endl;

    // Keep the SDK callback short and process downloads in the main loop instead.
    queue_twin_update(*listener_context, update_state, payload_to_string(payload, size));
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO QUEUE DEVICE TWIN DATA: " << ex.what() << std::endl;
  }
}

class IoTHubRuntime
{
public:
  IoTHubRuntime()
  {
    if (IoTHub_Init() != 0) {
      throw std::runtime_error(ERROR" IOTHUB INITIALIZATION FAILED");
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
  : handle_(IoTHubDeviceClient_LL_CreateFromConnectionString(connection_string.c_str(), MQTT_Protocol))
  {
    if (handle_ == nullptr) {
      throw std::runtime_error("failed to create IoT Hub device client from connection string");
    }
  }

  ~DeviceClient()
  {
    if (handle_ != nullptr) {
      IoTHubDeviceClient_LL_Destroy(handle_);
    }
  }

  DeviceClient(const DeviceClient &) = delete;
  DeviceClient & operator=(const DeviceClient &) = delete;

  IOTHUB_DEVICE_CLIENT_LL_HANDLE get() const
  {
    return handle_;
  }

private:
  IOTHUB_DEVICE_CLIENT_LL_HANDLE handle_ {};
};

}  // namespace

int main()
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  try {
    const std::string connection_string = connection_string_from_environment();
    const std::string robot_api_key = robot_api_key_from_environment();
    const std::string hostname = host_name_from_system();

    IoTHubRuntime runtime;
    DeviceClient device_client(connection_string);
    ListenerContext listener_context;
    listener_context.hostname = hostname;
    listener_context.robot_api_key = robot_api_key;
    listener_context.device_client_handle = device_client.get();
    const auto startup_twin_request_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);

    if (IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_client.get(), on_connection_status, &listener_context) != IOTHUB_CLIENT_OK) {
      std::cerr << "Warning: failed to register IoT Hub connection status callback" << std::endl;
    }

    if (IoTHubDeviceClient_LL_SetDeviceTwinCallback(device_client.get(), on_device_twin_update, &listener_context) != IOTHUB_CLIENT_OK) {
      throw std::runtime_error("failed to register device twin callback");
    }

    while (g_running != 0) {
      IoTHubDeviceClient_LL_DoWork(device_client.get());

      if (std::chrono::steady_clock::now() >= startup_twin_request_deadline &&
          try_begin_startup_twin_request(listener_context)) {
        std::cout << WORKING << " REQUESTING FULL DEVICE TWIN" << std::endl;
        if (IoTHubDeviceClient_LL_GetTwinAsync(device_client.get(), on_device_twin_update, &listener_context) != IOTHUB_CLIENT_OK) {
          std::cerr << ERROR << " FAILED TO REQUEST STARTUP DEVICE TWIN" << std::endl;
          mark_startup_twin_request_pending(listener_context);
        }
      }

      while (true) {
        const std::optional<PendingTwinUpdate> next_update = take_pending_twin_update(listener_context);
        if (!next_update.has_value()) {
          break;
        }

        process_device_twin_update(
          next_update->update_state,
          next_update->payload,
          listener_context);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << BREAK << " INTERRUPED BY USER" << std::endl;
    std::cout << EXIT << " DISCONNECTED" << std::endl;
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << ERROR << "ERROR: " << ex.what() << std::endl;
    return 1;
  }
}
