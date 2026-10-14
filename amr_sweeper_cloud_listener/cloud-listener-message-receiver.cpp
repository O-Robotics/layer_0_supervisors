#include "cloud-listener.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <archive.h>
#include <archive_entry.h>

#undef WORKING
#undef WARNING
#undef ERROR
#define WORKING 231
#define WARNING 331
#define ERROR "431"

namespace cloud_listener
{

namespace
{

constexpr const char * kPackageApiBaseUrl =
  "https://orobotics-apim.azure-api.net/robot-api/v1/robots/";
constexpr const char * kPackageDownloadOutputFileName = "last_package.zip";

class PackageArchiveCleanup
{
public:
  explicit PackageArchiveCleanup(const std::filesystem::path & path)
  : path_(path)
  {
  }

  PackageArchiveCleanup(const PackageArchiveCleanup &) = delete;
  PackageArchiveCleanup & operator=(const PackageArchiveCleanup &) = delete;

  ~PackageArchiveCleanup() noexcept
  {
    if (!removed_) {
      try {
        remove();
      } catch (const std::exception & ex) {
        std::cerr << ERROR << " " << ex.what() << std::endl;
      }
    }
  }

  void remove()
  {
    std::error_code remove_error;
    const bool removed = std::filesystem::remove(path_, remove_error);
    if (remove_error) {
      throw std::runtime_error(
        "FAILED TO REMOVE PACKAGE ZIP '" + path_.string() + "': " + remove_error.message());
    }
    removed_ = true;
    if (removed) {
      std::cout << WORKING << " REMOVING FILE: " << path_.string() << std::endl;
    }
  }

private:
  std::filesystem::path path_;
  bool removed_ = false;
};

[[noreturn]] void throw_zip_error(struct archive * archive_handle, const std::string & action)
{
  const char * detail = archive_error_string(archive_handle);
  throw std::runtime_error(action + ": " + (detail == nullptr ? "unknown ZIP error" : detail));
}

void extract_package_archive(const std::filesystem::path & archive_path)
{
  const auto output_directory = std::filesystem::canonical(archive_path.parent_path());
  const auto full_archive_path = output_directory / archive_path.filename();
  std::unique_ptr<struct archive, decltype(&archive_read_free)> reader(
    archive_read_new(), &archive_read_free);
  std::unique_ptr<struct archive, decltype(&archive_write_free)> writer(
    archive_write_disk_new(), &archive_write_free);
  if (!reader || !writer) {
    throw std::runtime_error("FAILED TO INITIALIZE ZIP EXTRACTOR");
  }
  if (archive_read_support_format_zip(reader.get()) != ARCHIVE_OK) {
    throw_zip_error(reader.get(), "FAILED TO ENABLE ZIP FORMAT");
  }
  if (archive_write_disk_set_options(
        writer.get(),
        ARCHIVE_EXTRACT_SECURE_NODOTDOT | ARCHIVE_EXTRACT_SECURE_SYMLINKS |
        ARCHIVE_EXTRACT_SAFE_WRITES) != ARCHIVE_OK) {
    throw_zip_error(writer.get(), "FAILED TO CONFIGURE ZIP EXTRACTION");
  }
  if (archive_read_open_filename(reader.get(), full_archive_path.string().c_str(), 65536) !=
      ARCHIVE_OK) {
    throw_zip_error(reader.get(), "FAILED TO OPEN PACKAGE ZIP");
  }

  struct archive_entry * entry = nullptr;
  while (true) {
    const int result = archive_read_next_header(reader.get(), &entry);
    if (result == ARCHIVE_EOF) {
      break;
    }
    if (result != ARCHIVE_OK) {
      throw_zip_error(reader.get(), "FAILED TO READ PACKAGE ZIP ENTRY");
    }

    const char * entry_name = archive_entry_pathname(entry);
    if (entry_name == nullptr || entry_name[0] == '\0') {
      throw std::runtime_error("PACKAGE ZIP HAS AN ENTRY WITHOUT A FILE NAME");
    }
    const std::filesystem::path relative_path(entry_name);
    if (relative_path.has_root_path()) {
      throw std::runtime_error("PACKAGE ZIP HAS AN ABSOLUTE ENTRY PATH: " + relative_path.string());
    }
    for (const auto & component : relative_path) {
      if (component == "..") {
        throw std::runtime_error("PACKAGE ZIP ENTRY LEAVES OUTPUT DIRECTORY: " + relative_path.string());
      }
    }
    const auto file_type = archive_entry_filetype(entry);
    if ((file_type != AE_IFREG && file_type != AE_IFDIR) ||
        archive_entry_symlink(entry) != nullptr || archive_entry_hardlink(entry) != nullptr) {
      throw std::runtime_error("PACKAGE ZIP HAS AN UNSUPPORTED ENTRY TYPE: " + relative_path.string());
    }

    const auto normalized_path = relative_path.lexically_normal();
    if (file_type == AE_IFDIR && normalized_path == ".") {
      continue;
    }
    const auto extraction_path = (output_directory / normalized_path).lexically_normal();
    if (extraction_path == output_directory || extraction_path == full_archive_path) {
      throw std::runtime_error("PACKAGE ZIP ENTRY WOULD OVERWRITE ITS ARCHIVE OR OUTPUT DIRECTORY");
    }
    if (!normalized_path.empty() && *normalized_path.begin() == kLastMessageOutputFileName) {
      throw std::runtime_error("PACKAGE ZIP ENTRY WOULD OVERWRITE LAST MESSAGE FILE");
    }
    const std::string extraction_name = extraction_path.string();
    archive_entry_set_pathname(entry, extraction_name.c_str());
    if (archive_read_extract2(reader.get(), entry, writer.get()) != ARCHIVE_OK) {
      throw_zip_error(reader.get(), "FAILED TO EXTRACT ZIP ENTRY '" + relative_path.string() + "'");
    }
  }

  if (archive_write_close(writer.get()) != ARCHIVE_OK) {
    throw_zip_error(writer.get(), "FAILED TO FINALIZE EXTRACTED PACKAGE FILES");
  }
  if (archive_read_close(reader.get()) != ARCHIVE_OK) {
    throw_zip_error(reader.get(), "FAILED TO CLOSE PACKAGE ZIP");
  }
}

std::string expected_package_sha256(const C2DMessage & message)
{
  const std::string checksum_field = trim(message.checksum);
  const std::string checksum = normalize_sha256_checksum(checksum_field);
  if (!starts_with(checksum_field, "sha256:") || checksum.size() != 64 ||
      checksum.find_first_not_of("0123456789abcdef") != std::string::npos) {
    throw std::runtime_error(
      "INVALID PACKAGE CHECKSUM: EXPECTED sha256: FOLLOWED BY 64 HEXADECIMAL CHARACTERS");
  }
  return checksum;
}

std::string encode_url_path_segment(const std::string & value)
{
  constexpr char hex_digits[] = "0123456789ABCDEF";
  std::string encoded;
  for (unsigned char ch : value) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
        (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      encoded.push_back(static_cast<char>(ch));
    } else {
      encoded.push_back('%');
      encoded.push_back(hex_digits[ch >> 4]);
      encoded.push_back(hex_digits[ch & 0x0F]);
    }
  }
  return encoded;
}

std::string build_package_url(const C2DMessage & message, const char * operation)
{
  if (message.robot_id.empty() || message.package_id.empty()) {
    throw std::runtime_error("PACKAGE REQUEST REQUIRES NONEMPTY robotId AND packageId");
  }

  return std::string(kPackageApiBaseUrl) + encode_url_path_segment(message.robot_id) +
         "/packages/" + encode_url_path_segment(message.package_id) + "/" + operation;
}

void report_package_ready(
  const C2DMessage & message,
  const std::string & verified_checksum,
  const ListenerContext & listener_context)
{
  const std::string ready_url = build_package_url(message, "ready");
  std::cout << WORKING << " REPORTING PACKAGE READY TO: " << ready_url << std::endl;

  std::string error_message;
  if (!post_package_ready(
        ready_url,
        listener_context.hostname,
        listener_context.robot_api_key,
        verified_checksum,
        &error_message)) {
    throw std::runtime_error(
      "FAILED TO REPORT PACKAGE READY TO '" + ready_url + "': " + error_message);
  }

  std::cout << WORKING << " PACKAGE READY CONFIRMED: " << message.package_id << std::endl;
}

std::optional<std::string> extract_message_payload(IOTHUB_MESSAGE_HANDLE message)
{
  const char * string_payload = IoTHubMessage_GetString(message);
  if (string_payload != nullptr) {
    return std::string(string_payload);
  }

  const unsigned char * buffer = nullptr;
  size_t size = 0;
  if (IoTHubMessage_GetByteArray(message, &buffer, &size) == IOTHUB_MESSAGE_OK) {
    return std::string(reinterpret_cast<const char *>(buffer), size);
  }

  return std::nullopt;
}

}  // namespace

C2DMessage C2DMessage::from_json(const std::string & json_object)
{
  const std::string message_json = trim(json_object);
  if (message_json.size() < 2 || message_json.front() != '{' || message_json.back() != '}') {
    throw std::runtime_error(std::string(ERROR) + " DEVICE MESSAGE IS NOT A JSON OBJECT");
  }

  C2DMessage message;
  message.message_type = require_json_string_field(message_json, "messageType");
  message.message_id = require_json_string_field(message_json, "messageId");
  message.robot_id = require_json_string_field(message_json, "robotId");
  message.package_id = require_json_string_field(message_json, "packageId");
  message.package_type = require_json_string_field(message_json, "packageType");
  message.package_version = require_json_integer_field(message_json, "packageVersion");
  message.checksum = require_json_string_field(message_json, "checksum");
  message.created_at = require_json_string_field(message_json, "createdAt");
  return message;
}

C2DMessage C2DMessage::load_from_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error(
      std::string(ERROR) + " FAILED TO OPEN DEVICE MESSAGE FILE: " + path.string());
  }

  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (input.bad() || buffer.bad()) {
    throw std::runtime_error(
      std::string(ERROR) + " FAILED TO READ DEVICE MESSAGE FILE: " + path.string());
  }

  try {
    return from_json(buffer.str());
  } catch (const std::exception & ex) {
    throw std::runtime_error(
      std::string(ERROR) + " FAILED TO PARSE DEVICE MESSAGE FILE '" +
      path.string() + "': " + ex.what());
  }
}

void process_received_message(
  const PendingMessage & message,
  ListenerContext & listener_context)
{
  try {
    const auto output_path = message_output_path();
    const auto last_message_path = output_path.parent_path() / kLastMessageOutputFileName;
    const auto download_path = output_path.parent_path() / kPackageDownloadOutputFileName;
    save_payload_to_path(output_path, message.payload);
    std::cout << WORKING << " SAVING FILE: " << output_path.string() << std::endl;

    const C2DMessage parsed_message = C2DMessage::load_from_file(output_path);
    {
      std::lock_guard<std::mutex> lock(listener_context.state_mutex);
      listener_context.last_received_message = parsed_message;
    }
    std::cout << WORKING << " DEVICE MESSAGE LOADED: " << parsed_message.message_type
              << " (" << parsed_message.message_id << ")" << std::endl;

    std::error_code exists_error;
    const bool last_message_exists = std::filesystem::exists(last_message_path, exists_error);
    if (exists_error) {
      throw std::runtime_error(
        "FAILED TO INSPECT LAST MESSAGE FILE '" + last_message_path.string() +
        "': " + exists_error.message());
    }
    if (last_message_exists) {
      const C2DMessage last_message = C2DMessage::load_from_file(last_message_path);
      const bool duplicated_message = parsed_message.message_id == last_message.message_id;
      const bool duplicated_download =
        normalize_sha256_checksum(parsed_message.checksum) ==
        normalize_sha256_checksum(last_message.checksum);
      if (duplicated_message) {
        std::cout << WARNING << " DUPLICATED MESSAGE: messageId "
                  << parsed_message.message_id << " (SKIPPING)" << std::endl;
      }
      if (duplicated_download) {
        std::cout << WARNING << " DUPLICATED PROPOSED DOWNLOAD: checksum "
                  << parsed_message.checksum << " (SKIPPING)" << std::endl;
        report_package_ready(
          parsed_message, expected_package_sha256(last_message), listener_context);
      }
      if (duplicated_message || duplicated_download) {
        std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
        return;
      }
    }

    const std::string expected_checksum = expected_package_sha256(parsed_message);
    const std::string download_url = build_package_url(parsed_message, "download");
    PackageArchiveCleanup archive_cleanup(download_path);
    std::cout << WORKING << " DOWNLOADING PACKAGE FROM: " << download_url << std::endl;

    std::string download_error;
    if (!download_file(
          download_url,
          download_path.string(),
          listener_context.hostname,
          listener_context.robot_api_key,
          &download_error)) {
      throw std::runtime_error(
        "FAILED TO DOWNLOAD PACKAGE FROM '" + download_url + "': " + download_error);
    }

    const std::string actual_checksum =
      normalize_sha256_checksum(calculate_sha256_checksum(download_path));
    if (actual_checksum != expected_checksum) {
      throw std::runtime_error(
        "PACKAGE SHA256 CHECKSUM MISMATCH FOR '" + download_path.string() +
        "': EXPECTED sha256:" + expected_checksum + ", ACTUAL sha256:" + actual_checksum);
    }

    std::cout << WORKING << " VERIFYING PACKAGE SHA256 CHECKSUM: OK" << std::endl;
    std::cout << WORKING << " PACKAGE DOWNLOAD SUCCESSFUL: " << download_path.string()
              << std::endl;
    // Remember a verified download independently of whether ZIP extraction succeeds.
    save_payload_to_path(last_message_path, message.payload);
    std::cout << WORKING << " SAVING FILE: " << last_message_path.string() << std::endl;
    std::cout << WORKING << " EXTRACTING PACKAGE ZIP INTO: "
              << download_path.parent_path().string() << std::endl;
    extract_package_archive(download_path);
    std::cout << WORKING << " PACKAGE EXTRACTION SUCCESSFUL: "
              << download_path.parent_path().string() << std::endl;
    report_package_ready(parsed_message, actual_checksum, listener_context);
    archive_cleanup.remove();
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO PROCESS DEVICE MESSAGE: " << ex.what() << std::endl;
  }

  std::cout << IDLING << " LISTENING (IDLING)" << std::endl;
}

IOTHUBMESSAGE_DISPOSITION_RESULT on_message_received(
  IOTHUB_MESSAGE_HANDLE message,
  void * user_context)
{
  try {
    ListenerContext * listener_context = static_cast<ListenerContext *>(user_context);
    if (listener_context == nullptr) {
      throw std::runtime_error(std::string(ERROR) + " LISTENER CONTEXT NOT INITIALIZED");
    }

    const std::optional<std::string> payload = extract_message_payload(message);
    if (!payload.has_value()) {
      std::cerr << ERROR << " FAILED TO READ DEVICE MESSAGE PAYLOAD" << std::endl;
      return IOTHUBMESSAGE_REJECTED;
    }

    std::cout << WORKING << " DEVICE MESSAGE RECEIVED" << std::endl;

    queue_message(*listener_context, *payload);
    return IOTHUBMESSAGE_ACCEPTED;
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " FAILED TO QUEUE DEVICE MESSAGE: " << ex.what() << std::endl;
    return IOTHUBMESSAGE_ABANDONED;
  }
}

}  // namespace cloud_listener
