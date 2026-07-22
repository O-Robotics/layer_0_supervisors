#include "cloud-listener.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

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

constexpr const char * kClearActivePackageAction = "clearActivePackage";
constexpr const char * kPreservedMessageFileName = "message.json";
constexpr const char * kPreservedScheduleFileName = "schedule_20260000T000000Z.ics";

struct PackageCommand
{
  std::string action;

  static PackageCommand from_json(const std::string & json_object)
  {
    PackageCommand command;
    command.action = require_json_string_field(json_object, "action");
    return command;
  }
};

struct DeviceMessage
{
  std::optional<PackageCommand> package_command;

  static DeviceMessage from_json(const std::string & json_object)
  {
    DeviceMessage message;

    if (extract_top_level_json_value(json_object, "packageCommand").has_value()) {
      message.package_command =
        PackageCommand::from_json(require_json_object_field(json_object, "packageCommand"));
    }

    return message;
  }

  static DeviceMessage load_from_file(const std::filesystem::path & path)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      throw std::runtime_error(std::string(ERROR) + "failed to open message file: " + path.string());
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    try {
      return from_json(buffer.str());
    } catch (const std::exception & ex) {
      throw std::runtime_error(std::string(ERROR) + "failed to parse message file '" +
                               path.string() + "': " + ex.what());
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
  return file_name == kPreservedMessageFileName || file_name == kPreservedScheduleFileName;
}

void clear_active_package_database_files()
{
  const std::filesystem::path database_directory = manifest_output_path_from_home().parent_path();

  std::error_code exists_error;
  const bool database_exists = std::filesystem::exists(database_directory, exists_error);
  if (exists_error) {
    throw std::runtime_error(
      std::string(ERROR) + "failed to inspect database directory: " + database_directory.string());
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
      std::string(ERROR) + "database path is not a directory: " + database_directory.string());
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
          std::string(ERROR) + "failed to remove file: " + file_path.string());
      }
      continue;
    }

    std::cout << WORKING << " REMOVING FILE: " << file_path.string() << std::endl;
  }
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

void save_message_payload(const std::string & payload)
{
  const std::filesystem::path output_path = message_output_path();
  const std::filesystem::path output_directory = output_path.parent_path();
  if (!output_directory.empty()) {
    std::error_code create_error;
    std::filesystem::create_directories(output_directory, create_error);
    if (create_error) {
      throw std::runtime_error(std::string(ERROR) + "failed to create output directory: " + output_directory.string());
    }
  }

  std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error(std::string(ERROR) + "failed to open output file: " + output_path.string());
  }

  output.write(payload.data(), static_cast<std::streamsize>(payload.size()));
  if (!output.good()) {
    throw std::runtime_error(std::string(ERROR) + "failed to write output file: " + output_path.string());
  }

  output.close();
  if (!output) {
    throw std::runtime_error(std::string(ERROR) + "failed to finalize output file: " + output_path.string());
  }
}

}  // namespace

void process_received_message(const PendingMessage & message)
{
  try {
    const std::filesystem::path output_path = message_output_path();
    save_message_payload(message.payload);
    std::cout << WORKING << " SAVING FILE: " << output_path.string() << std::endl;

    const DeviceMessage parsed_message = DeviceMessage::load_from_file(output_path);
    if (parsed_message.package_command.has_value()) {
      std::cout << WORKING << " RECEIVED COMMAND: " << parsed_message.package_command->action << std::endl;

      if (parsed_message.package_command->action == kClearActivePackageAction) {
        clear_active_package_database_files();
      }
    }
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
