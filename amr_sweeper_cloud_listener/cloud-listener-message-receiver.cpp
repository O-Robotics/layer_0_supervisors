#include "cloud-listener.hpp"

#include <iostream>
#include <optional>
#include <stdexcept>

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

void process_received_message(const PendingMessage & message)
{
  try {
    const auto output_path = message_output_path();
    save_payload_to_path(output_path, message.payload);
    std::cout << WORKING << " SAVING FILE: " << output_path.string() << std::endl;
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
