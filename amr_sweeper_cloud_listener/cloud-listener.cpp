#include "cloud-listener.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

int main()
{
  std::signal(SIGINT, cloud_listener::handle_signal);
  std::signal(SIGTERM, cloud_listener::handle_signal);

  try {
    const std::string connection_string = cloud_listener::connection_string_from_environment();
    const std::string robot_api_key = cloud_listener::robot_api_key_from_environment();
    const std::string hostname = cloud_listener::host_name_from_system();

    cloud_listener::IoTHubRuntime runtime;
    cloud_listener::DeviceClient device_client(connection_string);
    cloud_listener::ListenerContext listener_context;
    listener_context.hostname = hostname;
    listener_context.robot_api_key = robot_api_key;

    if (IoTHubDeviceClient_LL_SetConnectionStatusCallback(
          device_client.get(),
          cloud_listener::on_connection_status,
          &listener_context) != IOTHUB_CLIENT_OK) {
      std::cerr << "Warning: failed to register IoT Hub connection status callback" << std::endl;
    }

    if (IoTHubDeviceClient_LL_SetMessageCallback(
          device_client.get(),
          cloud_listener::on_message_received,
          &listener_context) != IOTHUB_CLIENT_OK) {
      throw std::runtime_error(std::string(ERROR) + " failed to register device message callback");
    }

    if (IoTHubDeviceClient_LL_SetDeviceMethodCallback(
          device_client.get(),
          cloud_listener::on_direct_method_invoked,
          &listener_context) != IOTHUB_CLIENT_OK) {
      throw std::runtime_error(std::string(ERROR) + " failed to register direct method callback");
    }

    while (cloud_listener::g_running != 0) {
      IoTHubDeviceClient_LL_DoWork(device_client.get());

      while (true) {
        const auto next_message = cloud_listener::take_pending_message(listener_context);
        if (!next_message.has_value()) {
          break;
        }

        cloud_listener::process_received_message(*next_message, listener_context);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl << BREAK << " INTERRUPED BY USER" << std::endl;
    std::cout << EXIT << " DISCONNECTED" << std::endl;
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << ERROR << " ERROR: " << ex.what() << std::endl;
    return 1;
  }
}
