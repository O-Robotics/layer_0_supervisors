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
    listener_context.device_client_handle = device_client.get();
    const auto startup_twin_request_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);

    if (IoTHubDeviceClient_LL_SetConnectionStatusCallback(
          device_client.get(),
          cloud_listener::on_connection_status,
          &listener_context) != IOTHUB_CLIENT_OK) {
      std::cerr << "Warning: failed to register IoT Hub connection status callback" << std::endl;
    }

    if (IoTHubDeviceClient_LL_SetDeviceTwinCallback(
          device_client.get(),
          cloud_listener::on_device_twin_update,
          &listener_context) != IOTHUB_CLIENT_OK) {
      throw std::runtime_error(std::string(ERROR) + " failed to register device twin callback");
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

      if (std::chrono::steady_clock::now() >= startup_twin_request_deadline &&
          cloud_listener::try_begin_startup_twin_request(listener_context)) {
        std::cout << WORKING << " REQUESTING FULL DEVICE TWIN" << std::endl;
        if (IoTHubDeviceClient_LL_GetTwinAsync(
              device_client.get(),
              cloud_listener::on_device_twin_update,
              &listener_context) != IOTHUB_CLIENT_OK) {
          std::cerr << ERROR << " FAILED TO REQUEST STARTUP DEVICE TWIN" << std::endl;
          cloud_listener::mark_startup_twin_request_pending(listener_context);
        }
      }

      while (true) {
        const auto next_update = cloud_listener::take_pending_twin_update(listener_context);
        if (!next_update.has_value()) {
          break;
        }

        cloud_listener::process_device_twin_update(
          next_update->update_state,
          next_update->payload,
          listener_context);
      }

      while (true) {
        const auto next_message = cloud_listener::take_pending_message(listener_context);
        if (!next_message.has_value()) {
          break;
        }

        cloud_listener::process_received_message(*next_message);
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
