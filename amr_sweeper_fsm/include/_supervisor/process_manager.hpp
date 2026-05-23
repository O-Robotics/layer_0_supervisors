#pragma once

#include <sys/types.h>

#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace fsm_layer_0
{

/**
 * @brief Minimal POSIX process manager for launching/stopping ROS 2 launch files
 *        from within lifecycle state nodes.
 *
 * We intentionally keep this lightweight (no shell pipelines). Commands are executed
 * via /bin/sh -c "<command>" to allow typical ROS usage (ros2 launch ... args).
 *
 * Stop behavior:
 *   - SIGINT, wait sigint_timeout
 *   - SIGTERM, wait sigterm_timeout
 *   - SIGKILL, wait sigkill_timeout
 */
class ProcessManager
{
public:
  struct Proc
  {
    pid_t pid{-1};
    std::string command;
    std::chrono::steady_clock::time_point started_at;
  };

  struct StopPolicy
  {
    std::chrono::milliseconds sigint_timeout{std::chrono::milliseconds(2000)};
    std::chrono::milliseconds sigterm_timeout{std::chrono::milliseconds(2000)};
    std::chrono::milliseconds sigkill_timeout{std::chrono::milliseconds(500)};
  };


  ProcessManager() = default;
  ~ProcessManager();

  ProcessManager(const ProcessManager &) = delete;
  ProcessManager & operator=(const ProcessManager &) = delete;

  bool start(const std::string & command, std::string & err_out);
  bool stop(const std::string & command, std::string & err_out);
  bool stop(const std::string & command, std::string & err_out, const StopPolicy & policy);
  void stop_all();
  void stop_all(const StopPolicy & policy);

  bool is_running(const std::string & command) const;
  std::vector<Proc> list() const;

private:
  std::map<std::string, Proc> procs_;

  static bool pid_alive(pid_t pid);
  static void send_signal(pid_t pid, int sig);
  static bool wait_dead(pid_t pid, std::chrono::milliseconds timeout);
};

}  // namespace fsm_layer_0
