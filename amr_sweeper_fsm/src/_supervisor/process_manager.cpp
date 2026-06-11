#include "_supervisor/process_manager.hpp"

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstring>
#include <cstdlib>
#include <filesystem>

namespace fsm_layer_0
{

ProcessManager::~ProcessManager()
{
  stop_all();
}

bool ProcessManager::pid_alive(pid_t pid)
{
  if (pid <= 0) {
    return false;
  }
  // kill(pid,0) checks existence/permission without sending a signal.
  if (::kill(pid, 0) == 0) {
    return true;
  }
  return errno == EPERM;
}

bool ProcessManager::process_group_alive(pid_t pgid)
{
  if (pgid <= 0) {
    return false;
  }
  if (::kill(-pgid, 0) == 0) {
    return true;
  }
  return errno == EPERM;
}

void ProcessManager::send_signal(pid_t pid, int sig)
{
  if (pid > 0) {
    ::kill(pid, sig);
  }
}

bool ProcessManager::wait_dead(pid_t pid, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    const pid_t r = ::waitpid(pid, &status, WNOHANG);
    if (r == pid) {
      return true;
    }
    if (!pid_alive(pid)) {
      return true;
    }
    ::usleep(20 * 1000);
  }
  return !pid_alive(pid);
}

bool ProcessManager::wait_process_group_dead(pid_t pgid, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    (void)::waitpid(pgid, &status, WNOHANG);
    if (!process_group_alive(pgid)) {
      return true;
    }
    ::usleep(20 * 1000);
  }
  return !process_group_alive(pgid);
}

bool ProcessManager::start(const std::string & command, std::string & err_out)
{
  err_out.clear();

  if (command.empty()) {
    err_out = "Empty command";
    return false;
  }
  if (is_running(command)) {
    return true;
  }

  const pid_t pid = ::fork();
  if (pid < 0) {
    err_out = std::string("fork() failed: ") + std::strerror(errno);
    return false;
  }
  if (pid == 0) {
    // Child: new process group so we can signal the whole tree.
    ::setpgid(0, 0);

    // Give each FSM-managed launch tree an isolated ROS log directory so concurrent
    // `ros2 launch` invocations do not race on ~/.ros/log/latest.
    const std::filesystem::path ros_log_dir =
      std::filesystem::temp_directory_path() /
      ("amr_sweeper_fsm_roslog_" + std::to_string(::getpid()));
    std::error_code ec;
    std::filesystem::create_directories(ros_log_dir, ec);
    ::setenv("ROS_LOG_DIR", ros_log_dir.c_str(), 1);

    // Force ANSI severity colors for FSM-managed launch trees even when they are
    // spawned under a supervisor instead of directly from an interactive shell.
    ::setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);

    // Fast DDS shared-memory transport can leave stale lock files behind when many
    // short-lived launch trees cycle quickly. Disable SHM for FSM-managed subprocesses
    // to avoid spurious RTPS transport startup errors.
    ::setenv("RMW_FASTRTPS_USE_SHM", "0", 1);

    ::execl("/bin/sh", "sh", "-c", command.c_str(), (char *)nullptr);
    _exit(127);
  }

  // Parent
  ::setpgid(pid, pid);

  Proc p;
  p.pid = pid;
  p.command = command;
  p.started_at = std::chrono::steady_clock::now();
  procs_[command] = p;
  return true;
}

bool ProcessManager::stop(const std::string & command, std::string & err_out)
{
  return stop(command, err_out, StopPolicy{});
}

bool ProcessManager::stop(const std::string & command, std::string & err_out, const StopPolicy & policy)
{
  err_out.clear();

  auto it = procs_.find(command);
  if (it == procs_.end()) {
    return true;
  }

  const pid_t pid = it->second.pid;
  const pid_t pgid = pid;

  // Signal process group (-pid) so typical "ros2 launch" trees are handled.
  // Prefer SIGTERM first for FSM-managed transitions so launch trees do not
  // report this as a user Ctrl-C interruption.
  if (process_group_alive(pgid) || pid_alive(pid)) {
    ::kill(-pgid, SIGTERM);
    if (!wait_process_group_dead(pgid, policy.sigterm_timeout)) {
      ::kill(-pgid, SIGINT);
      if (!wait_process_group_dead(pgid, policy.sigint_timeout)) {
        ::kill(-pgid, SIGKILL);
        (void)wait_process_group_dead(pgid, policy.sigkill_timeout);
      }
    }
  }

  // Reap if still present.
  int status = 0;
  (void)::waitpid(pid, &status, WNOHANG);

  procs_.erase(it);
  return true;
}

void ProcessManager::stop_all()
{
  stop_all(StopPolicy{});
}

void ProcessManager::stop_all(const StopPolicy & policy)
{
  std::vector<std::string> cmds;
  cmds.reserve(procs_.size());
  for (const auto & kv : procs_) {
    cmds.push_back(kv.first);
  }
  for (const auto & c : cmds) {
    std::string err;
    (void)stop(c, err, policy);
  }
}

bool ProcessManager::is_running(const std::string & command) const
{
  auto it = procs_.find(command);
  if (it == procs_.end()) {
    return false;
  }
  return process_group_alive(it->second.pid) || pid_alive(it->second.pid);
}

std::vector<ProcessManager::Proc> ProcessManager::list() const
{
  std::vector<Proc> out;
  out.reserve(procs_.size());
  for (const auto & kv : procs_) {
    out.push_back(kv.second);
  }
  return out;
}

}  // namespace fsm_layer_0
