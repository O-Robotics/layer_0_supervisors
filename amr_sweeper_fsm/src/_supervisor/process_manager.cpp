#include "_supervisor/process_manager.hpp"

#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <string>

namespace {
volatile sig_atomic_t g_wrapper_stop_signal = 0;

bool process_group_alive_unscoped(pid_t pgid)
{
  if (pgid <= 0) {
    return false;
  }
  if (::kill(-pgid, 0) == 0) {
    return true;
  }
  return errno == EPERM;
}

void wrapper_signal_handler(int sig)
{
  if (g_wrapper_stop_signal == 0) {
    g_wrapper_stop_signal = sig;
  }
}

int exit_code_from_status(int status)
{
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    return 128 + WTERMSIG(status);
  }
  return 1;
}

int supervise_managed_command(const std::string & command)
{
  const pid_t wrapper_pid = ::getpid();
  g_wrapper_stop_signal = 0;

  struct sigaction action;
  std::memset(&action, 0, sizeof(action));
  action.sa_handler = wrapper_signal_handler;
  ::sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  (void)::sigaction(SIGINT, &action, nullptr);
  (void)::sigaction(SIGTERM, &action, nullptr);
  (void)::sigaction(SIGHUP, &action, nullptr);

  const pid_t launch_pid = ::fork();
  if (launch_pid < 0) {
    return 125;
  }

  if (launch_pid == 0) {
    (void)::setpgid(0, wrapper_pid);
    const std::string exec_command = "exec " + command;
    ::execl("/bin/sh", "sh", "-c", exec_command.c_str(), (char *)nullptr);
    _exit(127);
  }

  (void)::setpgid(launch_pid, wrapper_pid);

  while (true) {
    int status = 0;
    const pid_t wait_result = ::waitpid(launch_pid, &status, WNOHANG);
    if (wait_result == launch_pid) {
      return exit_code_from_status(status);
    }

    if (g_wrapper_stop_signal != 0) {
      (void)::signal(SIGINT, SIG_IGN);
      (void)::signal(SIGTERM, SIG_IGN);
      (void)::signal(SIGHUP, SIG_IGN);

      ::kill(-wrapper_pid, SIGTERM);
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (std::chrono::steady_clock::now() < deadline) {
        int child_status = 0;
        const pid_t child_wait = ::waitpid(launch_pid, &child_status, WNOHANG);
        if (child_wait == launch_pid || !process_group_alive_unscoped(wrapper_pid)) {
          return 128 + g_wrapper_stop_signal;
        }
        ::usleep(20 * 1000);
      }

      ::kill(-wrapper_pid, SIGKILL);
      ::usleep(100 * 1000);
      return 128 + g_wrapper_stop_signal;
    }

    ::usleep(20 * 1000);
  }
}
}  // namespace

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
    ::setpgid(0, 0);

    (void)::prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (::getppid() == 1) {
      _exit(1);
    }

    const std::filesystem::path ros_log_dir =
      std::filesystem::temp_directory_path() /
      ("amr_sweeper_fsm_roslog_" + std::to_string(::getpid()));
    std::error_code ec;
    std::filesystem::create_directories(ros_log_dir, ec);
    ::setenv("ROS_LOG_DIR", ros_log_dir.c_str(), 1);
    ::setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);
    ::setenv("RMW_FASTRTPS_USE_SHM", "0", 1);

    const int exit_code = supervise_managed_command(command);
    _exit(exit_code);
  }

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

  if (pid_alive(pid)) {
    const bool is_ros2_launch = command.rfind("ros2 launch", 0) == 0;
    const int initial_signal = is_ros2_launch ? SIGTERM : SIGINT;
    const auto initial_timeout = is_ros2_launch ? policy.sigterm_timeout : policy.sigint_timeout;

    send_signal(pid, initial_signal);
    if (!wait_dead(pid, initial_timeout)) {
      send_signal(pid, SIGTERM);
      (void)wait_dead(pid, policy.sigterm_timeout);
    }
  }

  if (process_group_alive(pgid)) {
    ::kill(-pgid, SIGTERM);
    if (!wait_process_group_dead(pgid, policy.sigterm_timeout)) {
      ::kill(-pgid, SIGKILL);
      if (!wait_process_group_dead(pgid, policy.sigkill_timeout)) {
        err_out =
          "process group remained alive after SIGTERM/SIGKILL escalation for command '" + command + "'";
      }
    }
  }

  int status = 0;
  (void)::waitpid(pid, &status, WNOHANG);

  const bool stopped_cleanly = err_out.empty() && !pid_alive(pid) && !process_group_alive(pgid);
  procs_.erase(it);
  return stopped_cleanly;
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