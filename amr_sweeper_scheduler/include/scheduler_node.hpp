#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "amr_sweeper_mission_executor/srv/execute_mission.hpp"
#include "amr_sweeper_mission_executor/srv/prepare_manual_mission.hpp"
#include "amr_sweeper_scheduler/srv/prepare_mission_execution.hpp"

namespace amr_sweeper_scheduler
{

enum class ScheduleType
{
  WORK,
  NO_WORK,
  SAFETY
};

std::optional<ScheduleType> schedule_type_from_string(const std::string & value);
const char * schedule_type_to_cstr(ScheduleType type);

struct ScheduleEvent
{
  std::string uid;
  std::string robot_id;
  ScheduleType type{ScheduleType::WORK};
  std::optional<std::string> mission_id;
  bool record_rosbag{false};
  std::optional<std::string> runtime_status;
  std::string dtstart_tzid;
  std::string dtstart_local;
  std::optional<std::string> duration;
  std::optional<std::string> dtend_local;
  std::optional<std::string> rrule;
};

struct ScheduleModel
{
  std::string calendar_tzid;
  std::vector<ScheduleEvent> events;
};

struct TimeWindow
{
  std::string uid;
  std::string robot_id;
  ScheduleType type{ScheduleType::WORK};
  std::optional<std::string> mission_id;
  bool record_rosbag{false};
  std::optional<std::string> runtime_status;
  std::optional<std::string> mission_path;
  std::string tzid;
  std::string start_local;
  std::string end_local;
};

struct ParserConfig
{
  bool strict_validation{true};
  int max_events{2000};
  bool require_x_robot_id{false};
  bool require_x_schedule_type{true};
  bool require_x_mission_id_for_work{true};
};

class IcalParser
{
public:
  virtual ~IcalParser() = default;
  virtual ScheduleModel parse_file(const std::string & ics_path, const ParserConfig & config) = 0;
};

class IcalParserMinimal final : public IcalParser
{
public:
  ScheduleModel parse_file(const std::string & ics_path, const ParserConfig & config) override;
};

class ScheduleExpander
{
public:
  virtual ~ScheduleExpander() = default;
  virtual std::vector<TimeWindow> expand(
    const ScheduleModel & model,
    const std::chrono::system_clock::time_point & now,
    const std::chrono::hours & horizon) = 0;
};

class ScheduleExpanderStub final : public ScheduleExpander
{
public:
  std::vector<TimeWindow> expand(
    const ScheduleModel & model,
    const std::chrono::system_clock::time_point & now,
    const std::chrono::hours & horizon) override;
};

std::vector<TimeWindow> apply_blackout_overlay(const std::vector<TimeWindow> & windows);

class SchedulerNode : public rclcpp::Node
{
public:
  explicit SchedulerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void enter_fatal_state(const std::string & message);
  void report_supervision_issue(const std::string & message);
  void log_escalating_issue(int count, const std::string & message);
  void reset_supervision_issue_count();
  void publish_info_message(const std::string & message);
  void tick();
  void poll_schedule();
  void load_schedule();
  void refresh_mission_catalog();
  void publish_windows(const std::vector<TimeWindow> & windows);
  void maybe_promote_mission(const std::vector<TimeWindow> & windows);
  [[nodiscard]] std::chrono::system_clock::time_point current_schedule_time() const;
  [[nodiscard]] bool actual_schedule_has_terminal_run_for_window(const TimeWindow & window) const;
  [[nodiscard]] std::string resolved_actual_schedule_log_path() const;
  [[nodiscard]] bool mission_json_or_folder_exists(const std::string & mission_id) const;
  void request_mission_execution(const TimeWindow & window);
  [[nodiscard]] std::string resolved_schedule_path() const;
  [[nodiscard]] std::optional<std::filesystem::path> discover_latest_schedule_path() const;
  [[nodiscard]] std::optional<std::string> resolve_timestamped_mission_alias(
    const std::string & mission_id) const;
  [[nodiscard]] std::optional<std::string> resolve_mission_path(
    const std::string & mission_id) const;
  void trigger_info(const std::string & code, const std::string & kv = "");
  void trigger_warn(const std::string & code, const std::string & kv = "");
  void trigger_error(const std::string & code, const std::string & kv = "");

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planned_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trigger_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<amr_sweeper_scheduler::srv::PrepareMissionExecution>::SharedPtr
    prepare_mission_execution_srv_;
  rclcpp::CallbackGroup::SharedPtr mission_executor_client_callback_group_;
  rclcpp::Client<amr_sweeper_mission_executor::srv::ExecuteMission>::SharedPtr
    mission_executor_execute_client_;
  rclcpp::Client<amr_sweeper_mission_executor::srv::PrepareManualMission>::SharedPtr
    mission_executor_prepare_client_;
  std::unique_ptr<IcalParser> parser_;
  std::unique_ptr<ScheduleExpander> expander_;
  ScheduleModel schedule_;
  bool schedule_loaded_{false};
  std::string schedule_ics_path_;
  std::string missions_directory_;
  std::string default_schedule_filename_;
  std::string mission_file_extension_;
  std::string robot_id_;
  std::string robot_config_env_path_;
  std::string actual_schedule_log_path_;
  std::string mission_executor_execute_service_;
  std::string mission_executor_prepare_service_;
  int horizon_hours_{72};
  double tick_seconds_{1.0};
  bool trigger_running_on_work_window_{true};
  bool force_record_rosbag_{false};
  bool use_sim_time_for_schedule_clock_{false};
  double schedule_poll_interval_sec_{60.0};
  int retry_attempts_before_error_{3};
  int fatal_after_consecutive_errors_{10};
  bool reload_on_mtime_change_{true};
  bool reload_on_every_poll_{false};
  bool emit_rosout_triggers_{true};
  bool emit_trigger_topic_{true};
  std::string trigger_topic_name_{"scheduler_node/triggers"};
  std::string last_trigger_message_;
  std::string last_planned_windows_payload_;
  bool running_request_in_flight_{false};
  std::string running_request_window_uid_;
  std::string rejected_running_window_uid_;
  std::optional<rclcpp::Time> next_running_request_retry_time_;
  std::optional<std::time_t> last_mtime_;
  std::unordered_map<std::string, std::string> mission_catalog_;
  std::set<std::string> warned_missing_mission_ids_;
  std::set<std::string> warned_mission_alias_ids_;
  int supervision_issue_count_{0};
  bool ready_message_emitted_{false};
  bool fatal_error_{false};
  bool schedule_has_no_events_{false};
  bool use_sim_time_{false};
  mutable bool schedule_time_anchor_initialized_{false};
  mutable rclcpp::Time schedule_time_anchor_ros_;
  mutable std::chrono::system_clock::time_point schedule_time_anchor_wall_;
};

}  // namespace amr_sweeper_scheduler
