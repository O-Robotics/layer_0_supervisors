#pragma once

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "amr_sweeper_fsm/srv/request_state.hpp"
#include "amr_sweeper_scheduler/srv/prepare_mission_execution.hpp"

namespace amr_sweeper_scheduler
{

enum class ScheduleType
{
  WORK,
  NO_WORK
};

std::optional<ScheduleType> schedule_type_from_string(const std::string & value);
const char * schedule_type_to_cstr(ScheduleType type);

struct ScheduleEvent
{
  std::string uid;
  std::string robot_id;
  ScheduleType type{ScheduleType::WORK};
  std::optional<std::string> mission_id;
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
  std::optional<std::string> mission_path;
  std::string start_local;
  std::string end_local;
};

struct ParserConfig
{
  bool strict_validation{true};
  int max_events{2000};
  bool require_x_robot_id{true};
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
    const std::string & robot_id,
    const std::chrono::system_clock::time_point & now,
    const std::chrono::hours & horizon) = 0;
};

class ScheduleExpanderStub final : public ScheduleExpander
{
public:
  std::vector<TimeWindow> expand(
    const ScheduleModel & model,
    const std::string & robot_id,
    const std::chrono::system_clock::time_point & now,
    const std::chrono::hours & horizon) override;
};

std::vector<TimeWindow> apply_blackout_overlay(const std::vector<TimeWindow> & windows);

class SchedulerNode : public rclcpp::Node
{
public:
  explicit SchedulerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();
  void poll_schedule();
  void load_schedule();
  void refresh_mission_catalog();
  void publish_windows(const std::vector<TimeWindow> & windows);
  void maybe_promote_mission(const std::vector<TimeWindow> & windows);
  void request_mission_build(const std::string & mission_path);
  [[nodiscard]] bool mission_json_or_folder_exists(const std::string & mission_id) const;
  [[nodiscard]] bool prepare_active_mission_execution(const TimeWindow & window);
  [[nodiscard]] bool prepare_mission_execution(
    const std::string & mission_id,
    const std::string & mission_path,
    const std::string & window_start,
    const std::string & window_end);
  void request_running_state(const TimeWindow & window);
  [[nodiscard]] bool mission_artifacts_ready(const std::string & mission_path) const;
  [[nodiscard]] std::string mission_costmap_yaml_path(const std::string & mission_path) const;
  [[nodiscard]] std::string mission_route_path(const std::string & mission_path) const;
  [[nodiscard]] std::filesystem::path mission_folder_path(const std::string & mission_path) const;
  [[nodiscard]] std::string mission_costmap_image_path(const std::string & mission_path) const;
  [[nodiscard]] std::string active_costmap_image_path() const;
  [[nodiscard]] std::string active_route_path() const;
  [[nodiscard]] std::string active_costmap_yaml_path() const;
  [[nodiscard]] std::string resolved_schedule_path() const;
  [[nodiscard]] std::optional<std::filesystem::path> discover_latest_schedule_path() const;
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
  rclcpp::AsyncParametersClient::SharedPtr mission_builder_parameter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mission_builder_build_client_;
  rclcpp::Client<amr_sweeper_fsm::srv::RequestState>::SharedPtr fsm_request_client_;
  std::unique_ptr<IcalParser> parser_;
  std::unique_ptr<ScheduleExpander> expander_;
  ScheduleModel schedule_;
  bool schedule_loaded_{false};
  std::string schedule_ics_path_;
  std::string missions_directory_;
  std::string default_schedule_filename_;
  std::string mission_file_extension_;
  std::string robot_id_;
  std::string mission_builder_node_name_;
  std::string mission_builder_build_service_;
  std::string fsm_request_service_;
  std::string active_costmap_output_basename_;
  std::string active_route_output_basename_;
  std::string active_execution_pointer_filename_;
  int horizon_hours_{72};
  int running_profile_id_{201};
  double tick_seconds_{1.0};
  bool trigger_running_on_work_window_{true};
  double schedule_poll_interval_sec_{60.0};
  bool reload_on_mtime_change_{true};
  bool reload_on_every_poll_{false};
  bool emit_rosout_triggers_{true};
  std::string rosout_trigger_prefix_{"FSM_TRIGGER"};
  bool emit_trigger_topic_{true};
  std::string trigger_topic_name_{"scheduler_triggers"};
  bool mission_build_in_flight_{false};
  bool running_request_in_flight_{false};
  std::string mission_build_target_;
  std::string prepared_active_mission_;
  std::string prepared_active_window_uid_;
  std::string prepared_execution_directory_;
  std::string running_request_window_uid_;
  std::optional<std::time_t> last_mtime_;
  std::unordered_map<std::string, std::string> mission_catalog_;
};

}  // namespace amr_sweeper_scheduler
