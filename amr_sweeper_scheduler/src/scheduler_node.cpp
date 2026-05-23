#include "scheduler_node.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace amr_sweeper_scheduler
{

namespace
{

std::optional<std::time_t> file_mtime(const std::string & path)
{
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return std::nullopt;
  }
  return st.st_mtime;
}

std::filesystem::path resolve_path(const std::string & path)
{
  const std::filesystem::path configured(path);
  if (configured.is_absolute()) {
    return configured;
  }
  const std::filesystem::path workspace_relative = std::filesystem::current_path() / configured;
  if (std::filesystem::exists(workspace_relative)) {
    return workspace_relative;
  }
  return configured;
}

std::tm parse_local_tm(const std::string & value)
{
  if (value.size() < 15) {
    throw std::runtime_error("Invalid local timestamp: " + value);
  }

  std::tm tm{};
  tm.tm_year = std::stoi(value.substr(0, 4)) - 1900;
  tm.tm_mon = std::stoi(value.substr(4, 2)) - 1;
  tm.tm_mday = std::stoi(value.substr(6, 2));
  tm.tm_hour = std::stoi(value.substr(9, 2));
  tm.tm_min = std::stoi(value.substr(11, 2));
  tm.tm_sec = std::stoi(value.substr(13, 2));
  tm.tm_isdst = -1;
  return tm;
}

std::chrono::system_clock::time_point to_time_point(const std::string & value)
{
  auto tm = parse_local_tm(value);
  return std::chrono::system_clock::from_time_t(std::mktime(&tm));
}

std::string format_local_timestamp(const std::tm & tm)
{
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%S", &tm);
  return buffer;
}

std::string format_utc_timestamp(const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  const std::tm utc_tm = *std::gmtime(&as_time_t);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%SZ", &utc_tm);
  return buffer;
}

std::tm time_point_to_tm(const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  return *std::localtime(&as_time_t);
}

std::string trim_cr(std::string value)
{
  if (!value.empty() && value.back() == '\r') {
    value.pop_back();
  }
  return value;
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::pair<std::string, std::string> split_kv(const std::string & line)
{
  const auto pos = line.find(':');
  if (pos == std::string::npos) {
    return {line, ""};
  }
  return {line.substr(0, pos), line.substr(pos + 1)};
}

struct ParsedRRule
{
  std::string freq;
  std::optional<int> by_set_pos;
  std::vector<std::string> by_day;
};

std::chrono::seconds parse_duration_seconds(const std::string & value)
{
  if (value.size() < 3 || value.front() != 'P') {
    throw std::runtime_error("Unsupported DURATION format: " + value);
  }

  int hours = 0;
  int minutes = 0;
  int seconds = 0;
  std::string number;
  bool in_time = false;
  for (char character : value.substr(1)) {
    if (character == 'T') {
      in_time = true;
      continue;
    }
    if (std::isdigit(static_cast<unsigned char>(character))) {
      number.push_back(character);
      continue;
    }
    if (!in_time || number.empty()) {
      continue;
    }
    const int parsed = std::stoi(number);
    if (character == 'H') {
      hours = parsed;
    } else if (character == 'M') {
      minutes = parsed;
    } else if (character == 'S') {
      seconds = parsed;
    }
    number.clear();
  }

  return std::chrono::hours(hours) + std::chrono::minutes(minutes) + std::chrono::seconds(seconds);
}

ParsedRRule parse_rrule(const std::string & rule)
{
  ParsedRRule parsed;
  std::stringstream stream(rule);
  std::string token;
  while (std::getline(stream, token, ';')) {
    const auto equals = token.find('=');
    if (equals == std::string::npos) {
      continue;
    }
    const std::string key = token.substr(0, equals);
    const std::string value = token.substr(equals + 1);
    if (key == "FREQ") {
      parsed.freq = value;
    } else if (key == "BYSETPOS") {
      parsed.by_set_pos = std::stoi(value);
    } else if (key == "BYDAY") {
      std::stringstream day_stream(value);
      std::string day_token;
      while (std::getline(day_stream, day_token, ',')) {
        parsed.by_day.push_back(day_token);
      }
    }
  }
  return parsed;
}

int weekday_from_byday(const std::string & value)
{
  static const std::unordered_map<std::string, int> lookup{
    {"SU", 0}, {"MO", 1}, {"TU", 2}, {"WE", 3}, {"TH", 4}, {"FR", 5}, {"SA", 6}};
  return lookup.at(value);
}

int days_in_month(int year, int month)
{
  static const std::array<int, 12> month_lengths{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 1) {
    const bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    return leap ? 29 : 28;
  }
  return month_lengths.at(static_cast<std::size_t>(month));
}

std::optional<std::tm> nth_weekday_of_month(
  const std::tm & seed,
  int year,
  int month,
  int weekday,
  int setpos)
{
  std::vector<int> matching_days;
  const int last_day = days_in_month(year, month);
  for (int day = 1; day <= last_day; ++day) {
    std::tm candidate = seed;
    candidate.tm_year = year - 1900;
    candidate.tm_mon = month;
    candidate.tm_mday = day;
    std::mktime(&candidate);
    if (candidate.tm_wday == weekday) {
      matching_days.push_back(day);
    }
  }

  if (matching_days.empty()) {
    return std::nullopt;
  }

  int index = setpos;
  if (index < 0) {
    index = static_cast<int>(matching_days.size()) + index + 1;
  }
  if (index <= 0 || index > static_cast<int>(matching_days.size())) {
    return std::nullopt;
  }

  std::tm result = seed;
  result.tm_year = year - 1900;
  result.tm_mon = month;
  result.tm_mday = matching_days.at(static_cast<std::size_t>(index - 1));
  std::mktime(&result);
  return result;
}

std::optional<std::chrono::system_clock::time_point> compute_window_end(
  const ScheduleEvent & event,
  const std::chrono::system_clock::time_point & start_time)
{
  if (event.duration) {
    return start_time + parse_duration_seconds(*event.duration);
  }
  if (event.dtend_local) {
    return to_time_point(*event.dtend_local);
  }
  return std::nullopt;
}

TimeWindow build_window(
  const ScheduleEvent & event,
  const std::chrono::system_clock::time_point & start_time,
  const std::chrono::system_clock::time_point & end_time)
{
  TimeWindow window;
  window.uid = event.uid;
  window.robot_id = event.robot_id;
  window.type = event.type;
  window.mission_id = event.mission_id;
  window.start_local = format_local_timestamp(time_point_to_tm(start_time));
  window.end_local = format_local_timestamp(time_point_to_tm(end_time));
  return window;
}

bool overlaps(const TimeWindow & left, const TimeWindow & right)
{
  const auto left_start = to_time_point(left.start_local);
  const auto left_end = to_time_point(left.end_local);
  const auto right_start = to_time_point(right.start_local);
  const auto right_end = to_time_point(right.end_local);
  return !(left_end <= right_start || left_start >= right_end);
}

}  // namespace

std::optional<ScheduleType> schedule_type_from_string(const std::string & value)
{
  if (value == "WORK") {
    return ScheduleType::WORK;
  }
  if (value == "NO_WORK") {
    return ScheduleType::NO_WORK;
  }
  return std::nullopt;
}

const char * schedule_type_to_cstr(const ScheduleType type)
{
  return type == ScheduleType::WORK ? "WORK" : "NO_WORK";
}

ScheduleModel IcalParserMinimal::parse_file(const std::string & ics_path, const ParserConfig & config)
{
  std::ifstream input_stream(ics_path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open ICS file: " + ics_path);
  }

  ScheduleModel model;
  bool in_vevent = false;
  ScheduleEvent current;
  int event_count = 0;

  std::string line;
  while (std::getline(input_stream, line)) {
    line = trim_cr(line);

    if (!in_vevent) {
      if (starts_with(line, "X-WR-TIMEZONE:")) {
        model.calendar_tzid = line.substr(std::string("X-WR-TIMEZONE:").size());
      }
      if (line == "BEGIN:VEVENT") {
        in_vevent = true;
        current = ScheduleEvent{};
      }
      continue;
    }

    if (line == "END:VEVENT") {
      if (current.uid.empty()) {
        throw std::runtime_error("ICS invalid VEVENT: missing UID");
      }
      if (current.dtstart_local.empty() || current.dtstart_tzid.empty()) {
        throw std::runtime_error("ICS invalid VEVENT uid=" + current.uid + ": missing DTSTART/TZID");
      }
      if (!current.duration && !current.dtend_local) {
        throw std::runtime_error("ICS invalid VEVENT uid=" + current.uid + ": missing DURATION/DTEND");
      }
      if (config.require_x_robot_id && current.robot_id.empty()) {
        throw std::runtime_error("ICS invalid VEVENT uid=" + current.uid + ": missing X-ROBOT-ID");
      }
      if (config.require_x_mission_id_for_work && current.type == ScheduleType::WORK) {
        if (!current.mission_id || current.mission_id->empty()) {
          throw std::runtime_error("ICS invalid VEVENT uid=" + current.uid + ": WORK missing X-MISSION-ID");
        }
      }

      model.events.push_back(current);
      ++event_count;
      if (event_count > config.max_events) {
        throw std::runtime_error("ICS too many VEVENTs (max_events exceeded)");
      }

      in_vevent = false;
      continue;
    }

    if (starts_with(line, "UID:")) {
      current.uid = line.substr(4);
      continue;
    }
    if (starts_with(line, "RRULE:")) {
      current.rrule = line.substr(6);
      continue;
    }
    if (starts_with(line, "DURATION:")) {
      current.duration = line.substr(9);
      continue;
    }
    if (starts_with(line, "DTEND")) {
      current.dtend_local = split_kv(line).second;
      continue;
    }
    if (starts_with(line, "DTSTART")) {
      const auto key_value = split_kv(line);
      const std::string & key = key_value.first;
      current.dtstart_local = key_value.second;
      const std::string tzid_tag = "TZID=";
      const auto tz_pos = key.find(tzid_tag);
      if (tz_pos != std::string::npos) {
        auto start = tz_pos + tzid_tag.size();
        auto end = key.find(';', start);
        if (end == std::string::npos) {
          end = key.size();
        }
        current.dtstart_tzid = key.substr(start, end - start);
      } else {
        current.dtstart_tzid.clear();
      }
      continue;
    }
    if (starts_with(line, "X-ROBOT-ID:")) {
      current.robot_id = line.substr(std::string("X-ROBOT-ID:").size());
      continue;
    }
    if (starts_with(line, "X-SCHEDULE-TYPE:")) {
      const auto value = line.substr(std::string("X-SCHEDULE-TYPE:").size());
      const auto type = schedule_type_from_string(value);
      if (!type) {
        if (config.strict_validation) {
          throw std::runtime_error("ICS invalid VEVENT: unknown X-SCHEDULE-TYPE=" + value);
        }
      } else {
        current.type = *type;
      }
      continue;
    }
    if (starts_with(line, "X-MISSION-ID:")) {
      current.mission_id = line.substr(std::string("X-MISSION-ID:").size());
      continue;
    }
  }

  if (in_vevent) {
    throw std::runtime_error("ICS parse error: unterminated VEVENT");
  }
  if (model.events.empty()) {
    throw std::runtime_error("ICS contains no VEVENTs");
  }

  return model;
}

std::vector<TimeWindow> ScheduleExpanderStub::expand(
  const ScheduleModel & model,
  const std::string & robot_id,
  const std::chrono::system_clock::time_point & now,
  const std::chrono::hours & horizon)
{
  std::vector<TimeWindow> windows;
  const auto horizon_end = now + horizon;

  for (const auto & event : model.events) {
    if (!event.robot_id.empty() && event.robot_id != robot_id) {
      continue;
    }

    const std::tm seed_tm = parse_local_tm(event.dtstart_local);
    const auto seed_start = to_time_point(event.dtstart_local);
    const auto seed_end = compute_window_end(event, seed_start);
    if (!seed_end) {
      continue;
    }

    if (!event.rrule) {
      if (*seed_end >= now && seed_start <= horizon_end) {
        windows.push_back(build_window(event, seed_start, *seed_end));
      }
      continue;
    }

    const ParsedRRule rule = parse_rrule(*event.rrule);
    if (rule.freq == "DAILY") {
      for (auto occurrence = seed_start; occurrence <= horizon_end; occurrence += std::chrono::hours(24)) {
        const auto occurrence_end = occurrence + (*seed_end - seed_start);
        if (occurrence_end < now) {
          continue;
        }
        windows.push_back(build_window(event, occurrence, occurrence_end));
      }
      continue;
    }

    if (rule.freq == "MONTHLY" && rule.by_set_pos && !rule.by_day.empty()) {
      std::tm cursor = time_point_to_tm(now);
      for (int month_offset = 0; month_offset <= horizon.count() / 24 / 28 + 2; ++month_offset) {
        const int month_index = cursor.tm_mon + month_offset;
        const int year = cursor.tm_year + 1900 + month_index / 12;
        const int month = month_index % 12;
        const auto occurrence_tm = nth_weekday_of_month(
          seed_tm,
          year,
          month,
          weekday_from_byday(rule.by_day.front()),
          *rule.by_set_pos);
        if (!occurrence_tm) {
          continue;
        }

        auto occurrence_tm_value = *occurrence_tm;
        const auto occurrence_start =
          std::chrono::system_clock::from_time_t(std::mktime(&occurrence_tm_value));
        if (occurrence_start < seed_start) {
          continue;
        }
        const auto occurrence_end = occurrence_start + (*seed_end - seed_start);
        if (occurrence_end < now) {
          continue;
        }
        if (occurrence_start > horizon_end) {
          break;
        }
        windows.push_back(build_window(event, occurrence_start, occurrence_end));
      }
    }
  }

  std::sort(
    windows.begin(),
    windows.end(),
    [](const TimeWindow & left, const TimeWindow & right) {
      return left.start_local < right.start_local;
    });
  return windows;
}

std::vector<TimeWindow> apply_blackout_overlay(const std::vector<TimeWindow> & windows)
{
  std::vector<TimeWindow> work_windows;
  std::vector<TimeWindow> no_work_windows;

  for (const auto & window : windows) {
    if (window.type == ScheduleType::NO_WORK) {
      no_work_windows.push_back(window);
    } else {
      work_windows.push_back(window);
    }
  }

  std::vector<TimeWindow> filtered_windows;
  filtered_windows.reserve(work_windows.size());
  for (const auto & work_window : work_windows) {
    bool blocked = false;
    for (const auto & no_work_window : no_work_windows) {
      if (overlaps(work_window, no_work_window)) {
        blocked = true;
        break;
      }
    }
    if (!blocked) {
      filtered_windows.push_back(work_window);
    }
  }

  return filtered_windows;
}

SchedulerNode::SchedulerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("amr_sweeper_scheduler", options)
{
  schedule_ics_path_ = declare_parameter<std::string>("schedule_ics_path", "");
  missions_directory_ = declare_parameter<std::string>("missions_directory", "src/missions");
  default_schedule_filename_ = declare_parameter<std::string>(
    "default_schedule_filename",
    "");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  robot_id_ = declare_parameter<std::string>("robot_id", "");
  mission_builder_node_name_ = declare_parameter<std::string>(
    "mission_builder_node_name",
    "mission_builder_node");
  mission_builder_build_service_ = declare_parameter<std::string>(
    "mission_builder_build_service",
    "build_current_mission");
  fsm_request_service_ = declare_parameter<std::string>("fsm_request_service", "request_state");
  active_costmap_output_basename_ = declare_parameter<std::string>(
    "active_costmap_output_basename",
    "global_costmap");
  active_route_output_basename_ = declare_parameter<std::string>(
    "active_route_output_basename",
    "active_mission_path");
  active_execution_pointer_filename_ = declare_parameter<std::string>(
    "active_execution_pointer_filename",
    "active_execution.json");
  horizon_hours_ = declare_parameter<int>("horizon_hours", 72);
  running_profile_id_ = declare_parameter<int>("running_profile_id", 201);
  tick_seconds_ = declare_parameter<double>("tick_seconds", 2.0);
  trigger_running_on_work_window_ = declare_parameter<bool>(
    "trigger_running_on_work_window",
    true);
  schedule_poll_interval_sec_ = declare_parameter<double>("schedule_poll_interval_sec", 60.0);
  reload_on_mtime_change_ = declare_parameter<bool>("reload_on_mtime_change", true);
  reload_on_every_poll_ = declare_parameter<bool>("reload_on_every_poll", false);
  declare_parameter<bool>("strict_validation", true);
  declare_parameter<int>("max_events", 2000);
  declare_parameter<bool>("require_x_robot_id", true);
  declare_parameter<bool>("require_x_schedule_type", true);
  declare_parameter<bool>("require_x_mission_id_for_work", true);
  emit_rosout_triggers_ = declare_parameter<bool>("emit_rosout_triggers", true);
  rosout_trigger_prefix_ = declare_parameter<std::string>(
    "rosout_trigger_prefix",
    "FSM_TRIGGER");
  emit_trigger_topic_ = declare_parameter<bool>("emit_trigger_topic", true);
  trigger_topic_name_ = declare_parameter<std::string>(
    "trigger_topic_name",
    "scheduler_triggers");

  planned_pub_ = create_publisher<std_msgs::msg::String>("planned_windows", 10);
  if (emit_trigger_topic_) {
    trigger_pub_ = create_publisher<std_msgs::msg::String>(trigger_topic_name_, 10);
  }

  mission_builder_parameter_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, mission_builder_node_name_);
  mission_builder_build_client_ = create_client<std_srvs::srv::Trigger>(
    mission_builder_build_service_);
  fsm_request_client_ = create_client<amr_sweeper_layer_0_fsm::srv::RequestState>(
    fsm_request_service_);

  reload_srv_ = create_service<std_srvs::srv::Trigger>(
    "reload_schedule",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      try {
        last_mtime_.reset();
        load_schedule();
        response->success = true;
        response->message = "Schedule reloaded";
      } catch (const std::exception & exception) {
        response->success = false;
        response->message = exception.what();
      }
    });

  parser_ = std::make_unique<IcalParserMinimal>();
  expander_ = std::make_unique<ScheduleExpanderStub>();

  tick_timer_ = create_wall_timer(
    std::chrono::duration<double>(tick_seconds_),
    std::bind(&SchedulerNode::tick, this));
  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(schedule_poll_interval_sec_),
    std::bind(&SchedulerNode::poll_schedule, this));

  poll_schedule();
}

void SchedulerNode::trigger_info(const std::string & code, const std::string & kv)
{
  if (emit_rosout_triggers_) {
    RCLCPP_INFO(get_logger(), "%s %s %s", rosout_trigger_prefix_.c_str(), code.c_str(), kv.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = rosout_trigger_prefix_ + std::string(" ") + code + (kv.empty() ? "" : " " + kv);
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::trigger_warn(const std::string & code, const std::string & kv)
{
  if (emit_rosout_triggers_) {
    RCLCPP_WARN(get_logger(), "%s %s %s", rosout_trigger_prefix_.c_str(), code.c_str(), kv.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = rosout_trigger_prefix_ + std::string(" ") + code + (kv.empty() ? "" : " " + kv);
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::trigger_error(const std::string & code, const std::string & kv)
{
  if (emit_rosout_triggers_) {
    RCLCPP_ERROR(get_logger(), "%s %s %s", rosout_trigger_prefix_.c_str(), code.c_str(), kv.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = rosout_trigger_prefix_ + std::string(" ") + code + (kv.empty() ? "" : " " + kv);
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::poll_schedule()
{
  if (robot_id_.empty()) {
    trigger_warn("SCHED_PARAMS_MISSING", "set robot_id");
    return;
  }

  refresh_mission_catalog();

  const std::string schedule_path = resolved_schedule_path();
  const auto mtime = file_mtime(schedule_path);
  if (!mtime) {
    trigger_error("SCHED_ICS_NOT_FOUND", "path=" + schedule_path);
    schedule_loaded_ = false;
    return;
  }

  const bool must_reload =
    reload_on_every_poll_ || !last_mtime_.has_value() ||
    (reload_on_mtime_change_ && last_mtime_.value() != mtime.value());
  if (!must_reload) {
    return;
  }

  try {
    load_schedule();
    last_mtime_ = mtime;
    trigger_info("SCHED_ICS_LOADED", "events=" + std::to_string(schedule_.events.size()));
  } catch (const std::exception & exception) {
    trigger_error("SCHED_ICS_LOAD_FAILED", std::string("reason=") + exception.what());
    schedule_loaded_ = false;
  }
}

void SchedulerNode::load_schedule()
{
  ParserConfig config;
  config.strict_validation = get_parameter("strict_validation").as_bool();
  config.max_events = static_cast<int>(get_parameter("max_events").as_int());
  config.require_x_robot_id = get_parameter("require_x_robot_id").as_bool();
  config.require_x_schedule_type = get_parameter("require_x_schedule_type").as_bool();
  config.require_x_mission_id_for_work =
    get_parameter("require_x_mission_id_for_work").as_bool();

  schedule_ = parser_->parse_file(resolved_schedule_path(), config);
  schedule_loaded_ = true;
  if (schedule_.calendar_tzid.empty()) {
    trigger_warn("SCHED_ICS_NO_CAL_TZ", "using DTSTART TZID only");
  }
}

void SchedulerNode::refresh_mission_catalog()
{
  mission_catalog_.clear();
  const std::filesystem::path directory = resolve_path(missions_directory_);
  if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
    return;
  }

  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == mission_file_extension_) {
      const std::filesystem::path canonical_path =
        directory / entry.path().stem() / (entry.path().stem().string() + mission_file_extension_);
      mission_catalog_[entry.path().stem().string()] = canonical_path.string();
      continue;
    }
    if (!entry.is_directory()) {
      continue;
    }
    for (const auto & nested_entry : std::filesystem::directory_iterator(entry.path())) {
      if (!nested_entry.is_regular_file() || nested_entry.path().extension() != mission_file_extension_) {
        continue;
      }
      mission_catalog_[nested_entry.path().stem().string()] = nested_entry.path().string();
      mission_catalog_[entry.path().filename().string()] = nested_entry.path().string();
    }
  }
}

void SchedulerNode::tick()
{
  if (!schedule_loaded_) {
    return;
  }

  const auto now = std::chrono::system_clock::now();
  const auto horizon = std::chrono::hours(horizon_hours_);
  auto windows = expander_->expand(schedule_, robot_id_, now, horizon);
  windows = apply_blackout_overlay(windows);

  for (auto & window : windows) {
    if (window.type == ScheduleType::WORK && window.mission_id) {
      if (!mission_json_or_folder_exists(*window.mission_id)) {
        trigger_warn("SCHED_MISSION_NOT_FOUND", "mission_id=" + *window.mission_id);
        continue;
      }
      window.mission_path = resolve_mission_path(*window.mission_id);
      if (!window.mission_path) {
        trigger_warn("SCHED_MISSION_NOT_FOUND", "mission_id=" + *window.mission_id);
      }
    }
  }

  publish_windows(windows);
  maybe_promote_mission(windows);
}

void SchedulerNode::publish_windows(const std::vector<TimeWindow> & windows)
{
  std_msgs::msg::String msg;
  std::ostringstream stream;
  stream << "{"
         << "\"robot_id\":\"" << robot_id_ << "\","
         << "\"window_count\":" << windows.size() << ","
         << "\"windows\":[";

  bool first = true;
  for (const auto & window : windows) {
    if (!first) {
      stream << ",";
    }
    stream << "{"
           << "\"uid\":\"" << window.uid << "\","
           << "\"type\":\"" << schedule_type_to_cstr(window.type) << "\","
           << "\"start\":\"" << window.start_local << "\","
           << "\"end\":\"" << window.end_local << "\"";
    if (window.mission_id) {
      stream << ",\"mission_id\":\"" << *window.mission_id << "\"";
      if (window.mission_path) {
        stream << ",\"mission_path\":\"" << *window.mission_path << "\"";
      }
    }
    stream << "}";
    first = false;
  }
  stream << "]}";
  msg.data = stream.str();
  planned_pub_->publish(msg);

  if (windows.empty()) {
    trigger_warn("SCHED_NO_WINDOWS", "robot_id=" + robot_id_);
  }
}

void SchedulerNode::maybe_promote_mission(const std::vector<TimeWindow> & windows)
{
  if (!trigger_running_on_work_window_) {
    return;
  }

  const auto now = std::chrono::system_clock::now();
  for (const auto & window : windows) {
    if (window.type != ScheduleType::WORK || !window.mission_path) {
      continue;
    }

    const auto start = to_time_point(window.start_local);
    const auto end = to_time_point(window.end_local);
    if (now < start || now > end) {
      continue;
    }

    if (!mission_artifacts_ready(*window.mission_path)) {
      request_mission_build(*window.mission_path);
      return;
    }

    if (prepared_active_mission_ != *window.mission_path ||
      prepared_active_window_uid_ != window.uid)
    {
      if (!prepare_active_mission_execution(window)) {
        trigger_error("SCHED_MISSION_EXECUTION_PREP_FAILED", *window.mission_path);
        return;
      }
      prepared_active_mission_ = *window.mission_path;
      prepared_active_window_uid_ = window.uid;
    }

    if (!running_request_in_flight_ && running_request_window_uid_ != window.uid) {
      request_running_state(window);
    }
    return;
  }

  running_request_window_uid_.clear();
}

void SchedulerNode::request_mission_build(const std::string & mission_path)
{
  if (mission_build_in_flight_ || mission_build_target_ == mission_path) {
    return;
  }
  if (!mission_builder_parameter_client_->service_is_ready()) {
    trigger_warn("SCHED_MISSION_BUILDER_UNAVAILABLE", mission_builder_node_name_);
    return;
  }

  mission_build_in_flight_ = true;
  mission_build_target_ = mission_path;
  mission_builder_parameter_client_->set_parameters(
    {rclcpp::Parameter("mission_path", mission_path)},
    [this, mission_path](
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> result_future)
    {
      bool accepted = true;
      for (const auto & result : result_future.get()) {
        if (!result.successful) {
          accepted = false;
          trigger_error("SCHED_MISSION_BUILD_SET_PARAM_FAILED", result.reason);
        }
      }
      if (!accepted) {
        mission_build_in_flight_ = false;
        mission_build_target_.clear();
        return;
      }

      if (!mission_builder_build_client_->service_is_ready()) {
        trigger_warn("SCHED_MISSION_BUILD_SERVICE_UNAVAILABLE", mission_builder_build_service_);
        mission_build_in_flight_ = false;
        mission_build_target_.clear();
        return;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      mission_builder_build_client_->async_send_request(
        request,
        [this, mission_path](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response_future)
        {
          const auto response = response_future.get();
          mission_build_in_flight_ = false;
          mission_build_target_.clear();
          if (response->success) {
            trigger_info("SCHED_MISSION_BUILD_OK", mission_path);
          } else {
            trigger_error("SCHED_MISSION_BUILD_FAILED", response->message);
          }
        });
    });
}

bool SchedulerNode::mission_json_or_folder_exists(const std::string & mission_id) const
{
  const std::filesystem::path missions_directory = resolve_path(missions_directory_);
  const std::filesystem::path mission_file = missions_directory / (mission_id + mission_file_extension_);
  const std::filesystem::path mission_folder = missions_directory / mission_id;
  const std::filesystem::path mission_folder_file = mission_folder / (mission_id + mission_file_extension_);
  return std::filesystem::exists(mission_file) ||
         (std::filesystem::exists(mission_folder) && std::filesystem::exists(mission_folder_file));
}

bool SchedulerNode::prepare_active_mission_execution(const TimeWindow & window)
{
  if (!window.mission_path || !window.mission_id) {
    return false;
  }

  const std::filesystem::path mission_file(*window.mission_path);
  const std::filesystem::path mission_folder = mission_folder_path(*window.mission_path);
  const std::filesystem::path mission_costmap_yaml(mission_costmap_yaml_path(*window.mission_path));
  const std::filesystem::path mission_costmap_image(mission_costmap_image_path(*window.mission_path));
  const std::filesystem::path mission_route(mission_route_path(*window.mission_path));

  if (!std::filesystem::exists(mission_file) ||
    !std::filesystem::exists(mission_costmap_yaml) ||
    !std::filesystem::exists(mission_costmap_image) ||
    !std::filesystem::exists(mission_route))
  {
    return false;
  }

  const std::chrono::system_clock::time_point run_start = std::chrono::system_clock::now();
  const std::string run_timestamp = format_utc_timestamp(run_start);
  const std::filesystem::path mission_run_directory = mission_folder / run_timestamp;
  std::filesystem::create_directories(mission_run_directory);

  std::filesystem::copy_file(
    mission_costmap_yaml,
    active_costmap_yaml_path(),
    std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(
    mission_costmap_image,
    active_costmap_image_path(),
    std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(
    mission_route,
    active_route_path(),
    std::filesystem::copy_options::overwrite_existing);

  {
    std::ifstream yaml_input(active_costmap_yaml_path());
    if (!yaml_input.is_open()) {
      return false;
    }
    std::ostringstream yaml_buffer;
    std::string yaml_line;
    while (std::getline(yaml_input, yaml_line)) {
      if (yaml_line.rfind("image:", 0) == 0) {
        yaml_buffer << "image: " << active_costmap_output_basename_ << ".pgm\n";
      } else {
        yaml_buffer << yaml_line << "\n";
      }
    }
    std::ofstream yaml_output(active_costmap_yaml_path(), std::ios::trunc);
    if (!yaml_output.is_open()) {
      return false;
    }
    yaml_output << yaml_buffer.str();
  }

  nlohmann::json context{
    {"mission_id", *window.mission_id},
    {"mission_file", mission_file.string()},
    {"mission_folder", mission_folder.string()},
    {"mission_route_file", mission_route.string()},
    {"mission_costmap_yaml", mission_costmap_yaml.string()},
    {"mission_run_directory", mission_run_directory.string()},
    {"mission_window_start", window.start_local},
    {"mission_window_end", window.end_local},
    {"run_started_at", run_timestamp}};

  std::ofstream context_stream(mission_run_directory / "execution_context.json");
  if (!context_stream.is_open()) {
    return false;
  }
  context_stream << std::setw(2) << context << '\n';

  const nlohmann::json execution_pointer{
    {"mission_id", *window.mission_id},
    {"mission_folder", mission_folder.string()},
    {"mission_run_directory", mission_run_directory.string()},
    {"execution_context_file", (mission_run_directory / "execution_context.json").string()},
    {"mission_window_start", window.start_local},
    {"mission_window_end", window.end_local}};
  std::ofstream pointer_stream(
    resolve_path(missions_directory_) / active_execution_pointer_filename_,
    std::ios::trunc);
  if (!pointer_stream.is_open()) {
    return false;
  }
  pointer_stream << std::setw(2) << execution_pointer << '\n';

  prepared_execution_directory_ = mission_run_directory.string();
  return true;
}

void SchedulerNode::request_running_state(const TimeWindow & window)
{
  if (!fsm_request_client_->service_is_ready()) {
    trigger_warn("SCHED_FSM_SERVICE_UNAVAILABLE", fsm_request_service_);
    return;
  }

  running_request_in_flight_ = true;
  auto request = std::make_shared<amr_sweeper_layer_0_fsm::srv::RequestState::Request>();
  request->target_state = "RUNNING";
  request->target_lifecycle = "Active";
  request->target_profile_id = static_cast<std::uint16_t>(running_profile_id_);
  request->requester = "amr_sweeper_scheduler";
  request->priority = 210;
  request->force = false;
  request->reason =
    "Scheduled mission window active; mission_id=" +
    window.mission_id.value_or(std::string("unknown")) +
    "; start=" + window.start_local +
    "; end=" + window.end_local;
  request->mission_execution_directory = prepared_execution_directory_;
  fsm_request_client_->async_send_request(
    request,
    [this, window](
      rclcpp::Client<amr_sweeper_layer_0_fsm::srv::RequestState>::SharedFuture response_future)
    {
      running_request_in_flight_ = false;
      const auto response = response_future.get();
      if (response->accepted) {
        running_request_window_uid_ = window.uid;
        trigger_info(
          "SCHED_PROMOTED_TO_RUNNING",
          "mission_id=" + window.mission_id.value_or(std::string("unknown")));
      } else {
        trigger_warn("SCHED_RUNNING_REQUEST_REJECTED", response->message);
      }
    });
}

bool SchedulerNode::mission_artifacts_ready(const std::string & mission_path) const
{
  const std::filesystem::path mission_file(mission_path);
  const std::filesystem::path mission_costmap(mission_costmap_yaml_path(mission_path));
  const std::filesystem::path mission_route(mission_route_path(mission_path));

  if (!std::filesystem::exists(mission_file) ||
    !std::filesystem::exists(mission_costmap) ||
    !std::filesystem::exists(mission_route))
  {
    return false;
  }

  const auto mission_stamp = std::filesystem::last_write_time(mission_file);
  return std::filesystem::last_write_time(mission_costmap) >= mission_stamp &&
         std::filesystem::last_write_time(mission_route) >= mission_stamp;
}

std::string SchedulerNode::mission_costmap_yaml_path(const std::string & mission_path) const
{
  const std::filesystem::path path(mission_path);
  return (mission_folder_path(mission_path) / (path.stem().string() + "_costmap.yaml")).string();
}

std::string SchedulerNode::mission_route_path(const std::string & mission_path) const
{
  const std::filesystem::path path(mission_path);
  return (mission_folder_path(mission_path) / (path.stem().string() + "_path.geojson")).string();
}

std::filesystem::path SchedulerNode::mission_folder_path(const std::string & mission_path) const
{
  const std::filesystem::path path(mission_path);
  return path.parent_path();
}

std::string SchedulerNode::mission_costmap_image_path(const std::string & mission_path) const
{
  const std::filesystem::path path(mission_path);
  return (mission_folder_path(mission_path) / (path.stem().string() + "_costmap.pgm")).string();
}

std::string SchedulerNode::active_route_path() const
{
  return (resolve_path(missions_directory_) / (active_route_output_basename_ + ".geojson")).string();
}

std::string SchedulerNode::active_costmap_yaml_path() const
{
  return (resolve_path(missions_directory_) / (active_costmap_output_basename_ + ".yaml")).string();
}

std::string SchedulerNode::active_costmap_image_path() const
{
  return (resolve_path(missions_directory_) / (active_costmap_output_basename_ + ".pgm")).string();
}

std::string SchedulerNode::resolved_schedule_path() const
{
  if (!schedule_ics_path_.empty()) {
    return resolve_path(schedule_ics_path_).string();
  }
  if (const auto discovered_path = discover_latest_schedule_path()) {
    return discovered_path->string();
  }
  if (!default_schedule_filename_.empty()) {
    return (resolve_path(missions_directory_) / default_schedule_filename_).string();
  }
  return (resolve_path(missions_directory_) / "schedule.ics").string();
}

std::optional<std::filesystem::path> SchedulerNode::discover_latest_schedule_path() const
{
  const std::filesystem::path missions_directory = resolve_path(missions_directory_);
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return std::nullopt;
  }

  std::optional<std::filesystem::path> latest_path;
  std::optional<std::filesystem::file_time_type> latest_mtime;
  for (const auto & entry : std::filesystem::directory_iterator(missions_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto & path = entry.path();
    if (path.extension() != ".ics") {
      continue;
    }
    const std::string filename = path.filename().string();
    if (filename.rfind("schedule_", 0) != 0) {
      continue;
    }

    const auto mtime = std::filesystem::last_write_time(path);
    if (!latest_mtime || mtime > *latest_mtime) {
      latest_mtime = mtime;
      latest_path = path;
    }
  }

  return latest_path;
}

std::optional<std::string> SchedulerNode::resolve_mission_path(const std::string & mission_id) const
{
  const auto mission_it = mission_catalog_.find(mission_id);
  if (mission_it != mission_catalog_.end()) {
    return mission_it->second;
  }
  if (mission_catalog_.size() == 1U) {
    return mission_catalog_.begin()->second;
  }
  return std::nullopt;
}

}  // namespace amr_sweeper_scheduler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_scheduler::SchedulerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
