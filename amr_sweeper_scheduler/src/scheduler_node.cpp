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

namespace amr_sweeper_scheduler
{

namespace
{

constexpr char kLowerPriorityRejectedPrefix[] = "Rejected: lower priority than last request";
constexpr auto kLowerPriorityRetryCooldown = std::chrono::seconds(30);
constexpr char kLegacyRobotId[] = "RBT-01";
constexpr char kDefaultRobotConfigEnvPath[] = "/opt/robot_config/robot_config.global.env";

std::string format_local_timestamp(const std::tm & tm);
std::tm time_point_to_tm(const std::chrono::system_clock::time_point & time_point);

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

std::string unique_archived_filename(
  const std::filesystem::path & archive_directory,
  const std::filesystem::path & source_path)
{
  const std::string stem = source_path.stem().string();
  const std::string extension = source_path.extension().string();
  std::filesystem::path candidate = archive_directory / source_path.filename();
  int suffix = 1;
  while (std::filesystem::exists(candidate)) {
    candidate = archive_directory / (stem + "_" + std::to_string(suffix) + extension);
    ++suffix;
  }
  return candidate.string();
}

std::optional<std::filesystem::path> archive_older_schedule_files(
  const std::filesystem::path & missions_directory)
{
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return std::nullopt;
  }

  std::vector<std::filesystem::directory_entry> schedule_entries;
  for (const auto & entry : std::filesystem::directory_iterator(missions_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string filename = entry.path().filename().string();
    if (entry.path().extension() != ".ics" || filename.rfind("schedule_", 0) != 0) {
      continue;
    }
    schedule_entries.push_back(entry);
  }

  if (schedule_entries.empty()) {
    return std::nullopt;
  }

  std::sort(
    schedule_entries.begin(),
    schedule_entries.end(),
    [](const auto & left, const auto & right) {
      return std::filesystem::last_write_time(left.path()) >
             std::filesystem::last_write_time(right.path());
    });

  const std::filesystem::path newest_path = schedule_entries.front().path();
  if (schedule_entries.size() == 1U) {
    return newest_path;
  }

  const std::filesystem::path archive_directory = missions_directory / "archive";
  std::filesystem::create_directories(archive_directory);
  for (std::size_t index = 1; index < schedule_entries.size(); ++index) {
    const auto & source_path = schedule_entries[index].path();
    const std::filesystem::path archived_path(unique_archived_filename(archive_directory, source_path));
    std::filesystem::rename(source_path, archived_path);
  }

  return newest_path;
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

std::chrono::system_clock::time_point utc_to_time_point(const std::string & value)
{
  if (value.size() < 16 || value.back() != 'Z') {
    throw std::runtime_error("Invalid UTC timestamp: " + value);
  }

  std::tm tm{};
  tm.tm_year = std::stoi(value.substr(0, 4)) - 1900;
  tm.tm_mon = std::stoi(value.substr(4, 2)) - 1;
  tm.tm_mday = std::stoi(value.substr(6, 2));
  tm.tm_hour = std::stoi(value.substr(9, 2));
  tm.tm_min = std::stoi(value.substr(11, 2));
  tm.tm_sec = std::stoi(value.substr(13, 2));
  tm.tm_isdst = 0;
#if defined(_WIN32)
  const std::time_t as_time_t = _mkgmtime(&tm);
#else
  const std::time_t as_time_t = timegm(&tm);
#endif
  return std::chrono::system_clock::from_time_t(as_time_t);
}

std::string normalize_schedule_timestamp_to_local(const std::string & value)
{
  if (!value.empty() && value.back() == 'Z') {
    return format_local_timestamp(time_point_to_tm(utc_to_time_point(value)));
  }
  return value;
}

std::string format_local_timestamp(const std::tm & tm)
{
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%S", &tm);
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

std::string trim(const std::string & value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

std::optional<std::string> derived_robot_id_from_env_file(const std::string & env_path)
{
  std::ifstream input_stream(env_path);
  if (!input_stream.is_open()) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(input_stream, line)) {
    line = trim(trim_cr(line));
    if (line.empty() || line.front() == '#') {
      continue;
    }

    const auto delimiter_pos = line.find('=');
    if (delimiter_pos == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, delimiter_pos));
    if (key != "ROBOT_NUMBER") {
      continue;
    }

    const int robot_number = std::stoi(trim(line.substr(delimiter_pos + 1)));
    if (robot_number < 0) {
      throw std::runtime_error("ROBOT_NUMBER must be non-negative");
    }

    std::ostringstream stream;
    stream << "AMR-Sweeper_" << std::setw(5) << std::setfill('0') << robot_number;
    return stream.str();
  }

  return std::nullopt;
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
  if (value == "SAFETY") {
    return ScheduleType::SAFETY;
  }
  return std::nullopt;
}

const char * schedule_type_to_cstr(const ScheduleType type)
{
  switch (type) {
    case ScheduleType::WORK:
      return "WORK";
    case ScheduleType::NO_WORK:
      return "NO_WORK";
    case ScheduleType::SAFETY:
      return "SAFETY";
    default:
      return "UNKNOWN";
  }
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
      current.dtend_local = normalize_schedule_timestamp_to_local(split_kv(line).second);
      continue;
    }
    if (starts_with(line, "DTSTART")) {
      const auto key_value = split_kv(line);
      const std::string & key = key_value.first;
      current.dtstart_local = normalize_schedule_timestamp_to_local(key_value.second);
      const std::string tzid_tag = "TZID=";
      const auto tz_pos = key.find(tzid_tag);
      if (tz_pos != std::string::npos) {
        auto start = tz_pos + tzid_tag.size();
        auto end = key.find(';', start);
        if (end == std::string::npos) {
          end = key.size();
        }
        current.dtstart_tzid = key.substr(start, end - start);
      } else if (!key_value.second.empty() && key_value.second.back() == 'Z') {
        current.dtstart_tzid = model.calendar_tzid.empty() ? "UTC" : model.calendar_tzid;
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
    if (event.type == ScheduleType::SAFETY) {
      continue;
    }
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
    } else if (window.type == ScheduleType::WORK) {
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
: rclcpp::Node("scheduler_node", options)
{
  schedule_ics_path_ = declare_parameter<std::string>("schedule_ics_path", "");
  missions_directory_ = declare_parameter<std::string>("missions_directory", "missions/database");
  default_schedule_filename_ = declare_parameter<std::string>(
    "default_schedule_filename",
    "");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  robot_id_ = declare_parameter<std::string>("robot_id", "");
  robot_config_env_path_ = declare_parameter<std::string>(
    "robot_config_env_path",
    kDefaultRobotConfigEnvPath);
  if (robot_id_.empty() || robot_id_ == kLegacyRobotId) {
    try {
      const auto derived_robot_id = derived_robot_id_from_env_file(robot_config_env_path_);
      if (derived_robot_id) {
        robot_id_ = *derived_robot_id;
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to derive robot_id from %s: %s",
        robot_config_env_path_.c_str(),
        exception.what());
    }
  }
  mission_executor_execute_service_ = declare_parameter<std::string>(
    "mission_executor_execute_service",
    "execute_mission");
  mission_executor_prepare_service_ = declare_parameter<std::string>(
    "mission_executor_prepare_service",
    "prepare_manual_mission");
  horizon_hours_ = declare_parameter<int>("horizon_hours", 72);
  tick_seconds_ = declare_parameter<double>("tick_seconds", 2.0);
  trigger_running_on_work_window_ = declare_parameter<bool>(
    "trigger_running_on_work_window",
    true);
  schedule_poll_interval_sec_ = declare_parameter<double>("schedule_poll_interval_sec", 60.0);
  retry_attempts_before_error_ = declare_parameter<int>("retry_attempts_before_error", 3);
  fatal_after_consecutive_errors_ = declare_parameter<int>("fatal_after_consecutive_errors", 10);
  reload_on_mtime_change_ = declare_parameter<bool>("reload_on_mtime_change", true);
  reload_on_every_poll_ = declare_parameter<bool>("reload_on_every_poll", false);
  declare_parameter<bool>("strict_validation", true);
  declare_parameter<int>("max_events", 2000);
  declare_parameter<bool>("require_x_robot_id", true);
  declare_parameter<bool>("require_x_schedule_type", true);
  declare_parameter<bool>("require_x_mission_id_for_work", true);
  emit_rosout_triggers_ = declare_parameter<bool>("emit_rosout_triggers", true);
  emit_trigger_topic_ = declare_parameter<bool>("emit_trigger_topic", true);
  trigger_topic_name_ = declare_parameter<std::string>(
    "trigger_topic_name",
    "scheduler_node/triggers");
  if (retry_attempts_before_error_ < 1) {
    retry_attempts_before_error_ = 1;
  }
  if (fatal_after_consecutive_errors_ < 1) {
    fatal_after_consecutive_errors_ = 1;
  }
  if (fatal_after_consecutive_errors_ < retry_attempts_before_error_) {
    fatal_after_consecutive_errors_ = retry_attempts_before_error_;
  }

  planned_pub_ = create_publisher<std_msgs::msg::String>("scheduler_node/planned_windows", 10);
  if (emit_trigger_topic_) {
    trigger_pub_ = create_publisher<std_msgs::msg::String>(trigger_topic_name_, 10);
  }

  mission_executor_client_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mission_executor_execute_client_ =
    create_client<amr_sweeper_mission_executor::srv::ExecuteMission>(
    mission_executor_execute_service_,
    rclcpp::ServicesQoS(),
    mission_executor_client_callback_group_);
  mission_executor_prepare_client_ =
    create_client<amr_sweeper_mission_executor::srv::PrepareManualMission>(
    mission_executor_prepare_service_,
    rclcpp::ServicesQoS(),
    mission_executor_client_callback_group_);

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

  prepare_mission_execution_srv_ =
    create_service<amr_sweeper_scheduler::srv::PrepareMissionExecution>(
    "prepare_mission_execution",
    [this](
      const std::shared_ptr<amr_sweeper_scheduler::srv::PrepareMissionExecution::Request> request,
      std::shared_ptr<amr_sweeper_scheduler::srv::PrepareMissionExecution::Response> response)
    {
      refresh_mission_catalog();

      if (request->mission_id.empty()) {
        response->success = false;
        response->message = "mission_id must not be empty";
        return;
      }

      const auto mission_path = resolve_mission_path(request->mission_id);
      if (!mission_path) {
        response->success = false;
        response->message = "Mission not found for mission_id=" + request->mission_id;
        trigger_warn("SCHED_MANUAL_MISSION_NOT_FOUND", "mission_id=" + request->mission_id);
        return;
      }

      if (!mission_executor_prepare_client_->wait_for_service(std::chrono::seconds(5))) {
        response->success = false;
        response->message = "Mission executor prepare_manual_mission service is unavailable";
        trigger_warn("SCHED_MISSION_EXECUTOR_UNAVAILABLE", mission_executor_prepare_service_);
        return;
      }

      auto prepare_request =
        std::make_shared<amr_sweeper_mission_executor::srv::PrepareManualMission::Request>();
      prepare_request->mission_id = request->mission_id;
      auto prepare_future = mission_executor_prepare_client_->async_send_request(prepare_request);
      if (prepare_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
        response->success = false;
        response->message = "Mission executor prepare_manual_mission request timed out";
        trigger_error("SCHED_MANUAL_MISSION_PREP_FAILED", "mission_id=" + request->mission_id);
        return;
      }

      const auto prepare_response = prepare_future.get();
      if (!prepare_response->success) {
        response->success = false;
        response->message = prepare_response->message;
        trigger_error("SCHED_MANUAL_MISSION_PREP_FAILED", "mission_id=" + request->mission_id);
        return;
      }

      response->success = true;
      response->message = "Mission execution context prepared";
      response->mission_execution_directory = prepare_response->mission_execution_directory;
      response->execution_context_file = prepare_response->execution_context_file;
      trigger_info("SCHED_MANUAL_MISSION_PREPARED", "mission_id=" + request->mission_id);
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

void SchedulerNode::enter_fatal_state(const std::string & message)
{
  fatal_error_ = true;
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  if (tick_timer_) {
    tick_timer_->cancel();
  }
  if (poll_timer_) {
    poll_timer_->cancel();
  }
  rclcpp::shutdown();
}

void SchedulerNode::report_supervision_issue(const std::string & message)
{
  ++supervision_issue_count_;
  log_escalating_issue(supervision_issue_count_, message);
}

void SchedulerNode::log_escalating_issue(int count, const std::string & message)
{
  if (count < retry_attempts_before_error_) {
    trigger_warn("SCHED_SELF_RECOVERY", message);
    return;
  }

  if (count < fatal_after_consecutive_errors_) {
    if (count == retry_attempts_before_error_) {
      trigger_error(
        "SCHED_SELF_RECOVERY",
        message + "; escalating_after_failures=" + std::to_string(count));
      return;
    }

    trigger_error("SCHED_SELF_RECOVERY", message + "; consecutive_failures=" + std::to_string(count));
    return;
  }

  enter_fatal_state(
    message + ". Reached fatal threshold after " + std::to_string(count) +
    " consecutive scheduler supervision failures");
}

void SchedulerNode::reset_supervision_issue_count()
{
  supervision_issue_count_ = 0;
  fatal_error_ = false;
}

void SchedulerNode::publish_info_message(const std::string & message)
{
  if (last_trigger_message_ == message) {
    return;
  }
  last_trigger_message_ = message;
  if (emit_rosout_triggers_) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = message;
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::trigger_info(const std::string & code, const std::string & kv)
{
  const std::string message = code + (kv.empty() ? "" : " " + kv);
  if (last_trigger_message_ == message) {
    return;
  }
  last_trigger_message_ = message;
  if (emit_rosout_triggers_) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = message;
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::trigger_warn(const std::string & code, const std::string & kv)
{
  const std::string message = code + (kv.empty() ? "" : " " + kv);
  if (last_trigger_message_ == message) {
    return;
  }
  last_trigger_message_ = message;
  if (emit_rosout_triggers_) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = message;
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::trigger_error(const std::string & code, const std::string & kv)
{
  const std::string message = code + (kv.empty() ? "" : " " + kv);
  if (last_trigger_message_ == message) {
    return;
  }
  last_trigger_message_ = message;
  if (emit_rosout_triggers_) {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }
  if (emit_trigger_topic_ && trigger_pub_) {
    std_msgs::msg::String msg;
    msg.data = message;
    trigger_pub_->publish(msg);
  }
}

void SchedulerNode::poll_schedule()
{
  if (fatal_error_) {
    return;
  }

  if (robot_id_.empty() || robot_id_ == kLegacyRobotId) {
    try {
      const auto derived_robot_id = derived_robot_id_from_env_file(robot_config_env_path_);
      if (derived_robot_id) {
        robot_id_ = *derived_robot_id;
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to derive robot_id from %s: %s",
        robot_config_env_path_.c_str(),
        exception.what());
    }
  }
  if (robot_id_.empty() || robot_id_ == kLegacyRobotId) {
    trigger_warn("SCHED_PARAMS_MISSING", "set robot_id or ROBOT_NUMBER");
    report_supervision_issue("robot_id unavailable; set robot_id or ROBOT_NUMBER");
    return;
  }

  refresh_mission_catalog();

  const std::string schedule_path = resolved_schedule_path();
  const auto mtime = file_mtime(schedule_path);
  if (!mtime) {
    trigger_error("SCHED_ICS_NOT_FOUND", "path=" + schedule_path);
    report_supervision_issue("schedule not found; path=" + schedule_path);
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
    reset_supervision_issue_count();
    last_mtime_ = mtime;
    trigger_info(
      "SCHED_ICS_LOADED",
      "events=" + std::to_string(schedule_.events.size()) +
      "; schedule=" + std::filesystem::path(schedule_path).filename().string() +
      "; robot_id=" + robot_id_);
    if (schedule_has_no_events_) {
      trigger_warn("SCHED_ICS_LOAD_FAILED", "reason=ICS contains no VEVENTs");
    }
    if (!ready_message_emitted_) {
      publish_info_message("Scheduler is now running");
      ready_message_emitted_ = true;
    }
  } catch (const std::exception & exception) {
    trigger_error("SCHED_ICS_LOAD_FAILED", std::string("reason=") + exception.what());
    report_supervision_issue(std::string("schedule load failed; reason=") + exception.what());
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
  schedule_has_no_events_ = schedule_.events.empty();
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
  std::unordered_map<std::string, std::size_t> missing_mission_counts;
  std::set<std::string> current_missing_mission_ids;

  for (auto & window : windows) {
    if (window.type == ScheduleType::WORK && window.mission_id) {
      if (!mission_json_or_folder_exists(*window.mission_id)) {
        ++missing_mission_counts[*window.mission_id];
        current_missing_mission_ids.insert(*window.mission_id);
        continue;
      }
      window.mission_path = resolve_mission_path(*window.mission_id);
      if (!window.mission_path) {
        ++missing_mission_counts[*window.mission_id];
        current_missing_mission_ids.insert(*window.mission_id);
      }
    }
  }

  for (const auto & entry : missing_mission_counts) {
    if (warned_missing_mission_ids_.count(entry.first) != 0U) {
      continue;
    }
    std::ostringstream detail;
    detail << "mission_id=" << entry.first;
    if (entry.second > 1U) {
      detail << "; windows=" << entry.second;
    }
    trigger_warn("SCHED_MISSION_NOT_FOUND", detail.str());
  }
  warned_missing_mission_ids_ = std::move(current_missing_mission_ids);

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
  if (last_planned_windows_payload_ != msg.data) {
    last_planned_windows_payload_ = msg.data;
    planned_pub_->publish(msg);
  }

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

    if (
      rejected_running_window_uid_ == window.uid &&
      next_running_request_retry_time_.has_value() &&
      this->now() < *next_running_request_retry_time_)
    {
      return;
    }

    if (!running_request_in_flight_ && running_request_window_uid_ != window.uid) {
      request_mission_execution(window);
    }
    return;
  }

  running_request_window_uid_.clear();
  rejected_running_window_uid_.clear();
  next_running_request_retry_time_.reset();
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

void SchedulerNode::request_mission_execution(const TimeWindow & window)
{
  if (!mission_executor_execute_client_->service_is_ready()) {
    trigger_warn("SCHED_MISSION_EXECUTOR_UNAVAILABLE", mission_executor_execute_service_);
    return;
  }

  running_request_in_flight_ = true;
  auto request = std::make_shared<amr_sweeper_mission_executor::srv::ExecuteMission::Request>();
  request->mission_id = window.mission_id.value_or(std::string("unknown"));
  request->mission_execution_directory = "";
  request->mission_window_start = window.start_local;
  request->mission_window_end = window.end_local;
  request->requester = "scheduler_node";
  request->priority = 210;
  request->force = false;
  request->reason =
    "Scheduled mission window active; mission_id=" +
    window.mission_id.value_or(std::string("unknown")) +
    "; start=" + window.start_local +
    "; end=" + window.end_local;
  mission_executor_execute_client_->async_send_request(
    request,
    [this, window](
      rclcpp::Client<amr_sweeper_mission_executor::srv::ExecuteMission>::SharedFuture response_future)
    {
      running_request_in_flight_ = false;
      const auto response = response_future.get();
      if (response->success) {
        running_request_window_uid_ = window.uid;
        rejected_running_window_uid_.clear();
        next_running_request_retry_time_.reset();
        trigger_info(
          "SCHED_PROMOTED_TO_RUNNING",
          "mission_id=" + window.mission_id.value_or(std::string("unknown")) +
          "; profile=" + std::to_string(response->running_profile_id));
      } else {
        if (response->message.rfind(kLowerPriorityRejectedPrefix, 0) == 0) {
          rejected_running_window_uid_ = window.uid;
          next_running_request_retry_time_ =
            this->now() + rclcpp::Duration(kLowerPriorityRetryCooldown);
        }
        trigger_warn("SCHED_MISSION_EXECUTION_REJECTED", response->message);
      }
    });
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
  return archive_older_schedule_files(missions_directory);
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
