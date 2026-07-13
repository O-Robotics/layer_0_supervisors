#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <sys/types.h>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "amr_sweeper_mission_executor/srv/create_recorded_mission.hpp"
#include "amr_sweeper_mission_executor/srv/end_mission.hpp"
#include "amr_sweeper_mission_executor/srv/execute_mission.hpp"
#include "amr_sweeper_mission_executor/srv/list_executable_missions.hpp"
#include "amr_sweeper_mission_executor/srv/list_manual_missions.hpp"
#include "amr_sweeper_mission_executor/srv/prepare_manual_mission.hpp"
#include "amr_sweeper_mission_executor/srv/upload_vda5050_mission.hpp"
#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "amr_sweeper_fsm/srv/request_state.hpp"

namespace amr_sweeper_mission_executor
{

struct ManualMissionInfo
{
  std::string mission_id;
  std::string mission_path;
  std::string mission_type;
  std::string execution_mode;
  std::uint16_t running_profile_id{0U};
  bool is_manual{false};
  bool artifacts_ready{false};
};

struct PreparedMissionContext
{
  std::string mission_execution_directory;
  std::string execution_context_file;
  std::uint16_t running_profile_id{0U};
};

class MissionExecutorNode : public rclcpp::Node
{
public:
  explicit MissionExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void shutdownForExit();

private:
  void handleListExecutableMissions(
    const std::shared_ptr<srv::ListExecutableMissions::Request> request,
    std::shared_ptr<srv::ListExecutableMissions::Response> response);
  void handleListManualMissions(
    const std::shared_ptr<srv::ListManualMissions::Request> request,
    std::shared_ptr<srv::ListManualMissions::Response> response);
  void handleUploadVda5050Mission(
    const std::shared_ptr<srv::UploadVda5050Mission::Request> request,
    std::shared_ptr<srv::UploadVda5050Mission::Response> response);
  void handleCreateRecordedMission(
    const std::shared_ptr<srv::CreateRecordedMission::Request> request,
    std::shared_ptr<srv::CreateRecordedMission::Response> response);
  void handlePrepareManualMission(
    const std::shared_ptr<srv::PrepareManualMission::Request> request,
    std::shared_ptr<srv::PrepareManualMission::Response> response);
  void handleExecuteMission(
    const std::shared_ptr<srv::ExecuteMission::Request> request,
    std::shared_ptr<srv::ExecuteMission::Response> response);
  void handleEndMission(
    const std::shared_ptr<srv::EndMission::Request> request,
    std::shared_ptr<srv::EndMission::Response> response);
  void handleSafetyStop(const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr message);
  void handleManualMissionOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleManualMissionNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message);
  void handleRoutedMissionOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void checkManualMissionInactivity();

  [[nodiscard]] std::vector<ManualMissionInfo> discoverManualMissions() const;
  [[nodiscard]] std::optional<ManualMissionInfo> findManualMission(const std::string & mission_id) const;
  [[nodiscard]] static std::string sanitizeMissionId(const std::string & mission_id);
  [[nodiscard]] static std::string deriveMissionId(
    const nlohmann::json & document,
    const std::string & requested_mission_id);
  [[nodiscard]] std::filesystem::path resolvePath(const std::string & configured_path) const;
  [[nodiscard]] std::filesystem::path resolveMissionsFromDbDirectory() const;
  [[nodiscard]] std::filesystem::path resolveMissionsLogDirectory() const;
  [[nodiscard]] std::filesystem::path resolveRosbagDirectory() const;
  [[nodiscard]] std::filesystem::path resolveManualMissionsDirectory() const;
  [[nodiscard]] std::vector<std::filesystem::path> executionContextFiles() const;
  [[nodiscard]] std::optional<ManualMissionInfo> findStagedScheduledMission(
    const std::string & mission_id) const;
  [[nodiscard]] std::filesystem::path missionFolderPath(const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::filesystem::path artifactsDirectoryForMission(const ManualMissionInfo & mission) const;
  [[nodiscard]] std::string missionStemForPath(const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::string missionStaticCostmapBasename(const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::string missionRouteBasename(const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::filesystem::path resolveMissionRoutePath(
    const ManualMissionInfo & mission,
    const std::filesystem::path & mission_file) const;
  [[nodiscard]] std::optional<std::filesystem::path> newestScheduledArtifactDirectory(
    const std::string & mission_id) const;
  [[nodiscard]] ManualMissionInfo resolveExecutableMissionSource(const ManualMissionInfo & mission) const;
  [[nodiscard]] std::filesystem::path missionHistoryDirectory(const ManualMissionInfo & mission) const;
  [[nodiscard]] std::optional<ManualMissionInfo> classifyMissionFile(
    const std::filesystem::path & mission_path) const;
  [[nodiscard]] PreparedMissionContext prepareMissionArtifacts(
    const ManualMissionInfo & mission,
    const std::string & mission_window_start,
    const std::string & mission_window_end) const;
  [[nodiscard]] std::optional<nlohmann::json> resolveExecutionContext(
    const std::string & mission_id) const;
  [[nodiscard]] bool requestIdlingState(
    const srv::EndMission::Request & request,
    std::string & message) const;
  [[nodiscard]] bool finalizeMissionExecution(
    const srv::EndMission::Request & request,
    std::string & message,
    std::optional<nlohmann::json> context_document = std::nullopt);
  void promoteRuntimeCostmapArtifacts(
    nlohmann::json & context_document,
    const srv::EndMission::Request & request) const;
  void updateRecordMapArtifacts(nlohmann::json & context_document) const;
  void writeLatestRecordedMapSnapshot(const nlohmann::json & context_document) const;
  void refreshActiveMissionState(const nlohmann::json & context_document);
  void clearActiveMissionState();
  void recordMissionExecutionStart(
    const ManualMissionInfo & mission,
    const PreparedMissionContext & context,
    const srv::ExecuteMission::Request & request) const;
  void recordMissionExecutionEnd(
    nlohmann::json & context_document,
    const srv::EndMission::Request & request) const;
  void recordSafetyEvent(
    const amr_sweeper_safety_msgs::msg::SafetyStop & event,
    const std::optional<nlohmann::json> & context_document) const;
  void rewriteBuiltinLocalPatternArtifacts(
    const ManualMissionInfo & mission,
    const PreparedMissionContext & context) const;
  [[nodiscard]] bool missionArtifactsReady(const ManualMissionInfo & mission) const;
  [[nodiscard]] bool ensureMissionArtifactsReady(
    const ManualMissionInfo & mission,
    const std::string & requester = std::string{},
    const std::string & reason = std::string{});
  [[nodiscard]] bool requestRunningState(
    const PreparedMissionContext & context,
    const srv::ExecuteMission::Request & request,
    std::string & message) const;
  void writeMissionExecutionPreferences(
    const std::filesystem::path & context_path,
    bool record_rosbag) const;
  [[nodiscard]] std::vector<std::string> loadRosbagTopics() const;
  [[nodiscard]] bool startMissionRosbagRecording(
    const PreparedMissionContext & context,
    bool record_rosbag_requested,
    std::string & warning_message);
  void stopMissionRosbagRecording();
  [[nodiscard]] std::filesystem::path resolveScheduleSourcePath() const;
  [[nodiscard]] std::filesystem::path ensureScheduleLogPath(
    const std::filesystem::path & schedule_source_path) const;
  [[nodiscard]] std::filesystem::path ensureActualScheduleLogPath(
    const std::filesystem::path & schedule_source_path) const;
  [[nodiscard]] static std::string formatUtcTimestamp(
    const std::chrono::system_clock::time_point & time_point);
  [[nodiscard]] static std::string formatLocalTimestamp(
    const std::chrono::system_clock::time_point & time_point);

  std::string missions_directory_;
  std::string missions_log_directory_;
  std::string actual_schedule_log_directory_;
  std::string manual_missions_directory_;
  std::string mission_file_extension_;
  std::string schedule_ics_path_;
  std::string rosbag_directory_;
  std::string robot_id_;
  std::string safety_stop_topic_;
  std::string teleop_odometry_topic_;
  std::string manual_mapping_odometry_topic_;
  std::string manual_mapping_navsat_topic_;
  std::string routed_mission_odometry_topic_;
  std::string rosbag_topics_file_;
  bool record_mission_rosbag_{false};
  std::string mission_parser_node_name_;
  std::string mission_parser_build_service_;
  std::string fsm_request_service_;
  bool use_simulation_{false};
  double manual_mission_inactivity_timeout_seconds_{300.0};
  double manual_mission_min_linear_speed_mps_{0.01};
  double manual_mission_min_angular_speed_rps_{0.01};
  double teleop_path_sample_distance_m_{0.1};
  double routed_mission_pose_max_age_seconds_{2.0};
  std::uint16_t idling_profile_id_{101U};
  std::uint16_t scheduled_running_profile_id_{201U};
  std::uint16_t manual_mapping_profile_id_{225U};
  std::uint16_t manual_routed_profile_id_{210U};
  std::uint16_t manual_teleop_profile_id_{220U};
  std::uint8_t default_activation_priority_{200U};
  bool promote_runtime_costmap_on_completed_mission_{true};
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::AsyncParametersClient::SharedPtr mission_parser_parameter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mission_parser_build_client_;
  rclcpp::Client<amr_sweeper_fsm::srv::RequestState>::SharedPtr fsm_request_client_;
  rclcpp::Service<srv::ListExecutableMissions>::SharedPtr list_executable_missions_service_;
  rclcpp::Service<srv::ListManualMissions>::SharedPtr list_manual_missions_service_;
  rclcpp::Service<srv::UploadVda5050Mission>::SharedPtr upload_vda5050_mission_service_;
  rclcpp::Service<srv::CreateRecordedMission>::SharedPtr create_recorded_mission_service_;
  rclcpp::Service<srv::PrepareManualMission>::SharedPtr prepare_manual_mission_service_;
  rclcpp::Service<srv::ExecuteMission>::SharedPtr execute_mission_service_;
  rclcpp::Service<srv::EndMission>::SharedPtr end_mission_service_;
  rclcpp::Subscription<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr safety_stop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr teleop_odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr manual_mapping_odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr manual_mapping_navsat_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr routed_mission_odometry_subscription_;
  rclcpp::TimerBase::SharedPtr manual_mission_watchdog_timer_;
  mutable std::mutex active_mission_mutex_;
  mutable std::mutex routed_mission_pose_mutex_;
  mutable std::mutex rosbag_process_mutex_;
  bool active_mission_running_{false};
  bool active_mission_is_teleop_{false};
  bool active_mission_is_manual_mapping_{false};
  bool active_mission_uses_inactivity_watchdog_{false};
  std::string active_mission_id_;
  std::string active_execution_mode_;
  std::string active_execution_context_file_;
  std::string active_actual_path_file_;
  std::string active_actual_navsat_path_file_;
  rclcpp::Time last_manual_mission_motion_time_;
  std::vector<geometry_msgs::msg::Point> teleop_traveled_path_points_;
  std::vector<geometry_msgs::msg::Point> manual_mapping_navsat_points_;
  pid_t rosbag_recording_pid_{-1};
  std::string active_rosbag_output_directory_;
  std::string active_rosbag_context_file_;
  std::string active_rosbag_log_file_;
  bool routed_mission_pose_ready_{false};
  geometry_msgs::msg::Point routed_mission_position_;
  geometry_msgs::msg::Quaternion routed_mission_orientation_;
  rclcpp::Time routed_mission_pose_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace amr_sweeper_mission_executor
