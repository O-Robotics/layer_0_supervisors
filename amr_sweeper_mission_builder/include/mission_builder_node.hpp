#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>
#include <nlohmann/json_fwd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace amr_sweeper_mission_builder
{

struct GeoPoint
{
  double latitude;
  double longitude;
};

struct MapPoint
{
  double x;
  double y;
};

struct PolygonZone
{
  std::string name;
  std::string zone_type;
  std::vector<MapPoint> vertices;
};

struct Vda5050MissionBuildConfig
{
  std::string mission_path{""};
  std::string zone_type_property{"zone_type"};
  std::string working_zone_value{"working_zone"};
  std::string no_go_zone_value{"no_go"};
  double edge_band_meters{1.0};
  int edge_band_cost{180};
  int inside_cost{0};
  int outside_cost{254};
  int no_go_cost{254};
  double origin_latitude{0.0};
  double origin_longitude{0.0};
  double origin_altitude{0.0};
  bool use_first_polygon_vertex_as_origin{true};
};

struct MapExtent
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

struct RasterizedMap
{
  std::vector<unsigned char> costs;
  std::vector<int8_t> occupancy;
  unsigned int width_cells;
  unsigned int height_cells;
  double resolution;
  double origin_x;
  double origin_y;
};

struct MissionPathWaypoint
{
  std::string node_id;
  GeoPoint geo_point;
  double theta;
};

struct MissionIdentity
{
  std::string order_id;
  std::string timestamp;
  std::string stem;
};

class Vda5050MissionBuilder
{
public:
  void loadMission(const Vda5050MissionBuildConfig & config);
  [[nodiscard]] MissionIdentity inspectMissionIdentity(const std::string & mission_path) const;
  [[nodiscard]] RasterizedMap buildSuggestedGlobalCostmap(
    double resolution,
    double padding_meters) const;
  void saveGlobalCostmapArtifacts(
    const RasterizedMap & map,
    const std::string & image_path,
    const std::string & yaml_path) const;
  void saveMissionWaypointsArtifact(const std::string & path) const;
  [[nodiscard]] bool hasWorkingZones() const;
  [[nodiscard]] bool hasMissionWaypoints() const;

private:
  [[nodiscard]] RasterizedMap buildGlobalCostmap(
    double origin_x,
    double origin_y,
    unsigned int width_cells,
    unsigned int height_cells,
    double resolution) const;
  [[nodiscard]] MapExtent computeExtent(double padding_meters) const;
  void projectAndStoreZone(
    const std::vector<GeoPoint> & geo_vertices,
    const std::string & zone_name,
    const std::string & zone_type);
  void loadCoveragePath(
    const nlohmann::json & coverage_edge_ids,
    const std::unordered_map<std::string, GeoPoint> & nodes_by_id,
    const std::unordered_map<std::string, double> & node_theta_by_id,
    const std::unordered_map<std::string, nlohmann::json> & edges_by_id);
  void loadFromLegacyGeoJson(const nlohmann::json & document);
  void loadFromVda5050Mission(const nlohmann::json & document);
  [[nodiscard]] bool pointInPolygon(const MapPoint & point, const PolygonZone & polygon) const;
  [[nodiscard]] double signedDistanceToPolygon(
    const MapPoint & point,
    const PolygonZone & polygon) const;
  [[nodiscard]] double distanceToSegment(
    const MapPoint & point,
    const MapPoint & start,
    const MapPoint & end) const;
  [[nodiscard]] unsigned char costForPoint(const MapPoint & point) const;
  [[nodiscard]] std::string resolveMissionPath(const std::string & configured_path) const;
  [[nodiscard]] MissionIdentity extractMissionIdentity(const nlohmann::json & document) const;
  [[nodiscard]] static std::string sanitizeTimestamp(const std::string & timestamp);
  [[nodiscard]] static std::string sanitizeStemToken(const std::string & value);
  [[nodiscard]] static std::string normalizeZoneType(const std::string & zone_type);

  Vda5050MissionBuildConfig config_;
  std::vector<PolygonZone> working_zones_;
  std::vector<PolygonZone> no_go_zones_;
  std::vector<MissionPathWaypoint> mission_waypoints_;
  bool projection_initialized_{false};
  GeographicLib::LocalCartesian projector_;
};

class MissionBuilderNode : public rclcpp::Node
{
public:
  explicit MissionBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void buildIfNeeded();
  bool buildCurrentMissionArtifacts();
  void buildDiscoveredMissionArtifacts();
  [[nodiscard]] std::vector<std::filesystem::path> discoverMissionPaths();
  [[nodiscard]] std::optional<std::filesystem::path> selectActiveMissionPath();
  [[nodiscard]] std::filesystem::path stageMissionFile(
    const std::filesystem::path & mission_path);
  bool buildArtifactsForMission(
    const std::filesystem::path & mission_path,
    bool write_active_aliases);
  void handleBuildCurrentMission(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  [[nodiscard]] std::filesystem::path resolveMissionPath() const;
  [[nodiscard]] std::filesystem::path resolvePath(const std::string & path) const;
  [[nodiscard]] std::filesystem::file_time_type currentMissionStamp(
    const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::filesystem::path missionFolderPath(
    const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::string missionStemForPath(const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::string coverageBasenameForMission(
    const std::filesystem::path & mission_path) const;
  [[nodiscard]] std::string costmapBasenameForMission(
    const std::filesystem::path & mission_path) const;
  void publishStatus(const std::string & state, const std::string & detail) const;

  std::string mission_path_;
  std::string missions_directory_;
  std::string mission_file_extension_;
  std::string costmap_output_basename_;
  std::string coverage_path_basename_;
  double mission_build_resolution_{0.1};
  double mission_build_padding_meters_{2.0};
  bool auto_build_on_start_{true};
  bool watch_for_updates_{true};
  std::filesystem::path last_active_alias_mission_;
  std::map<std::string, std::filesystem::file_time_type> mission_build_stamps_;
  std::unique_ptr<Vda5050MissionBuilder> mission_builder_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr build_current_mission_service_;
  rclcpp::TimerBase::SharedPtr build_timer_;
};

}  // namespace amr_sweeper_mission_builder
