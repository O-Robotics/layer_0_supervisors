#include "vda5050_parser_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace amr_sweeper_vda5050_parser
{

namespace
{

bool isValidVda5050MissionDocument(const nlohmann::json & document)
{
  return document.is_object() &&
         document.contains("nodes") &&
         document.at("nodes").is_array() &&
         !document.at("nodes").empty() &&
         document.contains("edges") &&
         document.at("edges").is_array() &&
         document.contains("missionGeometries") &&
         document.at("missionGeometries").is_object();
}

nlohmann::json loadJsonDocument(const std::filesystem::path & path)
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission file: " + path.string());
  }

  nlohmann::json document;
  input_stream >> document;
  return document;
}

std::string sanitize_token(const std::string & value)
{
  std::string sanitized;
  sanitized.reserve(value.size());
  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0U || character == '_' || character == '-') {
      sanitized.push_back(static_cast<char>(character));
    }
  }
  return sanitized;
}

std::vector<GeoPoint> parseOuterRing(const nlohmann::json & geometry)
{
  if (!geometry.contains("type") || geometry.at("type") != "Polygon") {
    throw std::runtime_error("Only Polygon geometries are supported");
  }

  const auto & coordinates = geometry.at("coordinates");
  if (!coordinates.is_array() || coordinates.empty()) {
    throw std::runtime_error("Polygon coordinates are missing");
  }

  const auto & outer_ring = coordinates.at(0);
  if (!outer_ring.is_array() || outer_ring.size() < 4) {
    throw std::runtime_error("Polygon outer ring must contain at least four points");
  }

  std::vector<GeoPoint> points;
  points.reserve(outer_ring.size());
  for (const auto & coordinate : outer_ring) {
    points.push_back(GeoPoint{coordinate.at(1).get<double>(), coordinate.at(0).get<double>()});
  }

  if (points.front().latitude == points.back().latitude &&
    points.front().longitude == points.back().longitude)
  {
    points.pop_back();
  }

  if (points.size() < 3) {
    throw std::runtime_error("Polygon must have at least three unique vertices");
  }

  return points;
}

double clampToUnitInterval(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

std::pair<std::string, std::string> getEdgeEndpoints(const nlohmann::json & edge)
{
  return {
    edge.at("startNodeId").get<std::string>(),
    edge.at("endNodeId").get<std::string>()};
}

std::vector<std::string> buildNodeSequenceFromEdgeIds(
  const nlohmann::json & edge_ids_json,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id,
  const bool require_closed_loop)
{
  if (!edge_ids_json.is_array() || edge_ids_json.empty()) {
    throw std::runtime_error("Mission geometry edgeIds must be a non-empty array");
  }

  const std::string first_edge_id = edge_ids_json.at(0).get<std::string>();
  const auto first_edge_it = edges_by_id.find(first_edge_id);
  if (first_edge_it == edges_by_id.end()) {
    throw std::runtime_error("Mission geometry references unknown edgeId: " + first_edge_id);
  }

  const auto [first_start, first_end] = getEdgeEndpoints(first_edge_it->second);
  std::vector<std::string> node_sequence{first_start, first_end};

  for (std::size_t index = 1; index < edge_ids_json.size(); ++index) {
    const std::string edge_id = edge_ids_json.at(index).get<std::string>();
    const auto edge_it = edges_by_id.find(edge_id);
    if (edge_it == edges_by_id.end()) {
      throw std::runtime_error("Mission geometry references unknown edgeId: " + edge_id);
    }

    const auto [start_node_id, end_node_id] = getEdgeEndpoints(edge_it->second);
    const std::string & tail = node_sequence.back();
    if (start_node_id == tail) {
      node_sequence.push_back(end_node_id);
      continue;
    }
    if (end_node_id == tail) {
      node_sequence.push_back(start_node_id);
      continue;
    }

    throw std::runtime_error("Mission geometry edge chain is disconnected at edgeId: " + edge_id);
  }

  if (require_closed_loop && node_sequence.front() != node_sequence.back()) {
    throw std::runtime_error("Mission polygon edges do not form a closed loop");
  }

  return node_sequence;
}

std::vector<GeoPoint> buildPolygonFromEdgeIds(
  const nlohmann::json & edge_ids_json,
  const std::unordered_map<std::string, GeoPoint> & nodes_by_id,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id)
{
  const auto node_sequence = buildNodeSequenceFromEdgeIds(edge_ids_json, edges_by_id, true);
  std::vector<GeoPoint> polygon;
  polygon.reserve(node_sequence.size());
  for (const auto & node_id : node_sequence) {
    polygon.push_back(nodes_by_id.at(node_id));
  }
  if (polygon.front().latitude == polygon.back().latitude &&
    polygon.front().longitude == polygon.back().longitude)
  {
    polygon.pop_back();
  }
  return polygon;
}

std::vector<MissionPathWaypoint> buildCoverageWaypoints(
  const nlohmann::json & edge_ids_json,
  const std::unordered_map<std::string, GeoPoint> & nodes_by_id,
  const std::unordered_map<std::string, double> & node_theta_by_id,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id)
{
  const auto node_sequence = buildNodeSequenceFromEdgeIds(edge_ids_json, edges_by_id, false);
  std::vector<MissionPathWaypoint> waypoints;
  waypoints.reserve(node_sequence.size());
  for (const auto & node_id : node_sequence) {
    if (!waypoints.empty() && waypoints.back().node_id == node_id) {
      continue;
    }
    waypoints.push_back(
      MissionPathWaypoint{node_id, nodes_by_id.at(node_id), MapPoint{}, node_theta_by_id.at(node_id), false});
  }
  return waypoints;
}

std::vector<MapPoint> buildLocalPolygonFromEdgeIds(
  const nlohmann::json & edge_ids_json,
  const std::unordered_map<std::string, MapPoint> & nodes_by_id,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id)
{
  const auto node_sequence = buildNodeSequenceFromEdgeIds(edge_ids_json, edges_by_id, true);
  std::vector<MapPoint> polygon;
  polygon.reserve(node_sequence.size());
  for (const auto & node_id : node_sequence) {
    polygon.push_back(nodes_by_id.at(node_id));
  }
  if (!polygon.empty() &&
    polygon.front().x == polygon.back().x &&
    polygon.front().y == polygon.back().y)
  {
    polygon.pop_back();
  }
  return polygon;
}

std::vector<MissionPathWaypoint> buildLocalCoverageWaypoints(
  const nlohmann::json & edge_ids_json,
  const std::unordered_map<std::string, MapPoint> & nodes_by_id,
  const std::unordered_map<std::string, double> & node_theta_by_id,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id)
{
  const auto node_sequence = buildNodeSequenceFromEdgeIds(edge_ids_json, edges_by_id, false);
  std::vector<MissionPathWaypoint> waypoints;
  waypoints.reserve(node_sequence.size());
  for (const auto & node_id : node_sequence) {
    if (!waypoints.empty() && waypoints.back().node_id == node_id) {
      continue;
    }
    waypoints.push_back(
      MissionPathWaypoint{node_id, GeoPoint{}, nodes_by_id.at(node_id), node_theta_by_id.at(node_id), true});
  }
  return waypoints;
}

}  // namespace

void Vda5050MissionParser::loadMission(const Vda5050MissionBuildConfig & config)
{
  config_ = config;
  working_zones_.clear();
  no_go_zones_.clear();
  mission_waypoints_.clear();
  projection_initialized_ = false;

  std::ifstream input_stream(resolveMissionPath(config.mission_path));
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission file: " + config.mission_path);
  }

  nlohmann::json document;
  input_stream >> document;

  if (document.contains("features") && document.at("features").is_array()) {
    loadFromLegacyGeoJson(document);
  } else if (document.contains("nodes") && document.contains("edges")) {
    loadFromVda5050Mission(document);
  } else {
    throw std::runtime_error(
            "Mission file is neither legacy GeoJSON nor VDA5050-style mission JSON");
  }

  if (working_zones_.empty()) {
    throw std::runtime_error("At least one working zone is required in the mission file");
  }
}

MissionIdentity Vda5050MissionParser::inspectMissionIdentity(const std::string & mission_path) const
{
  return extractMissionIdentity(loadJsonDocument(resolveMissionPath(mission_path)));
}

void Vda5050MissionParser::loadFromLegacyGeoJson(const nlohmann::json & document)
{
  for (const auto & feature : document.at("features")) {
    auto zone_type = config_.working_zone_value;
    auto zone_name = std::string("zone");
    if (feature.contains("properties") && feature.at("properties").is_object()) {
      const auto & properties = feature.at("properties");
      if (properties.contains(config_.zone_type_property) &&
        properties.at(config_.zone_type_property).is_string())
      {
        zone_type = properties.at(config_.zone_type_property).get<std::string>();
      }
      if (properties.contains("name") && properties.at("name").is_string()) {
        zone_name = properties.at("name").get<std::string>();
      }
    }
    projectAndStoreZone(parseOuterRing(feature.at("geometry")), zone_name, normalizeZoneType(zone_type));
  }
}

void Vda5050MissionParser::loadFromVda5050Mission(const nlohmann::json & document)
{
  const std::string coordinate_frame = document.contains("missionReference") &&
    document.at("missionReference").is_object() &&
    document.at("missionReference").contains("coordinateFrame") &&
    document.at("missionReference").at("coordinateFrame").is_string() ?
    normalizeZoneType(document.at("missionReference").at("coordinateFrame").get<std::string>()) :
    std::string{};
  const bool use_local_frame = coordinate_frame == "odom" || coordinate_frame == "local";

  std::unordered_map<std::string, GeoPoint> nodes_by_id;
  std::unordered_map<std::string, MapPoint> local_nodes_by_id;
  std::unordered_map<std::string, double> node_theta_by_id;
  std::unordered_map<std::string, nlohmann::json> edges_by_id;

  for (const auto & node : document.at("nodes")) {
    const auto & position = node.at("nodePosition");
    const std::string node_id = node.at("nodeId").get<std::string>();
    if (use_local_frame) {
      local_nodes_by_id.emplace(
        node_id,
        MapPoint{position.at("x").get<double>(), position.at("y").get<double>()});
    } else {
      nodes_by_id.emplace(
        node_id,
        GeoPoint{position.at("y").get<double>(), position.at("x").get<double>()});
    }
    node_theta_by_id.emplace(node_id, position.value("theta", 0.0));
  }

  for (const auto & edge : document.at("edges")) {
    edges_by_id.emplace(edge.at("edgeId").get<std::string>(), edge);
  }

  const auto & mission_geometries = document.at("missionGeometries");

  if (mission_geometries.contains("workingZones")) {
    for (const auto & zone : mission_geometries.at("workingZones")) {
      if (use_local_frame) {
        PolygonZone local_zone;
        local_zone.name = zone.value("zoneId", "working_zone");
        local_zone.zone_type = normalizeZoneType(zone.value("zoneType", config_.working_zone_value));
        local_zone.vertices = buildLocalPolygonFromEdgeIds(zone.at("edgeIds"), local_nodes_by_id, edges_by_id);
        if (local_zone.zone_type == normalizeZoneType(config_.no_go_zone_value)) {
          no_go_zones_.push_back(local_zone);
        } else {
          working_zones_.push_back(local_zone);
        }
      } else {
        projectAndStoreZone(
          buildPolygonFromEdgeIds(zone.at("edgeIds"), nodes_by_id, edges_by_id),
          zone.value("zoneId", "working_zone"),
          normalizeZoneType(zone.value("zoneType", config_.working_zone_value)));
      }
    }
  }

  if (mission_geometries.contains("noGoZones")) {
    for (const auto & zone : mission_geometries.at("noGoZones")) {
      if (use_local_frame) {
        PolygonZone local_zone;
        local_zone.name = zone.value("zoneId", "no_go_zone");
        local_zone.zone_type = normalizeZoneType(zone.value("zoneType", config_.no_go_zone_value));
        local_zone.vertices = buildLocalPolygonFromEdgeIds(zone.at("edgeIds"), local_nodes_by_id, edges_by_id);
        no_go_zones_.push_back(local_zone);
      } else {
        projectAndStoreZone(
          buildPolygonFromEdgeIds(zone.at("edgeIds"), nodes_by_id, edges_by_id),
          zone.value("zoneId", "no_go_zone"),
          normalizeZoneType(zone.value("zoneType", config_.no_go_zone_value)));
      }
    }
  }

  if (mission_geometries.contains("coveragePathEdgeIds")) {
    if (use_local_frame) {
      mission_waypoints_ = buildLocalCoverageWaypoints(
        mission_geometries.at("coveragePathEdgeIds"),
        local_nodes_by_id,
        node_theta_by_id,
        edges_by_id);
    } else {
      loadCoveragePath(
        mission_geometries.at("coveragePathEdgeIds"),
        nodes_by_id,
        node_theta_by_id,
        edges_by_id);
    }
  }
}

RasterizedMap Vda5050MissionParser::buildSuggestedGlobalCostmap(
  const double resolution,
  const double padding_meters) const
{
  const MapExtent extent = computeExtent(padding_meters);
  const unsigned int width_cells = std::max(
    1U,
    static_cast<unsigned int>(std::ceil((extent.max_x - extent.min_x) / resolution)));
  const unsigned int height_cells = std::max(
    1U,
    static_cast<unsigned int>(std::ceil((extent.max_y - extent.min_y) / resolution)));
  return buildGlobalCostmap(extent.min_x, extent.min_y, width_cells, height_cells, resolution);
}

RasterizedMap Vda5050MissionParser::buildGlobalCostmap(
  const double origin_x,
  const double origin_y,
  const unsigned int width_cells,
  const unsigned int height_cells,
  const double resolution) const
{
  RasterizedMap result;
  result.width_cells = width_cells;
  result.height_cells = height_cells;
  result.resolution = resolution;
  result.origin_x = origin_x;
  result.origin_y = origin_y;
  result.costs.resize(width_cells * height_cells);
  result.occupancy.resize(width_cells * height_cells);

  for (unsigned int iy = 0; iy < height_cells; ++iy) {
    for (unsigned int ix = 0; ix < width_cells; ++ix) {
      const double x = origin_x + (static_cast<double>(ix) + 0.5) * resolution;
      const double y = origin_y + (static_cast<double>(iy) + 0.5) * resolution;
      const unsigned char cost = costForPoint(MapPoint{x, y});
      const std::size_t index = static_cast<std::size_t>(iy) * width_cells + ix;
      result.costs[index] = cost;
      result.occupancy[index] =
        static_cast<int8_t>(std::lround((static_cast<double>(cost) / 254.0) * 100.0));
    }
  }

  return result;
}

MapExtent Vda5050MissionParser::computeExtent(const double padding_meters) const
{
  MapExtent extent{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};

  const auto update_extent = [&extent](const PolygonZone & zone) {
      for (const auto & vertex : zone.vertices) {
        extent.min_x = std::min(extent.min_x, vertex.x);
        extent.min_y = std::min(extent.min_y, vertex.y);
        extent.max_x = std::max(extent.max_x, vertex.x);
        extent.max_y = std::max(extent.max_y, vertex.y);
      }
    };

  for (const auto & zone : working_zones_) {
    update_extent(zone);
  }
  for (const auto & zone : no_go_zones_) {
    update_extent(zone);
  }

  extent.min_x -= padding_meters;
  extent.min_y -= padding_meters;
  extent.max_x += padding_meters;
  extent.max_y += padding_meters;
  return extent;
}

void Vda5050MissionParser::saveGlobalCostmapArtifacts(
  const RasterizedMap & map,
  const std::string & image_path,
  const std::string & yaml_path) const
{
  namespace fs = std::filesystem;
  fs::create_directories(fs::path(image_path).parent_path());
  fs::create_directories(fs::path(yaml_path).parent_path());

  std::ofstream image_stream(image_path, std::ios::binary);
  image_stream << "P5\n" << map.width_cells << " " << map.height_cells << "\n255\n";
  for (int row = static_cast<int>(map.height_cells) - 1; row >= 0; --row) {
    for (unsigned int col = 0; col < map.width_cells; ++col) {
      const std::size_t index = static_cast<std::size_t>(row) * map.width_cells + col;
      const unsigned char pixel = static_cast<unsigned char>(255U - map.costs.at(index));
      image_stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  std::ofstream yaml_stream(yaml_path);
  yaml_stream
    << "image: " << fs::path(image_path).filename().string() << "\n"
    << "resolution: " << map.resolution << "\n"
    << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n"
    << "mode: trinary\n";
}

void Vda5050MissionParser::saveMissionWaypointsArtifact(const std::string & path) const
{
  namespace fs = std::filesystem;
  fs::create_directories(fs::path(path).parent_path());

  nlohmann::json coordinates = nlohmann::json::array();
  bool use_local_frame = false;
  for (const auto & waypoint : mission_waypoints_) {
    if (waypoint.use_local_frame) {
      coordinates.push_back({waypoint.map_point.x, waypoint.map_point.y});
      use_local_frame = true;
    } else {
      coordinates.push_back({waypoint.geo_point.longitude, waypoint.geo_point.latitude});
    }
  }

  nlohmann::json document = {
    {"type", "FeatureCollection"},
    {"features", {{
      {"type", "Feature"},
      {"properties", {
         {"name", "coverage_path"},
         {"source", "vda5050_mission"},
         {"coordinate_frame", use_local_frame ? "odom" : "wgs84"}}},
      {"geometry", {{"type", "LineString"}, {"coordinates", coordinates}}}
    }}}
  };

  std::ofstream output_stream(path);
  output_stream << document.dump(2) << "\n";
}

bool Vda5050MissionParser::hasWorkingZones() const
{
  return !working_zones_.empty();
}

bool Vda5050MissionParser::hasMissionWaypoints() const
{
  return mission_waypoints_.size() >= 2U;
}

void Vda5050MissionParser::projectAndStoreZone(
  const std::vector<GeoPoint> & geo_vertices,
  const std::string & zone_name,
  const std::string & zone_type)
{
  if (!projection_initialized_) {
    if (config_.use_first_polygon_vertex_as_origin) {
      projector_.Reset(
        geo_vertices.front().latitude,
        geo_vertices.front().longitude,
        config_.origin_altitude);
    } else {
      projector_.Reset(
        config_.origin_latitude,
        config_.origin_longitude,
        config_.origin_altitude);
    }
    projection_initialized_ = true;
  }

  PolygonZone zone;
  zone.name = zone_name;
  zone.zone_type = zone_type;
  for (const auto & geo_point : geo_vertices) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    projector_.Forward(geo_point.latitude, geo_point.longitude, 0.0, x, y, z);
    zone.vertices.push_back(MapPoint{x, y});
  }

  if (zone_type == normalizeZoneType(config_.no_go_zone_value)) {
    no_go_zones_.push_back(zone);
  } else {
    working_zones_.push_back(zone);
  }
}

void Vda5050MissionParser::loadCoveragePath(
  const nlohmann::json & coverage_edge_ids,
  const std::unordered_map<std::string, GeoPoint> & nodes_by_id,
  const std::unordered_map<std::string, double> & node_theta_by_id,
  const std::unordered_map<std::string, nlohmann::json> & edges_by_id)
{
  mission_waypoints_ = buildCoverageWaypoints(
    coverage_edge_ids,
    nodes_by_id,
    node_theta_by_id,
    edges_by_id);
}

bool Vda5050MissionParser::pointInPolygon(const MapPoint & point, const PolygonZone & polygon) const
{
  bool inside = false;
  const std::size_t vertex_count = polygon.vertices.size();
  for (std::size_t i = 0, j = vertex_count - 1; i < vertex_count; j = i++) {
    const MapPoint & a = polygon.vertices.at(i);
    const MapPoint & b = polygon.vertices.at(j);
    const bool intersect =
      ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) /
      ((b.y - a.y) + std::numeric_limits<double>::epsilon()) + a.x);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

double Vda5050MissionParser::signedDistanceToPolygon(
  const MapPoint & point,
  const PolygonZone & polygon) const
{
  double min_distance = std::numeric_limits<double>::max();
  const std::size_t vertex_count = polygon.vertices.size();
  for (std::size_t i = 0; i < vertex_count; ++i) {
    min_distance = std::min(
      min_distance,
      distanceToSegment(
        point,
        polygon.vertices.at(i),
        polygon.vertices.at((i + 1U) % vertex_count)));
  }
  return pointInPolygon(point, polygon) ? min_distance : -min_distance;
}

double Vda5050MissionParser::distanceToSegment(
  const MapPoint & point,
  const MapPoint & start,
  const MapPoint & end) const
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double segment_length_squared = (dx * dx) + (dy * dy);
  if (segment_length_squared <= std::numeric_limits<double>::epsilon()) {
    const double point_dx = point.x - start.x;
    const double point_dy = point.y - start.y;
    return std::sqrt((point_dx * point_dx) + (point_dy * point_dy));
  }

  const double projection =
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / segment_length_squared;
  const double clamped_projection = clampToUnitInterval(projection);
  const double closest_x = start.x + clamped_projection * dx;
  const double closest_y = start.y + clamped_projection * dy;
  const double distance_x = point.x - closest_x;
  const double distance_y = point.y - closest_y;
  return std::sqrt((distance_x * distance_x) + (distance_y * distance_y));
}

unsigned char Vda5050MissionParser::costForPoint(const MapPoint & point) const
{
  bool inside_any_working_zone = false;
  double nearest_working_zone_distance = -std::numeric_limits<double>::max();

  for (const auto & working_zone : working_zones_) {
    const double signed_distance = signedDistanceToPolygon(point, working_zone);
    inside_any_working_zone = inside_any_working_zone || (signed_distance >= 0.0);
    nearest_working_zone_distance = std::max(nearest_working_zone_distance, signed_distance);
  }

  if (inside_any_working_zone) {
    for (const auto & no_go_zone : no_go_zones_) {
      if (pointInPolygon(point, no_go_zone)) {
        return static_cast<unsigned char>(config_.no_go_cost);
      }
    }
    if (nearest_working_zone_distance <= config_.edge_band_meters) {
      return static_cast<unsigned char>(config_.edge_band_cost);
    }
    return static_cast<unsigned char>(config_.inside_cost);
  }

  if (nearest_working_zone_distance >= -config_.edge_band_meters) {
    return static_cast<unsigned char>(config_.edge_band_cost);
  }
  return static_cast<unsigned char>(config_.outside_cost);
}

std::string Vda5050MissionParser::resolveMissionPath(const std::string & configured_path) const
{
  namespace fs = std::filesystem;
  const fs::path configured(configured_path);
  if (configured.is_absolute()) {
    return configured.string();
  }
  const fs::path workspace_relative = fs::current_path() / configured;
  if (fs::exists(workspace_relative)) {
    return workspace_relative.string();
  }
  return configured.string();
}

MissionIdentity Vda5050MissionParser::extractMissionIdentity(const nlohmann::json & document) const
{
  if (!document.contains("orderId") || !document.at("orderId").is_string()) {
    throw std::runtime_error("VDA5050 mission is missing string orderId");
  }
  if (!document.contains("timestamp") || !document.at("timestamp").is_string()) {
    throw std::runtime_error("VDA5050 mission is missing string timestamp");
  }

  MissionIdentity identity;
  identity.order_id = sanitizeStemToken(document.at("orderId").get<std::string>());
  identity.timestamp = sanitizeTimestamp(document.at("timestamp").get<std::string>());
  if (identity.order_id.empty()) {
    throw std::runtime_error("VDA5050 mission orderId cannot sanitize to an empty folder stem");
  }
  if (identity.timestamp.empty()) {
    throw std::runtime_error("VDA5050 mission timestamp cannot sanitize to an empty folder stem");
  }
  identity.stem = identity.order_id + "_" + identity.timestamp;
  return identity;
}

std::string Vda5050MissionParser::sanitizeTimestamp(const std::string & timestamp)
{
  std::string sanitized;
  sanitized.reserve(timestamp.size());
  for (const unsigned char character : timestamp) {
    if (std::isdigit(character) != 0U || character == 'T' || character == 'Z') {
      sanitized.push_back(static_cast<char>(character));
    }
  }
  return sanitized;
}

std::string Vda5050MissionParser::sanitizeStemToken(const std::string & value)
{
  return sanitize_token(value);
}

std::string Vda5050MissionParser::normalizeZoneType(const std::string & zone_type)
{
  std::string normalized = zone_type;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized;
}

MissionParserNode::MissionParserNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vda5050_parser_node", options)
{
  mission_path_ = declare_parameter<std::string>("mission_path", "");
  missions_directory_ = declare_parameter<std::string>("missions_directory", "src/missions_from_db");
  missions_log_directory_ = declare_parameter<std::string>("missions_log_directory", "src/missions_log");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  costmap_output_basename_ = declare_parameter<std::string>("costmap_output_basename", "global_costmap");
  coverage_path_basename_ = declare_parameter<std::string>(
    "coverage_path_basename",
    "active_mission_path");
  mission_build_resolution_ = declare_parameter<double>("mission_build_resolution", 0.1);
  mission_build_padding_meters_ = declare_parameter<double>("mission_build_padding_meters", 2.0);
  auto_build_on_start_ = declare_parameter<bool>("auto_build_on_start", true);
  watch_for_updates_ = declare_parameter<bool>("watch_for_updates", true);

  mission_parser_ = std::make_unique<Vda5050MissionParser>();
  status_publisher_ = create_publisher<std_msgs::msg::String>("mission_parser/status", 10);
  build_current_mission_service_ = create_service<std_srvs::srv::Trigger>(
    "build_current_mission",
    std::bind(
      &MissionParserNode::handleBuildCurrentMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  build_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&MissionParserNode::buildIfNeeded, this));

  RCLCPP_INFO(
    get_logger(),
    "MissionParserNode watching %s, reading source JSON from %s, and writing staged mission artifacts to %s",
    mission_path_.empty() ? "<auto-discovery>" : mission_path_.c_str(),
    missions_directory_.c_str(),
    missions_log_directory_.c_str());

  if (auto_build_on_start_) {
    buildIfNeeded();
  }
}

void MissionParserNode::buildIfNeeded()
{
  if (!watch_for_updates_ && !mission_build_stamps_.empty()) {
    return;
  }

  buildDiscoveredMissionArtifacts();
  (void)buildCurrentMissionArtifacts();
}

bool MissionParserNode::buildCurrentMissionArtifacts()
{
  const auto mission_path = selectActiveMissionPath();
  if (!mission_path) {
    publishStatus("waiting", "no active mission selected");
    if (!waiting_for_active_mission_logged_) {
      RCLCPP_INFO(get_logger(), "Waiting for an active mission selection");
      waiting_for_active_mission_logged_ = true;
    }
    return false;
  }
  waiting_for_active_mission_logged_ = false;

  const auto current_stamp = currentMissionStamp(*mission_path);
  const auto mission_key = mission_path->string();
  const auto stamp_it = mission_build_stamps_.find(mission_key);
  const bool mission_changed =
    stamp_it == mission_build_stamps_.end() || stamp_it->second != current_stamp;
  if (!mission_changed) {
    return true;
  }

  return buildArtifactsForMission(*mission_path, true);
}

void MissionParserNode::buildDiscoveredMissionArtifacts()
{
  const std::filesystem::path missions_directory = resolvePath(missions_directory_);
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    publishStatus("waiting", "missions directory missing");
    return;
  }

  for (const auto & mission_path : discoverMissionPaths()) {
    const auto current_stamp = currentMissionStamp(mission_path);
    const auto mission_key = mission_path.string();
    const auto stamp_it = mission_build_stamps_.find(mission_key);
    if (stamp_it != mission_build_stamps_.end() && stamp_it->second == current_stamp) {
      continue;
    }
    (void)buildArtifactsForMission(mission_path, false);
  }
}

std::vector<std::filesystem::path> MissionParserNode::discoverMissionPaths()
{
  std::vector<std::filesystem::path> mission_paths;
  const std::filesystem::path missions_directory = resolvePath(missions_directory_);
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return mission_paths;
  }

  auto maybe_add_mission = [this, &mission_paths](const std::filesystem::path & candidate_path) {
      if (!std::filesystem::is_regular_file(candidate_path) ||
        candidate_path.extension() != mission_file_extension_)
      {
        return;
      }
      try {
        const auto document = loadJsonDocument(candidate_path);
        if (!isValidVda5050MissionDocument(document)) {
          return;
        }
        mission_paths.push_back(stageMissionFile(candidate_path));
      } catch (const std::exception &) {
        return;
      }
    };

  for (const auto & entry : std::filesystem::directory_iterator(missions_directory)) {
    if (entry.is_regular_file()) {
      maybe_add_mission(entry.path());
      continue;
    }
    if (!entry.is_directory()) {
      continue;
    }
    for (const auto & nested_entry : std::filesystem::directory_iterator(entry.path())) {
      if (nested_entry.is_regular_file()) {
        maybe_add_mission(nested_entry.path());
      }
    }
  }

  std::sort(mission_paths.begin(), mission_paths.end());
  mission_paths.erase(std::unique(mission_paths.begin(), mission_paths.end()), mission_paths.end());
  return mission_paths;
}

std::optional<std::filesystem::path> MissionParserNode::selectActiveMissionPath()
{
  if (!mission_path_.empty()) {
    const std::filesystem::path configured_path = resolveMissionPath();
    if (std::filesystem::exists(configured_path)) {
      return stageMissionFile(configured_path);
    }
    const std::filesystem::path fallback_source =
      resolvePath(missions_directory_) / (configured_path.stem().string() + mission_file_extension_);
    if (std::filesystem::exists(fallback_source)) {
      return stageMissionFile(fallback_source);
    }
    for (const auto & discovered_path : discoverMissionPaths()) {
      if (missionStemForPath(discovered_path) == configured_path.stem().string()) {
        return discovered_path;
      }
    }
  }

  const auto discovered_missions = discoverMissionPaths();
  if (discovered_missions.size() == 1U) {
    return discovered_missions.front();
  }
  return std::nullopt;
}

std::filesystem::path MissionParserNode::stageMissionFile(const std::filesystem::path & mission_path)
{
  const MissionIdentity identity = mission_parser_->inspectMissionIdentity(mission_path.string());
  const std::filesystem::path mission_folder = resolveMissionsLogDirectory() / identity.stem;
  const std::filesystem::path staged_path = mission_folder / (identity.stem + mission_file_extension_);
  std::filesystem::create_directories(mission_folder);

  if (mission_path == staged_path) {
    return staged_path;
  }

  std::filesystem::copy_file(
    mission_path,
    staged_path,
    std::filesystem::copy_options::overwrite_existing);
  return staged_path;
}

bool MissionParserNode::buildArtifactsForMission(
  const std::filesystem::path & mission_path,
  const bool write_active_aliases)
{
  try {
    const std::filesystem::path staged_mission_path = stageMissionFile(mission_path);
    Vda5050MissionBuildConfig config;
    config.mission_path = staged_mission_path.string();
    mission_parser_->loadMission(config);
    const RasterizedMap rasterized_map = mission_parser_->buildSuggestedGlobalCostmap(
      mission_build_resolution_,
      mission_build_padding_meters_);

    const std::filesystem::path missions_directory = resolvePath(missions_directory_);
    const std::filesystem::path mission_directory = missionFolderPath(staged_mission_path);
    const std::string costmap_basename = costmapBasenameForMission(staged_mission_path);
    const std::string coverage_basename = coverageBasenameForMission(staged_mission_path);
    const std::filesystem::path mission_image_path = mission_directory / (costmap_basename + ".pgm");
    const std::filesystem::path mission_yaml_path = mission_directory / (costmap_basename + ".yaml");
    const std::filesystem::path mission_coverage_path = mission_directory / (coverage_basename + ".geojson");

    mission_parser_->saveGlobalCostmapArtifacts(
      rasterized_map,
      mission_image_path.string(),
      mission_yaml_path.string());
    if (mission_parser_->hasMissionWaypoints()) {
      mission_parser_->saveMissionWaypointsArtifact(mission_coverage_path.string());
    }

    const std::filesystem::path legacy_image_path = missions_directory / (costmap_basename + ".pgm");
    const std::filesystem::path legacy_yaml_path = missions_directory / (costmap_basename + ".yaml");
    const std::filesystem::path legacy_coverage_path = missions_directory / (coverage_basename + ".geojson");
    if (legacy_image_path != mission_image_path) {
      std::filesystem::remove(legacy_image_path);
    }
    if (legacy_yaml_path != mission_yaml_path) {
      std::filesystem::remove(legacy_yaml_path);
    }
    if (legacy_coverage_path != mission_coverage_path) {
      std::filesystem::remove(legacy_coverage_path);
    }

    (void)write_active_aliases;

    mission_build_stamps_[staged_mission_path.string()] = currentMissionStamp(staged_mission_path);
    last_build_error_key_.clear();
    publishStatus("built", staged_mission_path.string());
    RCLCPP_INFO(get_logger(), "Built mission artifacts from %s", staged_mission_path.string().c_str());
    return true;
  } catch (const std::exception & exception) {
    publishStatus("error", exception.what());
    std::ostringstream error_key;
    error_key << mission_path.string() << '\n' << exception.what();
    if (last_build_error_key_ != error_key.str()) {
      last_build_error_key_ = error_key.str();
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build mission artifacts from %s: %s",
        mission_path.string().c_str(),
        exception.what());
    }
    return false;
  }
}

void MissionParserNode::handleBuildCurrentMission(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  mission_path_ = get_parameter("mission_path").as_string();
  const bool success = buildCurrentMissionArtifacts();
  response->success = success;
  response->message = success ?
    ("Built " + (mission_path_.empty() ? std::string("<auto-selected>") : resolveMissionPath().string())) :
    ("Failed to build " + (mission_path_.empty() ? std::string("<auto-selected>") : resolveMissionPath().string()));
}

std::filesystem::path MissionParserNode::resolveMissionPath() const
{
  if (mission_path_.empty()) {
    return {};
  }
  return resolvePath(mission_path_);
}

std::filesystem::path MissionParserNode::resolvePath(const std::string & path) const
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

std::filesystem::path MissionParserNode::resolveMissionsLogDirectory() const
{
  return resolvePath(missions_log_directory_);
}

std::filesystem::file_time_type MissionParserNode::currentMissionStamp(
  const std::filesystem::path & mission_path) const
{
  return std::filesystem::last_write_time(mission_path);
}

std::filesystem::path MissionParserNode::missionFolderPath(
  const std::filesystem::path & mission_path) const
{
  return mission_path.parent_path();
}

std::string MissionParserNode::missionStemForPath(const std::filesystem::path & mission_path) const
{
  if (mission_path.has_parent_path() && mission_path.parent_path() != resolvePath(missions_directory_)) {
    return mission_path.parent_path().filename().string();
  }
  return mission_path.stem().string();
}

std::string MissionParserNode::coverageBasenameForMission(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_path";
}

std::string MissionParserNode::costmapBasenameForMission(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_costmap";
}

void MissionParserNode::publishStatus(const std::string & state, const std::string & detail) const
{
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "state=" << state
         << "; detail=" << detail
         << "; active_mission=" << resolveMissionPath().string();
  message.data = stream.str();
  status_publisher_->publish(message);
}

}  // namespace amr_sweeper_vda5050_parser

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_vda5050_parser::MissionParserNode>());
  rclcpp::shutdown();
  return 0;
}
