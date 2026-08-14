#include "vda5050_parser_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <set>
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
         document.contains("version") &&
         document.at("version").is_string() &&
         document.at("version").get<std::string>().rfind("3.", 0) == 0 &&
         document.contains("orderId") &&
         document.at("orderId").is_string() &&
         document.contains("orderUpdateId") &&
         document.at("orderUpdateId").is_number_integer() &&
         document.contains("nodes") &&
         document.at("nodes").is_array() &&
         !document.at("nodes").empty() &&
         document.contains("edges") &&
         document.at("edges").is_array() &&
         !document.contains("missionGeometries") &&
         !document.contains("missionReference");
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

}  // namespace

bool isFiniteNumber(const nlohmann::json & value);
std::string requireString(
  const nlohmann::json & document,
  const std::string & key,
  const std::string & context);
std::uint32_t requireUint32(
  const nlohmann::json & document,
  const std::string & key,
  const std::string & context);
void requireActionsArray(const nlohmann::json & document, const std::string & context);
MapExtent parseBounds(const nlohmann::json & bounds);

void Vda5050MissionParser::loadMission(const Vda5050MissionBuildConfig & config)
{
  config_ = config;
  working_zones_.clear();
  no_go_zones_.clear();
  mission_waypoints_.clear();
  map_georeferences_.clear();
  order_map_ids_.clear();
  projection_initialized_ = false;

  const std::filesystem::path order_path = packageOrderPath(resolveMissionPath(config.mission_path));
  std::ifstream input_stream(order_path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission file: " + config.mission_path);
  }

  nlohmann::json document;
  input_stream >> document;

  if (document.contains("features") && document.at("features").is_array()) {
    loadFromLegacyGeoJson(document);
  } else if (document.contains("nodes") && document.contains("edges")) {
    validateVda5050Order(document);
    loadMapGeoreference(order_path);
    loadFromVda5050Order(document);
    const auto zone_set_path = packageZoneSetPath(order_path);
    if (std::filesystem::exists(zone_set_path)) {
      const auto zone_set = loadJsonDocument(zone_set_path);
      validateVda5050ZoneSet(zone_set);
      loadVda5050ZoneSet(zone_set);
    }
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
  return extractMissionIdentity(loadJsonDocument(packageOrderPath(resolveMissionPath(mission_path))));
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

void Vda5050MissionParser::loadFromVda5050Order(const nlohmann::json & document)
{
  std::map<std::uint32_t, nlohmann::json> nodes_by_sequence_id;
  std::map<std::uint32_t, nlohmann::json> edges_by_sequence_id;
  for (const auto & node : document.at("nodes")) {
    nodes_by_sequence_id.emplace(node.at("sequenceId").get<std::uint32_t>(), node);
    order_map_ids_.insert(node.at("nodePosition").at("mapId").get<std::string>());
  }
  for (const auto & edge : document.at("edges")) {
    edges_by_sequence_id.emplace(edge.at("sequenceId").get<std::uint32_t>(), edge);
  }

  for (const auto & map_id : order_map_ids_) {
    if (map_georeferences_.find(map_id) == map_georeferences_.end()) {
      throw std::runtime_error("VDA5050 order references mapId without map_georeference: " + map_id);
    }
  }

  std::vector<MissionPathWaypoint> route;
  route.reserve(nodes_by_sequence_id.size());
  const auto append_node_position = [&route](const nlohmann::json & node) {
      const auto & position = node.at("nodePosition");
      route.push_back(
        MissionPathWaypoint{
          node.at("nodeId").get<std::string>(),
          GeoPoint{},
          MapPoint{position.at("x").get<double>(), position.at("y").get<double>()},
          position.value("theta", 0.0),
          true});
    };

  append_node_position(nodes_by_sequence_id.at(0U));
  for (std::uint32_t sequence_id = 1U; sequence_id < nodes_by_sequence_id.rbegin()->first;
    sequence_id += 2U)
  {
    const auto & edge = edges_by_sequence_id.at(sequence_id);
    if (edge.contains("trajectory") && edge.at("trajectory").is_object() &&
      edge.at("trajectory").contains("controlPoints") &&
      edge.at("trajectory").at("controlPoints").is_array())
    {
      for (const auto & control_point : edge.at("trajectory").at("controlPoints")) {
        route.push_back(
          MissionPathWaypoint{
            edge.at("edgeId").get<std::string>(),
            GeoPoint{},
            MapPoint{control_point.at("x").get<double>(), control_point.at("y").get<double>()},
            0.0,
            true});
      }
    }
    append_node_position(nodes_by_sequence_id.at(sequence_id + 1U));
  }
  loadCoveragePath(route);
}

void Vda5050MissionParser::loadVda5050ZoneSet(const nlohmann::json & document)
{
  const auto & zone_set = document.at("zoneSet");
  const std::string map_id = zone_set.at("mapId").get<std::string>();
  if (map_georeferences_.find(map_id) == map_georeferences_.end()) {
    throw std::runtime_error("zoneSet references mapId without map_georeference: " + map_id);
  }
  for (const auto & zone_document : zone_set.at("zones")) {
    PolygonZone zone;
    zone.name = zone_document.at("zoneId").get<std::string>();
    zone.zone_type = zone_document.at("zoneType").get<std::string>();
    for (const auto & vertex : zone_document.at("vertices")) {
      zone.vertices.push_back(MapPoint{vertex.at("x").get<double>(), vertex.at("y").get<double>()});
    }
    if (zone.zone_type == "BLOCKED") {
      no_go_zones_.push_back(zone);
    }
  }
}

void Vda5050MissionParser::loadMapGeoreference(const std::filesystem::path & order_path)
{
  const auto metadata_path = packageMapGeoreferencePath(order_path);
  if (!std::filesystem::exists(metadata_path)) {
    throw std::runtime_error("VDA5050 mission package is missing map_georeference.json");
  }
  const auto document = loadJsonDocument(metadata_path);
  if (!document.is_object()) {
    throw std::runtime_error("map_georeference.json must be a JSON object");
  }
  const auto maps_document = document.contains("maps") ? document.at("maps") : nlohmann::json::array({document});
  if (!maps_document.is_array() || maps_document.empty()) {
    throw std::runtime_error("map_georeference.json must define at least one map");
  }

  for (const auto & map_document : maps_document) {
    MapGeoreference georeference;
    georeference.map_id = requireString(map_document, "mapId", "map_georeference map");
    georeference.map_version = map_document.value("mapVersion", std::string{});
    georeference.crs = map_document.value("crs", std::string{"EPSG:4326"});
    georeference.units = map_document.value("units", std::string{"m"});
    georeference.frame = map_document.value("frame", std::string{"ENU"});
    if (georeference.units != "m") {
      throw std::runtime_error("map_georeference units must be \"m\" for mapId " + georeference.map_id);
    }
    for (const auto & key : {"originLatitude", "originLongitude"}) {
      if (!map_document.contains(key) || !isFiniteNumber(map_document.at(key))) {
        throw std::runtime_error(std::string("map_georeference map is missing finite ") + key);
      }
    }
    georeference.origin_latitude = map_document.at("originLatitude").get<double>();
    georeference.origin_longitude = map_document.at("originLongitude").get<double>();
    georeference.origin_altitude = map_document.value("originAltitude", 0.0);
    georeference.yaw = map_document.value("yaw", 0.0);
    georeference.bounds = parseBounds(map_document.at("bounds"));
    if (georeference.bounds.max_x <= georeference.bounds.min_x ||
      georeference.bounds.max_y <= georeference.bounds.min_y)
    {
      throw std::runtime_error("map_georeference bounds are empty for mapId " + georeference.map_id);
    }

    map_georeferences_.emplace(georeference.map_id, georeference);

    PolygonZone bounds_zone;
    bounds_zone.name = georeference.map_id + "_bounds";
    bounds_zone.zone_type = normalizeZoneType(config_.working_zone_value);
    bounds_zone.vertices = {
      MapPoint{georeference.bounds.min_x, georeference.bounds.min_y},
      MapPoint{georeference.bounds.max_x, georeference.bounds.min_y},
      MapPoint{georeference.bounds.max_x, georeference.bounds.max_y},
      MapPoint{georeference.bounds.min_x, georeference.bounds.max_y}};
    working_zones_.push_back(bounds_zone);

    if (!projection_initialized_) {
      projector_.Reset(
        georeference.origin_latitude,
        georeference.origin_longitude,
        georeference.origin_altitude);
      projection_initialized_ = true;
    }
  }
}

void Vda5050MissionParser::validateVda5050Order(const nlohmann::json & document) const
{
  for (const auto & forbidden_key : {"missionReference", "missionGeometries", "coveragePathEdgeIds",
      "workingZones", "noGoZones", "mission_type"})
  {
    if (document.contains(forbidden_key)) {
      throw std::runtime_error(std::string("VDA5050 order contains non-compliant field: ") + forbidden_key);
    }
  }
  for (const auto & key : {"headerId", "timestamp", "version", "manufacturer", "serialNumber",
      "orderId", "orderUpdateId", "nodes", "edges"})
  {
    if (!document.contains(key)) {
      throw std::runtime_error(std::string("VDA5050 order is missing required field: ") + key);
    }
  }
  const std::string version = document.at("version").get<std::string>();
  if (!isSupportedVda5050Version(version)) {
    throw std::runtime_error("Unsupported VDA5050 version: " + version);
  }
  if (!document.at("nodes").is_array() || document.at("nodes").empty()) {
    throw std::runtime_error("VDA5050 order nodes must be a non-empty array");
  }
  if (!document.at("edges").is_array()) {
    throw std::runtime_error("VDA5050 order edges must be an array");
  }
  const auto & nodes = document.at("nodes");
  const auto & edges = document.at("edges");
  if (nodes.size() != edges.size() + 1U) {
    throw std::runtime_error("VDA5050 order requires edges.size() == nodes.size() - 1");
  }

  std::map<std::uint32_t, std::string> node_id_by_sequence;
  std::map<std::uint32_t, nlohmann::json> edge_by_sequence;
  bool horizon_started = false;
  for (const auto & node : nodes) {
    const std::uint32_t sequence_id = requireUint32(node, "sequenceId", "VDA5050 node");
    requireString(node, "nodeId", "VDA5050 node");
    requireActionsArray(node, "VDA5050 node");
    if (!node.contains("released") || !node.at("released").is_boolean()) {
      throw std::runtime_error("VDA5050 node is missing boolean released");
    }
    if ((sequence_id % 2U) != 0U) {
      throw std::runtime_error("VDA5050 node sequenceId must be even");
    }
    if (node_id_by_sequence.find(sequence_id) != node_id_by_sequence.end()) {
      throw std::runtime_error("Duplicate VDA5050 node sequenceId");
    }
    const bool released = node.at("released").get<bool>();
    if (sequence_id == 0U && !released) {
      throw std::runtime_error("First VDA5050 node must be released");
    }
    if (horizon_started && released) {
      throw std::runtime_error("Released VDA5050 node appears after horizon started");
    }
    horizon_started = horizon_started || !released;
    if (!node.contains("nodePosition") || !node.at("nodePosition").is_object()) {
      throw std::runtime_error("VDA5050 node is missing nodePosition object");
    }
    const auto & position = node.at("nodePosition");
    if (!position.contains("x") || !isFiniteNumber(position.at("x")) ||
      !position.contains("y") || !isFiniteNumber(position.at("y")) ||
      !position.contains("mapId") || !position.at("mapId").is_string())
    {
      throw std::runtime_error("VDA5050 nodePosition requires finite x/y meters and string mapId");
    }
    if (position.contains("theta") && !isFiniteNumber(position.at("theta"))) {
      throw std::runtime_error("VDA5050 nodePosition theta must be finite when present");
    }
    node_id_by_sequence.emplace(sequence_id, node.at("nodeId").get<std::string>());
  }
  horizon_started = false;
  for (const auto & edge : edges) {
    const std::uint32_t sequence_id = requireUint32(edge, "sequenceId", "VDA5050 edge");
    requireString(edge, "edgeId", "VDA5050 edge");
    requireString(edge, "startNodeId", "VDA5050 edge");
    requireString(edge, "endNodeId", "VDA5050 edge");
    requireActionsArray(edge, "VDA5050 edge");
    if (!edge.contains("released") || !edge.at("released").is_boolean()) {
      throw std::runtime_error("VDA5050 edge is missing boolean released");
    }
    if ((sequence_id % 2U) != 1U) {
      throw std::runtime_error("VDA5050 edge sequenceId must be odd");
    }
    if (edge_by_sequence.find(sequence_id) != edge_by_sequence.end()) {
      throw std::runtime_error("Duplicate VDA5050 edge sequenceId");
    }
    const bool released = edge.at("released").get<bool>();
    if (horizon_started && released) {
      throw std::runtime_error("Released VDA5050 edge appears after horizon started");
    }
    horizon_started = horizon_started || !released;
    edge_by_sequence.emplace(sequence_id, edge);
  }
  for (std::uint32_t sequence_id = 0U; sequence_id < nodes.size() * 2U; sequence_id += 2U) {
    if (node_id_by_sequence.find(sequence_id) == node_id_by_sequence.end()) {
      throw std::runtime_error("VDA5050 node sequenceIds are not continuous");
    }
  }
  for (std::uint32_t sequence_id = 1U; sequence_id < edges.size() * 2U; sequence_id += 2U) {
    const auto edge_it = edge_by_sequence.find(sequence_id);
    if (edge_it == edge_by_sequence.end()) {
      throw std::runtime_error("VDA5050 edge sequenceIds are not continuous");
    }
    const auto & edge = edge_it->second;
    if (edge.at("startNodeId").get<std::string>() != node_id_by_sequence.at(sequence_id - 1U) ||
      edge.at("endNodeId").get<std::string>() != node_id_by_sequence.at(sequence_id + 1U))
    {
      throw std::runtime_error("VDA5050 edge endpoints do not match adjacent sequence nodes");
    }
  }
}

void Vda5050MissionParser::validateVda5050ZoneSet(const nlohmann::json & document) const
{
  if (!document.is_object() || !document.contains("zoneSet") || !document.at("zoneSet").is_object()) {
    throw std::runtime_error("zoneSet.json must be an official VDA5050 zoneSet message");
  }
  const std::string version = requireString(document, "version", "VDA5050 zoneSet");
  if (!isSupportedVda5050Version(version)) {
    throw std::runtime_error("Unsupported VDA5050 zoneSet version: " + version);
  }
  const auto & zone_set = document.at("zoneSet");
  requireString(zone_set, "mapId", "VDA5050 zoneSet");
  requireString(zone_set, "zoneSetId", "VDA5050 zoneSet");
  if (!zone_set.contains("zones") || !zone_set.at("zones").is_array()) {
    throw std::runtime_error("VDA5050 zoneSet is missing zones array");
  }
  static const std::set<std::string> kSupportedZoneTypes{
    "BLOCKED", "LINE_GUIDED", "RELEASE", "COORDINATED_REPLANNING", "SPEED_LIMIT",
    "ACTION", "PRIORITY", "PENALTY", "DIRECTED", "BIDIRECTED"};
  for (const auto & zone : zone_set.at("zones")) {
    requireString(zone, "zoneId", "VDA5050 zone");
    const std::string zone_type = requireString(zone, "zoneType", "VDA5050 zone");
    if (kSupportedZoneTypes.find(zone_type) == kSupportedZoneTypes.end()) {
      throw std::runtime_error("Unsupported VDA5050 zoneType: " + zone_type);
    }
    if (!zone.contains("vertices") || !zone.at("vertices").is_array() ||
      zone.at("vertices").size() < 3U)
    {
      throw std::runtime_error("VDA5050 zone requires at least three vertices");
    }
    for (const auto & vertex : zone.at("vertices")) {
      if (!vertex.is_object() || !isFiniteNumber(vertex.at("x")) || !isFiniteNumber(vertex.at("y"))) {
        throw std::runtime_error("VDA5050 zone vertex requires finite x/y meters");
      }
    }
  }
}

bool Vda5050MissionParser::isSupportedVda5050Version(const std::string & version) const
{
  static const std::regex version_pattern(R"(^3\.[0-9]+\.[0-9]+$)");
  if (!std::regex_match(version, version_pattern)) {
    return false;
  }
  if (config_.supported_vda5050_versions.empty()) {
    return true;
  }
  return std::find(
    config_.supported_vda5050_versions.begin(),
    config_.supported_vda5050_versions.end(),
    version) != config_.supported_vda5050_versions.end();
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


  if (projection_initialized_) {
    double latitude_at_origin = 0.0;
    double longitude_at_origin = 0.0;
    double altitude_at_origin = 0.0;
    double latitude_at_unit_x = 0.0;
    double longitude_at_unit_x = 0.0;
    double altitude_at_unit_x = 0.0;
    double latitude_at_unit_y = 0.0;
    double longitude_at_unit_y = 0.0;
    double altitude_at_unit_y = 0.0;

    projector_.Reverse(0.0, 0.0, 0.0, latitude_at_origin, longitude_at_origin, altitude_at_origin);
    projector_.Reverse(1.0, 0.0, 0.0, latitude_at_unit_x, longitude_at_unit_x, altitude_at_unit_x);
    projector_.Reverse(0.0, 1.0, 0.0, latitude_at_unit_y, longitude_at_unit_y, altitude_at_unit_y);

    result.georeference_valid = true;
    result.georeference_type = "affine_xy_to_wgs84";
    result.georeference_source_crs = "EPSG:4326";
    result.georeference_sample_count = 3U;
    result.longitude_coefficients = {
      longitude_at_unit_x - longitude_at_origin,
      longitude_at_unit_y - longitude_at_origin,
      longitude_at_origin};
    result.latitude_coefficients = {
      latitude_at_unit_x - latitude_at_origin,
      latitude_at_unit_y - latitude_at_origin,
      latitude_at_origin};
  }

  clearCoveragePathCorridor(result);

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
  yaml_stream << std::setprecision(std::numeric_limits<double>::max_digits10);
  yaml_stream
    << "image: " << fs::path(image_path).filename().string() << "\n"
    << "resolution: " << map.resolution << "\n"
    << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.99\n"
    << "free_thresh: 0.01\n"
    << "mode: scale\n";
  if (map.georeference_valid) {
    yaml_stream
      << "georeference_type: "
      << (map.georeference_type.empty() ? "affine_xy_to_wgs84" : map.georeference_type) << "\n"
      << "georeference_source_crs: "
      << (map.georeference_source_crs.empty() ? "EPSG:4326" : map.georeference_source_crs) << "\n"
      << "georeference_sample_count: " << map.georeference_sample_count << "\n"
      << "georeference_longitude_coefficients: ["
      << map.longitude_coefficients[0] << ", "
      << map.longitude_coefficients[1] << ", "
      << map.longitude_coefficients[2] << "]\n"
      << "georeference_latitude_coefficients: ["
      << map.latitude_coefficients[0] << ", "
      << map.latitude_coefficients[1] << ", "
      << map.latitude_coefficients[2] << "]\n";
  }
}

void Vda5050MissionParser::saveMissionWaypointsArtifact(const std::string & path) const
{
  namespace fs = std::filesystem;
  fs::create_directories(fs::path(path).parent_path());

  nlohmann::json coordinates = nlohmann::json::array();
  nlohmann::json wgs84_coordinates = nlohmann::json::array();
  for (const auto & waypoint : mission_waypoints_) {
    coordinates.push_back({waypoint.map_point.x, waypoint.map_point.y});
    if (projection_initialized_) {
      double latitude = 0.0;
      double longitude = 0.0;
      double altitude = 0.0;
      projector_.Reverse(waypoint.map_point.x, waypoint.map_point.y, 0.0, latitude, longitude, altitude);
      wgs84_coordinates.push_back({longitude, latitude});
    }
  }

  nlohmann::json map_ids = nlohmann::json::array();
  for (const auto & map_id : order_map_ids_) {
    map_ids.push_back(map_id);
  }
  nlohmann::json properties = {
    {"name", "coverage_path"},
    {"source", "vda5050_order"},
    {"coordinate_frame", "map"},
    {"map_ids", map_ids}};
  if (projection_initialized_) {
    properties["georeference_type"] = "local_enu_to_wgs84";
    properties["wgs84_coordinates"] = wgs84_coordinates;
  }

  nlohmann::json document = {
    {"type", "FeatureCollection"},
    {"features", {{
      {"type", "Feature"},
      {"properties", properties},
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
  const std::vector<MissionPathWaypoint> & coverage_path)
{
  mission_waypoints_ = coverage_path;
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

void Vda5050MissionParser::clearCoveragePathCorridor(RasterizedMap & map) const
{
  if (mission_waypoints_.size() < 2U || config_.coverage_path_clearance_meters <= 0.0) {
    return;
  }

  const unsigned char free_cost = static_cast<unsigned char>(
    std::clamp(config_.inside_cost, 0, 254));
  const int8_t free_occupancy = static_cast<int8_t>(
    std::lround((static_cast<double>(free_cost) / 254.0) * 100.0));

  const int clearance_cells = static_cast<int>(std::ceil(
    config_.coverage_path_clearance_meters / std::max(map.resolution, std::numeric_limits<double>::epsilon())));

  const auto clear_cell_if_allowed = [&](const int grid_x, const int grid_y) {
    if (grid_x < 0 || grid_y < 0 ||
      grid_x >= static_cast<int>(map.width_cells) ||
      grid_y >= static_cast<int>(map.height_cells))
    {
      return;
    }

    const MapPoint point{
      map.origin_x + (static_cast<double>(grid_x) + 0.5) * map.resolution,
      map.origin_y + (static_cast<double>(grid_y) + 0.5) * map.resolution};
    for (const auto & no_go_zone : no_go_zones_) {
      if (pointInPolygon(point, no_go_zone)) {
        return;
      }
    }

    const std::size_t index = static_cast<std::size_t>(grid_y) * map.width_cells +
      static_cast<std::size_t>(grid_x);
    map.costs.at(index) = free_cost;
    map.occupancy.at(index) = free_occupancy;
  };
  std::vector<MapPoint> corridor_waypoints;
  corridor_waypoints.reserve(mission_waypoints_.size());
  for (const auto & waypoint : mission_waypoints_) {
    corridor_waypoints.push_back(waypoint.map_point);
  }


  for (const auto & waypoint : corridor_waypoints) {
    const int center_x = static_cast<int>(std::floor((waypoint.x - map.origin_x) / map.resolution));
    const int center_y = static_cast<int>(std::floor((waypoint.y - map.origin_y) / map.resolution));
    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
      for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
        clear_cell_if_allowed(center_x + dx, center_y + dy);
      }
    }
  }

  for (unsigned int iy = 0; iy < map.height_cells; ++iy) {
    for (unsigned int ix = 0; ix < map.width_cells; ++ix) {
      const MapPoint point{
        map.origin_x + (static_cast<double>(ix) + 0.5) * map.resolution,
        map.origin_y + (static_cast<double>(iy) + 0.5) * map.resolution};

      bool inside_no_go_zone = false;
      for (const auto & no_go_zone : no_go_zones_) {
        if (pointInPolygon(point, no_go_zone)) {
          inside_no_go_zone = true;
          break;
        }
      }
      if (inside_no_go_zone) {
        continue;
      }

      bool inside_coverage_corridor = false;
      for (std::size_t waypoint_index = 1U; waypoint_index < corridor_waypoints.size(); ++waypoint_index) {
        const double distance = distanceToSegment(
          point,
          corridor_waypoints.at(waypoint_index - 1U),
          corridor_waypoints.at(waypoint_index));
        if (distance <= config_.coverage_path_clearance_meters) {
          inside_coverage_corridor = true;
          break;
        }
      }

      if (!inside_coverage_corridor) {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(iy) * map.width_cells + ix;
      map.costs.at(index) = free_cost;
      map.occupancy.at(index) = free_occupancy;
    }
  }
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
    if (config_.edge_band_meters > std::numeric_limits<double>::epsilon() &&
      nearest_working_zone_distance <= config_.edge_band_meters)
    {
      const double edge_ratio = clampToUnitInterval(
        (config_.edge_band_meters - nearest_working_zone_distance) /
        config_.edge_band_meters);
      const int inside_cost = std::clamp(config_.inside_cost, 0, 254);
      const int max_traversable_edge_cost = std::min({
          std::clamp(config_.edge_band_cost, 0, 254),
          std::max(inside_cost, std::clamp(config_.outside_cost, 0, 254) - 2),
          252});
      const double scaled_cost =
        static_cast<double>(inside_cost) +
        (static_cast<double>(max_traversable_edge_cost - inside_cost) * edge_ratio);
      return static_cast<unsigned char>(std::lround(scaled_cost));
    }
    return static_cast<unsigned char>(config_.inside_cost);
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

std::filesystem::path Vda5050MissionParser::packageOrderPath(
  const std::filesystem::path & mission_path)
{
  if (std::filesystem::is_directory(mission_path)) {
    return mission_path / "order.json";
  }
  return mission_path;
}

std::filesystem::path Vda5050MissionParser::packageZoneSetPath(
  const std::filesystem::path & order_path)
{
  return order_path.parent_path() / "zoneSet.json";
}

std::filesystem::path Vda5050MissionParser::packageMapGeoreferencePath(
  const std::filesystem::path & order_path)
{
  return order_path.parent_path() / "map_georeference.json";
}

MissionParserNode::MissionParserNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vda5050_parser_node", options)
{
  mission_path_ = declare_parameter<std::string>("mission_path", "");
  missions_directory_ = declare_parameter<std::string>("missions_directory", "missions/database");
  missions_log_directory_ = declare_parameter<std::string>("missions_log_directory", "missions/logs");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  mission_build_resolution_ = declare_parameter<double>("mission_build_resolution", 0.1);
  mission_build_padding_meters_ = declare_parameter<double>("mission_build_padding_meters", 2.0);
  mission_build_coverage_path_clearance_meters_ = declare_parameter<double>(
    "mission_build_coverage_path_clearance_meters", 1.0);
  mission_projection_use_first_polygon_vertex_as_origin_ = declare_parameter<bool>(
    "mission_projection_use_first_polygon_vertex_as_origin", true);
  mission_projection_origin_latitude_ = declare_parameter<double>(
    "mission_projection_origin_latitude", 0.0);
  mission_projection_origin_longitude_ = declare_parameter<double>(
    "mission_projection_origin_longitude", 0.0);
  mission_projection_origin_altitude_ = declare_parameter<double>(
    "mission_projection_origin_altitude", 0.0);
  supported_vda5050_versions_ = declare_parameter<std::vector<std::string>>(
    "supported_vda5050_versions",
    std::vector<std::string>{"3.0.0", "3.0.1", "3.1.0"});
  auto_build_on_start_ = declare_parameter<bool>("auto_build_on_start", true);
  watch_for_updates_ = declare_parameter<bool>("watch_for_updates", true);
  build_discovered_missions_ = declare_parameter<bool>("build_discovered_missions", false);

  mission_parser_ = std::make_unique<Vda5050MissionParser>();
  status_publisher_ = create_publisher<std_msgs::msg::String>("vda5050_parser/status", 10);
  build_current_mission_service_ = create_service<std_srvs::srv::Trigger>(
    "build_current_mission",
    std::bind(
      &MissionParserNode::handleBuildCurrentMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  build_timer_ = create_timer(
    std::chrono::seconds(2),
    std::bind(&MissionParserNode::buildIfNeeded, this));

  RCLCPP_INFO(
    get_logger(),
    "MissionParserNode watching %s, reading source JSON from %s, writing staged mission artifacts to %s, and %s discovered missions",
    mission_path_.empty() ? "<auto-discovery>" : mission_path_.c_str(),
    missions_directory_.c_str(),
    missions_log_directory_.c_str(),
    build_discovered_missions_ ? "eagerly building" : "lazily building");

  if (auto_build_on_start_) {
    buildIfNeeded();
  }
}

void MissionParserNode::buildIfNeeded()
{
  if (!watch_for_updates_ && !mission_build_stamps_.empty()) {
    return;
  }

  if (build_discovered_missions_) {
    buildDiscoveredMissionArtifacts();
  }
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

  return buildArtifactsForMission(*mission_path);
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
    (void)buildArtifactsForMission(mission_path);
  }
}

std::vector<std::filesystem::path> MissionParserNode::discoverMissionPaths()
{
  std::vector<std::filesystem::path> mission_paths;
  const std::filesystem::path missions_directory = resolvePath(missions_directory_);
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return mission_paths;
  }

  auto maybe_add_mission = [&mission_paths](const std::filesystem::path & candidate_path) {
      if (!std::filesystem::is_regular_file(candidate_path) ||
        candidate_path.filename() != "order.json")
      {
        return;
      }
      try {
        const auto document = loadJsonDocument(candidate_path);
        if (!isValidVda5050MissionDocument(document)) {
          return;
        }
        mission_paths.push_back(candidate_path);
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
    maybe_add_mission(entry.path() / "order.json");
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
  const std::filesystem::path mission_folder = resolveMissionsLogDirectory() / identity.order_id;
  const std::filesystem::path staged_path =
    mission_folder / (identity.order_id + "_vda5050" + mission_file_extension_);
  std::filesystem::create_directories(mission_folder);
  const std::filesystem::path source_order_path =
    Vda5050MissionParser::packageOrderPath(mission_path);

  if (source_order_path == staged_path) {
    return staged_path;
  }

  const auto source_stamp = currentMissionStamp(source_order_path);
  bool should_copy = true;
  if (std::filesystem::exists(staged_path) && std::filesystem::is_regular_file(staged_path)) {
    std::error_code size_error;
    const auto source_size = std::filesystem::file_size(source_order_path, size_error);
    if (!size_error) {
      std::error_code staged_size_error;
      const auto staged_size = std::filesystem::file_size(staged_path, staged_size_error);
      if (!staged_size_error) {
        const auto staged_stamp = currentMissionStamp(staged_path);
        should_copy = (source_size != staged_size) || (source_stamp != staged_stamp);
      }
    }
  }

  if (should_copy) {
    std::filesystem::copy_file(
      source_order_path,
      staged_path,
      std::filesystem::copy_options::overwrite_existing);
    std::error_code stamp_error;
    std::filesystem::last_write_time(staged_path, source_stamp, stamp_error);
  }

  const auto copy_support_file = [&mission_folder](const std::filesystem::path & source_path) {
      if (!std::filesystem::exists(source_path) || !std::filesystem::is_regular_file(source_path)) {
        return;
      }
      const auto destination_path = mission_folder / source_path.filename();
      std::filesystem::copy_file(
        source_path,
        destination_path,
        std::filesystem::copy_options::overwrite_existing);
    };
  copy_support_file(Vda5050MissionParser::packageZoneSetPath(source_order_path));
  copy_support_file(Vda5050MissionParser::packageMapGeoreferencePath(source_order_path));
  return staged_path;
}

bool MissionParserNode::buildArtifactsForMission(const std::filesystem::path & mission_path)
{
  try {
    const std::filesystem::path staged_mission_path = stageMissionFile(mission_path);
    Vda5050MissionBuildConfig config;
    config.mission_path = staged_mission_path.string();
    config.coverage_path_clearance_meters = mission_build_coverage_path_clearance_meters_;
    config.use_first_polygon_vertex_as_origin =
      mission_projection_use_first_polygon_vertex_as_origin_;
    config.origin_latitude = mission_projection_origin_latitude_;
    config.origin_longitude = mission_projection_origin_longitude_;
    config.origin_altitude = mission_projection_origin_altitude_;
    config.supported_vda5050_versions = supported_vda5050_versions_;
    mission_parser_->loadMission(config);
    const RasterizedMap rasterized_map = mission_parser_->buildSuggestedGlobalCostmap(
      mission_build_resolution_,
      mission_build_padding_meters_);

    const std::filesystem::path missions_directory = resolvePath(missions_directory_);
    const std::filesystem::path mission_directory = missionFolderPath(staged_mission_path);
    const std::string static_costmap_basename = staticCostmapBasenameForMission(staged_mission_path);
    const std::string coverage_basename = coverageBasenameForMission(staged_mission_path);
    const std::filesystem::path mission_image_path = mission_directory / (static_costmap_basename + ".pgm");
    const std::filesystem::path mission_yaml_path = mission_directory / (static_costmap_basename + ".yaml");
    const std::filesystem::path mission_coverage_path = mission_directory / (coverage_basename + ".geojson");

    mission_parser_->saveGlobalCostmapArtifacts(
      rasterized_map,
      mission_image_path.string(),
      mission_yaml_path.string());
    if (mission_parser_->hasMissionWaypoints()) {
      mission_parser_->saveMissionWaypointsArtifact(mission_coverage_path.string());
    }

    const std::filesystem::path legacy_image_path = missions_directory / (static_costmap_basename + ".pgm");
    const std::filesystem::path legacy_yaml_path = missions_directory / (static_costmap_basename + ".yaml");
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

    mission_build_stamps_[mission_path.string()] = currentMissionStamp(mission_path);
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
  const std::filesystem::path missions_directory = resolvePath(missions_directory_);
  if (mission_path.has_parent_path() && mission_path.parent_path() != missions_directory) {
    const std::filesystem::path parent = mission_path.parent_path();
    if (parent.filename() == "simulations" && parent.parent_path() == missions_directory) {
      return mission_path.stem().string();
    }
    return parent.filename().string();
  }
  return mission_path.stem().string();
}

std::string MissionParserNode::coverageBasenameForMission(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_path_planned";
}

std::string MissionParserNode::staticCostmapBasenameForMission(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_static_costmap";
}

bool isFiniteNumber(const nlohmann::json & value)
{
  return value.is_number() && std::isfinite(value.get<double>());
}

std::string requireString(
  const nlohmann::json & document,
  const std::string & key,
  const std::string & context)
{
  if (!document.contains(key) || !document.at(key).is_string()) {
    throw std::runtime_error(context + " is missing string " + key);
  }
  return document.at(key).get<std::string>();
}

std::uint32_t requireUint32(
  const nlohmann::json & document,
  const std::string & key,
  const std::string & context)
{
  if (!document.contains(key) || !document.at(key).is_number_integer()) {
    throw std::runtime_error(context + " is missing uint32 " + key);
  }
  const auto value = document.at(key).get<std::int64_t>();
  if (value < 0 || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error(context + " has out-of-range uint32 " + key);
  }
  return static_cast<std::uint32_t>(value);
}

void requireActionsArray(const nlohmann::json & document, const std::string & context)
{
  if (!document.contains("actions") || !document.at("actions").is_array()) {
    throw std::runtime_error(context + " is missing VDA5050 actions array");
  }
}

MapExtent parseBounds(const nlohmann::json & bounds)
{
  if (bounds.is_object()) {
    for (const auto & key : {"min_x", "min_y", "max_x", "max_y"}) {
      if (!bounds.contains(key) || !isFiniteNumber(bounds.at(key))) {
        throw std::runtime_error(std::string("map_georeference bounds is missing finite ") + key);
      }
    }
    return MapExtent{
      bounds.at("min_x").get<double>(),
      bounds.at("min_y").get<double>(),
      bounds.at("max_x").get<double>(),
      bounds.at("max_y").get<double>()};
  }
  if (bounds.is_array() && bounds.size() == 4U) {
    for (const auto & value : bounds) {
      if (!isFiniteNumber(value)) {
        throw std::runtime_error("map_georeference bounds array must contain finite numbers");
      }
    }
    return MapExtent{
      bounds.at(0).get<double>(),
      bounds.at(1).get<double>(),
      bounds.at(2).get<double>(),
      bounds.at(3).get<double>()};
  }
  throw std::runtime_error("map_georeference bounds must be an object or [min_x,min_y,max_x,max_y]");
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
