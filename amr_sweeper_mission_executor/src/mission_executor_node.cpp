#include "mission_executor_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <future>
#include <cmath>
#include <iomanip>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

namespace amr_sweeper_mission_executor
{

namespace
{

constexpr char kManualMappingExecutionMode[] = "manual_mapping";
constexpr char kTeleoperationExecutionMode[] = "teleoperation";
constexpr char kNavigateThroughPosesExecutionMode[] = "navigate_through_poses";
constexpr char kBuiltinManualMappingMissionType[] = "builtin_manual_mapping";
constexpr char kBuiltinLocalPatternMissionType[] = "builtin_local_pattern";
constexpr char kBuiltinTeleopMissionType[] = "builtin_teleop";
constexpr char kDefaultMissionsPackageName[] = "amr_sweeper_navigation";
constexpr char kRuntimeStatusStarted[] = "STARTED";
constexpr char kRuntimeStatusCompleted[] = "COMPLETED";
constexpr char kRuntimeStatusAborted[] = "ABORTED";
constexpr char kSafetyScheduleType[] = "SAFETY";
constexpr char kTeleopInactivityEndReason[] = "teleop mission auto-ended after 5 minutes without motion";
constexpr char kManualMappingInactivityEndReason[] =
  "manual mapping mission auto-ended after 5 minutes without motion";
constexpr char kScheduledMissionType[] = "vda5050_scheduled_mission";
constexpr char kLocalScheduledMissionType[] = "vda5050_scheduled_mission_local";
constexpr char kZigzagSweepPattern[] = "zigzag";
constexpr char kRandomSweepPattern[] = "random";
constexpr char kSpiralSweepPattern[] = "spiral";
constexpr char kLatestRecordedMapDirectoryName[] = "latest_recorded_map";
constexpr char kLatestRecordedMapMetadataFile[] = "latest_recorded_map.json";
constexpr char kLatestRecordedMapRouteStem[] = "latest_recorded_map_path";
constexpr char kLatestRecordedMapCostmapStem[] = "latest_recorded_map_costmap";
constexpr char kLatestRecordedMapNavSatStem[] = "latest_recorded_map_navsat";
constexpr char kActualScheduleLogFilename[] = "actual_schedule.ics";
constexpr char kSimulationActualScheduleLogFilename[] = "simulation_schedule.ics";
constexpr char kDepthCameraScanTopic[] = "/amr_sweeper/depth_camera/scan";
constexpr char kDepthCameraInfoTopic[] = "/amr_sweeper/depth_camera/depth/camera_info";
constexpr char kDepthCameraMotionSampleTopic[] = "/amr_sweeper/depth_camera/motion/sample";
constexpr char kSimulationPoseInfoTopic[] = "/amr_sweeper/simulation/pose/info";
constexpr char kImuDataRawTopic[] = "/amr_sweeper/imu/data_raw";
constexpr char kImuDataAccGyroTopic[] = "/amr_sweeper/imu/data_acc_gyro";
constexpr char kImuDataHeadingTopic[] = "/amr_sweeper/imu/data_heading";
constexpr char kImuAzimuthTopic[] = "/amr_sweeper/imu/azimuth";
constexpr double kRecordMapCostmapResolutionMeters = 0.1;
constexpr double kRecordMapCostmapPaddingMeters = 2.0;
constexpr double kRecordMapEdgeBandMeters = 1.0;
constexpr double kRecordMapObstacleRadiusMeters = 0.3;
constexpr double kSweepTrackSpacingMeters = 1.0;
constexpr double kSweepInsetMeters = 0.6;
constexpr double kNavSatSampleDistanceDegrees = 1.0e-6;
constexpr unsigned char kRecordMapInsideCost = 0U;
constexpr unsigned char kRecordMapEdgeBandCost = 180U;
constexpr unsigned char kRecordMapOutsideCost = 254U;
constexpr unsigned char kRecordMapObstacleCost = 254U;

struct MapPoint
{
  double x{0.0};
  double y{0.0};
};

struct GeoPoint
{
  double latitude{0.0};
  double longitude{0.0};
};

struct GeoTransform
{
  bool valid{false};
  double longitude_coefficients[3]{0.0, 0.0, 0.0};
  double latitude_coefficients[3]{0.0, 0.0, 0.0};
};

struct RasterizedCostmap
{
  std::vector<unsigned char> costs;
  unsigned int width_cells{0U};
  unsigned int height_cells{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  bool georeference_valid{false};
  std::string georeference_type;
  std::string georeference_source_crs{"EPSG:4326"};
  std::string georeference_companion_file;
  std::size_t georeference_sample_count{0U};
  std::array<double, 3> longitude_coefficients{0.0, 0.0, 0.0};
  std::array<double, 3> latitude_coefficients{0.0, 0.0, 0.0};
};

struct PolygonBounds
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

std::string toLower(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

std::string missionRunArtifactStem(
  const std::string & mission_id,
  const std::string & run_timestamp)
{
  if (mission_id.empty() || run_timestamp.empty()) {
    return {};
  }
  return mission_id + "_" + run_timestamp;
}

bool hasSuffix(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::filesystem::path resolveExecutionContextPath(
  const std::filesystem::path & mission_execution_directory)
{
  if (mission_execution_directory.empty()) {
    return {};
  }

  std::error_code error;
  if (std::filesystem::exists(mission_execution_directory / "execution_context.json", error)) {
    return mission_execution_directory / "execution_context.json";
  }

  std::vector<std::filesystem::path> matches;
  for (std::filesystem::directory_iterator iterator(
         mission_execution_directory,
         std::filesystem::directory_options::skip_permission_denied,
         error);
       iterator != std::filesystem::directory_iterator();
       iterator.increment(error))
  {
    if (error) {
      error.clear();
      continue;
    }
    if (!iterator->is_regular_file(error)) {
      error.clear();
      continue;
    }
    const auto filename = iterator->path().filename().string();
    if (filename.size() > std::string("_context.json").size() &&
      hasSuffix(filename, "_context.json"))
    {
      matches.push_back(iterator->path());
    }
  }

  if (matches.empty()) {
    return {};
  }
  std::sort(matches.begin(), matches.end());
  return matches.front();
}

bool isExecutionContextArtifact(const std::filesystem::path & path)
{
  const std::string filename = path.filename().string();
  return filename == "execution_context.json" || hasSuffix(filename, "_context.json");
}

bool has_timestamp_suffix(const std::string & candidate, const std::string & prefix)
{
  if (candidate.size() <= prefix.size() + 1U || candidate.rfind(prefix + "_", 0) != 0U) {
    return false;
  }
  const std::string suffix = candidate.substr(prefix.size() + 1U);
  if (suffix.size() != 16U || suffix.at(8) != 'T' || suffix.back() != 'Z') {
    return false;
  }
  for (std::size_t index = 0; index < suffix.size(); ++index) {
    if (index == 8U || index == suffix.size() - 1U) {
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(suffix.at(index)))) {
      return false;
    }
  }
  return true;
}

nlohmann::json loadJsonDocument(const std::filesystem::path & path)
{
  constexpr int kMaxAttempts = 3;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    std::ifstream input_stream(path);
    if (!input_stream.is_open()) {
      throw std::runtime_error("Failed to open mission file: " + path.string());
    }

    try {
      nlohmann::json document;
      input_stream >> document;
      return document;
    } catch (const nlohmann::json::parse_error &) {
      if (attempt >= kMaxAttempts) {
        throw;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  throw std::runtime_error("Failed to parse JSON document: " + path.string());
}

void writeJsonDocumentAtomic(
  const std::filesystem::path & path,
  const nlohmann::json & document)
{
  std::filesystem::create_directories(path.parent_path());
  const std::filesystem::path temp_path = path.string() + ".tmp";

  {
    std::ofstream output_stream(temp_path, std::ios::trunc);
    if (!output_stream.is_open()) {
      throw std::runtime_error("Failed to write JSON file: " + temp_path.string());
    }
    output_stream << std::setw(2) << document << '\n';
    output_stream.flush();
    if (!output_stream.good()) {
      throw std::runtime_error("Failed to flush JSON file: " + temp_path.string());
    }
  }

  std::filesystem::rename(temp_path, path);
}

std::string defaultIfEmpty(const std::string & value, const std::string & fallback)
{
  return value.empty() ? fallback : value;
}

std::string trimCopy(std::string value)
{
  const auto not_space = [](const unsigned char character) {
      return !std::isspace(character);
    };
  value.erase(
    value.begin(),
    std::find_if(value.begin(), value.end(), not_space));
  value.erase(
    std::find_if(value.rbegin(), value.rend(), not_space).base(),
    value.end());
  return value;
}

std::string stripQuotes(const std::string & value)
{
  if (value.size() >= 2U) {
    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return value.substr(1U, value.size() - 2U);
    }
  }
  return value;
}

void copyFileIfExists(
  const std::filesystem::path & source_path,
  const std::filesystem::path & destination_path)
{
  if (source_path.empty() || !std::filesystem::exists(source_path)) {
    return;
  }

  std::filesystem::create_directories(destination_path.parent_path());
  std::filesystem::copy_file(
    source_path,
    destination_path,
    std::filesystem::copy_options::overwrite_existing);
}

std::filesystem::path writeRosbagRuntimeQosOverridesFile(
  const std::filesystem::path & directory)
{
  std::filesystem::create_directories(directory);
  const std::filesystem::path overrides_path = directory / "rosbag_runtime_qos_overrides.yaml";
  std::ofstream output_stream(overrides_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    throw std::runtime_error(
            "Failed to write rosbag QoS overrides file: " + overrides_path.string());
  }

  output_stream
    << kDepthCameraScanTopic << ":\n"
    << "  reliability: best_effort\n"
    << "  history: keep_last\n"
    << "  depth: 5\n";
  for (const char * topic : {
      kSimulationPoseInfoTopic,
      kImuDataRawTopic,
      kImuDataAccGyroTopic,
      kImuDataHeadingTopic,
      kImuAzimuthTopic,
      kDepthCameraInfoTopic,
      kDepthCameraMotionSampleTopic,
    })
  {
    output_stream
      << topic << ":\n"
      << "  reliability: best_effort\n"
      << "  history: keep_last\n"
      << "  depth: 10\n";
  }
  output_stream.flush();
  if (!output_stream.good()) {
    throw std::runtime_error(
            "Failed to flush rosbag QoS overrides file: " + overrides_path.string());
  }

  return overrides_path;
}

std::string sanitizeUidToken(std::string value);

double clampToUnitInterval(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double computePathLengthMeters(const nlohmann::json & route_document)
{
  double total_length_meters = 0.0;
  if (!route_document.contains("features") || !route_document.at("features").is_array()) {
    return total_length_meters;
  }

  for (const auto & feature : route_document.at("features")) {
    if (!feature.contains("geometry") || !feature.at("geometry").is_object()) {
      continue;
    }
    const auto & geometry = feature.at("geometry");
    if (!geometry.contains("type") || geometry.at("type") != "LineString" ||
      !geometry.contains("coordinates") || !geometry.at("coordinates").is_array())
    {
      continue;
    }

    const auto & coordinates = geometry.at("coordinates");
    for (std::size_t index = 1; index < coordinates.size(); ++index) {
      const auto & previous = coordinates.at(index - 1U);
      const auto & current = coordinates.at(index);
      if (!previous.is_array() || !current.is_array() || previous.size() < 2U || current.size() < 2U) {
        continue;
      }
      const double dx = current.at(0).get<double>() - previous.at(0).get<double>();
      const double dy = current.at(1).get<double>() - previous.at(1).get<double>();
      total_length_meters += std::hypot(dx, dy);
    }
  }

  return total_length_meters;
}

std::vector<MapPoint> extractLineStringCoordinates(const nlohmann::json & route_document)
{
  std::vector<MapPoint> coordinates;
  if (!route_document.contains("features") || !route_document.at("features").is_array()) {
    return coordinates;
  }

  for (const auto & feature : route_document.at("features")) {
    if (!feature.contains("geometry") || !feature.at("geometry").is_object()) {
      continue;
    }
    const auto & geometry = feature.at("geometry");
    if (!geometry.contains("type") || geometry.at("type") != "LineString" ||
      !geometry.contains("coordinates") || !geometry.at("coordinates").is_array())
    {
      continue;
    }

    for (const auto & coordinate : geometry.at("coordinates")) {
      if (!coordinate.is_array() || coordinate.size() < 2U) {
        continue;
      }
      coordinates.push_back({coordinate.at(0).get<double>(), coordinate.at(1).get<double>()});
    }
    if (!coordinates.empty()) {
      return coordinates;
    }
  }

  return coordinates;
}

std::vector<GeoPoint> extractGeoLineStringCoordinates(const nlohmann::json & route_document)
{
  std::vector<GeoPoint> coordinates;
  if (!route_document.contains("features") || !route_document.at("features").is_array()) {
    return coordinates;
  }

  for (const auto & feature : route_document.at("features")) {
    if (!feature.contains("geometry") || !feature.at("geometry").is_object()) {
      continue;
    }
    const auto & geometry = feature.at("geometry");
    if (!geometry.contains("type") || geometry.at("type") != "LineString" ||
      !geometry.contains("coordinates") || !geometry.at("coordinates").is_array())
    {
      continue;
    }

    for (const auto & coordinate : geometry.at("coordinates")) {
      if (!coordinate.is_array() || coordinate.size() < 2U) {
        continue;
      }
      coordinates.push_back({coordinate.at(1).get<double>(), coordinate.at(0).get<double>()});
    }
    if (!coordinates.empty()) {
      return coordinates;
    }
  }

  return coordinates;
}

bool arePointsNear(const MapPoint & lhs, const MapPoint & rhs, const double tolerance = 1.0e-6)
{
  return std::abs(lhs.x - rhs.x) <= tolerance && std::abs(lhs.y - rhs.y) <= tolerance;
}

bool areGeoPointsNear(const GeoPoint & lhs, const GeoPoint & rhs, const double tolerance = 1.0e-9)
{
  return std::abs(lhs.latitude - rhs.latitude) <= tolerance &&
         std::abs(lhs.longitude - rhs.longitude) <= tolerance;
}

double distanceToSegment(const MapPoint & point, const MapPoint & start, const MapPoint & end)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double segment_length_squared = (dx * dx) + (dy * dy);
  if (segment_length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }

  const double projection =
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / segment_length_squared;
  const double clamped_projection = std::clamp(projection, 0.0, 1.0);
  const double closest_x = start.x + clamped_projection * dx;
  const double closest_y = start.y + clamped_projection * dy;
  return std::hypot(point.x - closest_x, point.y - closest_y);
}

bool pointInPolygon(const MapPoint & point, const std::vector<MapPoint> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    const auto & a = polygon.at(i);
    const auto & b = polygon.at(j);
    const bool intersects =
      ((a.y > point.y) != (b.y > point.y)) &&
      (point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + std::numeric_limits<double>::epsilon()) + a.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

double nearestPerimeterDistance(const MapPoint & point, const std::vector<MapPoint> & polygon)
{
  if (polygon.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < polygon.size(); ++index) {
    min_distance = std::min(min_distance, distanceToSegment(point, polygon.at(index), polygon.at(index + 1U)));
  }
  return min_distance;
}

std::vector<MapPoint> closePolygon(std::vector<MapPoint> polygon)
{
  if (polygon.size() >= 3U && !arePointsNear(polygon.front(), polygon.back())) {
    polygon.push_back(polygon.front());
  }
  return polygon;
}

std::vector<MapPoint> uniquePolygonVertices(std::vector<MapPoint> polygon)
{
  if (polygon.size() >= 2U && arePointsNear(polygon.front(), polygon.back())) {
    polygon.pop_back();
  }
  return polygon;
}

std::vector<GeoPoint> uniqueGeoPolygonVertices(std::vector<GeoPoint> polygon)
{
  if (polygon.size() >= 2U && areGeoPointsNear(polygon.front(), polygon.back())) {
    polygon.pop_back();
  }
  return polygon;
}

std::vector<MapPoint> loadGaussianObstaclePoints(const std::filesystem::path & gaussian_json_path)
{
  std::vector<MapPoint> points;
  if (!std::filesystem::exists(gaussian_json_path)) {
    return points;
  }

  const auto document = loadJsonDocument(gaussian_json_path);
  if (!document.contains("gaussians") || !document.at("gaussians").is_array()) {
    return points;
  }

  for (const auto & gaussian : document.at("gaussians")) {
    if (!gaussian.contains("position") || !gaussian.at("position").is_array() || gaussian.at("position").size() < 2U) {
      continue;
    }
    points.push_back({
      gaussian.at("position").at(0).get<double>(),
      gaussian.at("position").at(1).get<double>()});
  }
  return points;
}

void saveCostmapArtifacts(
  const RasterizedCostmap & map,
  const std::filesystem::path & image_path,
  const std::filesystem::path & yaml_path)
{
  namespace fs = std::filesystem;
  fs::create_directories(image_path.parent_path());
  fs::create_directories(yaml_path.parent_path());

  std::ofstream image_stream(image_path, std::ios::binary);
  if (!image_stream.is_open()) {
    throw std::runtime_error("Failed to write RecordMap costmap image: " + image_path.string());
  }
  image_stream << "P5\n" << map.width_cells << " " << map.height_cells << "\n255\n";
  for (int row = static_cast<int>(map.height_cells) - 1; row >= 0; --row) {
    for (unsigned int col = 0; col < map.width_cells; ++col) {
      const std::size_t index = static_cast<std::size_t>(row) * map.width_cells + col;
      const unsigned char pixel = static_cast<unsigned char>(255U - map.costs.at(index));
      image_stream.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }

  std::ofstream yaml_stream(yaml_path, std::ios::trunc);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to write RecordMap costmap yaml: " + yaml_path.string());
  }
  yaml_stream << std::setprecision(std::numeric_limits<double>::max_digits10);
  yaml_stream
    << "image: " << image_path.filename().string() << "\n"
    << "resolution: " << map.resolution << "\n"
    << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n"
    << "mode: trinary\n";
  if (map.georeference_valid) {
    yaml_stream << "georeference_type: " <<
      defaultIfEmpty(map.georeference_type, "affine_xy_to_wgs84") << "\n";
    yaml_stream << "georeference_source_crs: " <<
      defaultIfEmpty(map.georeference_source_crs, "EPSG:4326") << "\n";
    if (!map.georeference_companion_file.empty()) {
      yaml_stream << "georeference_companion_file: " << map.georeference_companion_file << "\n";
    }
    yaml_stream << "georeference_sample_count: " << map.georeference_sample_count << "\n";
    yaml_stream << "georeference_longitude_coefficients: [" <<
      map.longitude_coefficients[0] << ", " <<
      map.longitude_coefficients[1] << ", " <<
      map.longitude_coefficients[2] << "]\n";
    yaml_stream << "georeference_latitude_coefficients: [" <<
      map.latitude_coefficients[0] << ", " <<
      map.latitude_coefficients[1] << ", " <<
      map.latitude_coefficients[2] << "]\n";
  }
}

void rewriteCostmapYamlImageReference(
  const std::filesystem::path & yaml_path,
  const std::filesystem::path & image_path)
{
  std::ifstream input_stream(yaml_path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to reopen costmap yaml: " + yaml_path.string());
  }

  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string yaml_text = buffer.str();
  const auto image_key = yaml_text.find("image:");
  if (image_key == std::string::npos) {
    throw std::runtime_error("Costmap yaml does not contain an image key: " + yaml_path.string());
  }

  const auto line_end = yaml_text.find('\n', image_key);
  yaml_text.replace(
    image_key,
    (line_end == std::string::npos ? yaml_text.size() : line_end + 1U) - image_key,
    "image: " + image_path.filename().string() + "\n");

  std::ofstream output_stream(yaml_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    throw std::runtime_error("Failed to update costmap yaml: " + yaml_path.string());
  }
  output_stream << yaml_text;
}

RasterizedCostmap loadCostmapArtifacts(const std::filesystem::path & yaml_path)
{
  std::ifstream yaml_stream(yaml_path);
  if (!yaml_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap yaml: " + yaml_path.string());
  }

  std::string image_name;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  bool georeference_valid = false;
  std::string georeference_type;
  std::string georeference_source_crs = "EPSG:4326";
  std::string georeference_companion_file;
  std::size_t georeference_sample_count = 0U;
  std::array<double, 3> longitude_coefficients{0.0, 0.0, 0.0};
  std::array<double, 3> latitude_coefficients{0.0, 0.0, 0.0};
  bool negate = false;
  std::string line;
  while (std::getline(yaml_stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trimCopy(line.substr(0, colon));
    const std::string value = trimCopy(line.substr(colon + 1U));
    if (key == "image") {
      image_name = stripQuotes(value);
    } else if (key == "resolution") {
      resolution = std::stod(value);
    } else if (key == "origin") {
      const auto open = value.find('[');
      const auto comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', comma + 1U);
      origin_x = std::stod(trimCopy(value.substr(open + 1U, comma - open - 1U)));
      origin_y = std::stod(trimCopy(value.substr(comma + 1U, second_comma - comma - 1U)));
    } else if (key == "occupied_thresh") {
      occupied_thresh = std::stod(value);
    } else if (key == "free_thresh") {
      free_thresh = std::stod(value);
    } else if (key == "georeference_type") {
      georeference_type = stripQuotes(value);
      georeference_valid = !georeference_type.empty();
    } else if (key == "georeference_source_crs") {
      georeference_source_crs = stripQuotes(value);
    } else if (key == "georeference_companion_file") {
      georeference_companion_file = stripQuotes(value);
    } else if (key == "georeference_sample_count") {
      georeference_sample_count = static_cast<std::size_t>(std::stoul(value));
    } else if (key == "georeference_longitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      longitude_coefficients = {
        std::stod(trimCopy(value.substr(open + 1U, first_comma - open - 1U))),
        std::stod(trimCopy(value.substr(first_comma + 1U, second_comma - first_comma - 1U))),
        std::stod(trimCopy(value.substr(second_comma + 1U, close - second_comma - 1U)))};
      georeference_valid = true;
    } else if (key == "georeference_latitude_coefficients") {
      const auto open = value.find('[');
      const auto first_comma = value.find(',', open + 1U);
      const auto second_comma = value.find(',', first_comma + 1U);
      const auto close = value.find(']', second_comma + 1U);
      latitude_coefficients = {
        std::stod(trimCopy(value.substr(open + 1U, first_comma - open - 1U))),
        std::stod(trimCopy(value.substr(first_comma + 1U, second_comma - first_comma - 1U))),
        std::stod(trimCopy(value.substr(second_comma + 1U, close - second_comma - 1U)))};
      georeference_valid = true;
    } else if (key == "negate") {
      negate = std::stoi(value) != 0;
    }
  }

  const std::filesystem::path image_path = yaml_path.parent_path() / image_name;
  std::ifstream image_stream(image_path, std::ios::binary);
  if (!image_stream.is_open()) {
    throw std::runtime_error("Failed to open costmap image: " + image_path.string());
  }

  std::string magic;
  image_stream >> magic;
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("Unsupported costmap image format: " + magic);
  }

  unsigned int width = 0U;
  unsigned int height = 0U;
  int max_value = 0;
  image_stream >> width >> height >> max_value;

  RasterizedCostmap map;
  map.width_cells = width;
  map.height_cells = height;
  map.resolution = resolution;
  map.origin_x = origin_x;
  map.origin_y = origin_y;
  map.occupied_thresh = occupied_thresh;
  map.free_thresh = free_thresh;
  map.georeference_valid = georeference_valid;
  map.georeference_type = georeference_type;
  map.georeference_source_crs = georeference_source_crs;
  map.georeference_companion_file = georeference_companion_file;
  map.georeference_sample_count = georeference_sample_count;
  map.longitude_coefficients = longitude_coefficients;
  map.latitude_coefficients = latitude_coefficients;
  map.costs.resize(static_cast<std::size_t>(width) * height);

  auto to_cost = [max_value, negate](const int pixel_value) -> unsigned char {
    const int bounded = std::clamp(pixel_value, 0, std::max(1, max_value));
    const double normalized = static_cast<double>(bounded) / static_cast<double>(std::max(1, max_value));
    const double occupied = negate ? normalized : (1.0 - normalized);
    return static_cast<unsigned char>(std::lround(std::clamp(occupied, 0.0, 1.0) * 255.0));
  };

  if (magic == "P5") {
    image_stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
      for (unsigned int col = 0U; col < width; ++col) {
        unsigned char pixel = 0U;
        image_stream.read(reinterpret_cast<char *>(&pixel), 1);
        const std::size_t index = static_cast<std::size_t>(row) * width + col;
        map.costs.at(index) = to_cost(static_cast<int>(pixel));
      }
    }
    return map;
  }

  for (int row = static_cast<int>(height) - 1; row >= 0; --row) {
    for (unsigned int col = 0U; col < width; ++col) {
      int pixel_value = 0;
      image_stream >> pixel_value;
      const std::size_t index = static_cast<std::size_t>(row) * width + col;
      map.costs.at(index) = to_cost(pixel_value);
    }
  }

  return map;
}

bool compatibleCostmaps(const RasterizedCostmap & lhs, const RasterizedCostmap & rhs)
{
  constexpr double kTolerance = 1.0e-6;
  return lhs.width_cells == rhs.width_cells &&
         lhs.height_cells == rhs.height_cells &&
         std::abs(lhs.resolution - rhs.resolution) <= kTolerance &&
         std::abs(lhs.origin_x - rhs.origin_x) <= kTolerance &&
         std::abs(lhs.origin_y - rhs.origin_y) <= kTolerance;
}

RasterizedCostmap mergeCostmaps(
  const RasterizedCostmap & persistent,
  const RasterizedCostmap & runtime)
{
  RasterizedCostmap merged = persistent;
  if (!compatibleCostmaps(persistent, runtime)) {
    return runtime;
  }

  for (std::size_t index = 0U; index < merged.costs.size(); ++index) {
    const double persistent_ratio = static_cast<double>(persistent.costs.at(index)) / 255.0;
    const double runtime_ratio = static_cast<double>(runtime.costs.at(index)) / 255.0;
    const double merged_ratio = (persistent_ratio + runtime_ratio) * 0.5;
    merged.costs.at(index) = static_cast<unsigned char>(
      std::lround(std::clamp(merged_ratio, 0.0, 1.0) * 255.0));
  }
  merged.occupied_thresh = persistent.occupied_thresh;
  merged.free_thresh = persistent.free_thresh;
  merged.georeference_valid = persistent.georeference_valid || runtime.georeference_valid;
  if (persistent.georeference_valid) {
    merged.georeference_type = persistent.georeference_type;
    merged.georeference_source_crs = persistent.georeference_source_crs;
    merged.georeference_companion_file = persistent.georeference_companion_file;
    merged.georeference_sample_count = persistent.georeference_sample_count;
    merged.longitude_coefficients = persistent.longitude_coefficients;
    merged.latitude_coefficients = persistent.latitude_coefficients;
  } else if (runtime.georeference_valid) {
    merged.georeference_type = runtime.georeference_type;
    merged.georeference_source_crs = runtime.georeference_source_crs;
    merged.georeference_companion_file = runtime.georeference_companion_file;
    merged.georeference_sample_count = runtime.georeference_sample_count;
    merged.longitude_coefficients = runtime.longitude_coefficients;
    merged.latitude_coefficients = runtime.latitude_coefficients;
  }
  return merged;
}

std::optional<std::filesystem::path> findNewestGeoreferencedHistoricalCostmapYaml(
  const std::filesystem::path & mission_history_directory,
  const std::string & costmap_filename)
{
  namespace fs = std::filesystem;
  if (!fs::exists(mission_history_directory) || !fs::is_directory(mission_history_directory)) {
    return std::nullopt;
  }

  std::vector<fs::path> run_directories;
  for (const auto & entry : fs::directory_iterator(mission_history_directory)) {
    if (entry.is_directory()) {
      run_directories.push_back(entry.path());
    }
  }
  std::sort(run_directories.begin(), run_directories.end(), std::greater<fs::path>{});

  for (const auto & run_directory : run_directories) {
    const fs::path candidate_yaml = run_directory / costmap_filename;
    const fs::path candidate_image =
      candidate_yaml.parent_path() / (candidate_yaml.stem().string() + ".pgm");
    if (!fs::exists(candidate_yaml) || !fs::exists(candidate_image)) {
      continue;
    }

    try {
      const RasterizedCostmap candidate = loadCostmapArtifacts(candidate_yaml);
      if (candidate.georeference_valid) {
        return candidate_yaml;
      }
    } catch (const std::exception &) {
      continue;
    }
  }

  return std::nullopt;
}

nlohmann::json buildPerimeterGeoJson(const std::vector<MapPoint> & perimeter)
{
  nlohmann::json coordinates = nlohmann::json::array();
  for (const auto & point : perimeter) {
    coordinates.push_back({point.x, point.y});
  }

  return {
    {"type", "FeatureCollection"},
    {"features", nlohmann::json::array({
      {
        {"type", "Feature"},
        {"properties", {
           {"name", "recorded_work_area_perimeter"},
           {"source", "record_map"},
           {"coordinate_frame", "odom"}}},
        {"geometry", {
           {"type", "LineString"},
           {"coordinates", coordinates}}}
      }
    })}
  };
}

PolygonBounds computeBounds(const std::vector<MapPoint> & points)
{
  PolygonBounds bounds{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};
  for (const auto & point : points) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  return bounds;
}

std::vector<double> scanlineIntersections(const std::vector<MapPoint> & polygon, const double y)
{
  std::vector<double> intersections;
  if (polygon.size() < 2U) {
    return intersections;
  }

  for (std::size_t index = 0U; index + 1U < polygon.size(); ++index) {
    const auto & a = polygon.at(index);
    const auto & b = polygon.at(index + 1U);
    const bool crosses = ((a.y <= y) && (b.y > y)) || ((b.y <= y) && (a.y > y));
    if (!crosses) {
      continue;
    }
    const double t = (y - a.y) / (b.y - a.y);
    intersections.push_back(a.x + t * (b.x - a.x));
  }

  std::sort(intersections.begin(), intersections.end());
  return intersections;
}

void appendPointIfSeparated(std::vector<MapPoint> & route, const MapPoint & point, const double min_distance = 0.05)
{
  if (route.empty()) {
    route.push_back(point);
    return;
  }
  if (std::hypot(route.back().x - point.x, route.back().y - point.y) >= min_distance) {
    route.push_back(point);
  }
}

std::vector<MapPoint> buildZigzagSweepRoute(const std::vector<MapPoint> & perimeter)
{
  std::vector<MapPoint> route;
  const auto bounds = computeBounds(perimeter);
  bool reverse = false;
  for (double y = bounds.min_y + kSweepInsetMeters; y <= bounds.max_y - kSweepInsetMeters; y += kSweepTrackSpacingMeters) {
    const auto intersections = scanlineIntersections(perimeter, y);
    if (intersections.size() < 2U) {
      continue;
    }
    std::vector<MapPoint> row_points;
    for (std::size_t index = 0U; index + 1U < intersections.size(); index += 2U) {
      const double start_x = intersections.at(index) + kSweepInsetMeters;
      const double end_x = intersections.at(index + 1U) - kSweepInsetMeters;
      if (end_x <= start_x) {
        continue;
      }
      row_points.push_back({start_x, y});
      row_points.push_back({end_x, y});
    }
    if (row_points.empty()) {
      continue;
    }
    if (reverse) {
      std::reverse(row_points.begin(), row_points.end());
    }
    for (const auto & point : row_points) {
      appendPointIfSeparated(route, point);
    }
    reverse = !reverse;
  }
  return route;
}

std::vector<MapPoint> buildRandomSweepRoute(const std::vector<MapPoint> & perimeter)
{
  std::vector<MapPoint> candidates;
  const auto bounds = computeBounds(perimeter);
  for (double y = bounds.min_y + kSweepInsetMeters; y <= bounds.max_y - kSweepInsetMeters; y += kSweepTrackSpacingMeters) {
    const auto intersections = scanlineIntersections(perimeter, y);
    if (intersections.size() < 2U) {
      continue;
    }
    for (std::size_t index = 0U; index + 1U < intersections.size(); index += 2U) {
      const double start_x = intersections.at(index) + kSweepInsetMeters;
      const double end_x = intersections.at(index + 1U) - kSweepInsetMeters;
      if (end_x <= start_x) {
        continue;
      }
      candidates.push_back({start_x, y});
      if ((end_x - start_x) > (kSweepTrackSpacingMeters * 0.5)) {
        candidates.push_back({0.5 * (start_x + end_x), y});
      }
      candidates.push_back({end_x, y});
    }
  }

  if (candidates.size() < 2U) {
    return buildZigzagSweepRoute(perimeter);
  }

  std::mt19937 generator(42U);
  std::shuffle(candidates.begin(), candidates.end(), generator);

  std::vector<MapPoint> route;
  route.push_back(candidates.front());
  std::vector<bool> used(candidates.size(), false);
  used.front() = true;
  for (std::size_t step = 1U; step < candidates.size(); ++step) {
    std::size_t best_index = candidates.size();
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      if (used.at(index)) {
        continue;
      }
      const double distance = std::hypot(
        route.back().x - candidates.at(index).x,
        route.back().y - candidates.at(index).y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    if (best_index >= candidates.size()) {
      break;
    }
    used.at(best_index) = true;
    appendPointIfSeparated(route, candidates.at(best_index));
  }
  return route;
}

std::vector<MapPoint> buildSpiralSweepRoute(const std::vector<MapPoint> & perimeter)
{
  std::vector<MapPoint> route;
  const auto bounds = computeBounds(perimeter);
  double left = bounds.min_x + kSweepInsetMeters;
  double right = bounds.max_x - kSweepInsetMeters;
  double bottom = bounds.min_y + kSweepInsetMeters;
  double top = bounds.max_y - kSweepInsetMeters;

  while ((right - left) > 0.25 && (top - bottom) > 0.25) {
    const std::vector<MapPoint> candidate_points = {
      {left, bottom},
      {right, bottom},
      {right, top},
      {left, top}
    };
    for (const auto & point : candidate_points) {
      if (pointInPolygon(point, perimeter) || nearestPerimeterDistance(point, perimeter) <= kSweepInsetMeters) {
        appendPointIfSeparated(route, point);
      }
    }
    left += kSweepTrackSpacingMeters;
    right -= kSweepTrackSpacingMeters;
    bottom += kSweepTrackSpacingMeters;
    top -= kSweepTrackSpacingMeters;
  }

  if (route.size() < 2U) {
    return buildZigzagSweepRoute(perimeter);
  }
  return route;
}

std::vector<MapPoint> buildSweepRouteForPattern(
  const std::vector<MapPoint> & perimeter,
  const std::string & requested_pattern,
  std::string & applied_pattern)
{
  const std::string normalized_pattern = toLower(requested_pattern);
  if (normalized_pattern == kRandomSweepPattern) {
    applied_pattern = kRandomSweepPattern;
    return buildRandomSweepRoute(perimeter);
  }
  if (normalized_pattern == kSpiralSweepPattern) {
    applied_pattern = kSpiralSweepPattern;
    return buildSpiralSweepRoute(perimeter);
  }
  applied_pattern = kZigzagSweepPattern;
  return buildZigzagSweepRoute(perimeter);
}

std::vector<std::vector<MapPoint>> buildObstacleNoGoZones(const std::vector<MapPoint> & obstacle_points)
{
  std::vector<std::vector<MapPoint>> zones;
  constexpr double half_width_meters = 0.35;
  for (const auto & point : obstacle_points) {
    zones.push_back({
      {point.x - half_width_meters, point.y - half_width_meters},
      {point.x + half_width_meters, point.y - half_width_meters},
      {point.x + half_width_meters, point.y + half_width_meters},
      {point.x - half_width_meters, point.y + half_width_meters},
    });
  }
  return zones;
}

double computePolylineLength(const std::vector<MapPoint> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += std::hypot(points.at(index).x - points.at(index - 1U).x, points.at(index).y - points.at(index - 1U).y);
  }
  return length;
}

double computeGeoPolylineLength(const std::vector<GeoPoint> & points)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double dx = points.at(index).longitude - points.at(index - 1U).longitude;
    const double dy = points.at(index).latitude - points.at(index - 1U).latitude;
    length += std::hypot(dx, dy);
  }
  return length;
}

MapPoint interpolateAlongPolyline(const std::vector<MapPoint> & points, const double target_fraction)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1U) {
    return points.front();
  }
  const double total_length = computePolylineLength(points);
  if (total_length <= std::numeric_limits<double>::epsilon()) {
    return points.front();
  }
  const double target_distance = clampToUnitInterval(target_fraction) * total_length;
  double traversed = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const auto & previous = points.at(index - 1U);
    const auto & current = points.at(index);
    const double segment_length = std::hypot(current.x - previous.x, current.y - previous.y);
    if (traversed + segment_length >= target_distance) {
      const double local_fraction = (target_distance - traversed) / std::max(segment_length, std::numeric_limits<double>::epsilon());
      return {
        previous.x + local_fraction * (current.x - previous.x),
        previous.y + local_fraction * (current.y - previous.y)};
    }
    traversed += segment_length;
  }
  return points.back();
}

GeoPoint interpolateAlongGeoPolyline(const std::vector<GeoPoint> & points, const double target_fraction)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1U) {
    return points.front();
  }
  const double total_length = computeGeoPolylineLength(points);
  if (total_length <= std::numeric_limits<double>::epsilon()) {
    return points.front();
  }
  const double target_distance = clampToUnitInterval(target_fraction) * total_length;
  double traversed = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const auto & previous = points.at(index - 1U);
    const auto & current = points.at(index);
    const double segment_length = std::hypot(
      current.longitude - previous.longitude,
      current.latitude - previous.latitude);
    if (traversed + segment_length >= target_distance) {
      const double local_fraction = (target_distance - traversed) / std::max(segment_length, std::numeric_limits<double>::epsilon());
      return {
        previous.latitude + local_fraction * (current.latitude - previous.latitude),
        previous.longitude + local_fraction * (current.longitude - previous.longitude)};
    }
    traversed += segment_length;
  }
  return points.back();
}

bool solveLinear3x3(double matrix[3][4], double solution[3])
{
  for (int pivot = 0; pivot < 3; ++pivot) {
    int best_row = pivot;
    for (int row = pivot + 1; row < 3; ++row) {
      if (std::abs(matrix[row][pivot]) > std::abs(matrix[best_row][pivot])) {
        best_row = row;
      }
    }
    if (std::abs(matrix[best_row][pivot]) <= 1.0e-12) {
      return false;
    }
    if (best_row != pivot) {
      for (int column = pivot; column < 4; ++column) {
        std::swap(matrix[pivot][column], matrix[best_row][column]);
      }
    }
    const double pivot_value = matrix[pivot][pivot];
    for (int column = pivot; column < 4; ++column) {
      matrix[pivot][column] /= pivot_value;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix[row][pivot];
      for (int column = pivot; column < 4; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
    }
  }
  for (int row = 0; row < 3; ++row) {
    solution[row] = matrix[row][3];
  }
  return true;
}

bool fitAffineComponent(
  const std::vector<MapPoint> & local_points,
  const std::vector<double> & targets,
  double coefficients[3])
{
  if (local_points.size() != targets.size() || local_points.size() < 3U) {
    return false;
  }
  double ata[3][3] = {};
  double atb[3] = {};
  for (std::size_t index = 0U; index < local_points.size(); ++index) {
    const double row[3] = {local_points.at(index).x, local_points.at(index).y, 1.0};
    for (int i = 0; i < 3; ++i) {
      atb[i] += row[i] * targets.at(index);
      for (int j = 0; j < 3; ++j) {
        ata[i][j] += row[i] * row[j];
      }
    }
  }
  double augmented[3][4] = {
    {ata[0][0], ata[0][1], ata[0][2], atb[0]},
    {ata[1][0], ata[1][1], ata[1][2], atb[1]},
    {ata[2][0], ata[2][1], ata[2][2], atb[2]},
  };
  return solveLinear3x3(augmented, coefficients);
}

GeoTransform buildGeoTransform(
  const std::vector<MapPoint> & local_trace,
  const std::vector<GeoPoint> & geo_trace)
{
  GeoTransform transform;
  if (local_trace.size() < 3U || geo_trace.size() < 3U) {
    return transform;
  }

  const std::size_t sample_count = std::max<std::size_t>(3U, std::min<std::size_t>(12U, std::min(local_trace.size(), geo_trace.size())));
  std::vector<MapPoint> sampled_local_points;
  std::vector<double> sampled_longitudes;
  std::vector<double> sampled_latitudes;
  sampled_local_points.reserve(sample_count);
  sampled_longitudes.reserve(sample_count);
  sampled_latitudes.reserve(sample_count);

  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double fraction = sample_count == 1U ? 0.0 : static_cast<double>(index) / static_cast<double>(sample_count - 1U);
    const MapPoint local_point = interpolateAlongPolyline(local_trace, fraction);
    const GeoPoint geo_point = interpolateAlongGeoPolyline(geo_trace, fraction);
    sampled_local_points.push_back(local_point);
    sampled_longitudes.push_back(geo_point.longitude);
    sampled_latitudes.push_back(geo_point.latitude);
  }

  if (!fitAffineComponent(sampled_local_points, sampled_longitudes, transform.longitude_coefficients) ||
    !fitAffineComponent(sampled_local_points, sampled_latitudes, transform.latitude_coefficients))
  {
    return transform;
  }

  transform.valid = true;
  return transform;
}

GeoPoint applyGeoTransform(const GeoTransform & transform, const MapPoint & local_point)
{
  return {
    transform.latitude_coefficients[0] * local_point.x +
      transform.latitude_coefficients[1] * local_point.y +
      transform.latitude_coefficients[2],
    transform.longitude_coefficients[0] * local_point.x +
      transform.longitude_coefficients[1] * local_point.y +
      transform.longitude_coefficients[2]};
}

std::vector<GeoPoint> convertToGeoPoints(const std::vector<MapPoint> & points, const GeoTransform & transform)
{
  std::vector<GeoPoint> converted;
  converted.reserve(points.size());
  for (const auto & point : points) {
    converted.push_back(applyGeoTransform(transform, point));
  }
  return converted;
}

std::vector<std::vector<GeoPoint>> convertZonesToGeo(
  const std::vector<std::vector<MapPoint>> & zones,
  const GeoTransform & transform)
{
  std::vector<std::vector<GeoPoint>> converted;
  converted.reserve(zones.size());
  for (const auto & zone : zones) {
    converted.push_back(convertToGeoPoints(zone, transform));
  }
  return converted;
}

nlohmann::json buildNavSatGeoJson(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & name,
  const std::string & local_companion_file = std::string{})
{
  nlohmann::json coordinates = nlohmann::json::array();
  for (const auto & point : points) {
    coordinates.push_back({point.x, point.y});
  }

  nlohmann::json properties{
    {"name", name},
    {"coordinate_frame", "wgs84"}};
  if (!local_companion_file.empty()) {
    properties["local_companion_file"] = local_companion_file;
  }

  return {
    {"type", "FeatureCollection"},
    {"features", nlohmann::json::array({
      {
        {"type", "Feature"},
        {"properties", properties},
        {"geometry", {
           {"type", "LineString"},
           {"coordinates", coordinates}}}
      }
    })}
  };
}

nlohmann::json buildLocalPathGeoJson(
  const nlohmann::json & coordinates,
  const std::string & name,
  const std::string & geographic_companion_file = std::string{},
  const std::optional<nlohmann::json> & georeference = std::nullopt)
{
  nlohmann::json properties{
    {"name", name},
    {"coordinate_frame", "odom"}};
  if (!geographic_companion_file.empty()) {
    properties["geographic_companion_file"] = geographic_companion_file;
  }
  if (georeference.has_value()) {
    properties["georeference"] = *georeference;
  }

  return {
    {"type", "FeatureCollection"},
    {"features", nlohmann::json::array({
      {
        {"type", "Feature"},
        {"properties", properties},
        {"geometry", {{"type", "LineString"}, {"coordinates", coordinates}}}
      }
    })}
  };
}

std::optional<nlohmann::json> buildGeoReferenceMetadata(
  const std::vector<MapPoint> & local_trace,
  const std::vector<GeoPoint> & geo_trace,
  const std::string & companion_file = std::string{})
{
  const GeoTransform transform = buildGeoTransform(local_trace, geo_trace);
  if (!transform.valid) {
    return std::nullopt;
  }

  nlohmann::json georeference{
    {"type", "affine_xy_to_wgs84"},
    {"sample_count", std::min(local_trace.size(), geo_trace.size())},
    {"longitude_coefficients", {
       transform.longitude_coefficients[0],
       transform.longitude_coefficients[1],
       transform.longitude_coefficients[2]}},
    {"latitude_coefficients", {
       transform.latitude_coefficients[0],
       transform.latitude_coefficients[1],
       transform.latitude_coefficients[2]}}};
  if (!companion_file.empty()) {
    georeference["companion_file"] = companion_file;
  }
  return georeference;
}

void refreshLocalPathGeoReference(
  const std::filesystem::path & local_path_file,
  const std::filesystem::path & geographic_companion_path,
  const std::vector<GeoPoint> & geo_trace)
{
  if (local_path_file.empty() || geographic_companion_path.empty() || geo_trace.size() < 3U) {
    return;
  }
  if (!std::filesystem::exists(local_path_file)) {
    return;
  }

  const std::vector<MapPoint> local_trace = extractLineStringCoordinates(loadJsonDocument(local_path_file));
  if (local_trace.size() < 3U) {
    return;
  }

  nlohmann::json coordinates = nlohmann::json::array();
  for (const auto & point : local_trace) {
    coordinates.push_back({point.x, point.y});
  }

  const auto georeference = buildGeoReferenceMetadata(
    local_trace,
    geo_trace,
    geographic_companion_path.filename().string());
  if (!georeference.has_value()) {
    return;
  }

  const nlohmann::json document = buildLocalPathGeoJson(
    coordinates,
    "actual_path",
    geographic_companion_path.filename().string(),
    georeference);

  try {
    writeJsonDocumentAtomic(local_path_file, document);
  } catch (const std::exception &) {
    return;
  }
}

void refreshLocalPathGeoReferenceFromArtifacts(const nlohmann::json & context_document)
{
  const std::filesystem::path local_path_file(
    context_document.value("actual_path_file", std::string{}));
  const std::filesystem::path geographic_companion_path(
    context_document.value("actual_path_navsat_file", std::string{}));
  if (local_path_file.empty() || geographic_companion_path.empty() ||
    !std::filesystem::exists(local_path_file) || !std::filesystem::exists(geographic_companion_path))
  {
    return;
  }

  const std::vector<GeoPoint> geo_trace =
    extractGeoLineStringCoordinates(loadJsonDocument(geographic_companion_path));
  refreshLocalPathGeoReference(local_path_file, geographic_companion_path, geo_trace);
}

nlohmann::json buildGeoReferencedVda5050MissionDocument(
  const std::string & order_id,
  const std::string & timestamp,
  const std::vector<GeoPoint> & perimeter,
  const std::vector<GeoPoint> & coverage_route,
  const std::vector<std::vector<GeoPoint>> & no_go_zones,
  const std::string & applied_pattern,
  const std::string & source_recorded_map_id,
  const std::string & source_recorded_map_run_started_at)
{
  nlohmann::json nodes = nlohmann::json::array();
  nlohmann::json edges = nlohmann::json::array();
  nlohmann::json working_zone_edge_ids = nlohmann::json::array();
  nlohmann::json coverage_edge_ids = nlohmann::json::array();
  nlohmann::json no_go_zone_documents = nlohmann::json::array();

  auto append_node = [&nodes](const std::string & node_id, const GeoPoint & point, const double theta) {
    nodes.push_back({
      {"nodeId", node_id},
      {"sequenceId", static_cast<int>(nodes.size()) * 2},
      {"released", true},
      {"nodePosition", {
         {"x", point.longitude},
         {"y", point.latitude},
         {"theta", theta},
         {"mapId", "mission_wgs84"}}}
    });
  };
  auto append_edge = [&edges](
    const std::string & edge_id,
    const std::string & start_node_id,
    const std::string & end_node_id,
    const std::string & edge_type) {
    edges.push_back({
      {"edgeId", edge_id},
      {"sequenceId", static_cast<int>(edges.size()) * 2 + 1},
      {"released", true},
      {"startNodeId", start_node_id},
      {"endNodeId", end_node_id},
      {"edgeType", edge_type}});
  };

  for (std::size_t index = 0U; index < perimeter.size(); ++index) {
    const std::string node_id = "wz_node_" + std::to_string(index);
    const GeoPoint & point = perimeter.at(index);
    const GeoPoint & next = perimeter.at((index + 1U) % perimeter.size());
    append_node(node_id, point, std::atan2(next.latitude - point.latitude, next.longitude - point.longitude));
  }
  for (std::size_t index = 0U; index < perimeter.size(); ++index) {
    const std::string edge_id = "wz_edge_" + std::to_string(index);
    append_edge(
      edge_id,
      "wz_node_" + std::to_string(index),
      "wz_node_" + std::to_string((index + 1U) % perimeter.size()),
      "boundary");
    working_zone_edge_ids.push_back(edge_id);
  }

  for (std::size_t index = 0U; index < coverage_route.size(); ++index) {
    const std::string node_id = "cp_node_" + std::to_string(index);
    const GeoPoint & point = coverage_route.at(index);
    double theta = 0.0;
    if (index + 1U < coverage_route.size()) {
      const auto & next = coverage_route.at(index + 1U);
      theta = std::atan2(next.latitude - point.latitude, next.longitude - point.longitude);
    } else if (index > 0U) {
      const auto & previous = coverage_route.at(index - 1U);
      theta = std::atan2(point.latitude - previous.latitude, point.longitude - previous.longitude);
    }
    append_node(node_id, point, theta);
  }
  for (std::size_t index = 0U; index + 1U < coverage_route.size(); ++index) {
    const std::string edge_id = "cp_edge_" + std::to_string(index);
    append_edge(
      edge_id,
      "cp_node_" + std::to_string(index),
      "cp_node_" + std::to_string(index + 1U),
      "coverage_path");
    coverage_edge_ids.push_back(edge_id);
  }

  for (std::size_t zone_index = 0U; zone_index < no_go_zones.size(); ++zone_index) {
    const auto & zone = no_go_zones.at(zone_index);
    if (zone.size() < 4U) {
      continue;
    }
    nlohmann::json zone_edge_ids = nlohmann::json::array();
    for (std::size_t point_index = 0U; point_index < zone.size(); ++point_index) {
      const std::string node_id =
        "ngz_" + std::to_string(zone_index) + "_node_" + std::to_string(point_index);
      const GeoPoint & point = zone.at(point_index);
      const GeoPoint & next = zone.at((point_index + 1U) % zone.size());
      append_node(node_id, point, std::atan2(next.latitude - point.latitude, next.longitude - point.longitude));
    }
    for (std::size_t point_index = 0U; point_index < zone.size(); ++point_index) {
      const std::string edge_id =
        "ngz_" + std::to_string(zone_index) + "_edge_" + std::to_string(point_index);
      append_edge(
        edge_id,
        "ngz_" + std::to_string(zone_index) + "_node_" + std::to_string(point_index),
        "ngz_" + std::to_string(zone_index) + "_node_" + std::to_string((point_index + 1U) % zone.size()),
        "no_go_zone");
      zone_edge_ids.push_back(edge_id);
    }
    no_go_zone_documents.push_back({
      {"zoneId", "recorded_obstacle_" + std::to_string(zone_index)},
      {"zoneType", "no_go"},
      {"edgeIds", zone_edge_ids},
    });
  }

  return {
    {"orderId", order_id},
    {"timestamp", timestamp},
    {"version", "2.0.0"},
    {"manufacturer", "amr_sweeper"},
    {"serialNumber", "portable_recorded_mission"},
    {"orderUpdateId", 0},
    {"description", "Autonomous mission generated from RecordMap perimeter and embedded obstacle zones."},
    {"missionReference", {
       {"missionId", order_id + "_" + sanitizeUidToken(timestamp)},
       {"mapId", "mission_wgs84"},
       {"coordinateReferenceSystem", "EPSG:4326"},
       {"xIsLongitude", true},
       {"yIsLatitude", true}}},
    {"missionMetadata", {
       {"sourceRecordedMapId", source_recorded_map_id},
       {"sourceRecordedMapRunStartedAt", source_recorded_map_run_started_at},
       {"selectedSweepPattern", applied_pattern},
       {"embeddedNoGoZoneCount", no_go_zone_documents.size()},
       {"portableMission", true}}},
    {"nodes", nodes},
    {"edges", edges},
    {"missionGeometries", {
       {"workingZones", nlohmann::json::array({
         {
           {"zoneId", "recorded_work_area"},
           {"zoneType", "working_zone"},
           {"edgeIds", working_zone_edge_ids}
         }
       })},
       {"noGoZones", no_go_zone_documents},
       {"coveragePathEdgeIds", coverage_edge_ids}}}
  };
}

RasterizedCostmap buildRecordMapCostmap(
  const std::vector<MapPoint> & perimeter,
  const std::vector<MapPoint> & obstacles)
{
  if (perimeter.size() < 4U) {
    throw std::runtime_error("RecordMap perimeter is too small to build a working-area costmap");
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : perimeter) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  min_x -= kRecordMapCostmapPaddingMeters;
  min_y -= kRecordMapCostmapPaddingMeters;
  max_x += kRecordMapCostmapPaddingMeters;
  max_y += kRecordMapCostmapPaddingMeters;

  const unsigned int width_cells = std::max(
    1U,
    static_cast<unsigned int>(std::ceil((max_x - min_x) / kRecordMapCostmapResolutionMeters)));
  const unsigned int height_cells = std::max(
    1U,
    static_cast<unsigned int>(std::ceil((max_y - min_y) / kRecordMapCostmapResolutionMeters)));

  RasterizedCostmap map;
  map.width_cells = width_cells;
  map.height_cells = height_cells;
  map.resolution = kRecordMapCostmapResolutionMeters;
  map.origin_x = min_x;
  map.origin_y = min_y;
  map.costs.assign(static_cast<std::size_t>(width_cells) * height_cells, kRecordMapOutsideCost);

  for (unsigned int row = 0U; row < height_cells; ++row) {
    for (unsigned int col = 0U; col < width_cells; ++col) {
      const MapPoint point{
        min_x + (static_cast<double>(col) + 0.5) * map.resolution,
        min_y + (static_cast<double>(row) + 0.5) * map.resolution};

      const bool inside = pointInPolygon(point, perimeter);
      const double edge_distance = nearestPerimeterDistance(point, perimeter);
      unsigned char cost = kRecordMapOutsideCost;
      if (inside) {
        cost = edge_distance <= kRecordMapEdgeBandMeters ? kRecordMapEdgeBandCost : kRecordMapInsideCost;
      } else if (edge_distance <= kRecordMapEdgeBandMeters) {
        cost = kRecordMapEdgeBandCost;
      }

      map.costs.at(static_cast<std::size_t>(row) * width_cells + col) = cost;
    }
  }

  const int obstacle_radius_cells = std::max(
    1,
    static_cast<int>(std::ceil(kRecordMapObstacleRadiusMeters / map.resolution)));
  for (const auto & obstacle : obstacles) {
    const int center_x = static_cast<int>(std::floor((obstacle.x - min_x) / map.resolution));
    const int center_y = static_cast<int>(std::floor((obstacle.y - min_y) / map.resolution));
    for (int dy = -obstacle_radius_cells; dy <= obstacle_radius_cells; ++dy) {
      for (int dx = -obstacle_radius_cells; dx <= obstacle_radius_cells; ++dx) {
        const int grid_x = center_x + dx;
        const int grid_y = center_y + dy;
        if (grid_x < 0 || grid_y < 0 ||
          grid_x >= static_cast<int>(width_cells) ||
          grid_y >= static_cast<int>(height_cells))
        {
          continue;
        }

        const double offset_x = static_cast<double>(dx) * map.resolution;
        const double offset_y = static_cast<double>(dy) * map.resolution;
        if (std::hypot(offset_x, offset_y) > kRecordMapObstacleRadiusMeters) {
          continue;
        }

        const std::size_t index =
          static_cast<std::size_t>(grid_y) * width_cells + static_cast<std::size_t>(grid_x);
        map.costs.at(index) = kRecordMapObstacleCost;
      }
    }
  }

  return map;
}

std::chrono::system_clock::time_point parseUtcTimestamp(const std::string & value)
{
  std::tm time_info{};
  std::istringstream stream(value);
  stream >> std::get_time(&time_info, "%Y%m%dT%H%M%SZ");
  if (stream.fail()) {
    throw std::runtime_error("Failed to parse UTC timestamp: " + value);
  }
#if defined(_WIN32)
  const std::time_t as_time_t = _mkgmtime(&time_info);
#else
  const std::time_t as_time_t = timegm(&time_info);
#endif
  return std::chrono::system_clock::from_time_t(as_time_t);
}

std::string localTimestampToUtcTimestamp(const std::string & value)
{
  std::tm time_info{};
  std::istringstream stream(value);
  stream >> std::get_time(&time_info, "%Y%m%dT%H%M%S");
  if (stream.fail()) {
    throw std::runtime_error("Failed to parse local timestamp: " + value);
  }
  time_info.tm_isdst = -1;
  const std::time_t as_time_t = std::mktime(&time_info);
  std::tm utc_time_info{};
#if defined(_WIN32)
  gmtime_s(&utc_time_info, &as_time_t);
#else
  gmtime_r(&as_time_t, &utc_time_info);
#endif
  std::ostringstream output_stream;
  output_stream << std::put_time(&utc_time_info, "%Y%m%dT%H%M%SZ");
  return output_stream.str();
}

std::string sanitizeUidToken(std::string value)
{
  std::replace_if(
    value.begin(),
    value.end(),
    [](const char character) {
      return !(std::isalnum(static_cast<unsigned char>(character)) || character == '-' || character == '_');
    },
    '_');
  return value;
}

std::string escapeIcsText(std::string value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case ';':
        escaped += "\\;";
        break;
      case ',':
        escaped += "\\,";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        break;
      default:
        escaped.push_back(character);
        break;
    }
  }
  return escaped;
}

std::filesystem::path discoverNewestSchedulePath(const std::filesystem::path & missions_directory)
{
  if (!std::filesystem::exists(missions_directory) || !std::filesystem::is_directory(missions_directory)) {
    return {};
  }

  std::vector<std::filesystem::directory_entry> schedule_entries;
  for (const auto & entry : std::filesystem::directory_iterator(missions_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto filename = entry.path().filename().string();
    if (filename.rfind("schedule_", 0) != 0 || entry.path().extension() != ".ics") {
      continue;
    }
    schedule_entries.push_back(entry);
  }

  if (schedule_entries.empty()) {
    return {};
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
    std::filesystem::path archived_path = archive_directory / source_path.filename();
    int suffix = 1;
    while (std::filesystem::exists(archived_path)) {
      archived_path = archive_directory /
        (source_path.stem().string() + "_" + std::to_string(suffix) + source_path.extension().string());
      ++suffix;
    }
    std::filesystem::rename(source_path, archived_path);
  }

  return newest_path;
}

std::string discoverScheduleTimezone(const std::string & schedule_text)
{
  {
    constexpr char calendar_timezone_prefix[] = "X-WR-TIMEZONE:";
    const auto position = schedule_text.find(calendar_timezone_prefix);
    if (position != std::string::npos) {
      const auto start = position + std::char_traits<char>::length(calendar_timezone_prefix);
      const auto end = schedule_text.find('\n', start);
      return schedule_text.substr(start, end - start);
    }
  }

  constexpr char dtstart_timezone_prefix[] = "DTSTART;TZID=";
  const auto dtstart_position = schedule_text.find(dtstart_timezone_prefix);
  if (dtstart_position != std::string::npos) {
    const auto start = dtstart_position + std::char_traits<char>::length(dtstart_timezone_prefix);
    const auto end = schedule_text.find(':', start);
    if (end != std::string::npos) {
      return schedule_text.substr(start, end - start);
    }
  }

  return "UTC";
}

}  // namespace

MissionExecutorNode::MissionExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_executor_node", options)
{
  missions_directory_ = declare_parameter<std::string>("missions_directory", "missions/database");
  missions_log_directory_ = declare_parameter<std::string>(
    "missions_log_directory",
    "missions/logs");
  actual_schedule_log_directory_ = declare_parameter<std::string>(
    "actual_schedule_log_directory",
    "missions/simulations");
  rosbag_directory_ = declare_parameter<std::string>("rosbag_directory", "missions/logs");
  manual_missions_directory_ = declare_parameter<std::string>("manual_missions_directory", "");
  mission_file_extension_ = declare_parameter<std::string>("mission_file_extension", ".json");
  schedule_ics_path_ = declare_parameter<std::string>("schedule_ics_path", "");
  robot_id_ = declare_parameter<std::string>("robot_id", "RBT-01");
  safety_stop_topic_ = declare_parameter<std::string>("safety_stop_topic", "safety_msgs/stop");
  teleop_odometry_topic_ = declare_parameter<std::string>("teleop_odometry_topic", "drive_controller/odom");
  manual_mapping_odometry_topic_ = declare_parameter<std::string>(
    "manual_mapping_odometry_topic",
    "localization/odometry_fused");
  manual_mapping_navsat_topic_ = declare_parameter<std::string>(
    "manual_mapping_navsat_topic",
    "gnss/navsat");
  routed_mission_odometry_topic_ = declare_parameter<std::string>(
    "routed_mission_odometry_topic",
    "localization/odometry_fused");
  rosbag_topics_file_ = declare_parameter<std::string>(
    "rosbag_topics_file",
    "");
  record_mission_rosbag_ = declare_parameter<bool>("record_mission_rosbag", false);
  mission_parser_node_name_ = declare_parameter<std::string>(
    "mission_parser_node_name",
    "vda5050_parser_node");
  mission_parser_build_service_ = declare_parameter<std::string>(
    "mission_parser_build_service",
    "build_current_mission");
  fsm_request_service_ = declare_parameter<std::string>("fsm_request_service", "request_state");
  use_simulation_ = declare_parameter<bool>("use_simulation", false);
  manual_mission_inactivity_timeout_seconds_ = declare_parameter<double>(
    "manual_mission_inactivity_timeout_seconds",
    300.0);
  manual_mission_min_linear_speed_mps_ = declare_parameter<double>(
    "manual_mission_min_linear_speed_mps",
    0.01);
  manual_mission_min_angular_speed_rps_ = declare_parameter<double>(
    "manual_mission_min_angular_speed_rps",
    0.01);
  teleop_path_sample_distance_m_ = declare_parameter<double>("teleop_path_sample_distance_m", 0.1);
  routed_mission_pose_max_age_seconds_ = declare_parameter<double>(
    "routed_mission_pose_max_age_seconds",
    2.0);
  idling_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("idling_profile_id", 101));
  scheduled_running_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("scheduled_running_profile_id", 201));
  manual_mapping_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_mapping_profile_id", 225));
  manual_routed_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_routed_profile_id", 210));
  manual_teleop_profile_id_ = static_cast<std::uint16_t>(
    declare_parameter<int>("manual_teleop_profile_id", 220));
  default_activation_priority_ = static_cast<std::uint8_t>(
    declare_parameter<int>("default_activation_priority", 200));
  promote_runtime_costmap_on_completed_mission_ = declare_parameter<bool>(
    "promote_runtime_costmap_on_completed_mission",
    true);

  client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mission_parser_parameter_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(
    this,
    mission_parser_node_name_,
    rmw_qos_profile_parameters,
    client_callback_group_);
  mission_parser_build_client_ = create_client<std_srvs::srv::Trigger>(
    mission_parser_build_service_,
    rclcpp::ServicesQoS(),
    client_callback_group_);
  fsm_request_client_ = create_client<amr_sweeper_fsm::srv::RequestState>(
    fsm_request_service_,
    rclcpp::ServicesQoS(),
    client_callback_group_);

  list_executable_missions_service_ = create_service<srv::ListExecutableMissions>(
    "list_executable_missions",
    std::bind(
      &MissionExecutorNode::handleListExecutableMissions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  list_manual_missions_service_ = create_service<srv::ListManualMissions>(
    "list_manual_missions",
    std::bind(
      &MissionExecutorNode::handleListManualMissions,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  upload_vda5050_mission_service_ = create_service<srv::UploadVda5050Mission>(
    "upload_vda5050_mission",
    std::bind(
      &MissionExecutorNode::handleUploadVda5050Mission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  create_recorded_mission_service_ = create_service<srv::CreateRecordedMission>(
    "create_recorded_mission",
    std::bind(
      &MissionExecutorNode::handleCreateRecordedMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  prepare_manual_mission_service_ = create_service<srv::PrepareManualMission>(
    "prepare_manual_mission",
    std::bind(
      &MissionExecutorNode::handlePrepareManualMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  execute_mission_service_ = create_service<srv::ExecuteMission>(
    "execute_mission",
    std::bind(
      &MissionExecutorNode::handleExecuteMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  end_mission_service_ = create_service<srv::EndMission>(
    "end_mission",
    std::bind(
      &MissionExecutorNode::handleEndMission,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  safety_stop_subscription_ = create_subscription<amr_sweeper_safety_msgs::msg::SafetyStop>(
    safety_stop_topic_,
    rclcpp::QoS(10).reliable().transient_local(),
    std::bind(&MissionExecutorNode::handleSafetyStop, this, std::placeholders::_1));
  teleop_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    teleop_odometry_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleManualMissionOdometry, this, std::placeholders::_1));
  manual_mapping_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    manual_mapping_odometry_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleManualMissionOdometry, this, std::placeholders::_1));
  manual_mapping_navsat_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    manual_mapping_navsat_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleManualMissionNavSat, this, std::placeholders::_1));
  routed_mission_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    routed_mission_odometry_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MissionExecutorNode::handleRoutedMissionOdometry, this, std::placeholders::_1));
  manual_mission_watchdog_timer_ = create_timer(
    std::chrono::seconds(5),
    std::bind(&MissionExecutorNode::checkManualMissionInactivity, this));

  RCLCPP_INFO(
    get_logger(),
    "Mission executor watching %s and manual templates %s with profile mapping scheduled=%u manual_mapping=%u routed=%u teleop=%u teleop odometry %s manual mapping odometry %s routed mission odometry %s and navsat %s",
    missions_directory_.c_str(),
    resolveManualMissionsDirectory().string().c_str(),
    scheduled_running_profile_id_,
    manual_mapping_profile_id_,
    manual_routed_profile_id_,
    manual_teleop_profile_id_,
    teleop_odometry_topic_.c_str(),
    manual_mapping_odometry_topic_.c_str(),
    routed_mission_odometry_topic_.c_str(),
    manual_mapping_navsat_topic_.c_str());
}

void MissionExecutorNode::handleListExecutableMissions(
  const std::shared_ptr<srv::ListExecutableMissions::Request>,
  std::shared_ptr<srv::ListExecutableMissions::Response> response)
{
  const auto missions = discoverManualMissions();
  response->success = true;
  response->message = "Executable missions listed";
  for (const auto & mission : missions) {
    response->mission_ids.push_back(mission.mission_id);
    response->mission_types.push_back(mission.mission_type);
    response->execution_modes.push_back(mission.execution_mode);
    response->running_profile_ids.push_back(mission.running_profile_id);
    response->is_manual.push_back(mission.is_manual);
    response->artifacts_ready.push_back(mission.artifacts_ready);
  }
}

void MissionExecutorNode::handleListManualMissions(
  const std::shared_ptr<srv::ListManualMissions::Request>,
  std::shared_ptr<srv::ListManualMissions::Response> response)
{
  const auto missions = discoverManualMissions();
  response->success = true;
  response->message = "Manual missions listed";
  for (const auto & mission : missions) {
    if (!mission.is_manual) {
      continue;
    }
    response->mission_ids.push_back(mission.mission_id);
    response->mission_types.push_back(mission.mission_type);
    response->execution_modes.push_back(mission.execution_mode);
    response->running_profile_ids.push_back(mission.running_profile_id);
  }
}

void MissionExecutorNode::handleUploadVda5050Mission(
  const std::shared_ptr<srv::UploadVda5050Mission::Request> request,
  std::shared_ptr<srv::UploadVda5050Mission::Response> response)
{
  if (request->mission_json.empty()) {
    response->success = false;
    response->message = "mission_json is required";
    return;
  }

  try {
    auto mission_document = nlohmann::json::parse(request->mission_json);
    if (!mission_document.is_object()) {
      throw std::runtime_error("mission_json must describe a JSON object");
    }

    const std::string mission_id = deriveMissionId(mission_document, request->mission_id);
    const auto missions_root = resolveMissionsFromDbDirectory();
    const auto mission_file = missions_root / (mission_id + mission_file_extension_);

    if (std::filesystem::exists(mission_file) && !request->overwrite_existing)
    {
      response->success = false;
      response->message = "Mission already exists for mission_id=" + mission_id;
      return;
    }

    std::filesystem::create_directories(missions_root);

    if (mission_document.contains("mission_type") &&
      mission_document.at("mission_type").is_string() &&
      toLower(mission_document.at("mission_type").get<std::string>()) != kScheduledMissionType)
    {
      throw std::runtime_error("upload_vda5050_mission only accepts autonomous VDA5050 missions");
    }

    mission_document["mission_type"] = kScheduledMissionType;

    writeJsonDocumentAtomic(mission_file, mission_document);

    // Clear stale generated artifacts so the parser rebuilds from the new VDA5050 payload on execution.
    const auto mission_folder = resolveMissionsLogDirectory() / mission_id;
    std::filesystem::remove(mission_folder / (mission_id + "_costmap.yaml"));
    std::filesystem::remove(mission_folder / (mission_id + "_costmap.pgm"));
    std::filesystem::remove(mission_folder / (mission_id + "_path_planned.geojson"));
    std::filesystem::remove(mission_folder / (mission_id + "_vda5050" + mission_file_extension_));

    const auto mission = classifyMissionFile(mission_file);
    if (!mission) {
      throw std::runtime_error("Stored mission could not be classified");
    }

    response->success = true;
    response->message = "VDA5050 mission uploaded";
    response->mission_id = mission->mission_id;
    response->mission_file = mission->mission_path;
    response->mission_folder = mission_folder.string();
    response->mission_type = mission->mission_type;
    response->running_profile_id = mission->running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleCreateRecordedMission(
  const std::shared_ptr<srv::CreateRecordedMission::Request> request,
  std::shared_ptr<srv::CreateRecordedMission::Response> response)
{
  try {
    const std::filesystem::path latest_directory =
      resolveMissionsLogDirectory() / kLatestRecordedMapDirectoryName;
    const std::filesystem::path latest_metadata_file = latest_directory / kLatestRecordedMapMetadataFile;
    if (!std::filesystem::exists(latest_metadata_file)) {
      throw std::runtime_error("No latest recorded map is available yet");
    }

    const nlohmann::json latest_metadata = loadJsonDocument(latest_metadata_file);
    const std::filesystem::path perimeter_route_file(
      latest_metadata.value("recorded_work_area_route_file", std::string{}));
    const std::filesystem::path costmap_yaml_file(
      latest_metadata.value("recorded_work_area_costmap_yaml", std::string{}));
    const std::filesystem::path costmap_image_file(
      latest_metadata.value("recorded_work_area_costmap_image", std::string{}));
    if (perimeter_route_file.empty() || !std::filesystem::exists(perimeter_route_file) ||
      costmap_yaml_file.empty() || !std::filesystem::exists(costmap_yaml_file) ||
      costmap_image_file.empty() || !std::filesystem::exists(costmap_image_file))
    {
      throw std::runtime_error("Latest recorded map artifacts are incomplete");
    }

    const std::vector<MapPoint> perimeter =
      closePolygon(extractLineStringCoordinates(loadJsonDocument(perimeter_route_file)));
    if (perimeter.size() < 4U) {
      throw std::runtime_error("Latest recorded map perimeter is too small to create a mission");
    }

    std::string applied_pattern;
    std::vector<MapPoint> route = buildSweepRouteForPattern(perimeter, request->sweep_pattern, applied_pattern);
    if (route.size() < 2U) {
      route = buildZigzagSweepRoute(perimeter);
      applied_pattern = kZigzagSweepPattern;
    }
    std::vector<MapPoint> obstacle_points;
    if (latest_metadata.contains("recorded_obstacle_points") &&
      latest_metadata.at("recorded_obstacle_points").is_array())
    {
      for (const auto & point : latest_metadata.at("recorded_obstacle_points")) {
        if (!point.is_object()) {
          continue;
        }
        obstacle_points.push_back({
          point.value("x", 0.0),
          point.value("y", 0.0)});
      }
    }
    const auto no_go_zones = buildObstacleNoGoZones(obstacle_points);
    GeoTransform geo_transform;
    if (latest_metadata.contains("geo_transform") && latest_metadata.at("geo_transform").is_object()) {
      const auto & transform_document = latest_metadata.at("geo_transform");
      geo_transform.valid = transform_document.value("valid", false);
      if (transform_document.contains("longitude_coefficients") &&
        transform_document.at("longitude_coefficients").is_array() &&
        transform_document.at("longitude_coefficients").size() == 3U &&
        transform_document.contains("latitude_coefficients") &&
        transform_document.at("latitude_coefficients").is_array() &&
        transform_document.at("latitude_coefficients").size() == 3U)
      {
        for (std::size_t index = 0U; index < 3U; ++index) {
          geo_transform.longitude_coefficients[index] =
            transform_document.at("longitude_coefficients").at(index).get<double>();
          geo_transform.latitude_coefficients[index] =
            transform_document.at("latitude_coefficients").at(index).get<double>();
        }
      } else {
        geo_transform.valid = false;
      }
    }
    if (!geo_transform.valid) {
      throw std::runtime_error("Latest recorded map does not contain a valid WGS84 transform");
    }
    const std::vector<GeoPoint> geo_perimeter =
      uniqueGeoPolygonVertices(convertToGeoPoints(uniquePolygonVertices(perimeter), geo_transform));
    const std::vector<GeoPoint> geo_route = convertToGeoPoints(route, geo_transform);
    const auto geo_no_go_zones = convertZonesToGeo(no_go_zones, geo_transform);

    const auto now = std::chrono::system_clock::now();
    const std::string mission_id = sanitizeMissionId(defaultIfEmpty(
      request->mission_name,
      "recorded_map_" + formatUtcTimestamp(now)));
    if (mission_id.empty()) {
      throw std::runtime_error("mission_name is required");
    }

    const std::filesystem::path mission_file =
      resolveMissionsFromDbDirectory() / (mission_id + mission_file_extension_);
    if (std::filesystem::exists(mission_file) && !request->overwrite_existing) {
      throw std::runtime_error("Mission already exists for mission_name=" + mission_id);
    }

    std::filesystem::create_directories(resolveMissionsFromDbDirectory());
    const std::string timestamp = formatUtcTimestamp(now);
    nlohmann::json mission_document =
      buildGeoReferencedVda5050MissionDocument(
        mission_id,
        timestamp,
        geo_perimeter,
        geo_route,
        geo_no_go_zones,
        applied_pattern,
        latest_metadata.value("mission_id", std::string("RecordMap")),
        latest_metadata.value("run_started_at", std::string{}));
    mission_document["mission_type"] = kScheduledMissionType;
    mission_document["name"] = mission_id;

    {
      std::ofstream mission_stream(mission_file, std::ios::trunc);
      if (!mission_stream.is_open()) {
        throw std::runtime_error("Failed to write recorded mission file: " + mission_file.string());
      }
      writeJsonDocumentAtomic(mission_file, mission_document);
    }

    response->success = true;
    response->message = "Recorded mission created";
    response->mission_id = mission_id;
    response->mission_file = mission_file.string();
    response->mission_folder = (resolveMissionsLogDirectory() / mission_id).string();
    response->applied_sweep_pattern = applied_pattern;
    response->latest_recorded_map_file = latest_metadata_file.string();
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handlePrepareManualMission(
  const std::shared_ptr<srv::PrepareManualMission::Request> request,
  std::shared_ptr<srv::PrepareManualMission::Response> response)
{
  const auto mission = findManualMission(request->mission_id);
  if (!mission || !mission->is_manual) {
    response->success = false;
    response->message = "Manual mission not found for mission_id=" + request->mission_id;
    return;
  }

  try {
    if (!ensureMissionArtifactsReady(*mission)) {
      response->success = false;
      response->message = "Mission artifacts are not ready for mission_id=" + mission->mission_id;
      if (const auto staged_directory = newestScheduledArtifactDirectory(mission->mission_id)) {
        response->message +=
          "; newest_staged_variant=" + staged_directory->filename().string();
      }
      return;
    }
    const PreparedMissionContext context =
      prepareMissionArtifacts(resolveExecutableMissionSource(*mission), "", "");
    response->success = true;
    response->message = "Manual mission prepared";
    response->mission_execution_directory = context.mission_execution_directory;
    response->execution_context_file = context.execution_context_file;
    response->running_profile_id = context.running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleExecuteMission(
  const std::shared_ptr<srv::ExecuteMission::Request> request,
  std::shared_ptr<srv::ExecuteMission::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Received execute_mission request: mission_id=%s requester=%s priority=%u force=%s record_rosbag=%s reason=%s",
    request->mission_id.c_str(),
    request->requester.c_str(),
    request->priority,
    request->force ? "true" : "false",
    request->record_rosbag ? "true" : "false",
    request->reason.c_str());

  const auto mission = findManualMission(request->mission_id);
  if (!mission) {
    response->success = false;
    response->message = "Mission not found for mission_id=" + request->mission_id;
    return;
  }

  try {
    const auto resolved_mission = *mission;
    PreparedMissionContext context;
    if (!request->mission_execution_directory.empty()) {
      context.mission_execution_directory = request->mission_execution_directory;
      context.execution_context_file =
        resolveExecutionContextPath(request->mission_execution_directory).string();
      context.running_profile_id = resolved_mission.running_profile_id;
    } else {
      if (!ensureMissionArtifactsReady(resolved_mission, request->requester, request->reason)) {
        response->success = false;
        response->message = "Mission artifacts are not ready for mission_id=" + resolved_mission.mission_id;
        if (const auto staged_directory = newestScheduledArtifactDirectory(resolved_mission.mission_id)) {
          response->message +=
            "; newest_staged_variant=" + staged_directory->filename().string();
        }
        return;
      }
      const auto executable_mission = resolveExecutableMissionSource(resolved_mission);
      context = prepareMissionArtifacts(
        executable_mission,
        request->mission_window_start,
        request->mission_window_end);
    }
    const bool effective_record_rosbag = request->record_rosbag || record_mission_rosbag_;
    writeMissionExecutionPreferences(context.execution_context_file, effective_record_rosbag);
    rewriteBuiltinLocalPatternArtifacts(resolved_mission, context);
    std::string message;
    if (!requestRunningState(context, *request, message)) {
      response->success = false;
      response->message = message;
      response->mission_execution_directory = context.mission_execution_directory;
      response->execution_context_file = context.execution_context_file;
      response->running_profile_id = context.running_profile_id;
      return;
    }
    recordMissionExecutionStart(resolved_mission, context, *request);
    std::string rosbag_warning;
    if (!startMissionRosbagRecording(context, effective_record_rosbag, rosbag_warning) &&
      !rosbag_warning.empty())
    {
      message += " (" + rosbag_warning + ")";
    }
    auto context_document = loadJsonDocument(context.execution_context_file);
    context_document["execution_context_file"] = context.execution_context_file;
    refreshActiveMissionState(context_document);

    response->success = true;
    response->message = message;
    response->mission_execution_directory = context.mission_execution_directory;
    response->execution_context_file = context.execution_context_file;
    response->running_profile_id = context.running_profile_id;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleEndMission(
  const std::shared_ptr<srv::EndMission::Request> request,
  std::shared_ptr<srv::EndMission::Response> response)
{
  try {
    auto context_document = resolveExecutionContext(request->mission_id);
    const std::string mission_run_directory = context_document ?
      context_document->value("mission_run_directory", std::string{}) : std::string{};
    const std::string execution_context_file = context_document ?
      context_document->value("execution_context_file", std::string{}) : std::string{};
    std::string finalization_message;
    if (!finalizeMissionExecution(*request, finalization_message, std::move(context_document))) {
      response->success = false;
      response->message = finalization_message;
      return;
    }

    response->success = true;
    response->message = finalization_message;
    response->mission_execution_directory = mission_run_directory;
    response->execution_context_file = execution_context_file;
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = exception.what();
  }
}

void MissionExecutorNode::handleSafetyStop(
  const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr message)
{
  if (!message) {
    return;
  }
  try {
    recordSafetyEvent(*message, resolveExecutionContext(""));
  } catch (const std::exception & exception) {
    RCLCPP_WARN(get_logger(), "Failed to record safety stop event: %s", exception.what());
  }
}

void MissionExecutorNode::handleManualMissionOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::string actual_path_file;
  bool tracked_mission_active = false;
  bool should_write_path = false;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    tracked_mission_active = active_mission_running_ && active_mission_uses_inactivity_watchdog_;
    if (!tracked_mission_active) {
      return;
    }
    should_write_path = active_mission_is_teleop_;
    actual_path_file = active_actual_path_file_;
  }

  const auto & position = message->pose.pose.position;
  const double linear_speed = std::hypot(
    message->twist.twist.linear.x,
    message->twist.twist.linear.y);
  const double angular_speed = std::abs(message->twist.twist.angular.z);
  const bool moving =
    linear_speed >= manual_mission_min_linear_speed_mps_ ||
    angular_speed >= manual_mission_min_angular_speed_rps_;

  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (!active_mission_running_ || !active_mission_uses_inactivity_watchdog_) {
      return;
    }

    if (active_mission_is_teleop_ && teleop_traveled_path_points_.empty()) {
      geometry_msgs::msg::Point point;
      point.x = position.x;
      point.y = position.y;
      point.z = position.z;
      teleop_traveled_path_points_.push_back(point);
    } else if (active_mission_is_teleop_) {
      const auto & last_point = teleop_traveled_path_points_.back();
      const double dx = position.x - last_point.x;
      const double dy = position.y - last_point.y;
      if ((dx * dx + dy * dy) >= (teleop_path_sample_distance_m_ * teleop_path_sample_distance_m_)) {
        geometry_msgs::msg::Point point;
        point.x = position.x;
        point.y = position.y;
        point.z = position.z;
        teleop_traveled_path_points_.push_back(point);
      }
    }

    if (moving) {
      last_manual_mission_motion_time_ = message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
        now() : rclcpp::Time(message->header.stamp);
    }
  }

  if (should_write_path && !actual_path_file.empty()) {
    nlohmann::json coordinates = nlohmann::json::array();
    std::string navsat_companion_file;
    std::vector<MapPoint> local_trace;
    std::vector<GeoPoint> geo_trace;
    {
      std::lock_guard<std::mutex> lock(active_mission_mutex_);
      for (const auto & point : teleop_traveled_path_points_) {
        coordinates.push_back({point.x, point.y});
        local_trace.push_back({point.x, point.y});
      }
      if (!active_actual_navsat_path_file_.empty()) {
        navsat_companion_file =
          std::filesystem::path(active_actual_navsat_path_file_).filename().string();
      }
      geo_trace.reserve(manual_mapping_navsat_points_.size());
      for (const auto & point : manual_mapping_navsat_points_) {
        geo_trace.push_back({point.y, point.x});
      }
    }
    const auto georeference = buildGeoReferenceMetadata(local_trace, geo_trace, navsat_companion_file);
    nlohmann::json actual_path_document = buildLocalPathGeoJson(
      coordinates,
      "actual_path",
      navsat_companion_file,
      georeference);

    try {
      writeJsonDocumentAtomic(actual_path_file, actual_path_document);
    } catch (const std::exception &) {
    }
  }
}

void MissionExecutorNode::handleManualMissionNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::string actual_navsat_path_file;
  std::string local_companion_file;
  std::string actual_path_file;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (
      !active_mission_running_ ||
      !active_mission_is_manual_mapping_ ||
      active_actual_navsat_path_file_.empty())
    {
      return;
    }
    actual_navsat_path_file = active_actual_navsat_path_file_;
    if (!active_actual_path_file_.empty()) {
      actual_path_file = active_actual_path_file_;
      local_companion_file = std::filesystem::path(active_actual_path_file_).filename().string();
    }
  }

  geometry_msgs::msg::Point point;
  point.x = message->longitude;
  point.y = message->latitude;
  point.z = message->altitude;

  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (!active_mission_running_) {
      return;
    }
    if (!manual_mapping_navsat_points_.empty()) {
      const auto & previous = manual_mapping_navsat_points_.back();
      const double distance = std::hypot(point.x - previous.x, point.y - previous.y);
      if (distance < kNavSatSampleDistanceDegrees) {
        return;
      }
    }
    manual_mapping_navsat_points_.push_back(point);
  }

  nlohmann::json navsat_path_document;
  std::vector<GeoPoint> geo_trace;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    geo_trace.reserve(manual_mapping_navsat_points_.size());
    for (const auto & sampled_point : manual_mapping_navsat_points_) {
      geo_trace.push_back({sampled_point.y, sampled_point.x});
    }
    navsat_path_document = buildNavSatGeoJson(
      manual_mapping_navsat_points_,
      "actual_path_navsat",
      local_companion_file);
  }

  try {
    writeJsonDocumentAtomic(actual_navsat_path_file, navsat_path_document);
  } catch (const std::exception &) {
  }
  if (!actual_path_file.empty()) {
    refreshLocalPathGeoReference(actual_path_file, actual_navsat_path_file, geo_trace);
  }
}

void MissionExecutorNode::handleRoutedMissionOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::lock_guard<std::mutex> lock(routed_mission_pose_mutex_);
  routed_mission_position_ = message->pose.pose.position;
  routed_mission_orientation_ = message->pose.pose.orientation;
  routed_mission_pose_stamp_ = message->header.stamp;
  routed_mission_pose_ready_ = true;
}

void MissionExecutorNode::checkManualMissionInactivity()
{
  srv::EndMission::Request request;
  std::string end_reason;
  std::string requester;
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (!active_mission_running_ || !active_mission_uses_inactivity_watchdog_) {
      return;
    }
    if ((now() - last_manual_mission_motion_time_).seconds() < manual_mission_inactivity_timeout_seconds_) {
      return;
    }
    request.mission_id = active_mission_id_;
    if (active_mission_is_manual_mapping_) {
      end_reason = kManualMappingInactivityEndReason;
      requester = "manual_mapping_inactivity_watchdog";
    } else {
      end_reason = kTeleopInactivityEndReason;
      requester = "teleop_inactivity_watchdog";
    }
  }

  request.reason = end_reason;
  request.outcome = "completed";
  request.requester = requester;
  request.priority = default_activation_priority_;
  request.force = false;
  request.request_idling = true;

  std::string message;
  if (!finalizeMissionExecution(request, message)) {
    RCLCPP_WARN(get_logger(), "Failed to auto-end inactive manual mission: %s", message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

std::vector<ManualMissionInfo> MissionExecutorNode::discoverManualMissions() const
{
  std::vector<ManualMissionInfo> missions;
  std::set<std::string> seen_ids;
  auto maybe_add = [this, &missions, &seen_ids](const std::filesystem::path & candidate_path) {
      try {
      const auto mission = classifyMissionFile(candidate_path);
      if (!mission || !seen_ids.insert(mission->mission_id).second) {
        return;
      }
      missions.push_back(*mission);
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(),
          "Skipping mission candidate %s: %s",
          candidate_path.string().c_str(),
          exception.what());
      }
    };

  const auto scan_directory = [&maybe_add](const std::filesystem::path & directory) {
      if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
        return;
      }
      std::error_code error;
      for (std::filesystem::recursive_directory_iterator iterator(
             directory,
             std::filesystem::directory_options::skip_permission_denied,
             error);
           iterator != std::filesystem::recursive_directory_iterator();
           iterator.increment(error))
      {
        if (error) {
          error.clear();
          continue;
        }
        if (!iterator->is_regular_file(error)) {
          error.clear();
          continue;
        }
        if (iterator->path().extension() != ".json") {
          continue;
        }
        maybe_add(iterator->path());
      }
    };

  scan_directory(resolveManualMissionsDirectory());
  scan_directory(resolveMissionsFromDbDirectory());

  std::sort(
    missions.begin(),
    missions.end(),
    [](const ManualMissionInfo & left, const ManualMissionInfo & right) {
      return left.mission_id < right.mission_id;
    });
  return missions;
}

std::optional<ManualMissionInfo> MissionExecutorNode::findManualMission(
  const std::string & mission_id) const
{
  const auto missions = discoverManualMissions();
  const auto it = std::find_if(
    missions.begin(),
    missions.end(),
    [&mission_id](const ManualMissionInfo & mission) {return mission.mission_id == mission_id;});
  if (it == missions.end()) {
    return findStagedScheduledMission(mission_id);
  }
  return *it;
}

std::string MissionExecutorNode::sanitizeMissionId(const std::string & mission_id)
{
  std::string sanitized;
  sanitized.reserve(mission_id.size());
  for (const unsigned char character : mission_id) {
    if (std::isalnum(character) || character == '-' || character == '_') {
      sanitized.push_back(static_cast<char>(character));
      continue;
    }
    if (character == ' ' || character == '.' || character == '/') {
      sanitized.push_back('_');
    }
  }

  if (sanitized.empty()) {
    throw std::runtime_error("Mission id must contain at least one alphanumeric character");
  }
  return sanitized;
}

std::string MissionExecutorNode::deriveMissionId(
  const nlohmann::json & document,
  const std::string & requested_mission_id)
{
  if (!requested_mission_id.empty()) {
    return sanitizeMissionId(requested_mission_id);
  }

  if (document.contains("orderId") && document.at("orderId").is_string()) {
    return sanitizeMissionId(document.at("orderId").get<std::string>());
  }

  if (document.contains("name") && document.at("name").is_string()) {
    return sanitizeMissionId(document.at("name").get<std::string>());
  }

  throw std::runtime_error("Unable to derive mission_id from mission_json; provide mission_id explicitly");
}

std::filesystem::path MissionExecutorNode::resolvePath(const std::string & configured_path) const
{
  const std::filesystem::path configured(configured_path);
  if (configured.is_absolute()) {
    return configured;
  }

  const std::filesystem::path workspace_relative = std::filesystem::current_path() / configured;
  if (std::filesystem::exists(workspace_relative)) {
    return workspace_relative;
  }
  return configured;
}

std::filesystem::path MissionExecutorNode::resolveMissionsFromDbDirectory() const
{
  return resolvePath(missions_directory_);
}

std::filesystem::path MissionExecutorNode::resolveMissionsLogDirectory() const
{
  return resolvePath(missions_log_directory_);
}

std::filesystem::path MissionExecutorNode::resolveRosbagDirectory() const
{
  return resolvePath(rosbag_directory_);
}

std::filesystem::path MissionExecutorNode::resolveManualMissionsDirectory() const
{
  if (!manual_missions_directory_.empty()) {
    return resolvePath(manual_missions_directory_);
  }

  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory(kDefaultMissionsPackageName)) / "missions";
}

std::vector<std::filesystem::path> MissionExecutorNode::executionContextFiles() const
{
  std::vector<std::filesystem::path> results;
  const auto missions_log_directory = resolveMissionsLogDirectory();
  std::error_code error;
  if (!std::filesystem::exists(missions_log_directory, error)) {
    return results;
  }

  for (std::filesystem::recursive_directory_iterator iterator(
         missions_log_directory,
         std::filesystem::directory_options::skip_permission_denied,
         error);
       iterator != std::filesystem::recursive_directory_iterator();
       iterator.increment(error))
  {
    if (error) {
      error.clear();
      continue;
    }
    if (!iterator->is_regular_file(error)) {
      error.clear();
      continue;
    }
    if (isExecutionContextArtifact(iterator->path())) {
      results.push_back(iterator->path());
    }
  }
  return results;
}

std::optional<ManualMissionInfo> MissionExecutorNode::findStagedScheduledMission(
  const std::string & mission_id) const
{
  const std::filesystem::path missions_log_directory = resolveMissionsLogDirectory();
  std::error_code error;
  if (!std::filesystem::exists(missions_log_directory, error) ||
    !std::filesystem::is_directory(missions_log_directory, error))
  {
    return std::nullopt;
  }

  std::optional<std::filesystem::path> best_path;
  std::string best_stem;
  for (const auto & entry : std::filesystem::directory_iterator(missions_log_directory, error)) {
    if (error) {
      error.clear();
      continue;
    }
    if (!entry.is_directory(error)) {
      error.clear();
      continue;
    }

    const std::string stem = entry.path().filename().string();
    if (stem != mission_id && !has_timestamp_suffix(stem, mission_id)) {
      continue;
    }

    const std::filesystem::path candidate =
      entry.path() / (stem + "_vda5050" + mission_file_extension_);
    if (!std::filesystem::exists(candidate, error) || !std::filesystem::is_regular_file(candidate, error)) {
      error.clear();
      continue;
    }

    if (!best_path || stem > best_stem) {
      best_path = candidate;
      best_stem = stem;
    }
  }

  if (!best_path) {
    return std::nullopt;
  }

  try {
    return classifyMissionFile(*best_path);
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to classify staged scheduled mission %s: %s",
      best_path->string().c_str(),
      exception.what());
    return std::nullopt;
  }
}

std::filesystem::path MissionExecutorNode::missionFolderPath(
  const std::filesystem::path & mission_path) const
{
  return mission_path.parent_path();
}

std::filesystem::path MissionExecutorNode::artifactsDirectoryForMission(
  const ManualMissionInfo & mission) const
{
  if (toLower(mission.mission_type) == kScheduledMissionType) {
    const std::filesystem::path mission_path(mission.mission_path);
    if (mission_path.has_parent_path()) {
      const std::filesystem::path parent = mission_path.parent_path();
      const std::filesystem::path missions_database_directory = resolveMissionsFromDbDirectory();
      if (parent != missions_database_directory &&
        !(parent.filename() == "simulations" && parent.parent_path() == missions_database_directory))
      {
        return parent;
      }
    }
    return resolveMissionsLogDirectory() / mission.mission_id;
  }
  return missionFolderPath(std::filesystem::path(mission.mission_path));
}

std::string MissionExecutorNode::missionStemForPath(const std::filesystem::path & mission_path) const
{
  const std::filesystem::path missions_database_directory = resolveMissionsFromDbDirectory();
  if (mission_path.has_parent_path() && mission_path.parent_path() != missions_database_directory) {
    const std::filesystem::path parent = mission_path.parent_path();
    if (parent.filename() == "simulations" && parent.parent_path() == missions_database_directory) {
      return mission_path.stem().string();
    }
    return parent.filename().string();
  }
  return mission_path.stem().string();
}

std::string MissionExecutorNode::missionCostmapBasename(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_costmap";
}

std::string MissionExecutorNode::missionRouteBasename(
  const std::filesystem::path & mission_path) const
{
  return missionStemForPath(mission_path) + "_path_planned";
}

std::filesystem::path MissionExecutorNode::resolveMissionRoutePath(
  const ManualMissionInfo & mission,
  const std::filesystem::path & mission_file) const
{
  const std::filesystem::path mission_folder = artifactsDirectoryForMission(mission);
  const std::filesystem::path planned_route =
    mission_folder / (missionRouteBasename(mission_file) + ".geojson");
  if (std::filesystem::exists(planned_route)) {
    return planned_route;
  }

  const std::filesystem::path builtin_route =
    mission_folder / (missionStemForPath(mission_file) + "_path.geojson");
  if (std::filesystem::exists(builtin_route)) {
    return builtin_route;
  }

  return planned_route;
}

std::optional<std::filesystem::path> MissionExecutorNode::newestScheduledArtifactDirectory(
  const std::string & mission_id) const
{
  const std::filesystem::path missions_log_directory = resolveMissionsLogDirectory();
  std::error_code error;
  if (!std::filesystem::exists(missions_log_directory, error) ||
    !std::filesystem::is_directory(missions_log_directory, error))
  {
    return std::nullopt;
  }

  std::optional<std::filesystem::path> newest_directory;
  std::string newest_stem;
  for (const auto & entry : std::filesystem::directory_iterator(missions_log_directory, error)) {
    if (error) {
      error.clear();
      continue;
    }
    if (!entry.is_directory(error)) {
      error.clear();
      continue;
    }
    const std::string stem = entry.path().filename().string();
    if (stem != mission_id && !has_timestamp_suffix(stem, mission_id)) {
      continue;
    }
    const std::filesystem::path staged_mission_file =
      entry.path() / (stem + "_vda5050" + mission_file_extension_);
    if (!std::filesystem::exists(staged_mission_file, error)) {
      error.clear();
      continue;
    }
    if (!newest_directory || stem > newest_stem) {
      newest_directory = entry.path();
      newest_stem = stem;
    }
  }
  return newest_directory;
}

ManualMissionInfo MissionExecutorNode::resolveExecutableMissionSource(const ManualMissionInfo & mission) const
{
  if (toLower(mission.mission_type) != kScheduledMissionType) {
    return mission;
  }

  const std::filesystem::path canonical_history_directory = missionHistoryDirectory(mission);
  const std::filesystem::path canonical_history_mission_file =
    canonical_history_directory / (mission.mission_id + "_vda5050" + mission_file_extension_);
  if (std::filesystem::exists(canonical_history_mission_file)) {
    ManualMissionInfo resolved = mission;
    resolved.mission_path = canonical_history_mission_file.string();
    return resolved;
  }

  const std::filesystem::path mission_path(mission.mission_path);
  if (mission_path.has_parent_path() &&
    mission_path.parent_path() != resolveMissionsFromDbDirectory())
  {
    return mission;
  }

  const auto staged_directory = newestScheduledArtifactDirectory(mission.mission_id);
  if (!staged_directory) {
    return mission;
  }

  const std::string staged_stem = staged_directory->filename().string();
  const std::filesystem::path staged_mission_file =
    *staged_directory / (staged_stem + "_vda5050" + mission_file_extension_);
  if (!std::filesystem::exists(staged_mission_file)) {
    return mission;
  }

  ManualMissionInfo resolved = mission;
  resolved.mission_path = staged_mission_file.string();
  return resolved;
}

std::filesystem::path MissionExecutorNode::missionHistoryDirectory(const ManualMissionInfo & mission) const
{
  return resolveMissionsLogDirectory() / mission.mission_id;
}

std::optional<ManualMissionInfo> MissionExecutorNode::classifyMissionFile(
  const std::filesystem::path & mission_path) const
{
  if (!std::filesystem::is_regular_file(mission_path) || mission_path.extension() != mission_file_extension_) {
    return std::nullopt;
  }

  const nlohmann::json document = loadJsonDocument(mission_path);
  ManualMissionInfo mission;
  mission.mission_id = missionStemForPath(mission_path);
  mission.mission_path = mission_path.string();
  mission.mission_type =
    document.contains("mission_type") && document.at("mission_type").is_string() ?
    document.at("mission_type").get<std::string>() :
    kScheduledMissionType;
  mission.execution_mode = kNavigateThroughPosesExecutionMode;

  if (document.contains("execution_mode") && document.at("execution_mode").is_string()) {
    mission.execution_mode = toLower(document.at("execution_mode").get<std::string>());
  } else if (toLower(mission.mission_type) == kBuiltinManualMappingMissionType) {
    mission.execution_mode = kManualMappingExecutionMode;
  } else if (toLower(mission.mission_type) == kBuiltinTeleopMissionType) {
    mission.execution_mode = kTeleoperationExecutionMode;
  }

  const std::string lowered_mission_type = toLower(mission.mission_type);
  const bool scheduled_local_frame =
    lowered_mission_type == kScheduledMissionType &&
    document.contains("missionReference") &&
    document.at("missionReference").is_object() &&
    document.at("missionReference").contains("coordinateFrame") &&
    document.at("missionReference").at("coordinateFrame").is_string() &&
    [] (std::string value) {
      value = toLower(std::move(value));
      return value == "odom" || value == "local";
    }(document.at("missionReference").at("coordinateFrame").get<std::string>());
  if (scheduled_local_frame) {
    mission.mission_type = kLocalScheduledMissionType;
  }

  const std::string effective_mission_type = toLower(mission.mission_type);
  if (mission.execution_mode == kManualMappingExecutionMode) {
    mission.running_profile_id = manual_mapping_profile_id_;
  } else if (mission.execution_mode == kTeleoperationExecutionMode) {
    mission.running_profile_id = manual_teleop_profile_id_;
  } else if (
    effective_mission_type == kBuiltinLocalPatternMissionType ||
    effective_mission_type == kLocalScheduledMissionType)
  {
    mission.running_profile_id = manual_routed_profile_id_;
  } else {
    mission.running_profile_id = scheduled_running_profile_id_;
  }
  mission.is_manual =
    mission.execution_mode == kManualMappingExecutionMode ||
    mission.execution_mode == kTeleoperationExecutionMode ||
    effective_mission_type == kBuiltinManualMappingMissionType ||
    effective_mission_type == kBuiltinTeleopMissionType;
  mission.artifacts_ready = missionArtifactsReady(mission);
  return mission;
}

PreparedMissionContext MissionExecutorNode::prepareMissionArtifacts(
  const ManualMissionInfo & mission,
  const std::string & mission_window_start,
  const std::string & mission_window_end) const
{
  namespace fs = std::filesystem;
  const std::filesystem::path mission_file(mission.mission_path);
  const std::filesystem::path source_mission_folder = artifactsDirectoryForMission(mission);
  const std::filesystem::path mission_costmap_yaml =
    source_mission_folder / (missionCostmapBasename(mission_file) + ".yaml");
  const std::filesystem::path mission_costmap_image =
    source_mission_folder / (missionCostmapBasename(mission_file) + ".pgm");
  const std::filesystem::path mission_route = resolveMissionRoutePath(mission, mission_file);

  if (!fs::exists(mission_file) ||
    !fs::exists(mission_costmap_yaml) ||
    !fs::exists(mission_costmap_image) ||
    !fs::exists(mission_route))
  {
    throw std::runtime_error("Manual mission artifacts are incomplete for mission_id=" + mission.mission_id);
  }

  const auto run_start_time = std::chrono::system_clock::now();
  const std::string run_timestamp = formatUtcTimestamp(run_start_time);
  const fs::path mission_history_directory = missionHistoryDirectory(mission);
  fs::create_directories(mission_history_directory);

  std::filesystem::path selected_costmap_yaml = mission_costmap_yaml;
  std::filesystem::path selected_costmap_image = mission_costmap_image;
  try {
    const RasterizedCostmap source_costmap = loadCostmapArtifacts(mission_costmap_yaml);
    RCLCPP_INFO(
      get_logger(),
      "Mission startup costmap source %s parsed with georeference_valid=%s resolution=%.3f origin=(%.3f, %.3f) size=%ux%u samples=%zu.",
      mission_costmap_yaml.string().c_str(),
      source_costmap.georeference_valid ? "true" : "false",
      source_costmap.resolution,
      source_costmap.origin_x,
      source_costmap.origin_y,
      source_costmap.width_cells,
      source_costmap.height_cells,
      source_costmap.georeference_sample_count);
    if (!source_costmap.georeference_valid) {
      const auto historical_georeferenced_yaml = findNewestGeoreferencedHistoricalCostmapYaml(
        mission_history_directory,
        mission.mission_id + "_costmap.yaml");
      if (historical_georeferenced_yaml.has_value()) {
        const auto historical_georeferenced_image =
          historical_georeferenced_yaml->parent_path() /
          (historical_georeferenced_yaml->stem().string() + ".pgm");
        if (fs::exists(historical_georeferenced_image)) {
          selected_costmap_yaml = *historical_georeferenced_yaml;
          selected_costmap_image = historical_georeferenced_image;
          RCLCPP_WARN(
            get_logger(),
            "Mission source costmap %s is non-georeferenced. Reusing newest georeferenced historical costmap %s for startup seeding.",
            mission_costmap_yaml.string().c_str(),
            selected_costmap_yaml.string().c_str());
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Mission source costmap %s is non-georeferenced and no older georeferenced historical startup costmap was found under %s.",
          mission_costmap_yaml.string().c_str(),
          mission_history_directory.string().c_str());
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to inspect mission source costmap %s before preparing mission artifacts: %s",
      mission_costmap_yaml.string().c_str(),
      exception.what());
  }

  const fs::path mission_run_directory = mission_history_directory / run_timestamp;
  fs::create_directories(mission_run_directory);
  const std::string run_artifact_stem = missionRunArtifactStem(mission.mission_id, run_timestamp);

  const fs::path history_mission_file =
    mission_history_directory / (mission.mission_id + "_vda5050" + mission_file_extension_);
  const fs::path history_costmap_yaml =
    mission_history_directory / (mission.mission_id + "_costmap.yaml");
  const fs::path history_costmap_image =
    mission_history_directory / (mission.mission_id + "_costmap.pgm");
  const fs::path history_route =
    mission_history_directory / (mission.mission_id + "_path_planned.geojson");
  if (mission_file != history_mission_file) {
    fs::copy_file(mission_file, history_mission_file, fs::copy_options::overwrite_existing);
  }
  if (selected_costmap_yaml != history_costmap_yaml) {
    fs::copy_file(selected_costmap_yaml, history_costmap_yaml, fs::copy_options::overwrite_existing);
    rewriteCostmapYamlImageReference(history_costmap_yaml, history_costmap_image);
  }
  if (selected_costmap_image != history_costmap_image) {
    fs::copy_file(selected_costmap_image, history_costmap_image, fs::copy_options::overwrite_existing);
  }
  if (mission_route != history_route) {
    fs::copy_file(mission_route, history_route, fs::copy_options::overwrite_existing);
  }

  const fs::path run_mission_file =
    mission_run_directory / (run_artifact_stem + "_vda5050" + mission_file_extension_);
  const fs::path run_costmap_yaml =
    mission_run_directory / (run_artifact_stem + "_costmap.yaml");
  const fs::path run_costmap_image =
    mission_run_directory / (run_artifact_stem + "_costmap.pgm");
  const fs::path run_route =
    mission_run_directory / (run_artifact_stem + "_path_planned.geojson");
  const fs::path actual_path_file =
    mission_run_directory / (run_artifact_stem + "_path_actual.geojson");
  const fs::path actual_path_navsat_file =
    mission_run_directory / (run_artifact_stem + "_path_navsat.geojson");
  const fs::path gaussian_output_directory = mission_run_directory / "gaussian";
  const fs::path captured_images_directory = mission_run_directory / "captured_images";
  const fs::path collected_artifacts_directory = mission_run_directory / "artifacts";

  fs::copy_file(history_mission_file, run_mission_file, fs::copy_options::overwrite_existing);
  fs::copy_file(history_costmap_yaml, run_costmap_yaml, fs::copy_options::overwrite_existing);
  fs::copy_file(history_costmap_image, run_costmap_image, fs::copy_options::overwrite_existing);
  rewriteCostmapYamlImageReference(run_costmap_yaml, run_costmap_image);
  fs::copy_file(history_route, run_route, fs::copy_options::overwrite_existing);
  fs::create_directories(gaussian_output_directory);
  fs::create_directories(captured_images_directory);
  fs::create_directories(collected_artifacts_directory);

  {
    const std::string navsat_companion_file = actual_path_navsat_file.filename().string();
    nlohmann::json actual_path_document = buildLocalPathGeoJson(
      nlohmann::json::array(),
      "actual_path",
      navsat_companion_file);
    writeJsonDocumentAtomic(actual_path_file, actual_path_document);
  }
  {
    const std::string local_companion_file = actual_path_file.filename().string();
    nlohmann::json actual_path_navsat_document = buildNavSatGeoJson(
      {},
      "actual_path_navsat",
      local_companion_file);
    writeJsonDocumentAtomic(actual_path_navsat_file, actual_path_navsat_document);
  }

  const nlohmann::json context{
    {"mission_id", mission.mission_id},
    {"mission_type", mission.mission_type},
    {"execution_mode", mission.execution_mode},
    {"mission_file", run_mission_file.string()},
    {"mission_folder", mission_history_directory.string()},
    {"mission_route_file", run_route.string()},
    {"mission_costmap_yaml", run_costmap_yaml.string()},
    {"saved_costmap_yaml", run_costmap_yaml.string()},
    {"mission_run_directory", mission_run_directory.string()},
    {"persistent_mission_file", history_mission_file.string()},
    {"persistent_mission_route_file", history_route.string()},
    {"persistent_mission_costmap_yaml", history_costmap_yaml.string()},
    {"mission_window_start", mission_window_start},
    {"mission_window_end", mission_window_end},
    {"run_started_at", run_timestamp},
    {"source_mission_file", mission_file.string()},
    {"source_mission_route_file", mission_route.string()},
    {"source_mission_costmap_yaml", mission_costmap_yaml.string()},
    {"selected_startup_costmap_yaml", selected_costmap_yaml.string()},
    {"actual_path_file", actual_path_file.string()},
    {"actual_path_navsat_file", actual_path_navsat_file.string()},
    {"gaussian_output_directory", gaussian_output_directory.string()},
    {"captured_images_directory", captured_images_directory.string()},
    {"collected_artifacts_directory", collected_artifacts_directory.string()},
    {"schedule_log_path", ensureScheduleLogPath(resolveScheduleSourcePath()).string()},
    {"actual_schedule_log_path", ensureActualScheduleLogPath(resolveScheduleSourcePath()).string()}};

  RCLCPP_INFO(
    get_logger(),
    "Prepared mission artifacts for %s with startup costmap %s, run costmap %s, persistent costmap %s, source costmap %s.",
    mission.mission_id.c_str(),
    selected_costmap_yaml.string().c_str(),
    run_costmap_yaml.string().c_str(),
    history_costmap_yaml.string().c_str(),
    mission_costmap_yaml.string().c_str());

  const fs::path execution_context_file =
    mission_run_directory / (run_artifact_stem + "_context.json");
  writeJsonDocumentAtomic(execution_context_file, context);

  PreparedMissionContext prepared;
  prepared.mission_execution_directory = mission_run_directory.string();
  prepared.execution_context_file = execution_context_file.string();
  prepared.running_profile_id = mission.running_profile_id;
  return prepared;
}

void MissionExecutorNode::rewriteBuiltinLocalPatternArtifacts(
  const ManualMissionInfo & mission,
  const PreparedMissionContext & context) const
{
  if (toLower(mission.mission_type) != kBuiltinLocalPatternMissionType) {
    return;
  }

  const std::filesystem::path execution_context_file(context.execution_context_file);
  auto context_document = loadJsonDocument(execution_context_file);

  const std::filesystem::path run_route_file(
    context_document.value("mission_route_file", std::string{}));
  std::filesystem::path source_route_file(
    context_document.value("source_mission_route_file", std::string{}));
  if (source_route_file.empty()) {
    source_route_file = run_route_file;
  }

  if (run_route_file.empty() || source_route_file.empty()) {
    throw std::runtime_error("Mission execution context is missing local-pattern route artifacts");
  }

  auto route_document = loadJsonDocument(source_route_file);
  if (!route_document.contains("features") || !route_document.at("features").is_array()) {
    throw std::runtime_error("Local-pattern route artifact does not contain a GeoJSON feature list");
  }

  for (auto & feature : route_document["features"]) {
    if (!feature.contains("geometry") || !feature["geometry"].is_object()) {
      continue;
    }
    auto & geometry = feature["geometry"];
    if (!geometry.contains("type") || geometry["type"] != "LineString" ||
      !geometry.contains("coordinates") || !geometry["coordinates"].is_array())
    {
      continue;
    }

    if (!feature.contains("properties") || !feature["properties"].is_object()) {
      feature["properties"] = nlohmann::json::object();
    }
    feature["properties"]["coordinate_frame"] = "base_footprint";
  }

  writeJsonDocumentAtomic(run_route_file, route_document);

  context_document["mission_costmap_yaml"] = "";
  context_document["source_mission_costmap_yaml"] = "";
  writeJsonDocumentAtomic(execution_context_file, context_document);

  RCLCPP_INFO(
    get_logger(),
    "Prepared builtin local pattern %s for mission-start anchoring in layer 3; route frame set to "
    "base_footprint and mission costmap disabled.",
    mission.mission_id.c_str());
}

std::optional<nlohmann::json> MissionExecutorNode::resolveExecutionContext(
  const std::string & mission_id) const
{
  {
    std::lock_guard<std::mutex> lock(active_mission_mutex_);
    if (
      active_mission_running_ &&
      !active_execution_context_file_.empty() &&
      (mission_id.empty() || active_mission_id_ == mission_id) &&
      std::filesystem::exists(active_execution_context_file_))
    {
      auto context_document = loadJsonDocument(active_execution_context_file_);
      context_document["execution_context_file"] = active_execution_context_file_;
      return context_document;
    }
  }

  std::optional<nlohmann::json> selected_context;
  std::string selected_run_started_at;
  std::filesystem::path selected_path;
  for (const auto & context_path : executionContextFiles()) {
    nlohmann::json context_document;
    try {
      context_document = loadJsonDocument(context_path);
    } catch (const std::exception &) {
      continue;
    }
    const std::string context_mission_id = context_document.value("mission_id", std::string{});
    if (!mission_id.empty() && context_mission_id != mission_id) {
      continue;
    }

    const std::string runtime_status = toLower(context_document.value("runtime_status", std::string{}));
    const bool finished = context_document.contains("actual_end_utc") &&
      !context_document.value("actual_end_utc", std::string{}).empty();
    if (
      finished ||
      runtime_status == toLower(std::string{kRuntimeStatusCompleted}) ||
      runtime_status == toLower(std::string{kRuntimeStatusAborted}))
    {
      continue;
    }

    const std::string run_started_at = context_document.value("run_started_at", std::string{});
    if (!selected_context || run_started_at > selected_run_started_at) {
      context_document["execution_context_file"] = context_path.string();
      selected_run_started_at = run_started_at;
      selected_path = context_path;
      selected_context = std::move(context_document);
    }
  }

  if (selected_context) {
    (*selected_context)["execution_context_file"] = selected_path.string();
  }
  return selected_context;
}

bool MissionExecutorNode::requestIdlingState(
  const srv::EndMission::Request & request,
  std::string & message) const
{
  if (!fsm_request_client_->wait_for_service(std::chrono::seconds(5))) {
    message = "FSM request_state service is unavailable for end_mission";
    return false;
  }

  auto fsm_request = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  fsm_request->target_state = "IDLING";
  fsm_request->target_lifecycle = "Active";
  fsm_request->target_profile_id = idling_profile_id_;
  fsm_request->requester = defaultIfEmpty(request.requester, "mission_executor");
  fsm_request->priority = request.priority == 0U ? default_activation_priority_ : request.priority;
  fsm_request->force = request.force;
  fsm_request->reason = defaultIfEmpty(request.reason, "mission ended");
  fsm_request->mission_execution_directory = "";

  auto future = fsm_request_client_->async_send_request(fsm_request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    message = "Timed out waiting for FSM IDLING request response";
    return false;
  }

  const auto response = future.get();
  if (!response->accepted) {
    message = response->message;
    return false;
  }

  message = "Mission execution finalized and FSM requested IDLING";
  return true;
}

bool MissionExecutorNode::finalizeMissionExecution(
  const srv::EndMission::Request & request,
  std::string & message,
  std::optional<nlohmann::json> context_document)
{
  if (!context_document) {
    context_document = resolveExecutionContext(request.mission_id);
  }
  if (!context_document) {
    message = "No active mission execution context found";
    return false;
  }

  updateRecordMapArtifacts(*context_document);
  promoteRuntimeCostmapArtifacts(*context_document, request);
  refreshLocalPathGeoReferenceFromArtifacts(*context_document);
  writeLatestRecordedMapSnapshot(*context_document);
  recordMissionExecutionEnd(*context_document, request);
  stopMissionRosbagRecording();
  clearActiveMissionState();

  if (!requestIdlingState(request, message)) {
    return false;
  }

  return true;
}

void MissionExecutorNode::promoteRuntimeCostmapArtifacts(
  nlohmann::json & context_document,
  const srv::EndMission::Request & request) const
{
  if (!promote_runtime_costmap_on_completed_mission_) {
    context_document["persistent_costmap_promoted"] = false;
    context_document["persistent_costmap_promotion_skip_reason"] =
      "runtime_costmap_promotion_disabled";
    return;
  }
  const std::string mission_type = toLower(context_document.value("mission_type", std::string{}));
  if (mission_type == kScheduledMissionType || mission_type == kLocalScheduledMissionType) {
    context_document["persistent_costmap_promoted"] = false;
    context_document["persistent_costmap_promotion_skip_reason"] =
      "scheduled_mission_static_costmap_is_parser_owned";
    return;
  }
  const std::string normalized_outcome = toLower(defaultIfEmpty(request.outcome, "completed"));

  const std::filesystem::path runtime_costmap_yaml(
    context_document.value("mission_costmap_yaml", std::string{}));
  std::filesystem::path persistent_costmap_yaml(
    context_document.value("persistent_mission_costmap_yaml", std::string{}));
  if (persistent_costmap_yaml.empty()) {
    const std::filesystem::path mission_folder(
      context_document.value("mission_folder", std::string{}));
    if (!mission_folder.empty() && !runtime_costmap_yaml.empty()) {
      persistent_costmap_yaml = mission_folder / runtime_costmap_yaml.filename();
    }
  }

  if (runtime_costmap_yaml.empty() || persistent_costmap_yaml.empty() ||
    !std::filesystem::exists(runtime_costmap_yaml))
  {
    context_document["persistent_costmap_promoted"] = false;
    context_document["persistent_costmap_promotion_skip_reason"] =
      runtime_costmap_yaml.empty() ?
      "no_runtime_costmap_configured" :
      (persistent_costmap_yaml.empty() ?
      "no_persistent_costmap_destination_configured" :
      "runtime_costmap_file_not_found");
    return;
  }

  const std::filesystem::path persistent_costmap_image =
    persistent_costmap_yaml.parent_path() / (persistent_costmap_yaml.stem().string() + ".pgm");
  try {
    const RasterizedCostmap runtime_costmap = loadCostmapArtifacts(runtime_costmap_yaml);
    bool promote_runtime_costmap = (normalized_outcome == "completed");
    RasterizedCostmap merged_costmap = runtime_costmap;
    if (std::filesystem::exists(persistent_costmap_yaml)) {
      const RasterizedCostmap persistent_costmap = loadCostmapArtifacts(persistent_costmap_yaml);
      if (!promote_runtime_costmap && runtime_costmap.georeference_valid &&
        !persistent_costmap.georeference_valid)
      {
        promote_runtime_costmap = true;
        RCLCPP_WARN(
          get_logger(),
          "Promoting georeferenced runtime costmap %s into non-georeferenced persistent startup artifact %s despite mission outcome '%s'.",
          runtime_costmap_yaml.string().c_str(),
          persistent_costmap_yaml.string().c_str(),
          normalized_outcome.c_str());
      }
      if (!promote_runtime_costmap) {
        context_document["persistent_costmap_promoted"] = false;
        context_document["persistent_costmap_promotion_skip_reason"] =
          "mission_outcome_not_completed";
        return;
      }
      merged_costmap = mergeCostmaps(persistent_costmap, runtime_costmap);
    } else if (!promote_runtime_costmap && runtime_costmap.georeference_valid) {
      promote_runtime_costmap = true;
      RCLCPP_WARN(
        get_logger(),
        "Promoting georeferenced runtime costmap %s into missing persistent startup artifact %s despite mission outcome '%s'.",
        runtime_costmap_yaml.string().c_str(),
        persistent_costmap_yaml.string().c_str(),
        normalized_outcome.c_str());
    }
    if (!promote_runtime_costmap) {
      context_document["persistent_costmap_promoted"] = false;
      context_document["persistent_costmap_promotion_skip_reason"] =
        "mission_outcome_not_completed";
      return;
    }
    saveCostmapArtifacts(merged_costmap, persistent_costmap_image, persistent_costmap_yaml);
    context_document["persistent_mission_costmap_yaml"] = persistent_costmap_yaml.string();
    context_document["persistent_mission_costmap_image"] = persistent_costmap_image.string();
    context_document["persistent_costmap_merge_mode"] = "cell_average";
    context_document["persistent_costmap_promoted"] = true;
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to promote runtime costmap artifact from %s into %s: %s",
      runtime_costmap_yaml.string().c_str(),
      persistent_costmap_yaml.string().c_str(),
      exception.what());
    context_document["persistent_costmap_promoted"] = false;
    context_document["persistent_costmap_promotion_error"] = exception.what();
  }
}

void MissionExecutorNode::updateRecordMapArtifacts(nlohmann::json & context_document) const
{
  const std::string execution_mode = toLower(context_document.value("execution_mode", std::string{}));
  const std::string mission_type = toLower(context_document.value("mission_type", std::string{}));
  if (execution_mode != kManualMappingExecutionMode && mission_type != kBuiltinManualMappingMissionType) {
    return;
  }

  const std::filesystem::path actual_path_file(
    context_document.value("actual_path_file", std::string{}));
  const std::filesystem::path mission_route_file(
    context_document.value("mission_route_file", std::string{}));
  const std::filesystem::path mission_costmap_yaml(
    context_document.value("mission_costmap_yaml", std::string{}));
  const std::filesystem::path mission_folder(
    context_document.value("mission_folder", std::string{}));
  const std::filesystem::path gaussian_output_directory(
    context_document.value("gaussian_output_directory", std::string{}));
  const std::string mission_id = context_document.value("mission_id", std::string{});
  const std::filesystem::path actual_path_navsat_file(
    context_document.value("actual_path_navsat_file", std::string{}));

  if (actual_path_file.empty() || !std::filesystem::exists(actual_path_file) ||
    mission_route_file.empty() || mission_costmap_yaml.empty() || mission_folder.empty() || mission_id.empty())
  {
    return;
  }

  const auto perimeter_points = closePolygon(extractLineStringCoordinates(loadJsonDocument(actual_path_file)));
  if (perimeter_points.size() < 4U) {
    return;
  }

  std::vector<MapPoint> obstacle_points;
  if (!gaussian_output_directory.empty()) {
    const std::string run_started_at = context_document.value("run_started_at", std::string{});
    const std::string gaussian_stem = missionRunArtifactStem(mission_id, run_started_at);
    const auto gaussian_json_path = gaussian_output_directory / (gaussian_stem + ".json");
    obstacle_points = loadGaussianObstaclePoints(gaussian_json_path);
  }

  const RasterizedCostmap map = buildRecordMapCostmap(perimeter_points, obstacle_points);
  RasterizedCostmap georeferenced_map = map;
  if (!actual_path_navsat_file.empty() && std::filesystem::exists(actual_path_navsat_file)) {
    const auto local_trace = extractLineStringCoordinates(loadJsonDocument(actual_path_file));
    const auto geo_trace = extractGeoLineStringCoordinates(loadJsonDocument(actual_path_navsat_file));
    const auto georeference = buildGeoReferenceMetadata(
      local_trace,
      geo_trace,
      actual_path_navsat_file.filename().string());
    if (georeference.has_value()) {
      georeferenced_map.georeference_valid = true;
      georeferenced_map.georeference_type = georeference->value("type", std::string("affine_xy_to_wgs84"));
      georeferenced_map.georeference_source_crs = "EPSG:4326";
      georeferenced_map.georeference_companion_file =
        georeference->value("companion_file", std::string{});
      georeferenced_map.georeference_sample_count =
        georeference->value("sample_count", static_cast<std::size_t>(0U));
      const auto & longitude_coefficients = georeference->at("longitude_coefficients");
      const auto & latitude_coefficients = georeference->at("latitude_coefficients");
      georeferenced_map.longitude_coefficients = {
        longitude_coefficients.at(0).get<double>(),
        longitude_coefficients.at(1).get<double>(),
        longitude_coefficients.at(2).get<double>()};
      georeferenced_map.latitude_coefficients = {
        latitude_coefficients.at(0).get<double>(),
        latitude_coefficients.at(1).get<double>(),
        latitude_coefficients.at(2).get<double>()};
    }
  }
  const std::filesystem::path mission_costmap_image = mission_costmap_yaml.parent_path() /
    (mission_costmap_yaml.stem().string() + ".pgm");
  saveCostmapArtifacts(georeferenced_map, mission_costmap_image, mission_costmap_yaml);

  {
    std::ofstream route_stream(mission_route_file, std::ios::trunc);
    if (!route_stream.is_open()) {
      throw std::runtime_error("Failed to write RecordMap perimeter route artifact");
    }
    route_stream << std::setw(2) << buildPerimeterGeoJson(perimeter_points) << '\n';
  }

  const std::filesystem::path history_route_file = mission_folder / mission_route_file.filename();
  if (history_route_file != mission_route_file) {
    std::ofstream route_stream(history_route_file, std::ios::trunc);
    if (!route_stream.is_open()) {
      throw std::runtime_error("Failed to write RecordMap history perimeter route artifact");
    }
    route_stream << std::setw(2) << buildPerimeterGeoJson(perimeter_points) << '\n';
  }

  const std::filesystem::path history_costmap_yaml = mission_folder / mission_costmap_yaml.filename();
  const std::filesystem::path history_costmap_image = history_costmap_yaml.parent_path() /
    (history_costmap_yaml.stem().string() + ".pgm");
  if (history_costmap_yaml != mission_costmap_yaml) {
    saveCostmapArtifacts(georeferenced_map, history_costmap_image, history_costmap_yaml);
  }

  context_document["recorded_work_area_route_file"] = mission_route_file.string();
  context_document["recorded_work_area_costmap_yaml"] = mission_costmap_yaml.string();
  context_document["recorded_work_area_costmap_image"] = mission_costmap_image.string();
  context_document["recorded_obstacle_count"] = obstacle_points.size();
  nlohmann::json obstacle_points_document = nlohmann::json::array();
  for (const auto & point : obstacle_points) {
    obstacle_points_document.push_back({{"x", point.x}, {"y", point.y}});
  }
  context_document["recorded_obstacle_points"] = obstacle_points_document;
}

void MissionExecutorNode::writeLatestRecordedMapSnapshot(const nlohmann::json & context_document) const
{
  const std::string execution_mode = toLower(context_document.value("execution_mode", std::string{}));
  const std::string mission_type = toLower(context_document.value("mission_type", std::string{}));
  const std::string mission_id = context_document.value("mission_id", std::string{});
  if (execution_mode != kManualMappingExecutionMode ||
    mission_type != kBuiltinManualMappingMissionType ||
    mission_id != "RecordMap")
  {
    return;
  }

  const std::filesystem::path mission_route_file(
    context_document.value("mission_route_file", std::string{}));
  const std::filesystem::path actual_path_file(
    context_document.value("actual_path_file", std::string{}));
  const std::filesystem::path mission_costmap_yaml(
    context_document.value("mission_costmap_yaml", std::string{}));
  const std::filesystem::path mission_costmap_image(
    context_document.value("recorded_work_area_costmap_image", std::string{}));
  const std::string run_started_at = context_document.value("run_started_at", std::string{});

  if (mission_route_file.empty() || !std::filesystem::exists(mission_route_file) ||
    actual_path_file.empty() || !std::filesystem::exists(actual_path_file) ||
    mission_costmap_yaml.empty() || !std::filesystem::exists(mission_costmap_yaml) ||
    mission_costmap_image.empty() || !std::filesystem::exists(mission_costmap_image) ||
    run_started_at.empty())
  {
    return;
  }

  const auto perimeter = closePolygon(extractLineStringCoordinates(loadJsonDocument(mission_route_file)));
  if (perimeter.size() < 4U) {
    return;
  }
  const std::filesystem::path navsat_route_file(
    context_document.value("actual_path_navsat_file", std::string{}));
  GeoTransform geo_transform;
  if (!navsat_route_file.empty() && std::filesystem::exists(navsat_route_file)) {
    geo_transform = buildGeoTransform(
      extractLineStringCoordinates(loadJsonDocument(actual_path_file)),
      extractGeoLineStringCoordinates(loadJsonDocument(navsat_route_file)));
  }

  const auto latest_directory = resolveMissionsLogDirectory() / kLatestRecordedMapDirectoryName;
  const auto latest_metadata_file = latest_directory / kLatestRecordedMapMetadataFile;
  const auto latest_route_file = latest_directory / (std::string(kLatestRecordedMapRouteStem) + ".geojson");
  const auto latest_costmap_yaml_file = latest_directory / (std::string(kLatestRecordedMapCostmapStem) + ".yaml");
  const auto latest_costmap_image_file = latest_directory / (std::string(kLatestRecordedMapCostmapStem) + ".pgm");
  const auto latest_navsat_file = latest_directory / (std::string(kLatestRecordedMapNavSatStem) + ".geojson");
  std::filesystem::create_directories(latest_directory);

  {
    std::ofstream route_stream(latest_route_file, std::ios::trunc);
    if (!route_stream.is_open()) {
      throw std::runtime_error("Failed to write latest recorded map route artifact");
    }
    route_stream << std::setw(2) << buildPerimeterGeoJson(perimeter) << '\n';
  }

  std::filesystem::copy_file(
    mission_costmap_yaml,
    latest_costmap_yaml_file,
    std::filesystem::copy_options::overwrite_existing);
  std::filesystem::copy_file(
    mission_costmap_image,
    latest_costmap_image_file,
    std::filesystem::copy_options::overwrite_existing);

  rewriteCostmapYamlImageReference(latest_costmap_yaml_file, latest_costmap_image_file);

  if (!navsat_route_file.empty() && std::filesystem::exists(navsat_route_file)) {
    std::filesystem::copy_file(
      navsat_route_file,
      latest_navsat_file,
      std::filesystem::copy_options::overwrite_existing);
  }

  nlohmann::json latest_metadata{
    {"mission_id", mission_id},
    {"run_started_at", run_started_at},
    {"recorded_work_area_route_file", latest_route_file.string()},
    {"recorded_work_area_costmap_yaml", latest_costmap_yaml_file.string()},
    {"recorded_work_area_costmap_image", latest_costmap_image_file.string()},
    {"recorded_work_area_navsat_file", std::filesystem::exists(latest_navsat_file) ? latest_navsat_file.string() : std::string{}},
    {"recorded_obstacle_count", context_document.value("recorded_obstacle_count", 0)},
    {"recorded_obstacle_points", context_document.value("recorded_obstacle_points", nlohmann::json::array())},
    {"geo_transform", {
      {"valid", geo_transform.valid},
      {"longitude_coefficients", {
        geo_transform.longitude_coefficients[0],
        geo_transform.longitude_coefficients[1],
        geo_transform.longitude_coefficients[2]}},
      {"latitude_coefficients", {
        geo_transform.latitude_coefficients[0],
        geo_transform.latitude_coefficients[1],
        geo_transform.latitude_coefficients[2]}}}}};
  std::ofstream metadata_stream(latest_metadata_file, std::ios::trunc);
  if (!metadata_stream.is_open()) {
    throw std::runtime_error("Failed to write latest recorded map metadata");
  }
  metadata_stream << std::setw(2) << latest_metadata << '\n';
}

void MissionExecutorNode::refreshActiveMissionState(const nlohmann::json & context_document)
{
  std::lock_guard<std::mutex> lock(active_mission_mutex_);
  active_mission_running_ = true;
  active_mission_id_ = context_document.value("mission_id", std::string{});
  active_execution_mode_ = context_document.value("execution_mode", std::string{});
  active_execution_context_file_ = context_document.value("execution_context_file", std::string{});
  active_mission_is_teleop_ = toLower(active_execution_mode_) == kTeleoperationExecutionMode;
  active_mission_is_manual_mapping_ = toLower(active_execution_mode_) == kManualMappingExecutionMode;
  active_mission_uses_inactivity_watchdog_ = active_mission_is_teleop_ || active_mission_is_manual_mapping_;
  active_actual_path_file_ = context_document.value("actual_path_file", std::string{});
  active_actual_navsat_path_file_ = context_document.value("actual_path_navsat_file", std::string{});
  teleop_traveled_path_points_.clear();
  manual_mapping_navsat_points_.clear();
  last_manual_mission_motion_time_ = now();
}

void MissionExecutorNode::clearActiveMissionState()
{
  std::lock_guard<std::mutex> lock(active_mission_mutex_);
  active_mission_running_ = false;
  active_mission_is_teleop_ = false;
  active_mission_is_manual_mapping_ = false;
  active_mission_uses_inactivity_watchdog_ = false;
  active_mission_id_.clear();
  active_execution_mode_.clear();
  active_execution_context_file_.clear();
  active_actual_path_file_.clear();
  active_actual_navsat_path_file_.clear();
  teleop_traveled_path_points_.clear();
  manual_mapping_navsat_points_.clear();
  last_manual_mission_motion_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void MissionExecutorNode::recordMissionExecutionStart(
  const ManualMissionInfo & mission,
  const PreparedMissionContext & context,
  const srv::ExecuteMission::Request & request) const
{
  auto context_document = loadJsonDocument(context.execution_context_file);
  std::string schedule_path_string = resolveScheduleSourcePath().string();
  if (schedule_path_string.empty() &&
    context_document.contains("schedule_log_path") &&
    context_document.at("schedule_log_path").is_string())
  {
    schedule_path_string = context_document.at("schedule_log_path").get<std::string>();
  }
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();
  const std::string timezone = discoverScheduleTimezone(schedule_text);
  const auto now = std::chrono::system_clock::now();
  const std::string actual_start_utc = formatUtcTimestamp(now);
  const std::string actual_start_local = formatLocalTimestamp(now);
  std::string event_uid;

  if (!request.mission_window_start.empty()) {
    const std::string mission_tag = "X-MISSION-ID:" + mission.mission_id;
    const std::string start_tag = request.mission_window_start;
    std::string start_tag_utc;
    try {
      start_tag_utc = localTimestampToUtcTimestamp(request.mission_window_start);
    } catch (const std::exception &) {
      start_tag_utc.clear();
    }
    const auto mission_position = schedule_text.find(mission_tag);
    if (mission_position != std::string::npos) {
      const auto event_begin = schedule_text.rfind("BEGIN:VEVENT", mission_position);
      const auto event_end = schedule_text.find("END:VEVENT", mission_position);
      const auto local_start_position = schedule_text.find(start_tag, event_begin);
      const auto utc_start_position =
        start_tag_utc.empty() ? std::string::npos : schedule_text.find(start_tag_utc, event_begin);
      const bool matching_start =
        (local_start_position != std::string::npos && local_start_position < event_end) ||
        (utc_start_position != std::string::npos && utc_start_position < event_end);
      if (event_begin != std::string::npos && event_end != std::string::npos && matching_start)
      {
        const auto uid_position = schedule_text.find("UID:", event_begin);
        if (uid_position != std::string::npos && uid_position < event_end) {
          const auto uid_end = schedule_text.find('\n', uid_position);
          event_uid = schedule_text.substr(uid_position + 4, uid_end - (uid_position + 4));
        }
        const std::string runtime_line = "X-ACTUAL-START-UTC:" + actual_start_utc + "\n";
        if (schedule_text.find(runtime_line, event_begin) == std::string::npos ||
          schedule_text.find(runtime_line, event_begin) > event_end)
        {
          schedule_text.insert(event_end, runtime_line);
        }
        const std::string status_line = std::string("X-RUNTIME-STATUS:") + kRuntimeStatusStarted + "\n";
        if (schedule_text.find(status_line, event_begin) == std::string::npos ||
          schedule_text.find(status_line, event_begin) > event_end)
        {
          schedule_text.insert(event_end, status_line);
        }
      }
    }
  } else {
    event_uid = "manual-" + sanitizeUidToken(mission.mission_id) + "-" + sanitizeUidToken(actual_start_utc);
    std::ostringstream event_stream;
    event_stream
      << "BEGIN:VEVENT\n"
      << "UID:" << event_uid << "\n"
      << "DTSTART;TZID=" << timezone << ":" << actual_start_local << "\n"
      << "DURATION:PT0S\n"
      << "SUMMARY:Manual mission execution " << mission.mission_id << "\n"
      << "X-ROBOT-ID:" << robot_id_ << "\n"
      << "X-SCHEDULE-TYPE:WORK\n"
      << "X-MISSION-ID:" << mission.mission_id << "\n"
      << "X-ACTUAL-START-UTC:" << actual_start_utc << "\n"
      << "X-RUNTIME-STATUS:" << kRuntimeStatusStarted << "\n"
      << "END:VEVENT\n";

    const auto calendar_end = schedule_text.rfind("END:VCALENDAR");
    if (calendar_end == std::string::npos) {
      return;
    }
    schedule_text.insert(calendar_end, event_stream.str());
  }

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;

  const std::filesystem::path actual_schedule_path =
    ensureActualScheduleLogPath(resolveScheduleSourcePath());
  if (!actual_schedule_path.empty() && std::filesystem::exists(actual_schedule_path)) {
    std::ifstream actual_input_stream(actual_schedule_path);
    if (actual_input_stream.is_open()) {
      std::ostringstream actual_buffer;
      actual_buffer << actual_input_stream.rdbuf();
      std::string actual_schedule_text = actual_buffer.str();
      const std::string actual_timezone = discoverScheduleTimezone(actual_schedule_text);
      const std::string actual_event_uid = "actual-" + sanitizeUidToken(event_uid.empty() ? mission.mission_id + "-" + actual_start_utc : event_uid);
      const auto existing_anchor = actual_schedule_text.find("UID:" + actual_event_uid);
      if (existing_anchor == std::string::npos) {
        std::ostringstream actual_event_stream;
        actual_event_stream
          << "BEGIN:VEVENT\n"
          << "UID:" << actual_event_uid << "\n"
          << "DTSTART;TZID=" << actual_timezone << ":" << actual_start_local << "\n"
          << "DURATION:PT0S\n"
          << "SUMMARY:Actual mission run " << mission.mission_id << "\n"
          << "X-ROBOT-ID:" << robot_id_ << "\n"
          << "X-SCHEDULE-TYPE:WORK\n"
          << "X-MISSION-ID:" << mission.mission_id << "\n"
          << "X-ACTUAL-START-UTC:" << actual_start_utc << "\n"
          << "X-RUNTIME-STATUS:" << kRuntimeStatusStarted << "\n"
          << "END:VEVENT\n";
        const auto calendar_end = actual_schedule_text.rfind("END:VCALENDAR");
        if (calendar_end != std::string::npos) {
          actual_schedule_text.insert(calendar_end, actual_event_stream.str());
          std::ofstream actual_output_stream(actual_schedule_path, std::ios::trunc);
          if (actual_output_stream.is_open()) {
            actual_output_stream << actual_schedule_text;
          }
          context_document["actual_schedule_event_uid"] = actual_event_uid;
          context_document["actual_schedule_log_path"] = actual_schedule_path.string();
        }
      } else {
        context_document["actual_schedule_event_uid"] = actual_event_uid;
        context_document["actual_schedule_log_path"] = actual_schedule_path.string();
      }
    }
  }

  context_document["schedule_event_uid"] = event_uid;
  context_document["schedule_log_path"] = schedule_path.string();
  context_document["actual_start_utc"] = actual_start_utc;
  context_document["runtime_status"] = kRuntimeStatusStarted;
  try {
    writeJsonDocumentAtomic(context.execution_context_file, context_document);
  } catch (const std::exception &) {
    return;
  }
}

void MissionExecutorNode::recordMissionExecutionEnd(
  nlohmann::json & context_document,
  const srv::EndMission::Request & request) const
{
  const auto now = std::chrono::system_clock::now();
  const std::string actual_end_utc = formatUtcTimestamp(now);
  const std::string normalized_outcome = toLower(defaultIfEmpty(request.outcome, "completed"));
  const std::string runtime_status =
    normalized_outcome == "completed" ? kRuntimeStatusCompleted : kRuntimeStatusAborted;

  double actual_duration_seconds = 0.0;
  if (context_document.contains("run_started_at") && context_document.at("run_started_at").is_string()) {
    const auto started = parseUtcTimestamp(context_document.at("run_started_at").get<std::string>());
    actual_duration_seconds = std::chrono::duration<double>(now - started).count();
  }

  double actual_path_length_meters = 0.0;
  if (context_document.contains("actual_path_file") && context_document.at("actual_path_file").is_string()) {
    const std::filesystem::path actual_path_file = context_document.at("actual_path_file").get<std::string>();
    if (std::filesystem::exists(actual_path_file)) {
      actual_path_length_meters = computePathLengthMeters(loadJsonDocument(actual_path_file));
    }
  }

  context_document["actual_end_utc"] = actual_end_utc;
  context_document["runtime_status"] = runtime_status;
  context_document["mission_outcome"] = normalized_outcome;
  context_document["end_reason"] = request.reason;
  context_document["actual_duration_seconds"] = actual_duration_seconds;
  context_document["actual_path_length_meters"] = actual_path_length_meters;

  const std::filesystem::path context_path(
    context_document.value("execution_context_file", std::string{}));
  if (!context_path.empty()) {
    writeJsonDocumentAtomic(context_path, context_document);
  }

  std::string schedule_path_string = resolveScheduleSourcePath().string();
  if (schedule_path_string.empty()) {
    schedule_path_string = context_document.value("schedule_log_path", std::string{});
  }
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();

  const std::string event_uid = context_document.value("schedule_event_uid", std::string{});
  const std::string mission_id = context_document.value("mission_id", std::string{});
  const std::string mission_window_start = context_document.value("mission_window_start", std::string{});
  const auto event_anchor = !event_uid.empty() ? schedule_text.find("UID:" + event_uid) : std::string::npos;
  std::size_t event_begin = std::string::npos;
  std::size_t event_end = std::string::npos;
  if (event_anchor != std::string::npos) {
    event_begin = schedule_text.rfind("BEGIN:VEVENT", event_anchor);
    event_end = schedule_text.find("END:VEVENT", event_anchor);
  } else if (!mission_id.empty()) {
    const auto mission_anchor = schedule_text.find("X-MISSION-ID:" + mission_id);
    if (mission_anchor != std::string::npos) {
      event_begin = schedule_text.rfind("BEGIN:VEVENT", mission_anchor);
      event_end = schedule_text.find("END:VEVENT", mission_anchor);
      if (event_begin != std::string::npos && !mission_window_start.empty()) {
        auto local_start_anchor = schedule_text.find(mission_window_start, event_begin);
        std::size_t utc_start_anchor = std::string::npos;
        try {
          const std::string mission_window_start_utc = localTimestampToUtcTimestamp(mission_window_start);
          utc_start_anchor = schedule_text.find(mission_window_start_utc, event_begin);
        } catch (const std::exception &) {
          utc_start_anchor = std::string::npos;
        }
        const bool matching_start =
          (local_start_anchor != std::string::npos && local_start_anchor < event_end) ||
          (utc_start_anchor != std::string::npos && utc_start_anchor < event_end);
        if (!matching_start) {
          event_begin = std::string::npos;
          event_end = std::string::npos;
        }
      }
    }
  }

  if (event_begin == std::string::npos || event_end == std::string::npos) {
    return;
  }

  const auto insert_or_replace_line = [&schedule_text, event_begin, event_end](
      const std::string & prefix, const std::string & line) {
      const auto position = schedule_text.find(prefix, event_begin);
      if (position != std::string::npos && position < event_end) {
        const auto line_end = schedule_text.find('\n', position);
        schedule_text.replace(position, (line_end == std::string::npos ? event_end : line_end + 1) - position, line);
      } else {
        schedule_text.insert(event_end, line);
      }
    };

  insert_or_replace_line("X-ACTUAL-END-UTC:", "X-ACTUAL-END-UTC:" + actual_end_utc + "\n");
  insert_or_replace_line(
    "X-ACTUAL-DURATION-SECONDS:",
    "X-ACTUAL-DURATION-SECONDS:" + std::to_string(static_cast<long long>(actual_duration_seconds)) + "\n");
  insert_or_replace_line(
    "X-ACTUAL-PATH-LENGTH-METERS:",
    "X-ACTUAL-PATH-LENGTH-METERS:" + std::to_string(actual_path_length_meters) + "\n");
  insert_or_replace_line("X-RUNTIME-STATUS:", "X-RUNTIME-STATUS:" + runtime_status + "\n");
  insert_or_replace_line("X-END-REASON:", "X-END-REASON:" + request.reason + "\n");

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;

  std::string actual_schedule_path_string =
    context_document.value("actual_schedule_log_path", std::string{});
  if (actual_schedule_path_string.empty()) {
    actual_schedule_path_string = ensureActualScheduleLogPath(resolveScheduleSourcePath()).string();
  }
  if (!actual_schedule_path_string.empty()) {
    const std::filesystem::path actual_schedule_path(actual_schedule_path_string);
    if (std::filesystem::exists(actual_schedule_path)) {
      std::ifstream actual_input_stream(actual_schedule_path);
      if (actual_input_stream.is_open()) {
        std::ostringstream actual_buffer;
        actual_buffer << actual_input_stream.rdbuf();
        std::string actual_schedule_text = actual_buffer.str();
        const std::string actual_event_uid =
          context_document.value("actual_schedule_event_uid", std::string{});
        const auto actual_anchor = !actual_event_uid.empty() ?
          actual_schedule_text.find("UID:" + actual_event_uid) : std::string::npos;
        if (actual_anchor != std::string::npos) {
          const auto actual_begin = actual_schedule_text.rfind("BEGIN:VEVENT", actual_anchor);
          const auto actual_end = actual_schedule_text.find("END:VEVENT", actual_anchor);
          if (actual_begin != std::string::npos && actual_end != std::string::npos) {
            const auto insert_or_replace_actual_line =
              [&actual_schedule_text, actual_begin, actual_end](
                const std::string & prefix,
                const std::string & line)
              {
                const auto position = actual_schedule_text.find(prefix, actual_begin);
                if (position != std::string::npos && position < actual_end) {
                  const auto line_end = actual_schedule_text.find('\n', position);
                  actual_schedule_text.replace(
                    position,
                    (line_end == std::string::npos ? actual_end : line_end + 1) - position,
                    line);
                } else {
                  actual_schedule_text.insert(actual_end, line);
                }
              };
            insert_or_replace_actual_line(
              "DTEND;TZID=",
              "DTEND;TZID=" + discoverScheduleTimezone(actual_schedule_text) + ":" + formatLocalTimestamp(now) + "\n");
            insert_or_replace_actual_line(
              "X-ACTUAL-END-UTC:",
              "X-ACTUAL-END-UTC:" + actual_end_utc + "\n");
            insert_or_replace_actual_line(
              "X-ACTUAL-DURATION-SECONDS:",
              "X-ACTUAL-DURATION-SECONDS:" + std::to_string(static_cast<long long>(actual_duration_seconds)) + "\n");
            insert_or_replace_actual_line(
              "X-ACTUAL-PATH-LENGTH-METERS:",
              "X-ACTUAL-PATH-LENGTH-METERS:" + std::to_string(actual_path_length_meters) + "\n");
            insert_or_replace_actual_line(
              "X-RUNTIME-STATUS:",
              "X-RUNTIME-STATUS:" + runtime_status + "\n");
            insert_or_replace_actual_line(
              "X-END-REASON:",
              "X-END-REASON:" + request.reason + "\n");
            std::ofstream actual_output_stream(actual_schedule_path, std::ios::trunc);
            if (actual_output_stream.is_open()) {
              actual_output_stream << actual_schedule_text;
            }
          }
        }
      }
    }
  }
}

void MissionExecutorNode::recordSafetyEvent(
  const amr_sweeper_safety_msgs::msg::SafetyStop & event,
  const std::optional<nlohmann::json> & context_document) const
{
  std::string schedule_path_string = resolveScheduleSourcePath().string();
  std::string related_mission_id;
  std::string mission_run_directory;
  if (context_document) {
    if (schedule_path_string.empty()) {
      schedule_path_string = context_document->value("schedule_log_path", std::string{});
    }
    related_mission_id = context_document->value("mission_id", std::string{});
    mission_run_directory = context_document->value("mission_run_directory", std::string{});
  }
  if (schedule_path_string.empty()) {
    return;
  }

  const std::filesystem::path schedule_path(schedule_path_string);
  if (!std::filesystem::exists(schedule_path)) {
    return;
  }

  std::ifstream input_stream(schedule_path);
  if (!input_stream.is_open()) {
    return;
  }
  std::ostringstream buffer;
  buffer << input_stream.rdbuf();
  std::string schedule_text = buffer.str();
  const std::string timezone = discoverScheduleTimezone(schedule_text);

  rclcpp::Time event_time(event.stamp);
  const auto time_point = std::chrono::system_clock::time_point(std::chrono::nanoseconds(event_time.nanoseconds()));
  const std::string event_utc = formatUtcTimestamp(time_point);
  const std::string event_local = formatLocalTimestamp(time_point);
  std::ostringstream debug_description;
  debug_description
    << "sender=" << event.sender
    << "; reason=" << event.reason;
  if (!related_mission_id.empty()) {
    debug_description << "; mission_id=" << related_mission_id;
  }
  if (!mission_run_directory.empty()) {
    debug_description << "; mission_run_directory=" << mission_run_directory;
  }
  debug_description << "; recorded_by=mission_executor";
  const std::string escaped_debug_description = escapeIcsText(debug_description.str());
  const std::string escaped_reason = escapeIcsText(event.reason);

  std::ostringstream event_stream;
  event_stream
    << "BEGIN:VEVENT\n"
    << "UID:safety-" << sanitizeUidToken(event.sender) << "-" << sanitizeUidToken(event_utc) << "\n"
    << "DTSTART;TZID=" << timezone << ":" << event_local << "\n"
    << "DURATION:PT0S\n"
    << "SUMMARY:Safety stop " << event.sender << "\n"
    << "DESCRIPTION:" << escaped_debug_description << "\n"
    << "X-ROBOT-ID:" << robot_id_ << "\n"
    << "X-SCHEDULE-TYPE:" << kSafetyScheduleType << "\n"
    << "X-SAFETY-SENDER:" << event.sender << "\n"
    << "X-SAFETY-REASON:" << escaped_reason << "\n"
    << "X-SAFETY-DEBUG:" << escaped_debug_description << "\n"
    << "X-ACTUAL-START-UTC:" << event_utc << "\n";
  if (!related_mission_id.empty()) {
    event_stream << "X-MISSION-ID:" << related_mission_id << "\n";
  }
  if (!mission_run_directory.empty()) {
    event_stream << "X-MISSION-RUN-DIRECTORY:" << mission_run_directory << "\n";
  }
  event_stream << "END:VEVENT\n";

  const auto calendar_end = schedule_text.rfind("END:VCALENDAR");
  if (calendar_end == std::string::npos) {
    return;
  }
  schedule_text.insert(calendar_end, event_stream.str());

  std::ofstream output_stream(schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return;
  }
  output_stream << schedule_text;

  const std::filesystem::path actual_schedule_path =
    ensureActualScheduleLogPath(resolveScheduleSourcePath());
  if (!actual_schedule_path.empty() && std::filesystem::exists(actual_schedule_path)) {
    std::ifstream actual_input_stream(actual_schedule_path);
    if (actual_input_stream.is_open()) {
      std::ostringstream actual_buffer;
      actual_buffer << actual_input_stream.rdbuf();
      std::string actual_schedule_text = actual_buffer.str();
      const std::string actual_timezone = discoverScheduleTimezone(actual_schedule_text);
      std::ostringstream actual_event_stream;
      actual_event_stream
        << "BEGIN:VEVENT\n"
        << "UID:actual-safety-" << sanitizeUidToken(event.sender) << "-" << sanitizeUidToken(event_utc) << "\n"
        << "DTSTART;TZID=" << actual_timezone << ":" << event_local << "\n"
        << "DURATION:PT0S\n"
        << "SUMMARY:Actual safety stop " << event.sender << "\n"
        << "DESCRIPTION:" << escaped_debug_description << "\n"
        << "X-ROBOT-ID:" << robot_id_ << "\n"
        << "X-SCHEDULE-TYPE:" << kSafetyScheduleType << "\n"
        << "X-SAFETY-SENDER:" << event.sender << "\n"
        << "X-SAFETY-REASON:" << escaped_reason << "\n"
        << "X-SAFETY-DEBUG:" << escaped_debug_description << "\n"
        << "X-ACTUAL-START-UTC:" << event_utc << "\n";
      if (!related_mission_id.empty()) {
        actual_event_stream << "X-MISSION-ID:" << related_mission_id << "\n";
      }
      if (!mission_run_directory.empty()) {
        actual_event_stream << "X-MISSION-RUN-DIRECTORY:" << mission_run_directory << "\n";
      }
      actual_event_stream << "END:VEVENT\n";

      const auto calendar_end = actual_schedule_text.rfind("END:VCALENDAR");
      if (calendar_end != std::string::npos) {
        actual_schedule_text.insert(calendar_end, actual_event_stream.str());
        std::ofstream actual_output_stream(actual_schedule_path, std::ios::trunc);
        if (actual_output_stream.is_open()) {
          actual_output_stream << actual_schedule_text;
        }
      }
    }
  }
}

bool MissionExecutorNode::missionArtifactsReady(const ManualMissionInfo & mission) const
{
  const ManualMissionInfo executable_mission = resolveExecutableMissionSource(mission);
  const std::filesystem::path mission_file(executable_mission.mission_path);
  const std::filesystem::path mission_folder = artifactsDirectoryForMission(executable_mission);
  return std::filesystem::exists(mission_file) &&
         std::filesystem::exists(mission_folder / (missionCostmapBasename(mission_file) + ".yaml")) &&
         std::filesystem::exists(mission_folder / (missionCostmapBasename(mission_file) + ".pgm")) &&
         std::filesystem::exists(resolveMissionRoutePath(executable_mission, mission_file));
}

bool MissionExecutorNode::ensureMissionArtifactsReady(
  const ManualMissionInfo & mission,
  const std::string & requester,
  const std::string & reason)
{
  if (missionArtifactsReady(mission)) {
    return true;
  }
  if (toLower(mission.mission_type) != kScheduledMissionType) {
    return false;
  }
  if (!mission_parser_parameter_client_->service_is_ready() ||
    !mission_parser_build_client_->service_is_ready())
  {
    RCLCPP_WARN(
      get_logger(),
      "VDA5050 mission artifacts are missing for %s but the mission builder is unavailable.",
      mission.mission_id.c_str());
    return false;
  }

  auto parameter_future = mission_parser_parameter_client_->set_parameters(
    {rclcpp::Parameter("mission_path", mission.mission_path)});
  if (parameter_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(),
      "Timed out setting mission_path on the VDA5050 mission builder for %s; requester=%s; reason=%s",
      mission.mission_id.c_str(),
      requester.c_str(),
      reason.c_str());
    return false;
  }

  for (const auto & result : parameter_future.get()) {
    if (!result.successful) {
      RCLCPP_WARN(
        get_logger(),
        "VDA5050 mission builder rejected mission_path for %s: %s",
        mission.mission_id.c_str(),
        result.reason.c_str());
      return false;
    }
  }

  auto build_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto build_future = mission_parser_build_client_->async_send_request(build_request);
  if (build_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(),
      "Timed out waiting for the VDA5050 mission builder while preparing %s",
      mission.mission_id.c_str());
    return false;
  }

  const auto build_response = build_future.get();
  if (!build_response->success) {
    RCLCPP_WARN(
      get_logger(),
      "VDA5050 mission builder failed for %s: %s",
      mission.mission_id.c_str(),
      build_response->message.c_str());
    return false;
  }

  if (missionArtifactsReady(mission)) {
    return true;
  }

  const auto staged_directory = newestScheduledArtifactDirectory(mission.mission_id);
  if (staged_directory) {
    RCLCPP_WARN(
      get_logger(),
      "Mission builder completed for %s, but ready artifacts still did not resolve from newest staged "
      "folder %s. Scheduler may be referencing a base orderId while parser outputs timestamped mission "
      "stems.",
      mission.mission_id.c_str(),
      staged_directory->filename().string().c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Mission builder completed for %s, but no staged artifact folder matching %s or %s_<timestamp> "
      "was found under %s.",
      mission.mission_id.c_str(),
      mission.mission_id.c_str(),
      mission.mission_id.c_str(),
      resolveMissionsLogDirectory().string().c_str());
  }
  return false;
}

bool MissionExecutorNode::requestRunningState(
  const PreparedMissionContext & context,
  const srv::ExecuteMission::Request & request,
  std::string & message) const
{
  if (!fsm_request_client_->wait_for_service(std::chrono::seconds(5))) {
    message = "FSM request_state service is unavailable";
    return false;
  }

  auto fsm_request = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  fsm_request->target_state = "RUNNING";
  fsm_request->target_lifecycle = "Active";
  fsm_request->target_profile_id = context.running_profile_id;
  fsm_request->requester = defaultIfEmpty(request.requester, "mission_executor");
  fsm_request->priority = request.priority == 0U ? default_activation_priority_ : request.priority;
  fsm_request->force = request.force;
  fsm_request->reason = defaultIfEmpty(request.reason, "manual mission activation");
  fsm_request->mission_execution_directory = context.mission_execution_directory;

  auto future = fsm_request_client_->async_send_request(fsm_request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    message = "Timed out waiting for FSM RUNNING request response";
    return false;
  }

  const auto response = future.get();
  if (!response->accepted) {
    message = response->message;
    return false;
  }

  std::ostringstream stream;
  stream
    << "Requested FSM RUNNING profile " << context.running_profile_id
    << " for mission_id=" << request.mission_id;
  if (!response->message.empty()) {
    stream << " (" << response->message << ")";
  }
  message = stream.str();
  return true;
}

void MissionExecutorNode::writeMissionExecutionPreferences(
  const std::filesystem::path & context_path,
  const bool record_rosbag) const
{
  if (context_path.empty()) {
    return;
  }

  auto context_document = loadJsonDocument(context_path);
  context_document["record_rosbag"] = record_rosbag;
  if (!context_document.contains("rosbag_output_directory")) {
    context_document["rosbag_output_directory"] = "";
  }
  if (!context_document.contains("rosbag_recording_started")) {
    context_document["rosbag_recording_started"] = false;
  }
  if (!context_document.contains("rosbag_log_file")) {
    context_document["rosbag_log_file"] = "";
  }
  if (!context_document.contains("rosbag_config_snapshot_directory")) {
    context_document["rosbag_config_snapshot_directory"] = "";
  }
  if (!context_document.contains("rosbag_runtime_qos_overrides_file")) {
    context_document["rosbag_runtime_qos_overrides_file"] = "";
  }
  if (!context_document.contains("rosbag_metadata_present")) {
    context_document["rosbag_metadata_present"] = false;
  }
  if (!context_document.contains("rosbag_shutdown_clean")) {
    context_document["rosbag_shutdown_clean"] = false;
  }
  if (!context_document.contains("rosbag_shutdown_reason")) {
    context_document["rosbag_shutdown_reason"] = "";
  }
  writeJsonDocumentAtomic(context_path, context_document);
}

std::vector<std::string> MissionExecutorNode::loadRosbagTopics() const
{
  const auto topics_path = resolvePath(rosbag_topics_file_);
  if (topics_path.empty() || !std::filesystem::exists(topics_path)) {
    return {};
  }

  std::ifstream input_stream(topics_path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open rosbag topics file: " + topics_path.string());
  }

  std::vector<std::string> topics;
  std::string line;
  while (std::getline(input_stream, line)) {
    const std::string trimmed = trimCopy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }
    if (trimmed == "topics:" || trimmed.rfind("topics:", 0U) == 0U) {
      continue;
    }
    if (trimmed.rfind("- ", 0U) != 0U) {
      continue;
    }
    const std::string topic = trimCopy(trimmed.substr(2));
    if (!topic.empty() && topic[0] == '/') {
      topics.push_back(topic);
    }
  }

  return topics;
}

bool MissionExecutorNode::startMissionRosbagRecording(
  const PreparedMissionContext & context,
  const bool record_rosbag_requested,
  std::string & warning_message)
{
  warning_message.clear();
  stopMissionRosbagRecording();

  const std::filesystem::path context_path(context.execution_context_file);
  auto context_document = loadJsonDocument(context_path);
  context_document["record_rosbag"] = record_rosbag_requested;
  context_document["rosbag_recording_started"] = false;
  context_document["rosbag_output_directory"] = "";
  context_document["rosbag_topics_file"] = rosbag_topics_file_;
  context_document["rosbag_log_file"] = "";
  context_document["rosbag_config_snapshot_directory"] = "";
  context_document["rosbag_runtime_qos_overrides_file"] = "";
  context_document["rosbag_metadata_present"] = false;
  context_document["rosbag_shutdown_clean"] = false;
  context_document["rosbag_shutdown_reason"] = "";
  context_document["rosbag_storage_id"] = "mcap";

  if (!record_rosbag_requested) {
    writeJsonDocumentAtomic(context_path, context_document);
    return true;
  }

  std::filesystem::path artifacts_directory(
    context_document.value("collected_artifacts_directory", std::string{}));
  if (artifacts_directory.empty()) {
    artifacts_directory = std::filesystem::path(context.mission_execution_directory) / "artifacts";
  }
  if (use_simulation_) {
    const std::filesystem::path rosbag_root_directory = resolveRosbagDirectory();
    const std::string mission_id = context_document.value("mission_id", std::string{});
    const std::string run_started_at = context_document.value("run_started_at", std::string{});
    if (!mission_id.empty() && !run_started_at.empty()) {
      artifacts_directory = rosbag_root_directory / mission_id / run_started_at / "artifacts";
    }
  }
  std::filesystem::create_directories(artifacts_directory);
  const std::string mission_id = context_document.value("mission_id", std::string{});
  const std::string run_started_at = context_document.value("run_started_at", std::string{});
  const std::string rosbag_basename = missionRunArtifactStem(mission_id, run_started_at) + "_rosbag";
  const std::filesystem::path rosbag_artifacts_directory = artifacts_directory / "rosbag";
  std::filesystem::create_directories(rosbag_artifacts_directory);
  const std::filesystem::path rosbag_output_directory = rosbag_artifacts_directory / rosbag_basename;
  const std::filesystem::path rosbag_log_file = rosbag_artifacts_directory /
    (rosbag_basename + "_recorder.log");
  const std::filesystem::path rosbag_config_snapshot_directory = rosbag_artifacts_directory /
    (rosbag_basename + "_config");
  const std::filesystem::path resolved_topics_path = resolvePath(rosbag_topics_file_);

  const auto topics = loadRosbagTopics();
  if (topics.empty()) {
    warning_message = "Record rosbag requested but no topics are configured";
    writeJsonDocumentAtomic(context_path, context_document);
    RCLCPP_WARN(get_logger(), "%s", warning_message.c_str());
    return false;
  }

  copyFileIfExists(
    resolved_topics_path,
    rosbag_config_snapshot_directory / resolved_topics_path.filename());
  try {
    const auto localization_share =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("amr_sweeper_localization"));
    copyFileIfExists(
      localization_share / "config" / "amr_sweeper_localization.yaml",
      rosbag_config_snapshot_directory / "amr_sweeper_localization.yaml");
  } catch (const std::exception &) {
  }

  std::filesystem::path rosbag_runtime_qos_overrides_path;
  try {
    rosbag_runtime_qos_overrides_path =
      writeRosbagRuntimeQosOverridesFile(rosbag_config_snapshot_directory);
  } catch (const std::exception & exception) {
    warning_message = exception.what();
    context_document["rosbag_shutdown_reason"] = warning_message;
    writeJsonDocumentAtomic(context_path, context_document);
    RCLCPP_WARN(get_logger(), "%s", warning_message.c_str());
    return false;
  }
  try {
    const auto mapping_share =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("amr_sweeper_mapping"));
    copyFileIfExists(
      mapping_share / "config" / "map_pose.yaml",
      rosbag_config_snapshot_directory / "map_pose.yaml");
    copyFileIfExists(
      mapping_share / "config" / "mapping.yaml",
      rosbag_config_snapshot_directory / "mapping.yaml");
  } catch (const std::exception &) {
  }
  try {
    const auto navigation_share =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("amr_sweeper_navigation"));
    copyFileIfExists(
      navigation_share / "config" / "default_missions_navigation.yaml",
      rosbag_config_snapshot_directory / "default_missions_navigation.yaml");
    copyFileIfExists(
      navigation_share / "config" / "manual_missions_navigation.yaml",
      rosbag_config_snapshot_directory / "manual_missions_navigation.yaml");
    copyFileIfExists(
      navigation_share / "config" / "programmed_missions_navigation.yaml",
      rosbag_config_snapshot_directory / "programmed_missions_navigation.yaml");
  } catch (const std::exception &) {
  }

  std::vector<std::string> escaped_topics;
  escaped_topics.reserve(topics.size());
  for (const auto & topic : topics) {
    escaped_topics.push_back(topic);
  }
  std::ostringstream regex_stream;
  regex_stream << "^(";
  for (std::size_t index = 0; index < escaped_topics.size(); ++index) {
    if (index > 0U) {
      regex_stream << "|";
    }
    for (const char character : escaped_topics[index]) {
      switch (character) {
        case '.':
        case '^':
        case '$':
        case '|':
        case '(':
        case ')':
        case '[':
        case ']':
        case '{':
        case '}':
        case '*':
        case '+':
        case '?':
        case '\\':
          regex_stream << '\\';
          break;
        default:
          break;
      }
      regex_stream << character;
    }
  }
  regex_stream << ")$";

  const std::string rosbag_output_string = rosbag_output_directory.string();
  const std::string rosbag_log_file_string = rosbag_log_file.string();
  const std::string rosbag_regex = regex_stream.str();
  const std::string rosbag_runtime_qos_overrides_string =
    rosbag_runtime_qos_overrides_path.string();
  const pid_t child_pid = fork();
  if (child_pid < 0) {
    warning_message = "Failed to start rosbag recorder process";
    writeJsonDocumentAtomic(context_path, context_document);
    RCLCPP_WARN(get_logger(), "%s", warning_message.c_str());
    return false;
  }

  if (child_pid == 0) {
    ::setsid();
    const int log_fd = ::open(
      rosbag_log_file_string.c_str(),
      O_CREAT | O_WRONLY | O_TRUNC,
      0644);
    if (log_fd >= 0) {
      (void)::dup2(log_fd, STDOUT_FILENO);
      (void)::dup2(log_fd, STDERR_FILENO);
      (void)::close(log_fd);
    }

    std::vector<char *> arguments{
      const_cast<char *>("ros2"),
      const_cast<char *>("bag"),
      const_cast<char *>("record"),
      const_cast<char *>("--storage"),
      const_cast<char *>("mcap"),
      const_cast<char *>("--regex"),
      const_cast<char *>(rosbag_regex.c_str()),
      const_cast<char *>("--qos-profile-overrides-path"),
      const_cast<char *>(rosbag_runtime_qos_overrides_string.c_str()),
      const_cast<char *>("--storage-preset-profile"),
      const_cast<char *>("zstd_fast"),
    };
    arguments.push_back(const_cast<char *>("-o"));
    arguments.push_back(const_cast<char *>(rosbag_output_string.c_str()));
    arguments.push_back(nullptr);
    ::execvp("ros2", arguments.data());
    _exit(127);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  int status = 0;
  if (::waitpid(child_pid, &status, WNOHANG) == child_pid) {
    warning_message = "Rosbag recorder exited immediately; see " + rosbag_log_file_string;
    context_document["rosbag_log_file"] = rosbag_log_file_string;
    context_document["rosbag_shutdown_reason"] = warning_message;
    writeJsonDocumentAtomic(context_path, context_document);
    RCLCPP_WARN(get_logger(), "%s", warning_message.c_str());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(rosbag_process_mutex_);
    rosbag_recording_pid_ = child_pid;
    active_rosbag_output_directory_ = rosbag_output_string;
    active_rosbag_context_file_ = context.execution_context_file;
    active_rosbag_log_file_ = rosbag_log_file_string;
  }

  context_document["rosbag_recording_started"] = true;
  context_document["rosbag_output_directory"] = rosbag_output_string;
  context_document["rosbag_log_file"] = rosbag_log_file_string;
  context_document["rosbag_topics_file"] = resolved_topics_path.string();
  context_document["rosbag_config_snapshot_directory"] = rosbag_config_snapshot_directory.string();
  context_document["rosbag_runtime_qos_overrides_file"] = rosbag_runtime_qos_overrides_string;
  writeJsonDocumentAtomic(context_path, context_document);
  RCLCPP_INFO(get_logger(), "Started rosbag recording under %s", rosbag_output_string.c_str());
  return true;
}

void MissionExecutorNode::stopMissionRosbagRecording()
{
  auto process_group_alive = [](pid_t pgid) {
      if (pgid <= 0) {
        return false;
      }
      if (::kill(-pgid, 0) == 0) {
        return true;
      }
      return errno == EPERM;
    };

  auto wait_process_group_dead = [&process_group_alive](pid_t pgid, std::chrono::milliseconds timeout) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        int status = 0;
        (void)::waitpid(pgid, &status, WNOHANG);
        if (!process_group_alive(pgid)) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return !process_group_alive(pgid);
    };

  pid_t child_pid = -1;
  std::string rosbag_output_directory;
  std::string rosbag_context_file;
  std::string rosbag_log_file;
  {
    std::lock_guard<std::mutex> lock(rosbag_process_mutex_);
    child_pid = rosbag_recording_pid_;
    rosbag_output_directory = active_rosbag_output_directory_;
    rosbag_context_file = active_rosbag_context_file_;
    rosbag_log_file = active_rosbag_log_file_;
    rosbag_recording_pid_ = -1;
    active_rosbag_output_directory_.clear();
    active_rosbag_context_file_.clear();
    active_rosbag_log_file_.clear();
  }

  if (child_pid <= 0) {
    return;
  }

  bool clean_shutdown = false;
  std::string shutdown_reason = "stopped";
  ::kill(-child_pid, SIGINT);
  for (int attempt = 0; attempt < 300; ++attempt) {
    int status = 0;
    const pid_t result = ::waitpid(child_pid, &status, WNOHANG);
    if (result == child_pid) {
      clean_shutdown = true;
      shutdown_reason = "stopped via SIGINT";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!wait_process_group_dead(child_pid, std::chrono::milliseconds(0))) {
    shutdown_reason =
      clean_shutdown ?
      "recorder root exited but descendant processes remained; terminating process group with SIGTERM" :
      "recorder did not exit after SIGINT; terminating process group with SIGTERM";
    RCLCPP_WARN(get_logger(), "%s", shutdown_reason.c_str());
    ::kill(-child_pid, SIGTERM);
    if (!wait_process_group_dead(child_pid, std::chrono::seconds(5))) {
      shutdown_reason += "; process group survived SIGTERM and required SIGKILL";
      RCLCPP_WARN(get_logger(), "%s", shutdown_reason.c_str());
      ::kill(-child_pid, SIGKILL);
      if (!wait_process_group_dead(child_pid, std::chrono::seconds(1))) {
        shutdown_reason += "; process group still appears alive after SIGKILL";
      }
    }
  }

  clean_shutdown = !process_group_alive(child_pid);

  const std::filesystem::path metadata_path =    std::filesystem::path(rosbag_output_directory) / "metadata.yaml";
  bool metadata_present = std::filesystem::exists(metadata_path);
  for (int attempt = 0; attempt < 100 && !metadata_present; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    metadata_present = std::filesystem::exists(metadata_path);
  }

  if (!metadata_present) {
    const std::string metadata_warning =
      "Rosbag metadata.yaml missing after recorder shutdown for " + rosbag_output_directory;
    RCLCPP_WARN(get_logger(), "%s", metadata_warning.c_str());
    shutdown_reason += "; metadata.yaml missing";
  }

  if (!rosbag_context_file.empty() && std::filesystem::exists(rosbag_context_file)) {
    auto context_document = loadJsonDocument(rosbag_context_file);
    context_document["rosbag_output_directory"] = rosbag_output_directory;
    context_document["rosbag_log_file"] = rosbag_log_file;
    context_document["rosbag_metadata_present"] = metadata_present;
    context_document["rosbag_shutdown_clean"] = clean_shutdown && metadata_present;
    context_document["rosbag_shutdown_reason"] = shutdown_reason;
    if (!context_document.contains("rosbag_runtime_qos_overrides_file")) {
      context_document["rosbag_runtime_qos_overrides_file"] = "";
    }
    writeJsonDocumentAtomic(rosbag_context_file, context_document);
  }
}

void MissionExecutorNode::shutdownForExit()
{
  stopMissionRosbagRecording();
}

std::string MissionExecutorNode::formatUtcTimestamp(
  const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  std::tm time_info{};
#if defined(_WIN32)
  gmtime_s(&time_info, &as_time_t);
#else
  gmtime_r(&as_time_t, &time_info);
#endif
  std::ostringstream stream;
  stream << std::put_time(&time_info, "%Y%m%dT%H%M%SZ");
  return stream.str();
}

std::string MissionExecutorNode::formatLocalTimestamp(
  const std::chrono::system_clock::time_point & time_point)
{
  const std::time_t as_time_t = std::chrono::system_clock::to_time_t(time_point);
  std::tm time_info{};
#if defined(_WIN32)
  localtime_s(&time_info, &as_time_t);
#else
  localtime_r(&as_time_t, &time_info);
#endif
  std::ostringstream stream;
  stream << std::put_time(&time_info, "%Y%m%dT%H%M%S");
  return stream.str();
}

std::filesystem::path MissionExecutorNode::resolveScheduleSourcePath() const
{
  if (!schedule_ics_path_.empty()) {
    return resolvePath(schedule_ics_path_);
  }
  return discoverNewestSchedulePath(resolveMissionsFromDbDirectory());
}

std::filesystem::path MissionExecutorNode::ensureScheduleLogPath(
  const std::filesystem::path & schedule_source_path) const
{
  return schedule_source_path;
}

std::filesystem::path MissionExecutorNode::ensureActualScheduleLogPath(
  const std::filesystem::path & schedule_source_path) const
{
  (void)schedule_source_path;
  const std::filesystem::path actual_schedule_log_directory = use_simulation_ ?
    resolvePath(actual_schedule_log_directory_) :
    resolveMissionsLogDirectory();
  std::filesystem::create_directories(actual_schedule_log_directory);
  const std::filesystem::path actual_schedule_path =
    actual_schedule_log_directory /
    (use_simulation_ ? kSimulationActualScheduleLogFilename : kActualScheduleLogFilename);
  if (std::filesystem::exists(actual_schedule_path)) {
    return actual_schedule_path;
  }

  std::string timezone = "UTC";
  if (!schedule_source_path.empty() && std::filesystem::exists(schedule_source_path)) {
    std::ifstream input_stream(schedule_source_path);
    if (input_stream.is_open()) {
      std::ostringstream buffer;
      buffer << input_stream.rdbuf();
      timezone = discoverScheduleTimezone(buffer.str());
    }
  }

  std::ofstream output_stream(actual_schedule_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    return {};
  }
  output_stream
    << "BEGIN:VCALENDAR\n"
    << "VERSION:2.0\n"
    << "PRODID:-//O-Robotics//AMR Sweeper Actual Schedule//EN\n"
    << "CALSCALE:GREGORIAN\n"
    << "X-WR-CALNAME:AMR Sweeper Actual Mission Log\n"
    << "X-WR-TIMEZONE:" << timezone << "\n"
    << "END:VCALENDAR\n";
  return actual_schedule_path;
}

}  // namespace amr_sweeper_mission_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sweeper_mission_executor::MissionExecutorNode>();
  rclcpp::on_shutdown(
    [weak_node = std::weak_ptr<amr_sweeper_mission_executor::MissionExecutorNode>(node)]() {
      if (const auto locked = weak_node.lock()) {
        locked->shutdownForExit();
      }
    },
    node->get_node_base_interface()->get_context());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  node->shutdownForExit();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
