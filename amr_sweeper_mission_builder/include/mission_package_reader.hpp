// Copyright 2026 O-Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <zip.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace amr_sweeper_mission_builder
{

struct MissionPackageDocuments
{
  nlohmann::json order;
  nlohmann::json map_georeference;
  std::optional<nlohmann::json> zone_set;
};

inline bool isZipMissionPackage(const std::filesystem::path & path)
{
  return std::filesystem::is_regular_file(path) && path.extension() == ".zip";
}

inline nlohmann::json loadMissionPackageJsonFile(const std::filesystem::path & path)
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("Failed to open mission package file: " + path.string());
  }

  nlohmann::json document;
  input_stream >> document;
  return document;
}

inline std::vector<std::string> splitZipPath(const std::string & path)
{
  if (path.empty() || path.front() == '/' || path.find('\\') != std::string::npos) {
    throw std::runtime_error("VDA5050 zip package contains unsafe path: " + path);
  }

  std::vector<std::string> parts;
  std::string current;
  for (const char character : path) {
    if (character == '/') {
      if (current.empty() || current == "." || current == "..") {
        throw std::runtime_error("VDA5050 zip package contains unsafe path: " + path);
      }
      parts.push_back(current);
      current.clear();
      continue;
    }
    current.push_back(character);
  }
  if (current.empty()) {
    return parts;
  }
  if (current == "." || current == "..") {
    throw std::runtime_error("VDA5050 zip package contains unsafe path: " + path);
  }
  parts.push_back(current);
  return parts;
}

inline std::string readZipEntry(
  zip_t * archive,
  const zip_uint64_t index,
  const std::string & name)
{
  zip_file_t * file = zip_fopen_index(archive, index, 0);
  if (file == nullptr) {
    throw std::runtime_error("Failed to open VDA5050 zip entry: " + name);
  }

  std::string content;
  std::vector<char> buffer(4096);
  while (true) {
    const zip_int64_t bytes_read = zip_fread(file, buffer.data(), buffer.size());
    if (bytes_read < 0) {
      zip_fclose(file);
      throw std::runtime_error("Failed to read VDA5050 zip entry: " + name);
    }
    if (bytes_read == 0) {
      break;
    }
    content.append(buffer.data(), static_cast<std::size_t>(bytes_read));
  }
  zip_fclose(file);
  return content;
}

inline MissionPackageDocuments readZipMissionPackage(const std::filesystem::path & package_path)
{
  int error_code = 0;
  zip_t * archive = zip_open(package_path.c_str(), ZIP_RDONLY, &error_code);
  if (archive == nullptr) {
    zip_error_t error;
    zip_error_init_with_code(&error, error_code);
    const std::string message = zip_error_strerror(&error);
    zip_error_fini(&error);
    throw std::runtime_error(
            "Failed to open VDA5050 zip package " + package_path.string() + ": " +
            message);
  }

  struct Candidate
  {
    std::string root;
    std::string filename;
    zip_uint64_t index{0U};
    std::string entry_name;
  };

  std::vector<Candidate> candidates;
  const zip_int64_t entry_count = zip_get_num_entries(archive, 0);
  if (entry_count < 0) {
    zip_close(archive);
    throw std::runtime_error(
            "Failed to list VDA5050 zip package entries: " + package_path.string());
  }
  for (zip_uint64_t index = 0; index < static_cast<zip_uint64_t>(entry_count); ++index) {
    struct zip_stat stat;
    zip_stat_init(&stat);
    if (zip_stat_index(archive, index, 0, &stat) != 0 || stat.name == nullptr) {
      zip_close(archive);
      throw std::runtime_error("Failed to inspect VDA5050 zip package: " + package_path.string());
    }
    const std::string entry_name(stat.name);
    if (!entry_name.empty() && entry_name.back() == '/') {
      (void)splitZipPath(entry_name);
      continue;
    }
    const auto parts = splitZipPath(entry_name);
    if (parts.empty()) {
      continue;
    }
    const std::string filename = parts.back();
    if (filename != "order.json" && filename != "zoneSet.json" &&
      filename != "map_georeference.json")
    {
      continue;
    }
    if (parts.size() > 2U) {
      zip_close(archive);
      throw std::runtime_error(
              "VDA5050 zip package mission file must be at root or one top-level folder: " +
              entry_name);
    }
    candidates.push_back(
      Candidate{
        parts.size() == 2U ? parts.front() : std::string(),
        filename,
        index,
        entry_name});
  }

  if (candidates.empty()) {
    zip_close(archive);
    throw std::runtime_error(
            "VDA5050 zip package contains no mission files: " + package_path.string());
  }

  std::set<std::string> roots;
  for (const auto & candidate : candidates) {
    roots.insert(candidate.root);
  }
  if (roots.size() > 1U) {
    zip_close(archive);
    throw std::runtime_error(
            "VDA5050 zip package contains mission files in multiple roots: " +
            package_path.string());
  }

  auto find_candidate =
    [&candidates, archive, &package_path](
    const std::string & filename) -> std::optional<Candidate> {
      std::optional<Candidate> result;
      for (const auto & candidate : candidates) {
        if (candidate.filename != filename) {
          continue;
        }
        if (result.has_value()) {
          zip_close(archive);
          throw std::runtime_error(
                  "VDA5050 zip package contains duplicate " + filename + ": " +
                  package_path.string());
        }
        result = candidate;
      }
      return result;
    };

  const auto order = find_candidate("order.json");
  const auto map_georeference = find_candidate("map_georeference.json");
  if (!order.has_value() || !map_georeference.has_value()) {
    zip_close(archive);
    throw std::runtime_error(
            "VDA5050 zip package requires order.json and map_georeference.json: " +
            package_path.string());
  }
  const auto zone_set = find_candidate("zoneSet.json");

  MissionPackageDocuments documents;
  documents.order =
    nlohmann::json::parse(readZipEntry(archive, order->index, order->entry_name));
  documents.map_georeference =
    nlohmann::json::parse(
    readZipEntry(archive, map_georeference->index, map_georeference->entry_name));
  if (zone_set.has_value()) {
    documents.zone_set =
      nlohmann::json::parse(readZipEntry(archive, zone_set->index, zone_set->entry_name));
  }

  zip_close(archive);
  return documents;
}

inline MissionPackageDocuments readDirectoryMissionPackage(
  const std::filesystem::path & package_path)
{
  const std::filesystem::path order_path = package_path / "order.json";
  const std::filesystem::path map_georeference_path = package_path / "map_georeference.json";
  if (!std::filesystem::is_regular_file(order_path) ||
    !std::filesystem::is_regular_file(map_georeference_path))
  {
    throw std::runtime_error(
            "VDA5050 package requires order.json and map_georeference.json: " +
            package_path.string());
  }

  MissionPackageDocuments documents;
  documents.order = loadMissionPackageJsonFile(order_path);
  documents.map_georeference = loadMissionPackageJsonFile(map_georeference_path);
  const std::filesystem::path zone_set_path = package_path / "zoneSet.json";
  if (std::filesystem::is_regular_file(zone_set_path)) {
    documents.zone_set = loadMissionPackageJsonFile(zone_set_path);
  }
  return documents;
}

inline MissionPackageDocuments readVda5050MissionPackage(const std::filesystem::path & package_path)
{
  if (std::filesystem::is_directory(package_path)) {
    return readDirectoryMissionPackage(package_path);
  }
  if (isZipMissionPackage(package_path)) {
    return readZipMissionPackage(package_path);
  }
  return readDirectoryMissionPackage(package_path.parent_path());
}

inline std::string missionPackageStem(const std::filesystem::path & mission_path)
{
  if (mission_path.filename() == "order.json" && mission_path.has_parent_path()) {
    return mission_path.parent_path().filename().string();
  }
  return mission_path.stem().string();
}

}  // namespace amr_sweeper_mission_builder
