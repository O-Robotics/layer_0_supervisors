// Copyright 2026 O-Robotics
//
// Licensed under the Apache License, Version 2.0.

#include <zip.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mission_package_reader.hpp"

namespace
{

const char kOrderJson[] =
  R"({
  "headerId": 1,
  "timestamp": "2026-01-01T00:00:00Z",
  "version": "3.0.0",
  "manufacturer": "O-Robotics",
  "serialNumber": "test",
  "orderId": "zip_reader_test",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "n0",
      "sequenceId": 0,
      "released": true,
      "actions": [],
      "nodePosition": {"x": 0.0, "y": 0.0, "theta": 0.0, "mapId": "map"}
    }
  ],
  "edges": []
})";

const char kMapGeoreferenceJson[] =
  R"({
  "mapId": "map",
  "originLatitude": 55.0,
  "originLongitude": 12.0,
  "bounds": [0.0, 0.0, 10.0, 10.0]
})";

const char kZoneSetJson[] =
  R"({
  "headerId": 1,
  "timestamp": "2026-01-01T00:00:00Z",
  "version": "3.0.0",
  "manufacturer": "O-Robotics",
  "serialNumber": "test",
  "zoneSet": {
    "zoneSetId": "zones",
    "mapId": "map",
    "zones": []
  }
})";

void writeFile(const std::filesystem::path & path, const std::string & content)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::trunc);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to create " + path.string());
  }
  stream << content;
}

void addZipEntry(zip_t * archive, const std::string & name, const std::string & content)
{
  zip_source_t * source = zip_source_buffer(archive, content.data(), content.size(), 0);
  if (source == nullptr) {
    throw std::runtime_error("failed to allocate zip source for " + name);
  }
  if (zip_file_add(archive, name.c_str(), source, ZIP_FL_OVERWRITE) < 0) {
    zip_source_free(source);
    throw std::runtime_error("failed to add zip entry " + name);
  }
}

void writeZip(
  const std::filesystem::path & path,
  const std::string & prefix,
  const bool include_map,
  const bool ambiguous_root)
{
  int error_code = 0;
  zip_t * archive = zip_open(path.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &error_code);
  if (archive == nullptr) {
    throw std::runtime_error("failed to create zip " + path.string());
  }
  const std::string order_json = kOrderJson;
  const std::string map_georeference_json = kMapGeoreferenceJson;
  const std::string zone_set_json = kZoneSetJson;
  const std::string normalized_prefix = prefix.empty() ? std::string() : prefix + "/";
  addZipEntry(archive, normalized_prefix + "order.json", order_json);
  if (include_map) {
    addZipEntry(archive, normalized_prefix + "map_georeference.json", map_georeference_json);
  }
  addZipEntry(archive, normalized_prefix + "zoneSet.json", zone_set_json);
  if (ambiguous_root) {
    addZipEntry(archive, "other/order.json", order_json);
  }
  if (zip_close(archive) != 0) {
    throw std::runtime_error("failed to close zip " + path.string());
  }
}

void expectThrows(const std::string & label, const std::filesystem::path & package_path)
{
  try {
    (void)amr_sweeper_vda5050_parser::readVda5050MissionPackage(package_path);
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(label + " unexpectedly succeeded");
}

}  // namespace

int main()
{
  try {
    const std::filesystem::path temp_root =
      std::filesystem::temp_directory_path() / "amr_sweeper_mission_package_reader_test";
    std::filesystem::remove_all(temp_root);
    std::filesystem::create_directories(temp_root);

    const auto folder_package = temp_root / "folder_mission";
    writeFile(folder_package / "order.json", kOrderJson);
    writeFile(folder_package / "map_georeference.json", kMapGeoreferenceJson);
    writeFile(folder_package / "zoneSet.json", kZoneSetJson);
    const auto folder_documents =
      amr_sweeper_vda5050_parser::readVda5050MissionPackage(folder_package);
    if (folder_documents.order.at("orderId") != "zip_reader_test" ||
      !folder_documents.zone_set.has_value())
    {
      throw std::runtime_error("folder package did not parse expected documents");
    }

    const auto root_zip = temp_root / "root.zip";
    writeZip(root_zip, "", true, false);
    const auto root_documents =
      amr_sweeper_vda5050_parser::readVda5050MissionPackage(root_zip);
    if (root_documents.map_georeference.at("mapId") != "map") {
      throw std::runtime_error("root zip package did not parse map_georeference.json");
    }

    const auto nested_zip = temp_root / "nested.zip";
    writeZip(nested_zip, "nested", true, false);
    const auto nested_documents =
      amr_sweeper_vda5050_parser::readVda5050MissionPackage(nested_zip);
    if (nested_documents.order.at("orderId") != "zip_reader_test") {
      throw std::runtime_error("nested zip package did not parse order.json");
    }

    const auto missing_map_zip = temp_root / "missing_map.zip";
    writeZip(missing_map_zip, "", false, false);
    expectThrows("missing map zip", missing_map_zip);

    const auto ambiguous_zip = temp_root / "ambiguous.zip";
    writeZip(ambiguous_zip, "nested", true, true);
    expectThrows("ambiguous root zip", ambiguous_zip);

    std::filesystem::remove_all(temp_root);
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
