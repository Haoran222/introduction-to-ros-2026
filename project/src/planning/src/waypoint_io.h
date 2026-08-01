#pragma once

// YAML input/output for the shared waypoint route.

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace planning {

struct Waypoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline std::vector<Waypoint> LoadWaypoints(const std::string &path) {
  const YAML::Node root = YAML::LoadFile(path);  // throws YAML::BadFile if missing
  std::vector<Waypoint> waypoints;

  if (!root["waypoints"]) {
    return waypoints;
  }

  waypoints.reserve(root["waypoints"].size());

  for (const auto &node : root["waypoints"]) {
    Waypoint wp;
    wp.x = node["x"].as<double>();
    wp.y = node["y"].as<double>();
    wp.yaw = node["yaw"].as<double>(0.0);
    waypoints.push_back(wp);
  }

  return waypoints;
}

// Rewrite the complete route after each accepted waypoint.
inline void SaveWaypoints(
    const std::string &path,
    const std::vector<Waypoint> &waypoints) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;

  for (const auto &wp : waypoints) {
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "x" << YAML::Value << wp.x;
    out << YAML::Key << "y" << YAML::Value << wp.y;
    out << YAML::Key << "yaw" << YAML::Value << wp.yaw;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq << YAML::EndMap;

  std::ofstream file(path, std::ios::trunc);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open '" + path + "' for writing");
  }

  file << out.c_str() << "\n";
}

}  // namespace planning
