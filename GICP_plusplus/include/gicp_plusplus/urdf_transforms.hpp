#pragma once

// URDF static-transform helper for gicp_plusplus. This is a direct port of
// GLIM's glim/util/urdf_transforms.hpp so both pipelines resolve aux-LiDAR
// extrinsics from the SAME av24.urdf with identical math (rpy = Rz*Ry*Rx,
// fixed joints only) -- and neither needs a live /tf_static publisher to merge
// multiple LiDARs in offline replay.

#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include <sstream>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <libxml/parser.h>
#include <libxml/tree.h>

namespace gicp_plusplus {

/// Parse a URDF file and return a map child_link_name -> (parent_link_name, T_parent_child).
/// Only fixed joints are considered. Throws std::runtime_error on parse failure.
inline std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> parse_urdf_transforms(const std::string& urdf_path) {
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> transforms;

  xmlDocPtr doc = xmlReadFile(urdf_path.c_str(), nullptr, 0);
  if (!doc) {
    throw std::runtime_error("Failed to parse URDF: " + urdf_path);
  }

  xmlNodePtr root = xmlDocGetRootElement(doc);
  for (xmlNodePtr joint = root ? root->children : nullptr; joint; joint = joint->next) {
    if (joint->type != XML_ELEMENT_NODE || xmlStrcmp(joint->name, BAD_CAST "joint") != 0) {
      continue;
    }

    xmlChar* type_attr = xmlGetProp(joint, BAD_CAST "type");
    std::string joint_type = type_attr ? reinterpret_cast<const char*>(type_attr) : "";
    xmlFree(type_attr);
    if (joint_type != "fixed") {
      continue;
    }

    std::string parent_link, child_link;
    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;

    for (xmlNodePtr child = joint->children; child; child = child->next) {
      if (child->type != XML_ELEMENT_NODE) continue;

      if (xmlStrcmp(child->name, BAD_CAST "parent") == 0) {
        xmlChar* link = xmlGetProp(child, BAD_CAST "link");
        if (link) { parent_link = reinterpret_cast<const char*>(link); xmlFree(link); }
      } else if (xmlStrcmp(child->name, BAD_CAST "child") == 0) {
        xmlChar* link = xmlGetProp(child, BAD_CAST "link");
        if (link) { child_link = reinterpret_cast<const char*>(link); xmlFree(link); }
      } else if (xmlStrcmp(child->name, BAD_CAST "origin") == 0) {
        xmlChar* xyz_attr = xmlGetProp(child, BAD_CAST "xyz");
        if (xyz_attr) {
          std::istringstream ss(reinterpret_cast<const char*>(xyz_attr));
          ss >> x >> y >> z;
          xmlFree(xyz_attr);
        }
        xmlChar* rpy_attr = xmlGetProp(child, BAD_CAST "rpy");
        if (rpy_attr) {
          std::istringstream ss(reinterpret_cast<const char*>(rpy_attr));
          ss >> roll >> pitch >> yaw;
          xmlFree(rpy_attr);
        }
      }
    }

    if (!parent_link.empty() && !child_link.empty()) {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation() = Eigen::Vector3d(x, y, z);
      T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
      transforms[child_link] = {parent_link, T};
    }
  }

  xmlFreeDoc(doc);
  xmlCleanupParser();
  return transforms;
}

/// Compute T_from_to between two frames by walking the kinematic tree to their
/// common ancestor. Throws std::runtime_error if no common ancestor exists.
inline Eigen::Isometry3d compute_transform(
  const std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>>& transforms,
  const std::string& from_frame,
  const std::string& to_frame) {
  auto chain_to_root = [&](const std::string& frame) {
    std::vector<std::pair<std::string, Eigen::Isometry3d>> chain;
    std::string current = frame;
    while (transforms.count(current)) {
      const auto& entry = transforms.at(current);
      chain.push_back({current, entry.second});
      current = entry.first;
    }
    chain.push_back({current, Eigen::Isometry3d::Identity()});
    return chain;
  };

  auto from_chain = chain_to_root(from_frame);
  auto to_chain = chain_to_root(to_frame);

  std::unordered_map<std::string, size_t> from_set;
  for (size_t i = 0; i < from_chain.size(); i++) {
    from_set[from_chain[i].first] = i;
  }

  std::string ancestor;
  size_t to_idx = 0;
  for (size_t i = 0; i < to_chain.size(); i++) {
    if (from_set.count(to_chain[i].first)) {
      ancestor = to_chain[i].first;
      to_idx = i;
      break;
    }
  }

  if (ancestor.empty()) {
    throw std::runtime_error("No common ancestor between " + from_frame + " and " + to_frame);
  }

  size_t from_idx = from_set[ancestor];

  Eigen::Isometry3d T_ancestor_from = Eigen::Isometry3d::Identity();
  for (size_t i = from_idx; i > 0; i--) {
    T_ancestor_from = T_ancestor_from * from_chain[i - 1].second;
  }

  Eigen::Isometry3d T_ancestor_to = Eigen::Isometry3d::Identity();
  for (size_t i = to_idx; i > 0; i--) {
    T_ancestor_to = T_ancestor_to * to_chain[i - 1].second;
  }

  return T_ancestor_from.inverse() * T_ancestor_to;
}

}  // namespace gicp_plusplus
