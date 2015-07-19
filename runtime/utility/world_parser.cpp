// Copyright 2015 Native Client Authors

#include <json/json.h>
#include <cstring>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "utility/world_parser.h"
#include "geometry/aabb.h"

namespace diagrammar {
Vector2f ParsePoint(const Json::Value& pt) {
  Vector2f vec;
  vec(0) = pt.get("x", 0).asFloat();
  vec(1) = pt.get("y", 0).asFloat();
  return vec;
}


}  // namespace

namespace diagrammar {

std::string Stringify(const char* path) {
  std::ifstream file_handle(path, std::ios::binary | std::ios::in);
  std::string text;
  if (file_handle.is_open()) {
    file_handle.seekg(0, std::ios::end);
    text.resize(file_handle.tellg());
    file_handle.seekg(0, std::ios::beg);
    file_handle.read(&text[0], text.size());
    file_handle.close();
  } else {
    std::cout << "cannot open file\n";
  }
  return text;
}

Json::Value CreateJsonObject(const char* file) {
  std::string content = diagrammar::Stringify(file);
  Json::Reader reader;
  Json::Value json_obj;
  bool success = reader.parse(content, json_obj);
  if (!success) {
    std::cout << "not a valid json file" << std::endl;
    exit(-1);
  }
  return json_obj;
}

Isometry2f ParseTransformation2D(const Json::Value& array) {
  Isometry2f t = Isometry2f::Identity();
  assert(array.size() == 6);

  Matrix2f rot;
  rot << array[0].asFloat(), array[2].asFloat(), array[1].asFloat(),
      array[3].asFloat();
  // rotate then translate, the order is important;
  t.translate(Vector2f(array[4].asFloat(), array[5].asFloat())).rotate(rot);
  return t;
}

Path ParsePath2D(const Json::Value& pathobj) {
  Path mypath;
  Json::Value::const_iterator itr = pathobj.begin();
  for (; itr != pathobj.end(); ++itr) {
    const Json::Value& pt = *itr;
    mypath.emplace_back(ParsePoint(pt));
  }
  return mypath;
}

// load a "children" node from the json descriptor
Node ParseNode(const Json::Value& nodeobj) {

  std::string ntype = nodeobj.get("type", "").asString();
  if (ntype != "node" && ntype != "open_path") {
    std::cout << ntype << std::endl;
    assert(0);
  }

  Node node;
  Json::Value::const_iterator itr = nodeobj.begin();
  for (; itr != nodeobj.end(); ++itr) {

    if (strcmp(itr.memberName(), "id") == 0) {
      node.id = (*itr).asInt();
    }

    if (strcmp(itr.memberName(), "transform") == 0) {
      const Isometry2f tr = ParseTransformation2D(*itr);
      node.frame.SetRotation(tr.linear());
      node.frame.SetTranslation(tr.translation());
    }

    if (strcmp(itr.memberName(), "path") == 0) {
      if (ntype == "node") {
        Polygon parsed_polygon = Polygon(ParsePath2D(*itr));
        node.polygons = ResolveIntersections(parsed_polygon);
      }
      if (ntype == "open_path") {
        node.paths.emplace_back(ParsePath2D(*itr));
      }
    }

    if (strcmp(itr.memberName(), "inner_path") == 0) {
      const std::vector<Vector2f>& path = ParsePath2D(*itr);
      AABB bounding_box = GetAABBWithPadding(path, 2e-2);
      std::vector<Vector2f> box;
      Vector2f pt0 = bounding_box.lower_bound;
      Vector2f pt2 = bounding_box.upper_bound;
      Vector2f pt1(pt2(0), pt0(1));
      Vector2f pt3(pt0(0), pt2(1));
      box.emplace_back(pt0);
      box.emplace_back(pt1);
      box.emplace_back(pt2);
      box.emplace_back(pt3);
      Polygon geo(box);
      geo.holes.emplace_back(path);
      node.polygons = ResolveIntersections(geo);
    }
  }
  return node;
}

std::unique_ptr<Joint> ParseJoint(const Json::Value& jointobj) {
  std::unique_ptr<Joint> joint_ptr;
  std::string join_type = jointobj.get("type", "").asString();
  if (join_type == "revolute") {
    joint_ptr.reset(new RevoluteJoint);
    Json::Value::const_iterator itr = jointobj.begin();
    for (; itr != jointobj.end(); ++itr) {
      if (strcmp(itr.memberName(), "local_anchor_1") == 0) {
        joint_ptr->local_anchor_1 = ParsePoint(*itr);
        continue;
      }
      if (strcmp(itr.memberName(), "local_anchor_2") == 0) {
        joint_ptr->local_anchor_2 = ParsePoint(*itr);
      }

      if (strcmp(itr.memberName(), "node_1") == 0) {
        joint_ptr->node_1 = itr.key().asUInt();
      }

      if (strcmp(itr.memberName(), "node_2") == 0) {
        joint_ptr->node_2 = itr.key().asUInt();
      }
    }
  }

  return joint_ptr;
}


}  // namespace diagrammar
