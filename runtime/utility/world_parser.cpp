// Copyright 2015 Native Client Authors

#include <json/json.h>
#include <cstring>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "utility/world_parser.h"
#include "geometry/aabb.h"
#include "utility/stl_memory.h"

namespace diagrammar {
Vector2f ParsePoint(const Json::Value& pt) {
  Vector2f vec;
  vec(0) = pt.get("x", 0).asFloat();
  vec(1) = -pt.get("y", 0).asFloat();
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

  Matrix2f rot_mat;
  rot_mat << array[0].asFloat(), array[2].asFloat(), array[1].asFloat(),
      array[3].asFloat();
  Rotation2f rot(0);
  rot.fromRotationMatrix(rot_mat);
  // rotate then translate, the order is important;
  t.translate(Vector2f(array[4].asFloat(), -array[5].asFloat()))
      .rotate(rot.inverse());
  return t;
}

Path ParsePath2D(const Json::Value& path_obj) {
  Path mypath;
  Json::Value::const_iterator itr = path_obj.begin();
  for (; itr != path_obj.end(); ++itr) {
    const Json::Value& pt = *itr;
    mypath.emplace_back(ParsePoint(pt));
  }
  return mypath;
}

// load a "child" node from the json descriptor
std::unique_ptr<Node> ParseNode(const Json::Value& node_obj) {
  std::string ntype = node_obj.get("type", "").asString();
  if (ntype != "node" && ntype != "open_path") {
    std::cout << ntype << std::endl;
    assert(0);
  }

  std::unique_ptr<Node> node_ptr = make_unique<Node>();
  if (node_obj.isMember("id")) {
    node_ptr->id = node_obj["id"].asInt();
  }

  std::string motion_type_str =
      node_obj.get("motion_type", "static").asString();
  if (motion_type_str == "static") {
    // no_op
  } else if (motion_type_str == "dynamic") {
    node_ptr->motion_type = MotionType::kDynamic;
  } else {
    node_ptr->motion_type = MotionType::kKinematic;
  }

  if (node_obj.isMember("transform")) {
    const Isometry2f tr = ParseTransformation2D(node_obj["transform"]);
    node_ptr->frame.SetRotation(tr.linear());
    node_ptr->frame.SetTranslation(tr.translation());
  }

  if (node_obj.isMember("path")) {
    auto& path_obj = node_obj["path"];
    if (ntype == "node") {
      auto polys = ResolveIntersections(Polygon(ParsePath2D(path_obj)));
      node_ptr->polygons.insert(node_ptr->polygons.begin(),
                                std::make_move_iterator(polys.begin()),
                                std::make_move_iterator(polys.end()));
    }
    if (ntype == "open_path") {
      node_ptr->paths.emplace_back(ParsePath2D(path_obj));
    }
  }

  if (node_obj.isMember("inner_path")) {
    const std::vector<Vector2f>& path = ParsePath2D(node_obj["inner_path"]);
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
    Polygon poly(box);
    poly.holes.emplace_back(path);
    auto polys = ResolveIntersections(poly);
    node_ptr->polygons.insert(node_ptr->polygons.begin(),
                              std::make_move_iterator(polys.begin()),
                              std::make_move_iterator(polys.end()));
  }

  return node_ptr;
}

std::unique_ptr<Joint> ParseJoint(const Json::Value& joint_obj) {
  std::unique_ptr<Joint> joint_ptr;
  std::string joint_type = joint_obj.get("type", "").asString();

  if (joint_type == "revolute") {
    joint_ptr.reset(new RevoluteJoint);
  } else if (joint_type == "prismatic") {
    joint_ptr.reset(new PrismaticJoint);
  }

  // fill the base joint type
  joint_ptr->id = joint_obj.get("id", 0).asInt();

  if (joint_obj.isMember("local_anchor_1")) {
    joint_ptr->local_anchor_1 = ParsePoint(joint_obj["local_anchor_1"]);
  }

  if (joint_obj.isMember("local_anchor_2")) {
    joint_ptr->local_anchor_2 = ParsePoint(joint_obj["local_anchor_2"]);
  }

  if (joint_obj.isMember("node_1")) {
    joint_ptr->node_1 = joint_obj["node_1"].asUInt();
  }

  if (joint_obj.isMember("node_2")) {
    joint_ptr->node_2 = joint_obj["node_2"].asUInt();
  }

  if (joint_type == "revolute") {
    RevoluteJoint* derived_ptr = dynamic_cast<RevoluteJoint*>(joint_ptr.get());
    const Json::Value& limit_obj = joint_obj["limit"];
    if (limit_obj.isMember("angle_min")) {
      derived_ptr->enable_limit_min = true;
      derived_ptr->angle_min = limit_obj["angle_min"].asFloat();
    }

    if (limit_obj.isMember("angle_max")) {
      derived_ptr->enable_limit_max = true;
      derived_ptr->angle_max = limit_obj["angle_max"].asFloat();
    }
  }

  return joint_ptr;
}

}  // namespace diagrammar
