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

namespace {
diagrammar::Vector2f ParsePoint(const Json::Value& pt) {
  diagrammar::Vector2f vec;
  vec(0) = pt.get("x", 0).asFloat();
  vec(1) = -pt.get("y", 0).asFloat();
  return vec;
}

diagrammar::Path ParsePath(const Json::Value& path_obj) {
  diagrammar::Path mypath;
  Json::Value::const_iterator itr = path_obj.begin();
  for (; itr != path_obj.end(); ++itr) {
    const Json::Value& pt = *itr;
    mypath.emplace_back(ParsePoint(pt));
  }
  return mypath;
}

std::vector<diagrammar::Polygon> ParsePolygon(const Json::Value& poly_obj) {
  diagrammar::Polygon poly;
  if (poly_obj.isMember("path")) {
    poly.path = ParsePath(poly_obj["path"]);
  }

  if (poly_obj.isMember("hole")) {
    poly.holes.emplace_back(ParsePath(poly_obj["hole"]));
  }
  // If the polygon has no path but only holes, we compute a box that can
  // enclose all the holes
  if (poly.path.empty()) {
    std::vector<diagrammar::Vector2f> bounding_box(4,
                                                   diagrammar::Vector2f(0, 0));
    for (auto& hole : poly.holes) {
      diagrammar::AABB box = diagrammar::GetAABBWithPadding(hole, 0.01);
      // Expand the bounding box
      if (bounding_box[0](0) > box.lower_bound(0))
        bounding_box[0](0) = box.lower_bound(0);
      if (bounding_box[0](1) > box.lower_bound(1))
        bounding_box[0](1) = box.lower_bound(1);
      if (bounding_box[2](0) < box.upper_bound(0))
        bounding_box[2](0) = box.upper_bound(0);
      if (bounding_box[2](1) < box.upper_bound(1))
        bounding_box[2](1) = box.upper_bound(1);
    }
    bounding_box[1] =
        diagrammar::Vector2f(bounding_box[2](0), bounding_box[0](1));
    bounding_box[3] =
        diagrammar::Vector2f(bounding_box[0](0), bounding_box[2](1));
    poly.path = bounding_box;
  }
  return ResolveIntersections(poly);
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
  // rotate then translate, the order is important;
  t.translate(Vector2f(array[4].asFloat(), -array[5].asFloat()))
      .rotate(rot_mat.transpose());
  return t;
}

// load a "child" node from the json descriptor
std::unique_ptr<Node> ParseNode(const Json::Value& node_obj) {
  std::string ntype = node_obj.get("type", "").asString();
  if (ntype != "node") {
    std::cerr << ntype << std::endl;
    assert(0);
  }

  std::unique_ptr<Node> node_ptr = make_unique<Node>();
  if (node_obj.isMember("id")) {
    node_ptr->id = node_obj["id"].asInt();
  }

  std::string motion_type_str = node_obj.get("motion_type", "").asString();
  if (motion_type_str == "") {
    node_ptr->motion_type = MotionType::kStatic;
  } else if (motion_type_str == "dynamic") {
    node_ptr->motion_type = MotionType::kDynamic;
  } else {
    node_ptr->motion_type = MotionType::kKinematic;
  }

  if (node_obj.isMember("transform")) {
    const Isometry2f tr = ParseTransformation2D(node_obj["transform"]);
    Matrix3f rot_mat = Matrix3f::Identity();
    rot_mat.topLeftCorner<2, 2>() = tr.rotation();
    node_ptr->frame.SetRotation(rot_mat);
    Vector3f disp(0, 0, 0);
    disp.head<2>() = tr.translation();
    node_ptr->frame.SetTranslation(disp);
  }

  if (node_obj.isMember("open_path")) {
    // Allow later to have a set of open_paths
    auto& path_obj = node_obj["open_path"];
    node_ptr->paths.emplace_back(ParsePath(path_obj));
  }

  // TODO(tingnan) move polygon into a sub Json object: polygons
  auto& poly_obj = node_obj;
  auto polygons = ParsePolygon(poly_obj);
  node_ptr->polygons.insert(node_ptr->polygons.begin(),
                            std::make_move_iterator(polygons.begin()),
                            std::make_move_iterator(polygons.end()));
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
