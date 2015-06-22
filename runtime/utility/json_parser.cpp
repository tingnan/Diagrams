#include "json_parser.h"
#include "geometry/aabb.h"
#include <json/json.h>
#include <iostream>
#include <fstream>

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

Eigen::Isometry2f ParseTransformation2D(const Json::Value& array) {
  Eigen::Isometry2f t = Eigen::Isometry2f::Identity();
  assert(array.size() == 6);

  Eigen::Matrix2f rot;
  rot << array[0].asFloat(), array[2].asFloat(), array[1].asFloat(),
      array[3].asFloat();
  // rotate then translate, the order is important;
  t.translate(Vec2f(array[4].asFloat(), array[5].asFloat())).rotate(rot);
  return t;
}

std::vector<Vec2f> ParsePath2D(const Json::Value& pathobj) {
  std::vector<Vec2f> mypath;

  Json::Value::const_iterator itr = pathobj.begin();
  for (; itr != pathobj.end(); ++itr) {
    const Json::Value& pt = *itr;
    float x = pt.get("x", 0).asFloat();
    float y = pt.get("y", 0).asFloat();
    mypath.emplace_back(Vec2f(x, y));
  }
  return mypath;
}

// load a "children" node from the json descriptor
Node ParseNode(const Json::Value& nodeobj) {
  Node node;
  if (!nodeobj.isMember("type")) {
    return node;
  }
  std::string ntype = nodeobj["type"].asString();
  if (ntype != "node" && ntype != "open_path") {
    return node;
  }

  Json::Value::const_iterator itr = nodeobj.begin();
  for (; itr != nodeobj.end(); ++itr) {
    if (itr.key().asString() == "id") {
      node.SetID((*itr).asInt());
    }

    if (itr.key().asString() == "transform") {
      const Eigen::Isometry2f tr = ParseTransformation2D(*itr);
      node.SetRotationMatrix(tr.linear());
      node.SetPosition(tr.translation());
    }

    if (itr.key().asString() == "path") {
      Geometry2D geo;
      geo.SetPath(ParsePath2D(*itr));
      if (ntype == "open_path") {
        geo.SetPathClosed(false);
      }
      node.AddGeometry(std::move(geo));
    }

    if (itr.key().asString() == "inner_path") {
      const std::vector<Vec2f>& path = ParsePath2D(*itr);
      AABB bounding_box = GetAABB(path);
      Geometry2D geo;
      geo.AddHole(path);
      std::vector<Vec2f> box;
      Vec2f pt0 = bounding_box.lower_bound;
      Vec2f pt2 = bounding_box.upper_bound;
      Vec2f pt1(pt2(0), pt0(1));
      Vec2f pt3(pt0(0), pt2(1));
      box.emplace_back(pt0);
      box.emplace_back(pt1);
      box.emplace_back(pt2);
      box.emplace_back(pt3);
      geo.SetPath(box);
      node.AddGeometry(std::move(geo));
    }
  }
  // once the geometry boundary is initilalized, we will also
  // initilized the internal convexhull.
  // TO DO;
  //
  return node;
}

Json::Value ReadWorldFromFile(const char* file) {
  std::string content = diagrammar::Stringify(file);
  Json::Reader world_parser;
  Json::Value world_descriptor;
  bool success = world_parser.parse(content, world_descriptor);
  if (!success) {
    std::cout << "not a valid json file" << std::endl;
    exit(-1);
  }
  return world_descriptor;
}
}