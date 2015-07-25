// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_UTILITY_WORLD_PARSER_H_
#define RUNTIME_UTILITY_WORLD_PARSER_H_

#include <memory>
#include <string>

#include <geometry/geometry2.h>
#include <physics/node.h>

namespace Json {
class Value;
}  // namespace Json

namespace diagrammar {
// Open a file and make a string out of its content
std::string Stringify(const char* path);
// Read a json file and create a json object;
Json::Value CreateJsonObject(const char* file);
// Parse the coordinate transformation
Isometry2f ParseTransformation2D(const Json::Value& transformobj);
// Parse a node
std::unique_ptr<Node> ParseNode(const Json::Value& nodeobj);
// Parse a joint, joint is a polymorphic struct
std::unique_ptr<Joint> ParseJoint(const Json::Value& jointobj);
}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_WORLD_PARSER_H_
