// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_UTILITY_WORLD_PARSER_H_
#define RUNTIME_UTILITY_WORLD_PARSER_H_

#include <string>
#include <physics/node.h>

namespace Json {
class Value;
}  // namespace Json

namespace diagrammar {
// open a file and make a string out of its content
std::string Stringify(const char* path);
// read a json file and create a json object;
Json::Value CreateJsonObject(const char* file);
// parse the coordinate transformation
Isometry2f ParseTransformation2D(const Json::Value& transformobj);
// parse a path;
std::vector<Vector2f> ParsePath2D(const Json::Value& pathobj);
// parse a node;
Node ParseNode(const Json::Value& nodeobj);
}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_WORLD_PARSER_H_
