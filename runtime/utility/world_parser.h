#ifndef UTILITY_WORLD_PARSER_
#define UTILITY_WORLD_PARSER_

#include <string>
#include <physics/node.h>

namespace Json {
class Value;
}

namespace diagrammar {
// open a file and make a string out of its content
std::string Stringify(const char* path);
// read a json file and create a json object;
Json::Value CreateJsonObject(const char* file);
// parse the coordinate transformation
Eigen::Isometry2f ParseTransformation2D(const Json::Value& transformobj);
// parse a path;
std::vector<Vector2f> ParsePath2D(const Json::Value& pathobj);
// parse a node;
Node ParseNode(const Json::Value& nodeobj);

}

#endif