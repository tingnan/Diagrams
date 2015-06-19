#include <string>
#include <physics/node.h>

namespace Json {
class Value;
}

namespace diagrammar {
std::string Stringify(const char* path);
Eigen::Isometry2f ParseTransformation2D(const Json::Value& transformobj);
std::vector<Vec2f> ParsePath2D(const Json::Value& pathobj);
Node ParseNode(const Json::Value& nodeobj);
Json::Value ReadWorldFromFile(const char* file);
}