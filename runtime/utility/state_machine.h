#ifndef RUNTIME_UTILITY_STATE_MACHINE_H_
#define RUNTIME_UTILITY_STATE_MACHINE_H_

namespace Json{
  class Value;
}  // namespace Json

namespace diagrammar{

class StateMachine {

 public:
  void HandleMessage(const Json::Value& message);

 private:
  // use boost state chart or sth 
};

}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_STATE_MACHINE_H_