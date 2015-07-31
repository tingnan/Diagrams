#include "geometry/coordinate_frame.h"
#include "gtest/gtest.h"

class CoordinateFrameTest : public ::testing::Test {
 protected:
  // called before each test
  virtual void SetUp();
  // called after each test
  virtual void TearDown();
  diagrammar::CoordinateFrame frame_;
  diagrammar::Isometry3f frame_internal_;
};
