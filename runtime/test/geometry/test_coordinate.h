#include "geometry/coordinate_frame.h"
#include "gtest/gtest.h"

class CoordinateFrame2DTest : public ::testing::Test {
 protected:
  // called before each test
  virtual void SetUp();
  // called after each test
  virtual void TearDown();
  diagrammar::CoordinateFrame2D coordinate_;
  Eigen::Isometry2f mFrameInternal;
};
