#include "test_coordinate.h"
#include <iostream>

void CoordinateFrameTest::SetUp() {
  frame_ = diagrammar::CoordinateFrame(diagrammar::Isometry3f::Identity());
  frame_internal_.setIdentity();
}

void CoordinateFrameTest::TearDown() {}

#define FLOAT_ROUNDOFF 1e-3

TEST_F(CoordinateFrameTest, TranslateRotateOrder) {
  // do some stuff
  diagrammar::Vector3f disp(1.34, 0, 0);
}
