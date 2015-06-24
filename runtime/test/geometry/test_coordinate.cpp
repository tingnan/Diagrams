#include "test_coordinate.h"
#include <iostream>

void CoordinateFrame2DTest::SetUp() {
  coordinate_ = diagrammar::CoordinateFrame2D(diagrammar::Isometry2f::Identity());
  mFrameInternal.setIdentity();
}

void CoordinateFrame2DTest::TearDown() {}

#define FLOAT_ROUNDOFF 1e-3

TEST_F(CoordinateFrame2DTest, TranslateRotateOrder) {
  // do some stuff
  diagrammar::Vector2f disp(1.34, 0);
  diagrammar::Rotation2f rot(2.15);
  // the order does not matter here because we are rotating IN PLACE
  coordinate_.Rotate(rot);
  coordinate_.Translate(disp);
  coordinate_.Rotate(rot);
  coordinate_.Translate(disp);

  // the indernal order
  mFrameInternal.translate(disp).translate(disp).rotate(rot).rotate(rot);
  diagrammar::Matrix2f diff = mFrameInternal.linear() - coordinate_.GetRotationMatrix();
  ASSERT_LE(abs(diff(0, 0)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(0, 1)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(1, 0)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(1, 1)), FLOAT_ROUNDOFF);

  mFrameInternal.setIdentity();
  // prerotate and pretranslate
  mFrameInternal.prerotate(rot).prerotate(rot).pretranslate(disp).pretranslate(
      disp);
  diff = mFrameInternal.linear() - coordinate_.GetRotationMatrix();
  ASSERT_LE(abs(diff(0, 0)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(0, 1)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(1, 0)), FLOAT_ROUNDOFF);
  ASSERT_LE(abs(diff(1, 1)), FLOAT_ROUNDOFF);
}

TEST_F(CoordinateFrame2DTest, TransformPoint) {
  float dispx = 1.25;
  float dispy = 35.55;
  diagrammar::Vector2f disp(dispx, dispy);
  coordinate_.SetTranslation(disp);
  for (float rota = -M_PI; rota <= M_PI; rota += 0.01) {
    diagrammar::Rotation2f rot(rota);
    // the order does not matter here because we are rotating IN PLACE
    coordinate_.SetRotation(rot);

    // now transform a point (0, 0) from local frame to parent frame
    diagrammar::Vector2f p0 = coordinate_.TransformPoint(diagrammar::Vector2f(0, 0));
    ASSERT_LE(abs(p0(0) - dispx), FLOAT_ROUNDOFF);
    ASSERT_LE(abs(p0(1) - dispy), FLOAT_ROUNDOFF);

    // now transform a point (1, 1) from local frame to parent frame
    diagrammar::Vector2f p1 = coordinate_.TransformPoint(diagrammar::Vector2f(1, 1));
    // first rotate
    diagrammar::Vector2f p1m(cos(rota) * 1 - sin(rota) * 1, sin(rota) * 1 + cos(rota) * 1);
    // then translate
    p1m += disp;
    ASSERT_LE(abs(p1(0) - p1m(0)), FLOAT_ROUNDOFF);
    ASSERT_LE(abs(p1(1) - p1m(1)), FLOAT_ROUNDOFF);
  }
}