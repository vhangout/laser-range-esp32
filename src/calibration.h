#pragma once

#include <stddef.h>

struct Point {
  float x;
  float y;
};

struct Calibration {
  Point points[4];
  size_t count = 0;
  bool valid = false;
};

extern Calibration gCalibration;
