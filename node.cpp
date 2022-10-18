#include <boost/qvm/vec.hpp>
#include "helper.h"
#include <cmath>

typedef boost::qvm::vec<float,3> vec3;

using namespace boost;

// R = 6, 6.75 DEG
const float dy[] = { 0,        -0.0415893,  0.0415893};
const float dx[] = { 0.7068582,   0.705224,   0.705224};
const float dt[] = { 0,         0.1178097,   -0.1178097};

void createSuccessor(float x, float y, float t, const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
  }


  //return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}