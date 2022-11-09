#include<state.h>

State::State(): pos(), g(0), h(0), c(0), idx(0), o(0), prim(0), pred(nullptr)
{
}

State::State(const vec3& pos, float g, float h): pos(pos), g(g), h(h)
{
}

State::State(float x, float y, float theta, float g, float h, State* pred, int prim):pos({x,y,theta}),g(g), h(h),pred(pred),o(0),c(0),idx(-1),prim(prim)
{
}

State::State(const State& state): pos(state.pos), g(state.g), h(state.h)
{
}

State::~State()
{
}

const float dy[] = { 0,        -0.0415893,  0.0415893};
const float dx[] = { 0.7068582,   0.705224,   0.705224};
const float dt[] = { 0,         0.1178097,   -0.1178097};

State* State::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = pos.a[0] + dx[i] * cos(pos.a[2]) - dy[i] * sin(pos.a[2]);
    ySucc = pos.a[1] + dx[i] * sin(pos.a[2]) + dy[i] * cos(pos.a[2]);
    tSucc = Helper::normalizeHeadingRad(pos.a[2] + dt[i]);
  }
  // backwards
  else {
    xSucc = pos.a[0] + dx[i-3] * cos(pos.a[2]) - dy[i-3] * sin(pos.a[2]);
    ySucc = pos.a[1] + dx[i-3] * sin(pos.a[2]) + dy[i-3] * cos(pos.a[2]);
    tSucc = Helper::normalizeHeadingRad(pos.a[2] + dt[i-3]);
  }

  return new State(xSucc, ySucc, tSucc, g,0, this, i);
}

void State::updateG() {
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}

bool State::isOnGrid(const int width, const int height) const 
{
  return pos.a[0] >= 0 && pos.a[0] < width && pos.a[1] >= 0 && pos.a[1] < height && (int)(pos.a[2] / Constants::deltaHeadingRad) >= 0 && (int)(pos.a[2] / Constants::deltaHeadingRad) < Constants::headings;
}

bool State::operator == (const State& rhs) const {
  return (int)pos.a[0] == (int)rhs.pos.a[0] &&
         (int)pos.a[1] == (int)rhs.pos.a[1] &&
         (std::abs(pos.a[2] - rhs.pos.a[2]) <= Constants::deltaHeadingRad ||
          std::abs(pos.a[2] - rhs.pos.a[2]) >= Constants::deltaHeadingNegRad);
}

//###################################################
//                                        IS IN RANGE
//###################################################
bool State::isInRange(const State& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(pos.a[0] - goal.pos.a[0]) / random;
  float dy = std::abs(pos.a[1] - goal.pos.a[1]) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}