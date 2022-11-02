#pragma once

#include <boost/qvm/vec.hpp>
#include <cmath>
#include "helper.h"
#include "defs.h"

typedef boost::qvm::vec<float,3> vec3;

class State
{
  public:
  State();
  State(const vec3& pos, float g, float h);
  State(float x, float y, float theta, float g, float h, const State* pred, int prim = 0);

  State(const State& state);
  ~State();
  State* createSuccessor(const int i);
  /// позиция x, y, theta
	vec3 pos;
	/// текущая стоимость
	float g; 
	/// предполагаемая стоимость (эвристика)
	float h;
  /// индекс в 3d
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  const State* pred;

  void updateG();
    /// get the total estimated cost
  float getC() const { return g + h; }
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  int setIdx(int width, int height) { this->idx = (int)(pos.a[2] / Constants::deltaHeadingRad) * width * height + (int)(pos.a[1]) * width + (int)(pos.a[0]); return idx;}
    // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const State& rhs) const;
  bool isOnGrid(const int width, const int height) const;
  bool isInRange(const State& goal) const;
};

inline void getConfiguration(const State* node, float& x, float& y, float& t) {
  x = node->pos.a[0];
  y = node->pos.a[1];
  t = node->pos.a[2];
}