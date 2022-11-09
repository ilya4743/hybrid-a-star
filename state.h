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
  State(float x, float y, float theta, float g, float h, State* pred, int prim = 0);

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
   State* pred;

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

  // GETTER METHODS
  /// get the x position
  float getX() const { return pos.a[0]; }
  /// get the y position
  float getY() const { return pos.a[1]; }
  /// get the heading theta
  float getT() const { return pos.a[2]; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->pos.a[0] = x; }
  /// set the y position
  void setY(const float& y) { this->pos.a[1] = y; }
  /// set the heading theta
  void setT(const float& t) { this->pos.a[2] = t; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  // int setIdx(int width, int height) { this->idx = (int)(t / Constants::deltaHeadingRad) * width * height + (int)(y) * width + (int)(x); return idx;}
  // /// open the node
  // void open() { o = true; c = false;}
  // /// close the node
  // void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const State* pred) { this->pred = pred; }

};

inline void getConfiguration(const State* node, float& x, float& y, float& t) {
  x = node->pos.a[0];
  y = node->pos.a[1];
  t = node->pos.a[2];
}