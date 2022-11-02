#include <iostream>
#include "locationmap.h"
#include "hybrid_astar.h"
#include "helper.h"
#include <boost/qvm/vec.hpp>

typedef boost::qvm::vec<float,3> vec3;

using namespace std;

vector<int8_t> grid1 = {
  0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,
  0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,1,0,0,0,0,1,1,1,0,
  0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,
  0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,0,
  0,0,1,0,1,0,0,0,0,0,1,1,0,0,0,0,
  0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,
  0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,1,
  0,0,0,0,1,0,1,0,0,0,1,1,1,1,1,1,
  0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,1,
  0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,
  0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
vector<int> grid2 = {
  0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,
  0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,
  0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,
  0,0,1,0,0,0,0,1,0,0,0,0,1,1,1,0,
  0,0,1,0,0,0,1,0,0,0,0,1,1,1,0,0,
  0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,0,
  0,0,1,0,1,0,0,0,0,0,1,1,0,0,0,0,
  0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,
  0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,1,
  0,0,0,0,1,0,1,0,0,0,1,1,1,1,1,1,
  0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,1,
  0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,
  0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
#include <nav_msgs/OccupancyGrid.h>


int main()
{
  vector<int8_t> ggrid;
  ggrid.resize(10000);
  HybridAStar astar;


  int width = 100;
  int height = 100;
  int depth = Constants::headings;
  int length = width * height * depth;
  State* nodes3D = new State[length]();

  float x=20;
  float y=20;
  float theta=Helper::normalizeHeadingRad(0.0f);
  State start(x,y,theta,0,0,nullptr);

  x=30;
  y=20;
  theta=Helper::normalizeHeadingRad(0);
  State goal(x,y,theta,0,0,nullptr);

  //astar.Search(start, goal, nodes3D, mat);
  


nav_msgs::OccupancyGridPtr ptr(new nav_msgs::OccupancyGrid);
  ptr->info.width=100;
  ptr->info.height=100;
  ptr->data=ggrid;

  CollisionDetection cd;
  cd.grid=ptr;
  astar.Search(start, goal, nodes3D, ptr->info.width, ptr->info.height, cd);
  //grid.
  return 0;
}