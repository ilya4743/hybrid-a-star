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
  HybridAStar astar;
  OccurancyMatrix mat(15,15,1,grid2);

  int width = mat.width;
  int height = mat.height;
  int depth = 72;
  int length = width * height * depth;
  State* nodes3D = new State[length]();

  float x=0.5;
  float y=0.5;
  float theta=Helper::normalizeHeadingRad(0.0f);
  State start(x,y,theta,0,0,nullptr);

  x=15.5;
  y=15.5;
  theta=Helper::normalizeHeadingRad(M_PI_2);
  State goal(x,y,theta,0,0,nullptr);

  astar.Search(start, goal, nodes3D, mat);
  


  nav_msgs::OccupancyGrid grid;
  grid.info.width=15;
  grid.info.height=15;
  grid.data=grid1;
  grid.
  return 0;
}