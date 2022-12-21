#include <iostream>
#include "hybridastar.h"
#include "occupancygrid.h"
using namespace std;
int main()
{
  HybridAstarAlgo astar;
  OccupancyGrid g(float(100.0),float(100.0),float(1.0));
  for(int i=0; i<50;i++)
  g.data[i]=100;
  int i=0;
  while(i!=99){
  auto t=astar.searchHybridAStar(10,10,1.57,20,20,1.57,OccupancyGrid(float(100.0),float(100.0),float(1.0)));
i++;}
  return 0;
}