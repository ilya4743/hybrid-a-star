#include "node3d.h"
#include "algorithm.h"
#include "helper.h"
#include <nav_msgs/OccupancyGrid.h>
#include <smoother.h>
#include "path.h"
#include <vector>
#include "dynamicvoronoi.h"

using namespace std;
using namespace HybridAStar;

class HybridAstar
{
    private:
        DynamicVoronoi voronoiDiagram;
        nav_msgs::OccupancyGridPtr grid;
        CollisionDetection cd;
        vector<int8_t> ggrid;
        float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
        Smoother smoother;
        Path path;
    public:
    void searchHybridAStar(float x1, float y1, float t1, float x2, float y2, float t2, int w, int h);
};