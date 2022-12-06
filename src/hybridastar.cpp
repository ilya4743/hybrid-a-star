#include "hybridastar.h"

std::vector<Ogre::Vector3> HybridAstarAlgo::searchHybridAStar(float x1, float y1, float t1, float x2, float y2, float t2, int w, int h, const OccurancyGrid& occurancy)
{
    grid->info.width=occurancy.width;
    grid->info.height=occurancy.height;
    grid->data=occurancy.data;
    grid->info.resolution=occurancy.resolution;
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;

    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    x1 = x1 / Constants::cellSize;
    y1 = y1 / Constants::cellSize;
    t1 = Helper::normalizeHeadingRad(t1);
    Node3D nStart(x1, y1, t1, 0, 0, nullptr);

    x2 = x2 / Constants::cellSize;
    y2 = y2 / Constants::cellSize;
    t2 = Helper::normalizeHeadingRad(t2);
    const Node3D nGoal(x2, y2, t2, 0, 0, nullptr);

    cd.updateGrid(grid);

    Node3D* nSolution=Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, grid->info.width, grid->info.height, cd,dubinsLookup);

    //voronoiDiagram->initializeMap(grid->info.width, grid->info.height, grid->data);
    //voronoiDiagram->update();
    //voronoiDiagram->visualize();

      // TRACE THE PATH
    smoother.tracePath(nSolution);
    //smoother.smoothPath(voronoiDiagram);
    //delete voronoiDiagram;
    delete [] nodes3D;
    delete [] nodes2D;
    std::vector<Ogre::Vector3> out;
    auto p=smoother.getPath();
    out.reserve(p.size());
    for(int i=0; i<smoother.getPath().size(); i++)
    {
      Ogre::Vector3 ve(p[i].getX()*Constants::cellSize,p[i].getT(), p[i].getY()*Constants::cellSize);
      out.push_back(ve);
    }
    return out;
}