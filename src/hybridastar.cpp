#include "hybridastar.h"

void HybridAstarAlgo::searchHybridAStar(float x1, float y1, float t1, float x2, float y2, float t2, int w, int h)
{
    ggrid.resize(w*h);
    grid->info.width=w;
    grid->info.height=h;
    grid->data=ggrid;
    grid->info.resolution=1;

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

    voronoiDiagram->initializeMap(grid->info.width, grid->info.height, grid->data);
    voronoiDiagram->update();
    voronoiDiagram->visualize();

      // TRACE THE PATH
    smoother.tracePath(nSolution);
    smoother.smoothPath(voronoiDiagram);
    //delete voronoiDiagram;
    delete [] nodes3D;
    delete [] nodes2D;
}