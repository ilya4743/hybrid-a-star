#include "locationmap.h"

using namespace std;

IDistanceMatrix::~IDistanceMatrix(){}

void DistanceMatrix::init(){}

DistanceMatrix::~DistanceMatrix()
{
    matrix.clear();
}

DMQuadrangle::DMQuadrangle():width(0),height(0),center(0),step(0)
{
}

DMQuadrangle::DMQuadrangle(int width, int height, int center, float step):width(width),height(height),center(center),step(step)
{
}

DMQuadrangle::DMQuadrangle(const DMQuadrangle &o):width(o.width),height(o.height), center(o.center), step(o.step)
{
}

void DMQuadrangle::init()
{
    matrix.resize(width*height);
       float x= step, z=0;
       for (int i = center / width; i >= 0; i--)
       {
           for (int j = center % width; j >= 0; j--)
               matrix[i * width + j] = { x-=step,-z,0 };
           x = step;
           z-=step;
       }
       x = -step;
       z = 0;
       for (int i = center / width; i < height; i++)
       {
           for (int j = center % width; j < width; j++)
               matrix[i * width + j] = { x+=step,-z,0};
           x = -step;
           z+=step;
       }z = 0;
       for (int i = center/ width; i >= 0; i--)
       {
           for (int j = center % width; j < width; j++)
               matrix[i * width + j] = { x+=step,z,0 };
           x = -step;
           z+=step;
       }
       x = step;
       z = 0;
       for (int i = center/ width; i <height; i++)
       {
           for (int j = center%width; j >=0; j--)
               matrix[i * width + j] = { x-=step,z,0};
           x = step;
           z-=step;
       }
}

DMQuadrangle::~DMQuadrangle()
{
    matrix.clear();
};