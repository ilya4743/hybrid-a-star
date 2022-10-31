#include "locationmap.h"

using namespace std;

// IDistanceMatrix::~IDistanceMatrix(){}

// void DistanceMatrix::init(){}

// DistanceMatrix::~DistanceMatrix()
// {
//     matrix.clear();
// }

// DMQuadrangle::DMQuadrangle():width(0),height(0),center(0),step(0)
// {
// }

// DMQuadrangle::DMQuadrangle(int width, int height, int center, float step):width(width),height(height),center(center),step(step)
// {
// }

// DMQuadrangle::DMQuadrangle(const DMQuadrangle &o):width(o.width),height(o.height), center(o.center), step(o.step)
// {
// }

// void DMQuadrangle::init()
// {
//     matrix.resize(width*height);
//        float x= step, z=0;
//        for (int i = center / width; i >= 0; i--)
//        {
//            for (int j = center % width; j >= 0; j--)
//                matrix[i * width + j] = { x-=step,-z,0 };
//            x = step;
//            z-=step;
//        }
//        x = -step;
//        z = 0;
//        for (int i = center / width; i < height; i++)
//        {
//            for (int j = center % width; j < width; j++)
//                matrix[i * width + j] = { x+=step,-z,0};
//            x = -step;
//            z+=step;
//        }z = 0;
//        for (int i = center/ width; i >= 0; i--)
//        {
//            for (int j = center % width; j < width; j++)
//                matrix[i * width + j] = { x+=step,z,0 };
//            x = -step;
//            z+=step;
//        }
//        x = step;
//        z = 0;
//        for (int i = center/ width; i <height; i++)
//        {
//            for (int j = center%width; j >=0; j--)
//                matrix[i * width + j] = { x-=step,z,0};
//            x = step;
//            z-=step;
//        }
// }

// DMQuadrangle::~DMQuadrangle()
// {
//     matrix.clear();
// };

OccurancyMatrix::OccurancyMatrix():width(0),height(0), step(0)
{
}

OccurancyMatrix::    OccurancyMatrix(int width, int height, float step, const vector<int>& matrix):width(width),height(height),step(step), matrix(matrix)
{
}

OccurancyMatrix::OccurancyMatrix(const OccurancyMatrix& matrix): width(matrix.width),height(matrix.height),step(matrix.step), matrix(matrix.matrix)
{
}

OccurancyMatrix::~OccurancyMatrix()
{
    matrix.clear();
}

bool OccurancyMatrix::is_out_of_map(const vec2& p)
{
  if((p.a[0] < 0 || p.a[0] >= (width-1)*step) || 
     (p.a[1] < 0 || p.a[1] >= (height-1)*step))
    return true;
  else
    return false;  
}

int OccurancyMatrix::Idx(float float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}

bool OccurancyMatrix::is_grid_collision(const vec2& p)
{
  if (matrix[Idx(p.a[0])*width + Idx(p.a[1])] == 1)
    return true;
  else
    return false;
}