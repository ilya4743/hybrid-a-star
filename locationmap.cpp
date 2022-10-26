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

OccurancyMatrix::OccurancyMatrix():width(0),height(0), step(0)
{
}

OccurancyMatrix::OccurancyMatrix(int width, int height, float step):width(width),height(height),step(step)
{
}

OccurancyMatrix::OccurancyMatrix(const OccurancyMatrix& matrix): width(matrix.width),height(matrix.height),step(matrix.step)//добавить вектор
{
}

OccurancyMatrix::~OccurancyMatrix()
{
    matrix.clear();
}

bool OccurancyMatrix::is_collision(const vec3& p)
{
  float margin = 0.3;
  bool ret = false; 
  float rear_center_x = p.a[0];
  float rear_center_y = p.a[1];
  float front_center_x = rear_center_x + (VEHICLE_LEGTH+margin)*cos(p.a[2]);
  float front_center_y = rear_center_y + (VEHICLE_LEGTH+margin)*sin(p.a[2]);
  float left_rear_x = rear_center_x + (VEHICLE_WIDTH+margin)/2*cos(p.a[2] + M_PI/2);
  float left_rear_y = rear_center_y + (VEHICLE_WIDTH+margin)/2*sin(p.a[2] + M_PI/2);
  float right_rear_x = rear_center_x + (VEHICLE_WIDTH+margin)/2*cos(M_PI/2 - p.a[2]);
  float right_rear_y = rear_center_y - (VEHICLE_WIDTH+margin)/2*sin(M_PI/2 - p.a[2]);

  float left_front_x = front_center_x + (VEHICLE_WIDTH+margin)/2*cos(p.a[2] + M_PI/2);
  float left_front_y = front_center_y + (VEHICLE_WIDTH+margin)/2*sin(p.a[2] + M_PI/2);
  float right_front_x = front_center_x + (VEHICLE_WIDTH+margin)/2*cos(M_PI/2 - p.a[2]);
  float right_front_y = front_center_y - (VEHICLE_WIDTH+margin)/2*sin(M_PI/2 - p.a[2]);
  vector<vec2> vertices;
  vertices.push_back({left_rear_x, left_rear_y});
  vertices.push_back({right_rear_x, right_rear_y});
  vertices.push_back({left_front_x, left_front_y});
  vertices.push_back({right_front_x, right_front_y});
  for (int i = 0 ; i < vertices.size(); i ++)
  {
    bool out_map = is_out_of_map(vertices[i]);
    if (out_map == true)
    {
      ret = true;
      break;
    }
    bool collision_grid = is_grid_collision(vertices[i]);
    if (collision_grid == true)
    {
      ret = true;
      break;
    }
  }

  return ret;
}

bool OccurancyMatrix::is_out_of_map(vec3& p)
{
  if((p.a[0] < 0 || p.a[0] >= (width-1)*step) || 
     (p.a[1] < 0 || p.a[1] >= (height-1)*step))
    return true;
  else
    return false;  
}

bool OccurancyMatrix::is_out_of_map(vec2& p)
{
  if((p.a[0] < 0 || p.a[0] >= (width-1)*step) || 
     (p.a[1] < 0 || p.a[1] >= (height-1)*step))
    return true;
  else
    return false;  
}

int Idx(float float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}

bool OccurancyMatrix::is_grid_collision(vec2& p)
{
  if (matrix[Idx(p.a[0])*width + Idx(p.a[1])] == 1)
    return true;
  else
    return false;
}