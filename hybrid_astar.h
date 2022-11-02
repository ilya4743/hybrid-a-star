#include "state.h"
#include "defs.h"
#include "locationmap.h"
#include "collisiondetection.h"

#include <boost/heap/binomial_heap.hpp>

class HybridAStar
{
    private:
    //float Heuristic(const vec3& p1, const vec3& p2);
    //vector<State> Expand(const State &state, const vec3& goal);
    //bool is_collision(const vec3& p, OccurancyMatrix& mat);
    //int Idx(double float_num) {return int(floor(float_num));}
    
    public:
    State* Search(State& start, const State& goal, State* nodes3D, int width, int height, CollisionDetection& configurationSpace);
};