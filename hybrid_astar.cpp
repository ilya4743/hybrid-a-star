#include "hybrid_astar.h"

struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const State* lhs, const State* rhs) const {
    return lhs->c > rhs->c;
  }
};

typedef boost::heap::binomial_heap<State*,boost::heap::compare<CompareNodes>> priorityQueue;

using namespace std;

bool HybridAStar::is_collision(const vec3& p, OccurancyMatrix& mat)
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
    bool out_map = mat.is_out_of_map(vertices[i]);
    if (out_map == true)
    {
      ret = true;
      break;
    }
    bool collision_grid = mat.is_grid_collision(vertices[i]);
    if (collision_grid == true)
    {
      ret = true;
      break;
    }
  }

  return ret;
}

float HybridAStar::Heuristic(const vec3& p1,const vec3& p2)
{
    float dx=p1.a[0]-p2.a[0];
    float dy=p1.a[1]-p2.a[1];
    float euclidian_distance=sqrt(dx*dx+dy*dy);    
    float angle_err = abs(p1.a[2] - p2.a[2]);
    return euclidian_distance+angle_err;
}

// vector<State> HybridAStar::Expand(const State &state, const vec3& goal) {
    
//   int next_g = state.g+1;
//   float next_h;
//   float next_f;
//   vec3 next_pos;
//   vector<State> next_states;

//   for(float delta_i = -MAX_STEERING; delta_i <= MAX_STEERING; delta_i+=5) {
//     // kinematic model
//     float delta = DEG2RADIAN * delta_i;
//     float omega = SPEED / VEHICLE_LEGTH * (delta) * DT;
//     next_pos.a[2] = state.pos.a[2] + omega;
//     if(next_pos.a[2] < 0) {
//       next_pos.a[2] += 2*M_PI;
//     }
//     next_pos.a[0] = state.pos.a[0] + SPEED * cos(state.pos.a[2])* DT;
//     next_pos.a[1] = state.pos.a[1] + SPEED * sin(state.pos.a[2])* DT;

//     next_h = Heuristic(next_pos, goal);
//     next_f = next_g + next_h;
//     State next_state(next_g, next_h, next_f, next_pos);
//     next_states.push_back(next_state);
//   }

//   return next_states;
// }

State* HybridAStar::Search(State& start, const State& goal, State* nodes3D, OccurancyMatrix& matrix)
{  
  int iPred, iSucc;
  float newG;
  int iterations=0;
  int dir=6;
  priorityQueue O;
  
  start.open();
  O.push(&start);

  iPred=start.setIdx(matrix.width, matrix.height);
  nodes3D[iPred]=start;

  State* nPred;
  State* nSucc;

  while(!O.empty()) {
    
    nPred=O.top();
    iPred=nPred->setIdx(matrix.width, matrix.height);
    iterations++;

    if (nodes3D[iPred].isClosed()) {
      O.pop();
      continue;
    }    
    else if (nodes3D[iPred].isOpen()) 
    {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();
    
      if(*nPred==goal||iterations>30000)
      {
        std::cout << "found path to goal in " << " expansions" 
                  << std::endl;
        return nPred;
      }
        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
      for (int i = 0; i < dir; i++) {
        // create possible successor
        nSucc = nPred->createSuccessor(i);
        // set index of the successor
        iSucc = nSucc->setIdx(matrix.width, matrix.height);
        // ensure successor is on grid and traversable
        bool test=!is_collision(nSucc->pos,matrix);
        bool test1=nSucc->isOnGrid(matrix.width, matrix.height);
        if (nSucc->isOnGrid(matrix.width, matrix.height)&&(!is_collision(nSucc->pos,matrix))) {

          // ensure successor is not on closed list or it has the same index as the predecessor
          if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

            // calculate new G value
            nSucc->updateG();
            newG = nSucc->g;

            // if successor not on open list or found a shorter way to the cell
            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].g || iPred == iSucc) {

              // calculate H value
              //updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace);

              // if the successor is in the same cell but the C value is larger
              if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                delete nSucc;
                continue;
              }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
              else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                nSucc->pred=nPred->pred;
              }

              if (nSucc->pred == nSucc) {
                std::cout << "looping";
              }

              // put successor on open list
              nSucc->open();
              nodes3D[iSucc] = *nSucc;
              O.push(&nodes3D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        } else { delete nSucc; }
      }
    }
  }
  
  return nullptr;
}