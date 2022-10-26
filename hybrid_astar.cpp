#include "hybrid_astar.h"

using namespace std;

bool HybridAStar::is_collision(const vec3& p, const OccurancyMatrix& mat)
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

vector<State> HybridAStar::Expand(const State &state, const vec3& goal) {
    
  int next_g = state.g+1;
  float next_h;
  float next_f;
  vec3 next_pos;
  vector<State> next_states;

  for(float delta_i = -MAX_STEERING; delta_i <= MAX_STEERING; delta_i+=5) {
    // kinematic model
    float delta = DEG2RADIAN * delta_i;
    float omega = SPEED / VEHICLE_LEGTH * (delta) * DT;
    next_pos.a[2] = state.pos.a[2] + omega;
    if(next_pos.a[2] < 0) {
      next_pos.a[2] += 2*M_PI;
    }
    next_pos.a[0] = state.pos.a[0] + SPEED * cos(state.pos.a[2])* DT;
    next_pos.a[1] = state.pos.a[1] + SPEED * sin(state.pos.a[2])* DT;

    next_h = Heuristic(next_pos, goal);
    next_f = next_g + next_h;
    State next_state(next_g, next_h, next_f, next_pos);
    next_states.push_back(next_state);
  }

  return next_states;
}

int HybridAStar::Theta2Stack(float theta){
  // Takes an angle (in radians) and returns which "stack" in the 3D 
  //   configuration space this angle corresponds to. Angles near 0 go in the 
  //   lower stacks while angles near 2 * pi go in the higher stacks.
  float new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) 
                   % NUM_THETA_CELLS;

  return stack_number;
}

void HybridAStar::Search(const vec3& start, const vec3& goal, const OccurancyMatrix& matrix) 
{
  int goal_idx_x=int(floor(goal.a[0]));
  int goal_idx_y=int(floor(goal.a[1]));

  std::unordered_map<unsigned int, float> costs;
  std::unordered_map<unsigned int, float> f;
  std::unordered_map<unsigned int, float> pred;
  std::unordered_map<unsigned int, State> states;

  MyQueue <pair<float,int>,vector<pair<float,int>>,greater<pair<float,int>>> o;
  MyQueue <pair<float,int>,vector<pair<float,int>>,greater<pair<float,int>>> c;

  int stack = Theta2Stack(start.a[2]);

  int g = 0;
  float h = Heuristic(start, goal);
  float ff = g + h;
  State state(g,h,ff, start);
  
  o.push({ff,0});
  states[0]=state;
  bool finished = false;
  while(!o.empty()) {
    auto x=o.top();
    o.pop();
    c.push(x);

    o.pop(); // pop current state    
    State current = states[x.second]; 

    //opend_lists_visualizer_.push_back(opened); // just for visualizer

    if(Idx(current.pos.a[0]) == goal_idx_x && Idx(current.pos.a[1]) == goal_idx_y && (RADIAN2DEG*fabs(current.pos.a[2]-goal.a[2])<THRESHOLD_GOAL_THETA)) 
    {
      std::cout << "found path to goal in " << " expansions" 
                << std::endl;
      //return path;
    }

    vector<State> next_state = Expand(current, goal);

    for(int i = 0; i < next_state.size(); ++i) {
      if (is_collision(next_state[i].pos))
        continue;

      int stack2 = Theta2Stack(next_state[i].pos.a[2]);
      // if(!c.contains(xsuc))
      // {

      // }
      // if(closed[stack2][Idx(x2)][Idx(y2)] == 0) {
      //   opened.push_back(next_state[i]);
      //   closed[stack2][Idx(x2)][Idx(y2)] = 1;
      //   came_from[stack2][Idx(x2)][Idx(y2)] = current;
      //   ++total_closed;
      // }
    }
  }

  // std::cout << "no valid path." << std::endl;
  // Path path;
  // path.came_from = came_from;
  // path.closed = closed;
  // path.final = state;

  // return path;
}