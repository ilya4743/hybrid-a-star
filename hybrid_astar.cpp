#include "locationmap.h"
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <queue>
#include <helper.h>
#include "hybrid_astar.h"

using namespace boost;
using namespace std;

// R = 6, 6.75 DEG
const float dy[] = { 0,        -0.0415893,  0.0415893};
const float dx[] = { 0.7068582,   0.705224,   0.705224};
const float dt[] = { 0,         0.1178097,   -0.1178097};

/// Вес ребра
typedef boost::property<boost::edge_weight_t, float> weight;
/// Граф (список смежности)
typedef adjacency_list<vecS, vecS, undirectedS, boost::no_property, weight> my_graph;
/// Итератор дуг
typedef boost::graph_traits<my_graph>::edge_iterator edge_iterator;
typedef boost::graph_traits<my_graph>::out_edge_iterator out_edge_iterator;
/// Итератор вершин
typedef boost::graph_traits<my_graph>::vertex_iterator vertex_iterator;
/// Карта весов рёбер
typedef boost::property_map<my_graph, boost::edge_weight_t>::type Weight_Map;
/// Пара итераторов для ребра
typedef std::pair<edge_iterator, edge_iterator> edgePair;
typedef std::pair<out_edge_iterator, out_edge_iterator> outEdgePair;

/// Пара итераторов для вершины
typedef std::pair<vertex_iterator, vertex_iterator> vertexPair;
/// Дескриптор вершин
typedef typename graph_traits<my_graph>::vertex_descriptor vertex_descriptor;
/// Дескриптор рёбер
typedef typename graph_traits<my_graph>::edge_descriptor  edge_descriptor;

template<
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type>
> class MyQueue : public std::priority_queue<T, Container, Compare>
{
public:
    typedef typename
        std::priority_queue<
        T,
        Container,
        Compare>::container_type::const_iterator const_iterator;

    bool contains(const T&val) const
    {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first!=last) {
            if (*first==val) return true;
            ++first;
        }
        return false;
    }
};

template<
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type>
> class MyQueue1 : public std::priority_queue<T, Container, Compare>
{
public:
    typedef typename
        std::priority_queue<
        T,
        Container,
        Compare>::container_type::const_iterator const_iterator;

    bool contains(const T&val) const
    {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first!=last) {
            if (first->second==val.second) return true;
            ++first;
        }
        return false;
    }
};

vec3 expand11(float theta, float beta, float d, vec3 rearWheelPos)
{
    vec3 newRearWheelPos;

    if (abs(beta) < 0.00001f)
    {
        newRearWheelPos.a[0] = newRearWheelPos.a[0] + d * sin(theta);
        newRearWheelPos.a[2] = newRearWheelPos.a[2] + d * cos(theta);
    }
    //Turn
    else
    {
        //Turning radius 
        float R = d / beta;

        float cx = rearWheelPos.a[0] + cos(theta) * R;
        float cz = rearWheelPos.a[2] - sin(theta) * R;

        newRearWheelPos.a[0] = cx - cos(theta + beta) * R;
        newRearWheelPos.a[2] = cz + sin(theta + beta) * R;
    }

    return newRearWheelPos;
}

vec3 createSuccessor(const vec3 e, int i) 
{
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  //if (i < 3) {
    xSucc = e.a[0] + dx[i] * cos(e.a[2]) - dy[i] * sin(e.a[2]);
    ySucc = e.a[1] + dx[i] * sin(e.a[2]) + dy[i] * cos(e.a[2]);
    tSucc = Helper::normalizeHeadingRad(e.a[3] + dt[i]);
  //}
  // backwards
  //else {
  //  xSucc = e.a[0] - dx[i - 3] * cos(e.a[2]) - dy[i - 3] * sin(e.a[2]);
  //  ySucc = e.a[1] - dx[i - 3] * sin(e.a[2]) + dy[i - 3] * cos(e.a[2]);
  //  tSucc = Helper::normalizeHeadingRad(e.a[2] - dt[i - 3]);
  //}
vec3 v={xSucc, ySucc, tSucc};
  return v;
}

float heuristic1(const vec3& a, const vec3& b)
{
    float dx=a.a[0]-b.a[0];
    float dy=a.a[1]-b.a[1];
    float h=sqrt(dx*dx+dy*dy);
    return h;
}

vector<vec3> succ;
vector<vec3> mat;
vector<vec3> path;
list<int> hybrid_atar(my_graph& g, int start, int goal)
{
    try
    {
        start=0;
        goal=1;
        Weight_Map weight_map = get(edge_weight, g);
        std::unordered_map<unsigned int, float> costs;
        std::unordered_map<unsigned int, float> f;
        std::unordered_map<unsigned int, float> pred;
        MyQueue1 <pair<float,int>,vector<pair<float,int>>,greater<pair<float,int>>> o;
        MyQueue <int> c;   
        vec3 vvv{0,0,0};
        mat.push_back(vvv);
        vec3 vvv1{10,10,0};
        mat.push_back(vvv1);

        costs[start]=0;
        o.push({costs[start]+heuristic1(mat[start],mat[goal]),start});
        while(!o.empty())
        {
            int x=o.top().second;
            o.pop();
            c.push(x);
            path.push_back(mat[x]);
            if(x==goal)
            {
                
                list<int> path;
                list<float> cost;
                while(pred[x])
                {                    
                    path.push_front(x);
                    x=pred[x];
                }
                path.push_front(start);
                return path;
            }
            else
            {
                for(int i=0; i<3; i++)
                {
                    succ.push_back(createSuccessor(mat[x],i));
                    add_vertex(g);
                    mat.push_back(succ[succ.size()-1]);
                    int xsuc=num_vertices(g)-1;

                    //int xsuc=target(*e.first,g);
                    if(!c.contains(xsuc))
                    {
                        if(!o.contains({0,xsuc})||costs[x]>costs[x]+1)
                        {                    
                            costs[xsuc]=costs[x]+1;
                            pred[xsuc]=x;
                            //o1[costs[xsuc]+heuristic(d.matrix[x],d.matrix[goal])]=x;
                            if(!o.contains({0,xsuc}))
                                o.push({costs[xsuc]+heuristic1(succ[succ.size()-1],mat[goal]),xsuc});                            
                        }
                    }
                }
                for(int i=0; i<3; i++);

                /*for(outEdgePair e=out_edges(x,g); e.first != e.second; ++e.first)
                {
                    int xsuc=target(*e.first,g);
                    if(!c.contains(xsuc))
                    {
                        if(!o.contains({0,xsuc})||costs[x]>costs[x]+weight_map[*e.first])
                        {                    
                            costs[xsuc]=costs[x]+weight_map[*e.first];
                            pred[xsuc]=x;
                            //o1[costs[xsuc]+heuristic(d.matrix[x],d.matrix[goal])]=x;
                            //if(!o.contains({0,xsuc}))
                                //o.push({costs[xsuc]+heuristic(d.matrix[xsuc],d.matrix[goal]),xsuc});                            
                        }
                    }
                }*/
            }
        }
    }
    catch(const std::exception& e)
    {

    }
}

float Heuristic(const vec2& p1, const vec2& p2)
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
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid_[0].size(), vector<int>(grid_.size())));
  vector<vector<vector<State>>> came_from(
    NUM_THETA_CELLS, vector<vector<State>>(grid_[0].size(), vector<State>(grid_.size())));
  int stack = Theta2Stack(start.a[2]);
  
  int g = 0;
  float h = Heuristic(start, goal);
  float f = g + h;
  State state(g,h,f, start);

  closed[stack][Idx(state.x)][Idx(state.y)] = 1;
  came_from[stack][Idx(state.x)][Idx(state.y)] = state;
  int total_closed = 1;
  vector<State> opened = {state};
  bool finished = false;
  while(!opened.empty()) {
    Sort(&opened); // opened is sorted by f
    State current = opened[0]; 
    opened.erase(opened.begin()); // pop current state
    opend_lists_visualizer_.push_back(opened); // just for visualizer
    int x = current.x;
    int y = current.y;
    double theta = current.theta;
    if(Idx(x) == goal_idx_[0] && Idx(y) == goal_idx_[1] && (RADIAN2DEG*fabs(theta-goal_pos_[2])<THRESHOLD_GOAL_THETA)) {
      std::cout << "found path to goal in " << total_closed << " expansions" 
                << std::endl;
      Path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    vector<State> next_state = Expand(current, goal);

    for(int i = 0; i < next_state.size(); ++i) {
      if (is_collision(next_state[i].pos))
        continue;

      int stack2 = Theta2Stack(next_state[i].pos.a[2]);

      if(closed[stack2][Idx(x2)][Idx(y2)] == 0) {
        opened.push_back(next_state[i]);
        closed[stack2][Idx(x2)][Idx(y2)] = 1;
        came_from[stack2][Idx(x2)][Idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  Path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}