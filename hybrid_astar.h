#include <list>
#include "state.h"
#include "defs.h"
#include "locationmap.h"
#include <boost/graph/adjacency_list.hpp>
using namespace boost;

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

std::list<int> hybrid_atar(my_graph& g, int start, int goal);

class HybridAStar
{
    vector<State> Expand(const State &state, const vec3& goal);
    void Search(const vec3& start, const vec3& goal, const OccurancyMatrix& matrix) 
    float Heuristic(const vec2& p1, const vec2& p2);
    int HybridAStar::Theta2Stack(float theta);
};