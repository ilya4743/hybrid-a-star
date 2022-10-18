#include "locationmap.h"
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <queue>

using namespace boost;
using namespace std;

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

list<int> hybrid_atar(my_graph& g, int start, int goal, const DMQuadrangle& d)
{
    try
    {
        Weight_Map weight_map = get(edge_weight, g);
        std::unordered_map<unsigned int, float> costs;
        std::unordered_map<unsigned int, float> f;
        std::unordered_map<unsigned int, float> pred;
        MyQueue1 <pair<float,int>,vector<pair<float,int>>,greater<pair<float,int>>> o;
        MyQueue <int> c;        
        costs[start]=0;
        //o.push({costs[start]+heuristic(d.matrix[start],d.matrix[goal]),start});
        while(!o.empty())
        {
            int x=o.top().second;
            o.pop();
            c.push(x);
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
                for(outEdgePair e=out_edges(x,g); e.first != e.second; ++e.first)
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
                }
            }
        }
    }
    catch(const std::exception& e)
    {

    }
}
