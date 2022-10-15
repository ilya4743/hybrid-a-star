#include <iostream>
#include <queue>
#include <set>
#include <algorithm>
#include "locationmap.h"
#include <unordered_map>
#include <boost/graph/adjacency_list.hpp>
#include <boost/qvm/vec.hpp>
typedef boost::qvm::vec<float,3> vec3;
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

using namespace std;
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


void init_g(int width, int height, my_graph& g)
{
    Weight_Map weight_map = get(edge_weight, g);

    for (int i = 1; i < height; i++)
        for (int j = 0; j < width; j++)
            weight_map[add_edge(((i - 1) * width) + j, i * width + j, g).first]=1;
        
    for (int i = 0; i < height; i++)
        for (int j = 1; j < width; j++)
            weight_map[add_edge(i * width + j - 1, i * width + j, g).first]=1;
        
    for (int i = 0; i < height - 1; i++)
        for (int j = 0; j < width - 1; j++)
            weight_map[add_edge(i * width + j, i * width + (width + 1) + j, g).first]=1.1;

    for (int i = height - 1; i > 0; i--)
        for (int j = width - 2; j >= 0; j--)
            weight_map[add_edge(i * width + j, i * width - (width - 1) + j, g).first]=1.1;
}

int min_f(const priority_queue<int>& pq)
{
    
}

float heuristic(const vec3& a, const vec3& b)
{
    float dx=a.a[0]-b.a[0];
    float dy=a.a[1]-b.a[1];
    return sqrt(dx*dx+dy*dy);
}

list<int> astar(my_graph& g, int start, int goal, const DMQuadrangle& d)
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
        o.push({costs[start]+heuristic(d.matrix[start],d.matrix[goal]),start});
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
                            //o1[costs[xsuc]+heuristic(d.matrix[x],d.matrix[xsuc])]=x;
                            if(!o.contains({0,xsuc}))
                                o.push({costs[xsuc]+heuristic(d.matrix[x],d.matrix[xsuc]),xsuc});                            
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

list<int> dijkstra(my_graph& g, int start, int goal)
{
    try
    {
        Weight_Map weight_map = get(edge_weight, g);
        std::unordered_map<unsigned int, float> costs;
        std::unordered_map<unsigned int, float> pred;
        MyQueue <int> o;
        MyQueue <int> c;
        o.push(start);
        costs[start]=0;
        while(!o.empty())
        {
            int x=o.top();

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
                        if(!o.contains(xsuc)||costs[x]>costs[x]+weight_map[*e.first])
                        {                    
                            costs[xsuc]=costs[x]+weight_map[*e.first];
                            pred[xsuc]=x;
                            if(!o.contains(xsuc))
                                o.push(xsuc);                            
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

int main()
{
    my_graph g(110);
    DMQuadrangle dis(11,10,105,1);
    dis.init();
    init_g(11,10,g);
    list<int> path=dijkstra(g, 105, 0);
    list<int> path1=astar(g, 105, 0, dis);
    return 0;
}