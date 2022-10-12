#include <iostream>
#include <queue>
#include <set>
#include <algorithm>
#include "locationmap.h"
#include <boost/graph/adjacency_list.hpp>

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

int dijkstra(my_graph& g, int start, int goal)
{
    Weight_Map weight_map = get(edge_weight, g);
    float g1=0;

    MyQueue <int> o;
    MyQueue <int> c;
    queue<int> map;
    o.push(start);
    while(!o.empty())
    {
        int x=o.top();
        o.pop();
        c.push(x);
        if(x==goal)
        {
                cout<<endl<<g1;

            return x;
            }
        else
        {
            for(outEdgePair e=out_edges(x,g); e.first != e.second; ++e.first)
            {
                
                cout<<*e.first<<'\t'<<weight_map[*e.first]<<endl;
                int xsuc=target(*e.first,g);
                if(!c.contains(xsuc))
                {
                    float g2=g1+weight_map[*e.first];
                    if(!o.contains(xsuc)||g1>g2)
                    {
                        g1=g2;
                        //cout<<x<<',';
                        if(!o.contains(xsuc))
                            o.push(xsuc);                            
                    }
                }
            }
        }
    }
}

int main()
{
    my_graph g(4);
    DMQuadrangle dis(5,5,22,1);
    dis.init();
    init_g(2,2,g);
    dijkstra(g,3,0);
    return 0;
}