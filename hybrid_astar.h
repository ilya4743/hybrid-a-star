#include "state.h"
#include "defs.h"
#include "locationmap.h"
#include <queue>
#include <unordered_map>
#include <boost/qvm/vec.hpp>

typedef boost::qvm::vec<float,3> vec3;
typedef boost::qvm::vec<float,2> vec2;

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
            if (first->second==val.second) return true;
            ++first;
        }
        return false;
    }
};

class HybridAStar
{
    private:
    float Heuristic(const vec3& p1, const vec3& p2);
    int Theta2Stack(float theta);
    vector<State> Expand(const State &state, const vec3& goal);
    bool is_collision(const vec3& p, OccurancyMatrix& mat);
    int Idx(double float_num) {
    // Returns the index into the grid for continuous position. So if x is 3.621, 
    //   then this would return 3 to indicate that 3.621 corresponds to array 
    //   index 3.
    return int(floor(float_num));
    }
    public:
    void Search(const vec3& start, const vec3& goal, OccurancyMatrix& matrix);
};