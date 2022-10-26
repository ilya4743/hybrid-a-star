#include <boost/qvm/vec.hpp>
typedef boost::qvm::vec<float,3> vec3;

class State
{
    public:
    State();
    State(int g, float h, float f, vec3 pos);
    State(const State& state);
    ~State();
	int g; // iteration
	float h;
	float f; // cost
	vec3 pos;
};