#include<state.h>

State::State(): g(0), h(0), f(0), pos()
{
}

State::State(int g, float h, float f, const vec3& pos): g(g), h(h), f(f), pos(pos)
{
}

State::State(const State& state): g(state.g), h(state.h), f(state.f), pos(state.pos)
{
}

State::~State()
{
}