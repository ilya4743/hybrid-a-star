#pragma once

#define MAX_STEERING 90
#define DEG2RADIAN M_PI/180.0
#define RADIAN2DEG 57.2958
#define DT 1
#define SPEED 1.5
#define NUM_THETA_CELLS 180
#define THRESHOLD_GOAL_THETA 10
#define VEHICLE_LEGTH 0.5
#define VEHICLE_WIDTH 0.3

namespace Constants {

static const float tieBreaker = 0.01;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
/// [m] --- The number of discretizations in heading
static const int headings = 72;
/// [°] --- The discretization value of the heading (goal condition)
static const float deltaHeadingDeg = 360 / (float)headings;
/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
/// [c*M_PI] --- The heading part of the goal condition
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
};