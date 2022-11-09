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
  static const bool reverse = true;
static const bool dubins = false;
static const bool twoD = true;
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
/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;
/// A structure capturing the lookup for each theta configuration

struct relPos {
  /// the x position relative to the center
  int x;
  /// the y position relative to the center
  int y;
};

struct config {
  /// the number of cells occupied by this configuration of the vehicle
  int length;
  /*!
     \var relPos pos[64]
     \brief The maximum number of occupied cells
     \todo needs to be dynamic
  */
  relPos pos[64];
};

// ______________________
// DUBINS LOOKUP SPECIFIC

/// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
static const int dubinsWidth = 15;
/// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
static const int dubinsArea = dubinsWidth * dubinsWidth;

static const float cellSize = 1;

/// [m] --- Uniformly adds a padding around the vehicle
static const double bloating = 0;
/// [m] --- The width of the vehicle
static const double width = 1.75 + 2 * bloating;
/// [m] --- The length of the vehicle
static const double length = 2.65 + 2 * bloating;
/// [m] --- The minimum turning radius of the vehicle
static const float r = 6;

/// [m] -- The bounding box size length and width to precompute all possible headings
static const int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);

/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
static const float dubinsStepSize = 1;

/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = true;
};