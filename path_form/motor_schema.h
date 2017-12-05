#ifndef _MOTOR_SCHEMA_H
#define _MOTOR_SCHEMA_H

#include "vec2d.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>


// ============== declarations of motor schemas and claculations for vectors ==============

// conventions:
//  angle in degrees
//  unit of distance is in meters

// random decimal generator
double random_dec();

// prototypes of helper functions
double adjang(double angle);
Vec2D ang2vec(double angle);
double diffang(double ang1, double ang2);
double midang(double ang1, double ang2);

/* === conversions from Vec2D to velocities for differential drive ===
*/
double vel_forward(Vec2D vec,double max);
double vel_rotate(Vec2D vec,double max);

// weights for motor schemas
#define GAIN_ADJUST_DISTANCE 0.5
#define GAIN_PERPENDICULAR 0.1
#define GAIN_COLLISIONS 1.0
#define GAIN_RANDOM 0.5
#define GAIN_STRAIGHT 0.1
#define GAIN_ALIGN 0.008

/* ===adjust_distance===
  returns a vector that points towards an object at angle alpha
  if the current distance to the object d_current is larger than
  the desired distance d_desired, and in the opposite direction otherwise.
  The length of the returned vector is proportional to the value of
  |d_current−d_desired|. In order to avoid an oscillating behaviour,
  the vector is set to zero if |d_current − d_desired|<5cm.
*/
Vec2D adjust_distance(double alpha, double d_current, double d_desired);

/* ===move_perpendicular===
  returns a unit vector that is perpendicular to an object at angle alpha.
  The boolean parameter clockwise determines whether the vector is
  perpendicular in a clockwise sense or not.
*/
Vec2D move_perpendicular(double alpha, bool clockwise);

/* ===avoid_collisions===
  returns a vector that takes into account each activation of an IR sensor
  that is above a threshold. The direction of the vector is opposite to
  the direction of the sensor with maximum activation, and its length is
  proportional to the difference between the activation and the threshold.
*/
Vec2D avoid_collisions(double* IR_sensors, int n_sensors, double max_range);

/* ===move_random===
  returns a random unit vector.
*/
Vec2D move_random();

/* ===move_straight===
  returns a unit vector that points forward.
*/
Vec2D move_straight();

/* ===align===
  returns a vector that leads to the alignment between the previous and
  the next chain neighbour which are perceived at the angles αprevious
  and αnext. The length of the vector is proportional to the value of
  180◦ − |alpha_previous − alpha_next|. In order to avoid an oscillating
  behaviour, the vector is set to zero
  if |alpha_previous − alpha_next| > 170◦
  (with 180◦ representing perfect alignment).
*/
Vec2D align(double alpha_previous, double alpha_next);

#endif // _MOTOR_SCHEMA_H
