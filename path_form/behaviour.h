#ifndef _BEHAVIOUR_H
#define _BEHAVIOUR_H

#include "vec2d.h"
#include "motor_schema.h"

#include <cstdlib>
#include <ctime>

#define P_IN_DEF (0.3)
#define P_OUT_DEF (0.0)


#define CHAIN_INTERVAL (0.3)
#define DESIRED_DIST (0.3)

#define MAX_STICK_FOLLOW 0
#define MAX_STICK_CHAIN 5

double random_dec();

/*
Behaviours

Search
	required infor
		flag = chain_found()
		IR sensors

	schemas
		Move_straight, Random, Avoid_collisions

	flag
		=> Explore (* to_tail = T)

Explore
	required infor
		nearest(dist,ang,clr,dir,at_nest,prey_found)
		IR sensors

	schemas
		Move_perpendicular (based on dir, to_tail), Adjust_distance, Avoid_collisions
	
	at_nest = T
		to_tail = T

	dist = -1
		=> Search
	dist < & (dir = 0 | at_nest) & (P_in | prey_found)
		=> Chain (*LED on, and if prey_found, P_out = 0)

Chain
	required infor
		chainstat(dist_prev,ang_prev,ang_next,at_tail,preydist)
		IR sensors

	schemas
		if at tail
			Move_perpendicular, Adjust_distance, Align, Avoid_collisions
		else
			Adjust_distance, Align, Avoid_collisions

	preydist <
		=> Success
	at_tail = T & P_out
		=> Explore (*LED off, to_tail = F)
	dist_prev <
		=> Search (*LED off)
*/

class Behaviour
{
private:
	int state;
	//0: search
	//1: explore
	//2: chain
	//3: success

	// probabilities to join/leave a chain
	double P_in;
	double P_out;

	// indicates whether the robot is going to the tail
	bool to_tail;

	// ------------------------------
	// trials after losing members
	int n_stick_follow;
	// trials after being lost in Chain state
	int n_stick_chain;

	// ------------------------------

	// address to data section in the simulator
	void* data;

	// set LED light
	//0: off
	//1: blue
	//2: green
	//3: yellow
	//4: red
	void (*setLED)(void*, int);

	// get IR sensors
	void (*get_IR_sensors)(void*, double*, int*);


	//flag = chain_found()
	bool (*chain_found)(void*);

	//nearest(dist,ang,clr,dir,at_nest,prey_found)
	//clr
	//dir: + for left, - for right
	void (*nearest)(void*, double*,double*,int*,int*,bool*,bool*);

	//chainstat(dist_prev,ang_prev,ang_next,at_tail,preydist)
	void (*chainstat)(void*, double*,double*,double*,bool*,double*);


public:
	Behaviour
	(
		void* newdt,
		void (*setLED)(void*,int),
		void (*get_IR_sensors)(void*, double*, int*),
		bool (*chain_found)(void*),
		void (*nearest)(void*, double*,double*,int*,int*,bool*,bool*),
		void (*chainstat)(void*, double*,double*,double*,bool*,double*)
	);

	Vec2D getVec();
};

#endif //_BEHAVIOUR_H
