/////////////////////////////////
// File: pioneer_flocking.cc
// Desc: Flocking behaviour, Stage controller demo
// Created: 2009.7.8
// Author: Richard Vaughan <vaughan@sfu.ca>
// License: GPL
/////////////////////////////////

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "stage.hh"
using namespace Stg;

#include "behaviour.h"
#include "motor_schema.h"

typedef struct
{
	ModelPosition* position;
	ModelRanger* ranger;
	ModelFiducial* fiducial;
	ModelPosition* marker;

	Behaviour *behav;

	double IR_senses[24];

} robot_t;

//conversion functions for colors
Color ind2clr(int index)
{
	if(index == 0)
		return Color(0,0,0);
	else if(index == 1)
		return Color::blue;
	else if(index == 2)
		return Color::green;
	else if(index == 3)
		return Color::yellow;
	else if(index == 4)
		return Color::red;
	else
		return Color(0,0,0);
}
int clr2ind(Color clr)
{
	if(clr == Color::cyan || clr == Color::blue)
		return 1;
	else if(clr == Color::green)
		return 2;
	else if(clr == Color::yellow)
		return 3;
	else if(clr == Color::red)
		return 4;
	else
		return 0;
}

const double VSPEED = 0.1; // meters per second
const double EXPAND_WGAIN = 0.01; // turn speed gain
const double FLOCK_WGAIN = 0.01; // turn speed gain
const double SAFE_DIST = 0.05; // meters
const double SAFE_ANGLE = 0.5; // radians

const double MAX_FORWARD = 0.3;
const double MAX_ROTATE = 90.0;


// forward declare
int RangerUpdate( ModelRanger* mod, robot_t* robot );
int FiducialUpdate( ModelFiducial* fid, robot_t* robot );

// functions to be called back
void setLED(void*,int);
void get_IR_sensors(void*, double*, int*);
bool chain_found(void*);
void nearest(void*, double*,double*,int*,int*,bool*,bool*);
void chainstat(void*, double*,double*,double*,bool*,double*);

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{

	robot_t* robot = new robot_t;
	robot->position = (ModelPosition*)mod->GetUnusedModelOfType( "position" );

	robot->ranger = (ModelRanger*)mod->GetUnusedModelOfType( "ranger" );
	assert( robot->ranger );
	robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdate, robot );
 
	robot->fiducial = (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial" ) ;
	assert( robot->fiducial );
	robot->fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot );

	robot->fiducial->Subscribe();
	robot->ranger->Subscribe();
	robot->position->Subscribe();

	robot->marker = (ModelPosition*)mod->GetUnusedModelOfType( "position" );
	robot->position->SetColor(Color(0,0,0));
	robot->marker->SetColor(robot->position->GetColor());
	robot->marker->SetFiducialReturn(0);

	robot->behav = new Behaviour(robot,setLED,get_IR_sensors,chain_found,nearest,chainstat);

	return 0; //ok
}

int RangerUpdate( ModelRanger* rgr, robot_t* robot )
{
	const std::vector<ModelRanger::Sensor>& sensors = rgr->GetSensors();

	for (unsigned int i = 0; i < 24; i++)
	{
		robot->IR_senses[i] = sensors[i].ranges[0];
	}

	return 0;
}

int FiducialUpdate( ModelFiducial* fid, robot_t* robot )
{	
	//robot->fiducial = fid;
	
	Vec2D vec = robot->behav->getVec();
	
	robot->position->SetSpeed( vel_forward(vec,MAX_FORWARD), 0, vel_rotate(vec,MAX_ROTATE)/180.0*M_PI );

	return 0;
}


// ================================================================================== //
//  callbacks from Behaviour
// ================================================================================== //
void setLED(void* dt,int clrind)
{
	robot_t *robot = (robot_t*)dt;
	
	robot->position->SetColor(ind2clr(clrind));
	robot->marker->SetColor(robot->position->GetColor());
	if(clrind == 0)
		robot->marker->SetFiducialReturn(0);
	else
		robot->marker->SetFiducialReturn(1);
}
void get_IR_sensors(void* dt, double* senses, int* n)
{
	robot_t *robot = (robot_t*)dt;

	int max;
	if(*n < 24)
	{
		max = *n;
	}
	else
	{
		max = 24;
		*n = 24;
	}

	for(int i = 0; i < max; i++)
	{
		senses[i] = robot->IR_senses[i];
	}
}
bool chain_found(void* dt)
{
	robot_t *robot = (robot_t*)dt;
	bool flag = false;
 
	ModelFiducial* fid = robot->fiducial;
	double dist = 100.0;
	ModelFiducial::Fiducial* closest = NULL;

	FOR_EACH( it, fid->GetFiducials() )
	{
		ModelFiducial::Fiducial* detected = &(*it);
		
		if(detected->range < dist && detected->mod->GetColor() != Color::red)
		{
			closest = detected;
			dist = detected->range;
		}
	}

	if(closest != NULL)
		flag = true;
	
	return flag;
}
void nearest(void* dt, double* dist2obj, double* ang2obj, int* objclr, int* dir, bool* at_nest, bool* prey_found)
{
	*dist2obj = -1;
	*dir = 0;
	*at_nest = false;
	*prey_found = false;

	robot_t *robot = (robot_t*)dt;

	ModelFiducial* fid = robot->fiducial;
	double dist = 1000.0;
	double dist_second = 1000.0;
	ModelFiducial::Fiducial* closest = NULL;
	ModelFiducial::Fiducial* closest_second = NULL;

	FOR_EACH( it, fid->GetFiducials() )
	{
		ModelFiducial::Fiducial* detected = &(*it);
		
		if(detected->mod->GetColor() == Color::red)
		{
			*prey_found = true; // prey detected
		}
		else if(detected->range < dist)
		{
			closest_second = closest;
			dist_second = dist;
			closest = detected;
			dist = detected->range;
		}
		else if(detected->range < dist_second)
		{
			closest_second = detected;
			dist_second = detected->range;
		}
	}

	// calculate direction of the path
	if(closest != NULL && closest_second != NULL)
	{
		double ang1 = adjang(closest->bearing/M_PI*180.0);
		double ang2 = adjang(closest_second->bearing/M_PI*180.0);
		
		double diff = ang1 - ang2;
		
		int clrind1 = clr2ind(closest->mod->GetColor());
		int clrind2 = clr2ind(closest_second->mod->GetColor());

		if(-15 < diff && diff < 15)
		{
			*dir = 0;
		}else if((0 <= diff && diff < 180.0) || diff <= -180.0) // L: ang1, R: ang2
		{
			if((clrind1 - clrind2 + 3)%3 == 1)
				*dir = 1;
			else if((clrind2 - clrind1 + 3)%3 == 1)
				*dir = -1;
		}
		else // L: ang2, R: ang1
		{
			if((clrind2 - clrind1 + 3)%3 == 1)
				*dir = 1;
			else if((clrind1 - clrind2 + 3)%3 == 1)
				*dir = -1;
		}
	}

	// infor about the nearest
	if(closest != NULL)
	{
		*dist2obj = closest->range;
		*ang2obj = adjang(closest->bearing/M_PI*180.0);
		*objclr = clr2ind(closest->mod->GetColor());
	}
}
void chainstat(void* dt, double* dist_prev, double* ang_prev, double* ang_next, bool* at_tail, double* preydist)
{
	*dist_prev = -1;
	*at_tail = false;
	*preydist = -1;

	robot_t *robot = (robot_t*)dt;

	int clrind = clr2ind(robot->marker->GetColor());
	int clrind_prev = ((clrind - 1) - 1 + 3) % 3 + 1;
	int clrind_next = ((clrind - 1) + 1 + 3) % 3 + 1;

	ModelFiducial* fid = robot->fiducial;
	double distance_prev = 1000.0;
	double distance_next = 1000.0;
	double rad_prev=0;
	double rad_next=0;
	ModelFiducial::Fiducial* prev = NULL;
	ModelFiducial::Fiducial* next = NULL;

	FOR_EACH( it, fid->GetFiducials() )
	{
		ModelFiducial::Fiducial* detected = &(*it);
		Color clr = detected->mod->GetColor();

		if(clr == Color::red)
		{
			*preydist = detected->range; // prey detected
		}
		if(clr != Color::red && clr2ind(clr)==clrind_prev && detected->range < distance_prev)
		{
			distance_prev = detected->range;
			rad_prev = detected->bearing;
			prev = detected;
		}
		else if((clr == Color::red||clr2ind(clr)==clrind_next) && detected->range < distance_next)
		{
			distance_next = detected->range;
			rad_next = detected->bearing;
			next = detected;
		}
	}


	if(prev != NULL)
	{
		*dist_prev = distance_prev;
		*ang_prev = rad_prev/M_PI*180.0;
	}

	// if next is beyond prev, ignore it
	if(
		prev == NULL ||
		(
			next != NULL &&
			(
				distance_next / distance_prev > 1.0 && 
				diffang(adjang(rad_prev/M_PI*180.0),adjang(rad_next/M_PI*180.0)) < 60.0
			)
		)
    )
	{
		next = NULL;
	}
	
	if(next != NULL)
	{
		*ang_next = rad_next/M_PI*180.0;
	}
	else
	{
		*at_tail = true;
	}
}

