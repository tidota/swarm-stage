#include "behaviour.h"



Behaviour::Behaviour
(
	void* newdt,
	void (*setLED)(void*,int),
	void (*get_IR_sensors)(void*, double*, int*),
	bool (*chain_found)(void*),
	void (*nearest)(void*, double*,double*,int*,int*,bool*,bool*),
	void (*chainstat)(void*, double*,double*,double*,bool*,double*)
)
{
	state = 0;
	P_in = P_IN_DEF;
	P_out = P_OUT_DEF;
	to_tail = true;

	this->data = newdt;
	this->setLED = setLED;
	this->get_IR_sensors = get_IR_sensors;
	this->chain_found = chain_found;
	this->nearest = nearest;
	this->chainstat = chainstat;

	srand(time(NULL));

	n_stick_follow = 0;
	n_stick_chain = 0;
}

Vec2D Behaviour::getVec()
{
	Vec2D vector;

	// IR sensors
	double IR_sensors[24];
	int n_sensors = 24;
	this->get_IR_sensors(data,IR_sensors,&n_sensors);

	// motor schema: Avoid_collisions
	vector = avoid_collisions(IR_sensors,n_sensors,0.10);

	if(state == 0) // ==================== Search ====================
	{
		bool flag = this->chain_found(data);

		// motor schemas: Move_straight, Random
		vector = vector + move_straight();
		vector = vector + move_random();

		if(flag)
		{
			state = 1; // Explore
			to_tail = true;
		}
	}
	else if(state == 1) // ==================== Explore ====================
	{
		double dist2obj, ang2obj;
		int objclr, dir;
		bool at_nest, prey_found;
		this->nearest(data,&dist2obj,&ang2obj,&objclr,&dir,&at_nest,&prey_found);

		// motor schema: Move_perpendicular (based on dir, to_tail)
		if((to_tail && dir>0) || (to_tail==false && dir<0))
		{
			// counter-clockwise
			vector = vector + move_perpendicular(ang2obj,false);
		}
		else if((to_tail && dir<0) || (to_tail==false && dir>0))
		{
			// clockwise
			vector = vector + move_perpendicular(ang2obj,true);
		}

		// motor schema: Adjust_distance
		if(dist2obj != -1)
			vector = vector + adjust_distance(ang2obj,dist2obj,DESIRED_DIST);

		//printf("||| x= %8.2f, y= %8.2f, ang2obj= %8.2f, dist2obj= %8.2f, desired= %8.2f\n",vector.getX(),vector.getY(),ang2obj,dist2obj,DESIRED_DIST);

		// at nest, start going to the tail
		if(at_nest)
			to_tail = true;

		if(dist2obj == -1)
		{
			state = 0; // Search
		}
		else if(
			dist2obj <= (CHAIN_INTERVAL*1.2) &&
			(dir == 0 || at_nest) && //n_stick_follow >= MAX_STICK_FOLLOW &&
			(P_in > random_dec() || prey_found))
		{
			state = 2; // Chain
			this->setLED(data,(objclr%3)+1); // set next LED color
			if(prey_found)
				P_out = 0;
		}

		if(dir == 0 && state == 1)
		{
			n_stick_follow++;
		}
		else
		{
			n_stick_follow = 0;
		}
	}
	else if(state == 2) // ==================== Chain ====================
	{
		double dist_prev, ang_prev, ang_next, preydist;
		bool at_tail;
		this->chainstat(data,&dist_prev,&ang_prev,&ang_next,&at_tail,&preydist);

		if(at_tail)
		{
			// motor schema: Move_perpendicular
			vector = vector + (move_perpendicular(ang_prev,false) * 0.2);
		}
		else
		{
			// motor schemas: Align
			vector = vector + align(ang_prev,ang_next);
		}
		// motor schemas: Adjust_distance
		if(dist_prev != -1)
			vector = vector + adjust_distance(ang_prev,dist_prev,CHAIN_INTERVAL);

		if(preydist != -1 && preydist <= CHAIN_INTERVAL)
		{
			state = 3; // Success
			n_stick_chain = 0;
			printf("SUCCESS!!!\n");
		}
		else if(at_tail && P_out > random_dec())
		{
			state = 1; // Explore
			this->setLED(data,0);
			to_tail = false;
			n_stick_chain = 0;
		}
		else if(dist_prev == -1)
		{
			if(n_stick_chain < MAX_STICK_CHAIN)
			{
				n_stick_chain++;
			}
			else
			{
				printf("break!: n_stick_chain = %d\n",n_stick_chain);
				state = 0; // Search
				this->setLED(data,0);
				n_stick_chain = 0;
			}
		}
		else
		{
			n_stick_chain = 0;
		}
	}

	return vector;
}
