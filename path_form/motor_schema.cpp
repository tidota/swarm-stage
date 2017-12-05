#include "motor_schema.h"

// This defines motor schemas and claculations for vectors

// random decimal generator
double random_dec()
{
	return (double)rand()/RAND_MAX;
}

// =============================================== //
// conversion functions for differential drive
// =============================================== //
double vel_forward(Vec2D vec,double max)
{
	double x = vec.getX();
	double y = vec.getY();
	if(x <= 0)
		x = 0;
	//else if(fabs(atan2(y,x)/M_PI*180.0)>15)
	//	x = 0;
	else if(x > max)
		x = max;
	return x;
}
double vel_rotate(Vec2D vec,double max)
{
	double x = vec.getX();
	double y = vec.getY();
	double val;
	//if(fabs(atan2(y,x)/M_PI*180.0)>15)
	if(x <= 0)
	{
		if(y >= 0)
			val = sqrt(x*x+y*y);
		else
			val = -1*sqrt(x*x+y*y);
	}
	else
	{
		val = y;
	}
	val *= 180;
	if(val > max)
		val = max;
	else if(val < -1*max)
		val = -1*max;
	return val;
}


// =============================================== //
// helper functions for agnle calculations
// =============================================== //

// adjust an angle so that it is between -180 to 180
double adjang(double angle)
{
    while(angle < -180.0)
    {
        angle += 360.0;
    }
    while(angle > 180.0)
    {
        angle -= 360.0;
    }

    return angle;
}

// get a vector given an angle (in degree)
// the angle is adjusted if it is less than -180 or more than 180
Vec2D ang2vec(double angle)
{
    Vec2D result;

    while(angle < -180.0)
    {
        angle += 360.0;
    }
    while(angle > 180.0)
    {
        angle -= 360.0;
    }

    result.setVals(cos(angle/180.0*M_PI),sin(angle/180.0*M_PI));

    return result;
}

// get a difference between two angles
double diffang(double ang1, double ang2)
{
    ang1 = adjang(ang1);
    ang2 = adjang(ang2);

    double diff;
    diff = ang1 - ang2;
    if(diff < 0)
    {
        diff = diff + 360;
    }

    if(diff > 180)
    {
        diff = 360.0 - diff;
    }

    return diff;
}

// get a middle angle between the two
double midang(double ang1, double ang2)
{
    ang1 = adjang(ang1);
    ang2 = adjang(ang2);

    double diff;
    diff = ang2 - ang1;
    double mid;

    if(diff < -180)
    {
        mid = ang1 + (diff + 360)/2;
    }
    else if(diff > 180)
    {
        mid = ang1 + (diff - 360)/2;
    }
    else
    {
        mid = ang1 + diff/2;
    }

    mid = adjang(mid);

    return mid;
}

/*
void printvec(const char* str, Vec2D vec)
{
    printf("result (%s): %f, %f\n",str,vec.getX(),vec.getY());
}

int main()
{

    printvec("straight",move_straight());
    printvec("random  ",move_random());
    printvec("random  ",move_random());
    printvec("random  ",move_random());
    printvec("adj dist   0 deg, 10, 15",adjust_distance(0,10,15));
    printvec("adj dist   0 deg, 15, 10",adjust_distance(0,15,10));   
    printvec("adj dist  45 deg, 10, 15",adjust_distance(45,10,15));
    printvec("adj dist  45 deg, 15, 10",adjust_distance(45,15,10));    
    printvec("adj dist  90 deg, 10, 15",adjust_distance(90,10,15));
    printvec("adj dist  90 deg, 15, 10",adjust_distance(90,15,10));    
    printvec("adj dist 180 deg, 10, 15",adjust_distance(180,10,15));
    printvec("adj dist 180 deg, 15, 10",adjust_distance(180,15,10));    
    printvec("adj dist -90 deg, 10, 15",adjust_distance(-90,10,15));
    printvec("adj dist -90 deg, 15, 10",adjust_distance(-90,15,10));    
    printvec("move_perpendicular,   0, true",move_perpendicular(0,true)); 
    printvec("move_perpendicular,  45, true",move_perpendicular(45,true)); 
    printvec("move_perpendicular,  90, true",move_perpendicular(90,true)); 
    printvec("move_perpendicular, 180, true",move_perpendicular(180,true)); 
    printvec("move_perpendicular, -90, true",move_perpendicular(-90,true));
    printvec("move_perpendicular,   0, false",move_perpendicular(0,false)); 
    printvec("move_perpendicular,  45, false",move_perpendicular(45,false)); 
    printvec("move_perpendicular,  90, false",move_perpendicular(90,false)); 
    printvec("move_perpendicular, 180, false",move_perpendicular(180,false)); 
    printvec("move_perpendicular, -90, false",move_perpendicular(-90,false));

    double senses1[8] = { 0.03, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
    double senses2[8] = { 0.1, 0.1, 0.1, 0.1, 0.03, 0.1, 0.1, 0.1 };
    double senses3[8] = { 0.1, 0.03, 0.03, 0.03, 0.1, 0.1, 0.1, 0.1 };
    double senses4[8] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.03, 0.03, 0.03 };

    printvec("avoid_collisions1",avoid_collisions(senses1,8));
    printvec("avoid_collisions2",avoid_collisions(senses2,8));
    printvec("avoid_collisions3",avoid_collisions(senses3,8));
    printvec("avoid_collisions4",avoid_collisions(senses4,8));

    printvec("aling,   0,   30",align(0,30));
    printvec("aling,  30,  150",align(30,150));
    printvec("aling, -60,  -30",align(-60,-30));
    printvec("aling, 100, -100",align(100,-100));
    printvec("aling, 150, -150",align(150,-150));

    printf("adjang(30): %f\n",adjang(30));
    printf("adjang(190): %f\n",adjang(190));
    printf("adjang(-30): %f\n",adjang(-30));
    printf("adjang(-190): %f\n",adjang(-190));
    printf("  30,  60,diff: %f, mid: %f\n",diffang(30,60),midang(30,60));
    printf("  30, -30,diff: %f, mid: %f\n",diffang(30,-30),midang(30,-30));
    printf(" -30, -60,diff: %f, mid: %f\n",diffang(-30,-60),midang(-30,-60));

    return 0;
}
*/

// =============================================== //
// definitions of motor schemas
// =============================================== //

/* ===adjust_distance===
  returns a vector that points towards an object at angle alpha
  if the current distance to the object d_current is larger than
  the desired distance d_desired, and in the opposite direction otherwise.
  The length of the returned vector is proportional to the value of
  |d_current−d_desired|. In order to avoid an oscillating behaviour,
  the vector is set to zero if |d_current − d_desired|<5cm.
*/
Vec2D adjust_distance(double alpha, double d_current, double d_desired)
{
    Vec2D result;

    if(fabs(d_current-d_desired)<0.05)
    {
        result.setVals(0,0);
    }
    else
    {
        result = ang2vec(alpha);
        result = result * (d_current - d_desired);
    }

    return result * GAIN_ADJUST_DISTANCE;
}

/* ===move_perpendicular===
  returns a unit vector that is perpendicular to an object at angle alpha.
  The boolean parameter clockwise determines whether the vector is
  perpendicular in a clockwise sense or not.
*/
Vec2D move_perpendicular(double alpha, bool clockwise)
{
    Vec2D result;

    if(clockwise)
    {
        result = ang2vec(alpha - 90.0);
    }
    else
    {
        result = ang2vec(alpha + 90.0);
    }

    return result * GAIN_PERPENDICULAR;
}

/* ===avoid_collisions===
  returns a vector that takes into account each activation of an IR sensor
  that is above a threshold. The direction of the vector is opposite to
  the direction of the sensor with maximum activation, and its length is
  proportional to the difference between the activation and the threshold.
*/
Vec2D avoid_collisions(double* IR_sensors, int n_sensors, double max_range)
{
    Vec2D result;
    const double threshold = 1.0;
    result.setVals(0,0);

    for(int i = 0; i < n_sensors; i++)
    {
        if(IR_sensors[i]/max_range < threshold)
        {
            double angle = 360.0*i/n_sensors;
            result = result + (ang2vec(angle) * (IR_sensors[i]/max_range - threshold));
        }
    }

    return result * GAIN_COLLISIONS;
}

/* ===move_random===
  returns a random unit vector.
*/
Vec2D move_random()
{
    Vec2D result;
    //result.setVals(2.0*(random_dec()-0.5),2.0*(random_dec()-0.5));
    result.setVals(0,2.0*(random_dec()-0.5));
    return result * GAIN_RANDOM;
}

/* ===move_straight===
  returns a unit vector that points forward.
*/
Vec2D move_straight()
{
    Vec2D result;
    result.setVals(1,0);
    return result * GAIN_STRAIGHT;
}

/* ===align===
  returns a vector that leads to the alignment between the previous and
  the next chain neighbour which are perceived at the angles alpha_previous
  and alpha_next. The length of the vector is proportional to the value of
  180◦ − |alpha_previous − alpha_next|. In order to avoid an oscillating
  behaviour, the vector is set to zero
  if |alpha_previous − alpha_next| > 170◦
  (with 180◦ representing perfect alignment).
*/
Vec2D align(double alpha_previous, double alpha_next)
{
    Vec2D result;
    
    double diff = diffang(alpha_previous,alpha_next);
    double gain = 180.0 - diff;
    double mid = midang(alpha_previous,alpha_next);

    if(diff > 170.0)
    {
        result.setVals(0,0);
    }
    else
    {
        result = ang2vec(mid) * gain;
    }
    
    return result * GAIN_ALIGN;
}

