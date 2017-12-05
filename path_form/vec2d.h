#ifndef _VEC2D_H
#define _VEC2D_H

// class Vec2D defines a vector in two-dimensional world.
class Vec2D
{
private:
    double x, y;

public:
    void setVals(double newx, double newy){ x = newx; y = newy; }
    double getX(){ return x; }
    double getY(){ return y; }

    friend Vec2D operator+(Vec2D vec1, Vec2D vec2)
    {
        Vec2D result;
        result.x = vec1.x + vec2.x;
        result.y = vec1.y + vec2.y;

        return result;
    }
    friend Vec2D operator*(Vec2D vec, double norm)
    {
        Vec2D result;
        result.x = vec.x * norm;
        result.y = vec.y * norm;

        return result;
    }
    friend Vec2D operator*(double norm, Vec2D vec)
    {
        Vec2D result;
        result.x = vec.x * norm;
        result.y = vec.y * norm;

        return result;
    }
};

#endif //_VEC2D_H
