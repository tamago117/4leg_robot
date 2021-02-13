#pragma once

#include <math.h>

namespace leg4 {

class Vector
{
private:
    
public:
    double x,y,z = 0;
    double absVal();

    Vector(double x, double y);
    Vector(double x, double y, double z);
    Vector(){}
    
    Vector operator+(Vector vec);
    Vector operator-(Vector vec);
    Vector operator*(double scalar);

    Vector operator+=(Vector vec);
    Vector operator-=(Vector vec);
    Vector operator*=(double scalar);

    Vector operator=(Vector vec);

    bool operator==(Vector vec);
    bool operator!=(Vector vec);

    //Vector rot(double yaw);
    void clear();

};



Vector::Vector(double init_x, double init_y) : x(init_x),y(init_y)
{

}

Vector::Vector(double init_x, double init_y, double init_z) : x(init_y),y(init_y),z(init_z)
{

}

double Vector::absVal(){
    return sqrt(x*x + y*y + z*z);
}

Vector Vector::operator+(Vector vec)
{
    Vector res;
    res.x = this->x + vec.x;
    res.y = this->y + vec.y;
    res.z = this->z + vec.z;

    return res;
}

Vector Vector::operator-(Vector vec)
{
    Vector res;
    res.x = this->x - vec.x;
    res.y = this->y - vec.y;
    res.z = this->z - vec.z;

    return res;
}

Vector Vector::operator*(double scalar)
{
    Vector res;
    res.x = x * scalar;
    res.y = y * scalar;
    res.z = z * scalar;

    return res;
}

Vector Vector::operator+=(Vector vec)
{
    x = this->x + vec.x;
    y = this->y + vec.y;
    z = this->z + vec.z;

    return *this;
}

Vector Vector::operator-=(Vector vec)
{
    x = this->x - vec.x;
    y = this->y - vec.y;
    z = this->z - vec.z;

    return *this;
}

Vector Vector::operator*=(double scalar)
{
    x = x * scalar;
    y = y * scalar;
    z = z * scalar;

    return *this;
}

Vector Vector::operator=(Vector vec)
{
    this->x=vec.x;
    this->y=vec.y;
    this->z=vec.z;
    return *this;
}

bool Vector::operator==(Vector vec)
{
    if(x==vec.x&&y==vec.y){
        return true;
    }else{
        return false;
    }
}

bool Vector::operator!=(Vector vec)
{
    if(x==vec.x&&y==vec.y){
        return false;
    }else{
        return true;
    }
}

void Vector::clear()
{
    x=0;
    y=0;
    z=0;
}

}