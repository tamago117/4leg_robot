#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include "Vector.h"

class kinematics3Dof
{
private:
    double r0, r1, r2 ;
    std::vector<std::vector<double>> angRange;
    bool RunningState;

public:
    kinematics3Dof(std::vector<double> r, std::vector<std::vector<double>> angRnage_);
    double getDomain(double  x,double y, double  z);
    void forKinematics3Dof(std::vector<double>& position, const std::vector<double>& angle);
    void invKinematics3Dof(std::vector<double>& angle, const std::vector<double>& position);
};

kinematics3Dof::kinematics3Dof(std::vector<double> r, std::vector<std::vector<double>> angRnage_)
{
    r0 = r[0];
    r1 = r[1];
    r2 = r[2];
    angRange = angRnage_;
}

double kinematics3Dof::getDomain(double  x,double y, double  z)
{

    double D = (pow(-x, 2) + pow(y, 2) + pow(-z, 2) - pow(r0, 2) - pow(r1, 2) - pow(r2, 2))/(2*r1*r2);

    if (D > 1.0){
        D = 1.0;
    }else if(D < -1.0){
        D = -1.0;
    }

    return D;
}

void kinematics3Dof::forKinematics3Dof(std::vector<double>& position, const std::vector<double>& angle)
{
    for(int i=0; i<angle.size(); i++){
        if(angle[i]<angRange[i][0] || angle[i]>angRange[i][1]){
            std::cout<<"axis"<<i+1<<"is out of range!!";
            return;
        }
    }

    double c1 = cos(angle[0]);
    double s1 = sin(angle[0]);
    double c2 = cos(angle[1]);
    double s2 = sin(angle[1]);
    double c23 = cos(angle[1] + angle[2]);
    double s23 = sin(angle[1] + angle[2]);

    position[0] = c1*(r0 + r1*c2 + r2*c23);
    position[1] = s1*(r0 + r1*c2 + r2*c23);
    position[2] = r1*s2 + r2*s23;
}

void kinematics3Dof::invKinematics3Dof(std::vector<double>& angle, const std::vector<double>& position)
{
    leg4::Vector p(position[0], position[1], position[2]);
    double D = getDomain(p.x, p.y, p.z);
    double sqrt_component = pow(p.y, 2) + pow(-p.z, 2) - pow(r0, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
    
    angle[0] = -atan2(p.z, p.y) - atan2(sqrt(sqrt_component),-r0);
    angle[2] = atan2(-sqrt(1.0 - pow(D, 2)),D);
    double s3 = sin(angle[2]);
    double c3 = cos(angle[2]);
    angle[1] = atan2(-p.x, sqrt(sqrt_component)) - atan2(r2 * s3,r1 + r2 * c3);

    angle[0] = angle[0];
    angle[1] = -angle[1] + M_PI/2;
    angle[2] = -angle[2];

    //check range of angle
    for(int i=0; i<angle.size(); i++){
        if(angle[i]<angRange[i][0] || angle[i]>angRange[i][1]){
            std::cout<<"axis"<<i+1<<" is out of range!!"<<std::endl;
            RunningState = false;
            return;
        }
    }

    RunningState = true;
}

//r0=0 only
/*void kinematics3Dof::invKinematics3Dof(std::vector<double>& angle, const std::vector<double>& position)
{
    leg4::Vector p(position[0], position[1], position[2]);

    //check range of position
    if(!((r1-r2)*(r1-r2) < (p.x*p.x + p.y*p.y + p.z*p.z) || (r1+r2)*(r1+r2) > (p.x*p.x + p.y*p.y + p.z*p.z))){
        std::cout<<"posion is out of range!!"<<std::endl;
        RunningState = false;
        return;
    }
    if(p.x >= 0){
        angle[0] = atan2(p.y, p.x);
        angle[2] = acos((p.x*p.x + p.y*p.y + p.z*p.z - r1*r1 - r2*r2)/(2*r1*r2));
        double s3 = sin(angle[2]);
    double c3 = cos(angle[2]);
    angle[1] = atan2((-r2*s3*sqrt(p.x*p.x + p.y*p.y) + (r1+r2*c3)*p.z),((r1 + r2*c3)*sqrt(p.x*p.x + p.y*p.y) + r2*s3*p.z));
        
    }else{
        angle[0] = atan2(p.y, p.x)-3.14;
        angle[2] = acos((p.x*p.x + p.y*p.y + p.z*p.z - r1*r1 - r2*r2)/(2*r1*r2));
        double s3 = sin(angle[2]);
    double c3 = cos(angle[2]);
    angle[1] = atan2((r2*s3*sqrt(p.x*p.x + p.y*p.y) + (r1+r2*c3)*p.z),(-(r1 + r2*c3)*sqrt(p.x*p.x + p.y*p.y) + r2*s3*p.z));
    }
    
    
    

    //check range of angle
    for(int i=0; i<angle.size(); i++){
        if(angle[i]<angRange[i][0] || angle[i]>angRange[i][1]){
            std::cout<<"axis"<<i+1<<"is out of range!!"<<std::endl;
            RunningState = false;
            return;
        }
    }

    RunningState = true;
}*/