#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include "Vector.h"

class kinematics2Dof
{
private:
    double r1, r2;
    std::vector<std::vector<double>> angRange;
    bool RunningState;

public:
    kinematics2Dof(double l1, double l2, std::vector<std::vector<double>> angRnage_);
    void forKinematics2Dof(std::vector<double>& position, const std::vector<double>& angle);
    void invKinematics2Dof(std::vector<double>& angle, const std::vector<double>& position);
};

kinematics2Dof::kinematics2Dof(double l1, double l2, std::vector<std::vector<double>> angRnage_)
{
    r1 = l1;
    r2 = l2;
    angRange = angRnage_;
}

void kinematics2Dof::forKinematics2Dof(std::vector<double>& position, const std::vector<double>& angle)
{
    for(int i=0; i<angle.size(); i++){
        if(angle[i]<angRange[i][0] || angle[i]>angRange[i][1]){
            std::cout<<"axis"<<i+1<<"is out of range!!";
            return;
        }
    }

    double c1 = cos(angle[0]);
    double s1 = sin(angle[0]);
    double c12 = cos(angle[0] + angle[1]);
    double s12 = sin(angle[0] + angle[1]);

    position[0] = r1*c1 + r2*c12;
    position[1] = r1*s1 + r2*s12;
}

void kinematics2Dof::invKinematics2Dof(std::vector<double>& angle, const std::vector<double>& position)
{
    leg4::Vector p(position[0], position[1]);

    //check range of position
    if(!((r1-r2)*(r1-r2) < (p.x*p.x + p.y*p.y) || (r1+r2)*(r1+r2) > (position[0]*position[0] + position[1]*position[1]))){
        std::cout<<"posion is out of range!!"<<std::endl;
        RunningState = false;
        return;
    }
    
    angle[1] = acos((p.x*p.x + p.y*p.y - r1*r1 - r2*r2)/(2*r1*r2));
    double s2 = sin(angle[1]);
    double c2 = cos(angle[1]);
    angle[0] = atan2((-r2*s2*p.x + (r1+r2*c2)*p.y),((r1+r2*c2)*p.x + r2*s2*p.y));

    //check range of angle
    for(int i=0; i<angle.size(); i++){
        if(angle[i]<angRange[i][0] || angle[i]>angRange[i][1]){
            std::cout<<"axis"<<i+1<<"is out of range!!";
            RunningState = false;
            return;
        }
    }

    RunningState = true;
}