#pragma once

#include <iostream>
#include <math.h>
#include <vector>

class kinematics2Dof
{
private:
    double r1, r2;
    std::vector<double> posRange;
    std::vector<double> angRange;

public:
    kinematics2Dof(std::vector<double> posRange_, std::vector<double> angRnage_);
    int forKinematics2Dof(std::vector<double>& position, const std::vector<double>& angle);
    int invKinematics2Dof(std::vector<double>& angle, const std::vector<double>& position);
};

kinematics2Dof::kinematics2Dof(std::vector<double> posRange_, std::vector<double> angRnage_)
{
    posRange = posRange_;
    angRange = angRnage_;
}

int kinematics2Dof::forKinematics2Dof(std::vector<double>& position, const std::vector<double>& angle)
{

}

int kinematics2Dof::invKinematics2Dof(std::vector<double>& angle, const std::vector<double>& position)
{
    if(!((r1-r2)*(r1-r2) < (position[0]*position[0] + position[1]*position[1]) || (r1+r2)*(r1+r2) > (position[0]*position[0] + position[1]*position[1]))){
        std::cout<<"posion is out of range!!"<<std::endl;
        return 1;
    }
    return 0;
}