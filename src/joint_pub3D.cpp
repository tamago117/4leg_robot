#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <math.h>
#include <chrono>
#include <iostream>
#include "velTrapezoidalRule.h"
#include "kinematics3Dof.h"


const std::vector<double> r{0.035, 0.13, 0.127};
const int dofNum = 12;
const std::vector<std::vector<double>> angRange{{-3.14,3.14},{-3.14,3.14},{-3.14,3.14}};
//const std::vector<std::vector<double>> tarPos{{0.1, 0, 0.1}, {0.1, -0.02, 0.18}, {0.1, 0.02, 0.16}, {0.05, 0, 0.18}};
const std::vector<std::vector<double>> tarPos{{0.1, 0, 0}, {0.15, 0, 0}};
//const std::vector<std::vector<double>> tarPos{{0, 0, 0.1}, {0, 0, 0.15}};
//const std::vector<std::vector<double>> tarPos{{0, 0.1, 0}, {0, 0.15, 0}};
//const std::vector<std::vector<double>> tarPos{{0, 0.1, 0.1}};


//accelTimeRate range:0-0.5
const double accelTimeRate = 0.5;
const double moveTime = 1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_pub3D");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    velTrapezoidalRule tra[3] = {velTrapezoidalRule(accelTimeRate, moveTime), velTrapezoidalRule(accelTimeRate, moveTime), velTrapezoidalRule(accelTimeRate, moveTime)};
    kinematics3Dof kin3d(r, angRange);

    ros::Rate loop_rate(100);
    int count = 0;
    std::vector<double> diffAngle(dofNum);
    std::vector<double> omegaMax(dofNum);
    std::vector<double> angularAcc(dofNum);
    std::vector<double> decT(dofNum);
    std::vector<double> tarAngle(dofNum);
    std::vector<float> startAngle{0,0,0};
    std::vector<double> angle{0, 0, 0};

    double cou=0;
    while(ros::ok())
    {
        sensor_msgs::JointState j;
        j.header.stamp = ros::Time::now();
        j.name.resize(dofNum);
        j.name[0] = "right_leg_front_joint";
        j.name[1] = "right_leg_front_leg_joint1";
        j.name[2] = "right_leg_front_leg_joint2";
        j.name[3] = "right_leg_back_joint";
        j.name[4] = "right_leg_back_leg_joint1";
        j.name[5] = "right_leg_back_leg_joint2";
        j.name[6] = "left_leg_front_joint";
        j.name[7] = "left_leg_front_leg_joint1";
        j.name[8] = "left_leg_front_leg_joint2";
        j.name[9] = "left_leg_back_joint";
        j.name[10] = "left_leg_back_leg_joint1";
        j.name[11] = "left_leg_back_leg_joint2";
        j.position.resize(dofNum);

        //std::vector<std::vector<double>> tarPos{{0.02*cos(cou), 0.02*sin(cou)+0.04, 0.1}};
        kin3d.invKinematics3Dof(angle, tarPos[count%tarPos.size()]);

        /*j.position[0] = angle[0];
        j.position[1] = angle[1];
        j.position[2] = angle[2];*/
        tarAngle[0] = angle[0];
        tarAngle[1] = angle[1];
        tarAngle[2] = angle[2];
        tarAngle[3] = 0;
        tarAngle[5] = 0;
        tarAngle[4] = 0;
        tarAngle[6] = 0;
        tarAngle[8] = 0;
        tarAngle[7] = 0;
        tarAngle[9] = 0;
        tarAngle[11] = 0;
        tarAngle[10] = 0;

        for(int i=0; i<3; ++i)
        {
            tra[i].update(j.position[i], tarAngle[i], startAngle[i]);
        }

        if(!tra[0].getRunningState()){
            for(int i=0;i<3; i++){
                startAngle[i] = tarAngle[i];
            }
            count++;
            continue;
        }

        
        j.position[3] = 0;
        j.position[5] = 0;
        j.position[4] = 0;
        j.position[6] = 0;
        j.position[8] = 0;
        j.position[7] = 0;
        j.position[9] = 0;
        j.position[11] = 0;
        j.position[10] = 0;

        cou +=0.05;

        pub.publish(j);

        ros::spinOnce();
        loop_rate.sleep();
 
    }
    return 0;
}