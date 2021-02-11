#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <math.h>
#include <chrono>
#include <iostream>
//#include "velTrapezoidalRule.h"

const double r0 = 0.035;
const double r1 = 0.13;
const double r2 = 0.127;
const int dofNum = 12;

//accelTimeRate range:0-0.5
const double accelTimeRate = 0.5;
const double moveT = 1;
const double accelT = accelTimeRate * moveT;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_pub2D");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(100);
    double count = 0;
    int i = 0;
    std::vector<double> angle(dofNum);
    std::vector<double> diffAngle(dofNum);
    std::vector<double> omegaMax(dofNum);
    std::vector<double> angularAcc(dofNum);
    std::vector<double> decT(dofNum);
    std::vector<double> tarAngle(dofNum);
    std::vector<std::vector<float>> tarPos{{0.08, 0.08}, {0.08, 0.2},{0, 0.2}, {-0.08, 0.2}, {-0.08, 0.08}, {0,0.08}};
    std::vector<float> startAngle{0,0,0};

    std::chrono::system_clock::time_point start,preT,end;
    start = std::chrono::system_clock::now();
    preT = std::chrono::system_clock::now();
    while(ros::ok())
    {
        end = std::chrono::system_clock::now();
        double nowT = std::chrono::duration_cast<std::chrono::milliseconds>(end-preT).count();
        nowT /= 1000;
        std::cout << nowT <<std::endl;

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

        tarAngle[0] = 0;
        tarAngle[2] = (double)std::acos((tarPos[i%6][0]*tarPos[i%6][0]+tarPos[i%6][1]*tarPos[i%6][1]-r1*r1-r2*r2)/(2*r1*r2));
        tarAngle[1] = (double)std::atan2((-r2*std::sin(tarAngle[2])*tarPos[i%6][0] + (r1 + r2*std::cos(tarAngle[2]))*tarPos[i%6][1]),((r1 + r2*cos(tarAngle[2]))*tarPos[i%6][0] + r2*sin(tarAngle[2])*tarPos[i%6][1]));
        tarAngle[3] = 0;
        tarAngle[5] = 0;
        tarAngle[4] = 0;
        tarAngle[6] = 0;
        tarAngle[8] = 0;
        tarAngle[7] = 0;
        tarAngle[9] = 0;
        tarAngle[11] = 0;
        tarAngle[10] = 0;

        for(int k=1; k<3; k++)
        {
            diffAngle[k] = tarAngle[k] - startAngle[k];
            omegaMax[k] = diffAngle[k] / (moveT*(1.0-accelTimeRate));
            //angularAcc[i] = omegaMax[i] / accelT;

            if(nowT < accelT){
                //加速
                j.position[k] = startAngle[k] + omegaMax[k] * nowT * nowT / (2.0*accelT);
                //j.position[1] = startAngle[1] + omegaMax[1] * nowT * nowT / (2.0*accelT)-1.57;

            }else if(accelT <= nowT && nowT <= (moveT - accelT)){
                //定速 
                
                j.position[k] = startAngle[k] + (omegaMax[k]*accelT/2.0) + omegaMax[k]*(nowT-accelT);
                //j.position[1] = startAngle[1] + (omegaMax[1]*accelT/2.0) + omegaMax[1]*(nowT-accelT)-1.57;
            
            }else{
                decT[k] = (nowT-(moveT-accelT));
                j.position[k] = startAngle[k] + (omegaMax[k]*accelT/2.0) + omegaMax[k]*(moveT-2*accelT) + (2*omegaMax[k]-omegaMax[k]*decT[k]/accelT)*decT[k]/2;
                //j.position[1] = startAngle[1] + (omegaMax[1]*accelT/2.0) + omegaMax[1]*(moveT-2*accelT) + (2*omegaMax[1]-omegaMax[1]*decT[1]/accelT)*decT[1]/2 -1.57;
            
            }
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

        end = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-preT).count();
        if(elapsed >= moveT*1000){
            for(int k=1;k<3; k++){
                startAngle[k] = tarAngle[k];
            }
            i++;
            preT = std::chrono::system_clock::now();
        }

        /*double x,y,x2,y2;
        x = 0.03*std::cos(count) + 0.1;
        y = 0.03*std::sin(count) + 0.15;
        x2 = 0.03*cos(count+3.14) + 0.1;
        y2 = 0.03*sin(count+3.14) + 0.16;
        j.position[0] = 0;
        j.position[2] = (double)std::acos((x*x+y*y-r1*r1-r2*r2)/(2*r1*r2));
        j.position[1] = (double)std::atan2((-r2*std::sin(j.position[2])*x + (r1 + r2*std::cos(j.position[2]))*y),((r1 + r2*cos(j.position[2]))*x + r2*sin(j.position[2])*y));
        j.position[3] = 0;
        j.position[5] = (double)std::acos((x2*x2+y2*y2-r1*r1-r2*r2)/(2*r1*r2));
        j.position[4] = (double)std::atan2((-r2*std::sin(j.position[5])*x2 + (r1 + r2*std::cos(j.position[5]))*y2),((r1 + r2*cos(j.position[5]))*x2 + r2*sin(j.position[5])*y2));
        j.position[6] = 0;
        j.position[8] = (double)std::acos((x2*x2+y2*y2-r1*r1-r2*r2)/(2*r1*r2));
        j.position[7] = -(double)std::atan2((-r2*std::sin(j.position[8])*x2 + (r1 + r2*std::cos(j.position[8]))*y2),((r1 + r2*cos(j.position[8]))*x2 + r2*sin(j.position[8])*y2));
        j.position[9] = 0;
        j.position[11] = (double)std::acos((x*x+y*y-r1*r1-r2*r2)/(2*r1*r2));
        j.position[10] = -(double)std::atan2((-r2*std::sin(j.position[11])*x + (r1 + r2*std::cos(j.position[11]))*y),((r1 + r2*cos(j.position[11]))*x + r2*sin(j.position[11])*y));
        count += 0.2;*/

        pub.publish(j);

        ros::spinOnce();
        loop_rate.sleep();
 
    }
    return 0;
}