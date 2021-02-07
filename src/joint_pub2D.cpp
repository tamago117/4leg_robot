#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <math.h>

const double r0 = 0.035;
const double r1 = 0.13;
const double r2 = 0.127;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_pub2D");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(10);
    double count = 0;
    while(ros::ok())
    {
        sensor_msgs::JointState j;
        j.header.stamp = ros::Time::now();
        j.name.resize(12);
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
        j.position.resize(12);
        double x,y,x2,y2;
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
        count += 0.2;

        pub.publish(j);

        ros::spinOnce();
        loop_rate.sleep();
 
    }
    return 0;
}