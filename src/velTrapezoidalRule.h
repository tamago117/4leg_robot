#pragma once

#include <chrono>

class velTrapezoidalRule
{
private:
    double accelTimeRate;
    double moveT;
    double accelT;
    double diffAngle;
    double omegaMax;
    double decT;
    double nowT;
    double angle;
    std::chrono::system_clock::time_point start,preT,end;
public:
    velTrapezoidalRule(double accelTimeRate_, double moveT_);
    int update(double& angle, double tarAngle, double startAngle);
};

velTrapezoidalRule::velTrapezoidalRule(double accelTimeRate_, double moveT_)
{
    accelTimeRate = accelTimeRate_;
    moveT = moveT_;
    accelT = accelTimeRate * moveT;
    start = std::chrono::system_clock::now();
    preT = std::chrono::system_clock::now();
}

int velTrapezoidalRule::update(double& angle, double tarAngle, double startAngle)
{
    end = std::chrono::system_clock::now();
    nowT = std::chrono::duration_cast<std::chrono::milliseconds>(end-preT).count();
    nowT /= 1000;

    diffAngle = tarAngle - startAngle;
    omegaMax = diffAngle / (moveT*(1.0-accelTimeRate));
    //angularAcc[i] = omegaMax[i] / accelT;

    if(nowT < accelT){
        //accel
        angle = startAngle + omegaMax * nowT * nowT / (2.0*accelT);

    }else if(accelT <= nowT && nowT <= (moveT - accelT)){
        //constant speed
        angle = startAngle + (omegaMax*accelT/2.0) + omegaMax*(nowT-accelT);
        
    }else{
        //deceleration
        decT = (nowT-(moveT-accelT));
        angle = startAngle + (omegaMax*accelT/2.0) + omegaMax*(moveT-2*accelT) + (2*omegaMax-omegaMax*decT/accelT)*decT/2;
        
    }

    //goal point process
    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-preT).count();
    if(elapsed >= moveT*1000){
        preT = std::chrono::system_clock::now();
        return 0;
    }
    return 1;
}