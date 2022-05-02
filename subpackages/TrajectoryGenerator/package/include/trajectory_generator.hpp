#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include "ros/ros.h"


#define DEBUG_TRAJ 2
using namespace std;

// Father Class

#define AUTOYAW
class TrajectoryGenerator{

protected:    

    float endTime_ = 0.0f;

public:

    TrajectoryGenerator(){};
    ~TrajectoryGenerator(){};

    virtual bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float medium_speed) = 0;
    virtual bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float medium_speed, const std::vector<float>&){
        std::cout<<"VIRTUAL METHOD CALLED WITHOUT MATCHING FUNCTIONS"<<std::endl;
        return false;
    };
    
    virtual bool evaluateTrajectory(float t ,  std::array<std::array<float,3>,4>& refs) = 0;
    float getEndTime() const {return endTime_;}
    virtual ros::Time getBeginTime(){
        std::cerr << "calling non implemented method"<< std::endl;
        return ros::Time::now();
    }
    virtual void plot(){
        std::cerr << "calling non implemented method"<< std::endl;
    };

    bool trajectory_generated_ = false;
    virtual bool getTrajectoryGenerated() {
        return trajectory_generated_;
    }

};

class CircleGenerator:public TrajectoryGenerator{
private :
    std::vector<float> center_;
    float radius_,omega_,height_,yaw_;
    ros::Time beginTime_;
public:
    CircleGenerator(){};
    ~CircleGenerator(){};
    ros::Time getBeginTime(){
        return beginTime_;
    }
    bool generateTrajectory(const vector<vector<float>>& waypoints, float speed);
    bool evaluateTrajectory(float t , std::array<std::array<float,3>,4>& refs_);
};


class LemniscateGenerator:public TrajectoryGenerator{
private :
    std::vector<float> center_;
    float radius_,omega_,height_,yaw_;
    
public:
    LemniscateGenerator(){};
    ~LemniscateGenerator(){};
    bool generateTrajectory(const vector<vector<float>>& waypoints, float speed);
    bool evaluateTrajectory(float time ,  std::array<std::array<float,3>,4>& refs_) ;

};


#endif