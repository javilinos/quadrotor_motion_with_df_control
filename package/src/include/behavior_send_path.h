/*!********************************************************************************
 * \brief     Send path behavior implementation 
 * \authors   Pablo Santamaria
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef SEND_PATH_H
#define SEND_PATH_H

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>
#include "trajectory_msgs/JointTrajectoryPoint.h"

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/TrajectoryWaypoints.h>

// Aerostack libraries
#include <BehaviorExecutionManager.h>
#include "ros_utils_lib/ros_utils.hpp"

const int MAX_DISTANCE = 1000; //maximum meters allowed

class BehaviorSendPath : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorSendPath();
  ~BehaviorSendPath();
  int main(int argc, char** argv);

private:
  ros::NodeHandle nh;
  std::string nspace;

  bool moving = false;
  bool new_traj_generated_ = false;
  float previous_time=0.0;
  std::string motion_reference_waypoints_path_topic;
  std::string motion_reference_traj_topic_;
  std::string path_blocked_topic_str;
  ros::Publisher path_references_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber path_blocked_sub;  
  bool path_blocked;
private:
  // BehaviorExecutionManager
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  void CallbackTrajectoryTopic(const trajectory_msgs::JointTrajectoryPoint& traj);

public:
  void pathBlockedCallBack(const std_msgs::Bool &msg);
};

#endif
