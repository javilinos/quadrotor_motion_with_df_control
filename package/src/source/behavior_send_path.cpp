/*!********************************************************************************
 * \brief     Send path behavior implementation 
 * \authors   Pablo Santamaria
 *            Miguel Fernandez Cortizas
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

#include "../include/behavior_send_path.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorSendPath behavior;
  behavior.start();
  return 0;
}

BehaviorSendPath::BehaviorSendPath() : BehaviorExecutionManager(){ 
  setName("send_path");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorSendPath::~BehaviorSendPath(){}

void BehaviorSendPath::onConfigure(){ 
  nh = getNodeHandle();
  nspace = getNamespace();
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_waypoints_path_topic"	  , motion_reference_waypoints_path_topic   ,"motion_reference/waypoints");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_traj_topic"    , motion_reference_traj_topic_,     "motion_reference/trajectory");
  ros_utils_lib::getPrivateParam<std::string>("~path_blocked_topic"	                      , path_blocked_topic_str                  ,"environnment/path_blocked_by_obstacle");
  path_references_pub_ = nh.advertise<aerostack_msgs::TrajectoryWaypoints>("/" + nspace + "/" + motion_reference_waypoints_path_topic, 1, false);
}

void BehaviorSendPath::onActivate(){
  traj_sub_ = nh.subscribe("/" + nspace + "/" + motion_reference_traj_topic_, 1, &BehaviorSendPath::CallbackTrajectoryTopic,this);
  path_blocked_sub = nh.subscribe("/" + nspace + "/"+path_blocked_topic_str, 1, &BehaviorSendPath::pathBlockedCallBack, this);
  int yaw_mode = aerostack_msgs::TrajectoryWaypoints::PATH_FACING;
  double speed = 1;
  path_blocked=false;
  aerostack_msgs::TrajectoryWaypoints reference_waypoints;
  //Checks if path distance is not too long and argument is defined
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["path"].IsDefined()){
    std::vector<std::vector<double>> points=config_file["path"].as<std::vector<std::vector<double>>>();
    for(int i=0;i<points.size();i++){
      geometry_msgs::PoseStamped path_point;
      path_point.header.frame_id="odom";
      path_point.pose.position.x = points[i][0];
      path_point.pose.position.y = points[i][1];
      path_point.pose.position.z = points[i][2];
      reference_waypoints.poses.emplace_back(path_point);
    }
  }else{
    setErrorMessage("Error: Path is not defined");
    std::cout<<"Error: Path is not defined"<<std::endl;
    return;
  }
  if(config_file["yaw_mode"].IsDefined()){
    yaw_mode = config_file["yaw_mode"].as<int>(); 
  }
  if(config_file["speed"].IsDefined()){
    speed = config_file["speed"].as<double>(); 
  }

  reference_waypoints.header.frame_id="odom";
  reference_waypoints.header.stamp = ros::Time::now();
  reference_waypoints.yaw_mode = yaw_mode;
  reference_waypoints.max_speed = speed;
  int ctrl_c = 0;
  while(ctrl_c == 0){
    if (path_references_pub_.getNumSubscribers() > 0){
      ctrl_c = 1;
      path_references_pub_.publish(reference_waypoints);
    }
  }
  moving = false;
  new_traj_generated_ = true;
}

void BehaviorSendPath::onDeactivate(){
  moving = false;
  traj_sub_.shutdown();
  //path_references_pub_.shutdown();
}

void BehaviorSendPath::onExecute(){
}

bool BehaviorSendPath::checkSituation(){
  return true;
}

void BehaviorSendPath::checkGoal(){
}

void BehaviorSendPath::checkProgress(){
  if(path_blocked){
    path_blocked=false;
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}

void BehaviorSendPath::checkProcesses(){
}

void BehaviorSendPath::pathBlockedCallBack(const std_msgs::Bool &msg){
  path_blocked = msg.data;
}

void BehaviorSendPath::CallbackTrajectoryTopic(const trajectory_msgs::JointTrajectoryPoint& traj){
  double value = 0.0f;
  
  //std::cout << traj.time_from_start.toSec() - previous_time <<std::endl;
  if (traj.time_from_start.toSec() - previous_time < -0.001 ){
    std::cout << "TRAJ GENERATED" <<std::endl;
    std::cout << "t0:" << traj.time_from_start.toSec() << "  t1:"  << previous_time <<std::endl;
    new_traj_generated_ = true;
  }
  
  if (new_traj_generated_  && traj.time_from_start.toSec() > 0.1){
    for (unsigned short int i =0; i<3; i++ ){
      value += fabs(traj.velocities[i]);
      value += fabs(traj.accelerations[i]);
    }

    if (value == 0.0f){
      
      if (moving)
      {
        new_traj_generated_ = false;  
        std::cout << "behavior finished" << std::endl;
        BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
      }
      else moving = true;
    }
  }
  previous_time = traj.time_from_start.toSec();
    
}