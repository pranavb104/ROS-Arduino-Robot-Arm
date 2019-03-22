/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Int16.h>

int domove;//Controller for saving positions

//Callback for save_states
void callback(const std_msgs::Int16& msg){
	domove = msg.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotstate");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "Body";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  /*const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);*/

  domove = 0;//Default do not save
  
  ros::Subscriber sub = nh.subscribe("save_state",50,callback);//Subscriber for save_state

  //Set ros loop rate
  ros::Rate loop_rate(10);

  while (ros::ok()) {

	  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    //Save staes when pressed button
    if(domove == 1) {
	  ROS_INFO_NAMED("cb1", "x position: %f", current_pose.pose.position.x);
	  ROS_INFO_NAMED("cb1", "y position: %f", current_pose.pose.position.y);
	  ROS_INFO_NAMED("cb1", "z position: %f", current_pose.pose.position.z);
	  ROS_INFO_NAMED("cb1", "x orientation: %f", current_pose.pose.orientation.x);
	  ROS_INFO_NAMED("cb1", "y orientation: %f", current_pose.pose.orientation.y);
	  ROS_INFO_NAMED("cb1", "z orientation: %f", current_pose.pose.orientation.z);
	  ROS_INFO_NAMED("cb1", "w orientation: %f", current_pose.pose.orientation.w);
	  ROS_INFO(" ");
	  
	  domove = 0;
     }
	  ros::spinOnce();//callback once	
	
          // This will adjust as needed per iteration
          loop_rate.sleep();

  }//while loop ends here

  

  ros::shutdown();
  return 0;
}
