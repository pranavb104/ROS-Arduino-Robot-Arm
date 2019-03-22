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
#include "sensor_msgs/JointState.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <fstream>
#include <sstream> 
#include <iostream>

int domove, spin_speed;//Controller for saving positions

float joint[3];

//String stream
std::stringstream ss2;//Saves all points

std::string disp_time;

//Callback for save_states
void callback1(const std_msgs::Int16& msg){
	domove = msg.data;
}

//Callback for sugar_disp times
void callback2(const std_msgs::String& msg){
	disp_time = msg.data;
	ss2 << disp_time << " "
      << 4 << std::endl;
	/*ROS_INFO("Value is %s",disp_time.c_str());
  ROS_INFO("Spin speed is  %d",spin_speed);*/
}

//Callback for spinner speed
void callback3(const std_msgs::Int16& msg){
  spin_speed = msg.data;
}

//Callback for joint_states
void cmd_cb(const sensor_msgs::JointState& arm){
	joint[0]  = arm.position[0];
	joint[1]  = arm.position[1];
	joint[2]  = arm.position[2];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_moves");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  domove = 0;//Default do not save
  
  ros::Subscriber sub0 = nh.subscribe("/cb1/joint_states",1000,cmd_cb); 
  ros::Subscriber sub1 = nh.subscribe("save_state",50,callback1);       //Subscriber for save_state
  ros::Subscriber sub2 = nh.subscribe("sugar_times",50,callback2);      //Subscriber for save_state
  ros::Subscriber sub3 = nh.subscribe("spinner_speed",50,callback3);    //Subscriber for save_state
  
  //Open a file for arm joint values
  std::ofstream output_file1 (argv[1],std::ios::app);
  if(!output_file1.is_open())
    {
	ROS_INFO("open failed");
    }

  //Open a file for sugar dispensing times
  std::ofstream output_file2 (argv[2]);
  if(!output_file2.is_open())
    {
	ROS_INFO("open failed");
    }

  // Benchmark runtime
  ros::Time start_time;
  bool doOnce = false;

  //Stringstream for points
  std::stringstream ss1;

  //Check loop rate of ROS to match with goal timestamps
  ros::Rate rate(5);

  while (ros::ok()) {

    //Save states when pressed button(start recording)
    if(domove == 1) {
    	if(doOnce == false){
    		start_time = ros::Time::now();//Start Timer now
        /*ss1 << "timestamp" << ","
            << "joint1" << ","
            << "joint2" << ","
            << "joint3" << ","
            << "spinspeed" << std::endl;*/
    		doOnce = true;
    		
    	}

  	  // Timestamp
  	  float set_time = (ros::Time::now() - start_time).toSec();
  	  ss1 << set_time << " ";

      //Iterate through joint states
  	   ss1 << joint[0] << " "
           << joint[1] << " "
           << joint[2] << " ";

  	  
      //Spin speed
      ss1 << spin_speed << std::endl;

      //domove = 0;

    }

  //Break from while loop(stop recording)
  if(domove == 2) {
	 output_file1<<ss1.str();
	 output_file2<<ss2.str();
	 output_file1.close();
	 output_file2.close();
   ROS_INFO("Files created");
   break;	
  }
  //Callback once
  ros::spinOnce();

  //Sleep
  rate.sleep();

  }//while loop ends here

  
  //Shudown ros
  ros::shutdown();
  return 0;
}
