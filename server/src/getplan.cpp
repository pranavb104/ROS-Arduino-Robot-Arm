// MoveIt!

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>

#include <fstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "getplan");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

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
 /* const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);*/

  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("moveo", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  //Planning to a Pose Goal variable
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  //bool success; //Movement success variable

 // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();//Get current pose
  geometry_msgs::Pose target_pose3;
  target_pose3.position.x = current_pose.pose.position.x;
  target_pose3.position.y = current_pose.pose.position.y;
  target_pose3.position.z = current_pose.pose.position.z;
  target_pose3.orientation.x = current_pose.pose.orientation.x;
  target_pose3.orientation.y = current_pose.pose.orientation.y;
  target_pose3.orientation.z = current_pose.pose.orientation.z;
  target_pose3.orientation.w = current_pose.pose.orientation.w;

  //move_group.setApproximateJointValueTarget(target_pose3, "link3");

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.x += 0.034;
  waypoints.push_back(target_pose3);  // right


  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "Body");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  // Finally plan and execute the trajectory
  my_plan.trajectory_ = trajectory;

  //Stringstream variable
  std::stringstream ss;

  //Counter for time
  float count = 0.0 ;

  if(success == true) {
    
    moveit_msgs::RobotTrajectory msg;  
    msg =my_plan.trajectory_;
                     
    for (unsigned int i=0; i<msg.joint_trajectory.points.size(); i++)  
    {  

        ss 
           << count
           << " " << msg.joint_trajectory.points[i].positions[0]  
           << " " << msg.joint_trajectory.points[i].positions[1]  
           << " " << msg.joint_trajectory.points[i].positions[2]
           << " " << msg.joint_trajectory.points[i].velocities[0]  
           << " " << msg.joint_trajectory.points[i].velocities[1]
           << " " << msg.joint_trajectory.points[i].velocities[2]  
           << " " << msg.joint_trajectory.points[i].accelerations[0]
           << " " << msg.joint_trajectory.points[i].accelerations[1]  
           << " " << msg.joint_trajectory.points[i].accelerations[2]
           << " " << 0 //Redundant speed
           << std::endl; 


     count+=0.2;
          
     if(i == (msg.joint_trajectory.points.size()-1))
      {  
        ROS_INFO("Total number of points in trajectory is : %d", i);
        std::ofstream outfile ("stick_grab.txt",std::ios::app);
        if(!outfile.is_open())
          {
            ROS_INFO("open failed");
          }
        else
          {  
            //outfile << "There are a total of " << i << " points in the trajectory" <<std::endl;
            //outfile<<msg.multi_dof_joint_trajectory.joint_names[0]<<std::endl;
            outfile<<ss.str()<<std::endl;
            outfile.close();
            ROS_INFO("File created");
          }
      } 
    }

    sleep(1.0);
    //Finally Move
    move_group.execute(my_plan);
    ROS_INFO("Movement Done!");
  }
  

  return 0;

}
