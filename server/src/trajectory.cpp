#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "server/RoboState.h"
#include "server/EncoderState.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"


double EPSILON = 0.005; // tolerance for determining whether goal is reached (in meters)

//Calibration values
int stepsPerRevolution[3] ={32000,32000,32000};//Stepper counts for each joint based on microstep and gearbox ratio
float prev_angle[3] = {0,0,0};
float total[3] = {0,0,0};
//float deviation[3] = {0,0,0};
int count = 0;

//Sending positionand velocity commads to arduino
server::RoboState arm;

//Spinner speed values
std_msgs::Int16 spin_spd;

//Fake encoder messages
server::EncoderState msg;

//Setup control_msgs actionlib server
actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* Server;

//Relative encoder messages for feedback
sensor_msgs::JointState encfb;

//Setup ros publishers
ros::Publisher pub0,pub1,pub2;

//Callback for sensor joint states
void cmd_cb(const sensor_msgs::JointState& meas){
	encfb = meas;
}

//Goal tolerance later used to check with positional feedback
void setGoalTolerance(const control_msgs::FollowJointTrajectoryGoalConstPtr goal, std::vector<double>& goal_tolerance) {
    unsigned int n_joints = goal->trajectory.joint_names.size();
    goal_tolerance.resize(n_joints, EPSILON);
    for (unsigned int i = 0; i < n_joints; i++) {
        for (unsigned int j = 0; j < goal->goal_tolerance.size(); j++) {
            if (goal->trajectory.joint_names[i] == goal->goal_tolerance[j].name) {
                goal_tolerance[i] = goal->goal_tolerance[j].position;
            }
        }
    }
}

void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& arm_goal) {

	//Ros node handle 
	ros::NodeHandle n;

    //Joint Trajectory Feedback
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = arm_goal;
    /*control_msgs::FollowJointTrajectoryFeedback feedback;	
    feedback.joint_names = goal->trajectory.joint_names;*/

    //Goal Tolerance
    std::vector<double> goal_tolerance;
    setGoalTolerance(goal, goal_tolerance);

    unsigned int goal_index = 0;
    unsigned int n_joints = goal->trajectory.joint_names.size();
    
    //Check loop rate of ROS to match with goal timestamps
    ros::Rate rate(5);

    while(n.ok() && Server->isActive()) {
	
		//Check if new goal is available
	     if (Server->isNewGoalAvailable()) {
            goal = Server->acceptNewGoal();
            goal_index = 0;
            n_joints = goal->trajectory.joint_names.size();
            //feedback.joint_names = goal->trajectory.joint_names;
            setGoalTolerance(goal, goal_tolerance);
	    	ROS_INFO("New goal received");
		}

		//Check if goal is finished
        if (goal_index == goal->trajectory.points.size()) {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = 0;
            Server->setSucceeded(result, "Goal succeeded!");
	
	    	/*sleep(1.0);//Sleep so that it has time to publish the 'actual' encoder values

			//Publish Feedback deviations to correct itself   
			for(unsigned int i = 0; i< n_joints; i++) {
			 	deviation[i] = (prev_angle[i] - encfb.position[i])*stepsPerRevolution[i]/(2*M_PI) ;
			 	ROS_INFO("Feedback is : %f", deviation[i]);
			}

			arm.position1 += deviation[0];
			arm.position2 += deviation[1];
			arm.position3 += deviation[2];

			//pub.publish(arm);//Publish values to correct itself

			//ROS_INFO("New arm position : %f", arm.position1);

			arm.position1 -= deviation[0];
			arm.position2 -= deviation[1];
			arm.position3 -= deviation[2];	*/		
	    
		}else{//Get new values of positions and velocities 

		    if(count == 0) {
			prev_angle[0] = (goal -> trajectory.points[goal_index].positions[0]);
	    	prev_angle[1] = (goal -> trajectory.points[goal_index].positions[1]);
		   	prev_angle[2] = (goal -> trajectory.points[goal_index].positions[2]);		
		    }

		    total[0] = ((goal -> trajectory.points[goal_index].positions[0])-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI);
		    total[1] = ((goal -> trajectory.points[goal_index].positions[1])-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI);
		    total[2] = ((goal -> trajectory.points[goal_index].positions[2])-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI);
		    
		    
		    //Absolute position values
		    arm.position1 += total[0];
		    arm.position2 += total[1];
	        arm.position3 += total[2];
		    
		    //vel values
		    arm.velocity1 = (goal -> trajectory.points[goal_index].velocities[0])*stepsPerRevolution[0]/(2*M_PI);
		    arm.velocity2 = (goal -> trajectory.points[goal_index].velocities[1])*stepsPerRevolution[1]/(2*M_PI);
		    arm.velocity3 = (goal -> trajectory.points[goal_index].velocities[2])*stepsPerRevolution[2]/(2*M_PI);
		    
		    //Acc values
	        arm.ax1 = (goal -> trajectory.points[goal_index].accelerations[0])*stepsPerRevolution[0]/(2*M_PI);
		    arm.ax2 = (goal -> trajectory.points[goal_index].accelerations[1])*stepsPerRevolution[1]/(2*M_PI);
		    arm.ax3 = (goal -> trajectory.points[goal_index].accelerations[2])*stepsPerRevolution[2]/(2*M_PI);

		    //Spinner values
		    spin_spd.data = (goal -> trajectory.points[goal_index].velocities[3]);


		    if(count!=0){
			    prev_angle[0] = (goal -> trajectory.points[goal_index].positions[0]);
		    	prev_angle[1] = (goal -> trajectory.points[goal_index].positions[1]);
			    prev_angle[2] = (goal -> trajectory.points[goal_index].positions[2]);
		    }
		

	  	    //Reference/Fake encoder position values
		    msg.angle1 = prev_angle[0];
		    msg.angle2 = prev_angle[1];
		    msg.angle3 = prev_angle[2];

		    count = 1;
		    //ROS_INFO("Position 1 is: %f", arm.position1);
		    //ROS_INFO("Position 3 is: %f", prev_angle[2]);
		    //ROS_INFO("Velocity 3 is: %f", arm.velocity3);
		    //ROS_INFO("Acc 1 is: %f", arm.ax1);
		
		    pub0.publish(arm);//Sends pos, vel and acc values to arduino
		    pub1.publish(spin_spd);//Sends spin speed
	        pub2.publish(msg);//Publish reference/fake encoder values   
			   
		}//else statement ends here

		unsigned int n_converged = 0;
	    for (unsigned int i = 0; i < n_joints; i++) {               
             ++n_converged;//Gonna later use it for goal tolerances calculation          	
     	}

		if (n_converged == n_joints) {
	           ++goal_index;
		}
		
	
		rate.sleep();
   	}//while loop ends here

}//function ends here

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle nh;
  //Sends steps to arduino
  pub0 = nh.advertise<server::RoboState>("arm_state",50);
  //Spinner speed values
  pub1 = nh.advertise<std_msgs::Int16>("spinner_speed",50);
  //Fake encoder values for reference
  pub2 = nh.advertise<server::EncoderState>("/cb1/encoder_states",50);

  //Subscriber to joint states
  ros::Subscriber sub = nh.subscribe("/cb1/joint_states",1000,cmd_cb);
  
  Server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh, "cb1/follow_joint_trajectory", &executeCB, false);
  Server->start();

  ROS_INFO("Action server is active and spinning...");

  ros::spin();
  return 0;
}
