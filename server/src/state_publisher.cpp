#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <server/EncoderState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

sensor_msgs::JointState joint_state;


//Relative joint states
float joint1,joint2,joint3,joint4;


//Callback for relative encoder states
void callback1(const std_msgs::Float32& msg){
	joint1 = msg.data;	
}

void callback2(const std_msgs::Float32& msg){
	joint2 = msg.data;	
}

void callback3(const std_msgs::Float32& msg){
	joint3 = msg.data;	
}

//Callback for encoder states
/*void callback(const server::EncoderState& msg){
	joint1 = msg.angle1;
	joint2 = msg.angle2;
	joint3 = msg.angle3;
}*/


int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    
    //Publish joint states to robot publisher
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);   
    
    //Fake encoders
    //ros::Subscriber sub = n.subscribe("encoder_states",1000,callback);

    //Relative encoder callback
    ros::Subscriber sub1 = n.subscribe("rel_state1",1000,callback1);
    ros::Subscriber sub2 = n.subscribe("rel_state2",1000,callback2);
    ros::Subscriber sub3 = n.subscribe("rel_state3",1000,callback3);
	
	
    //robot state
    /*joint1 = 0.4839;
    joint2 = 1.4788;
    joint3 = 0.9649;
    joint4 = 0.0;*/

    
    //Set ros loop rate
    //ros::Rate loop_rate(10);

    while (ros::ok()) {
	
	
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] ="joint1";

        joint_state.name[1] ="joint2";

        joint_state.name[2] ="joint3";

        joint_state.name[3] ="joint4";

        joint_state.position[0] = joint1;
        joint_state.position[1] = joint2;
        joint_state.position[2] = joint3+0.0469;           /*0.077402*/   //Offset for 3rd joint becoz of some reason??
        joint_state.position[3] = joint4;

        //send the joint state and transform
        joint_pub.publish(joint_state);

        ros::spinOnce();

        // This will adjust as needed per iteration
        //loop_rate.sleep();
    }


    return 0;
}

