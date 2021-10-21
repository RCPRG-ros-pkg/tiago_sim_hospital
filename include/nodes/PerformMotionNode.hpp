// ROS headers
#include <ros/ros.h>

// Play motion headers
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// C++ headers
#include <string>

class PerformMotionNode {
	private:
		ros::NodeHandle nh;
	    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> play_motion_client;
	    actionlib::SimpleClientGoalState state;
    	play_motion_msgs::PlayMotionGoal goal;
    	bool actionOk;
	public:
		PerformMotionNode();
		void performMotion(std::string name);
};