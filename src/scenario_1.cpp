// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

// #include <string>

class ScenarioExecutionNode{
	private:
	    ros::NodeHandle nh;
	    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> play_motion_client;
	    ros::Publisher pub_move_base;
	    ros::Timer timer;
    	geometry_msgs::PoseStamped pose_kitchen_pick_cup;
    	play_motion_msgs::PlayMotionGoal motion_home;
    	int state;
	public:
	    ScenarioExecutionNode(): nh("~"), play_motion_client("/play_motion", true){
	    	pose_kitchen_pick_cup.header.frame_id = "map";
	    	pose_kitchen_pick_cup.pose.position.x = 4.1;
	    	pose_kitchen_pick_cup.pose.position.y = -1;
	    	pose_kitchen_pick_cup.pose.orientation.z = 1;

	    	motion_home.motion_name = "fast_home";
			motion_home.skip_planning = false;
			motion_home.priority = 0;
	    	
	    	play_motion_client.waitForServer();

	    	pub_move_base = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

		    state = 0;
		    timer = nh.createTimer(ros::Duration(1), &ScenarioExecutionNode::run, this);
	    }

	    void run(const ros::TimerEvent& event) {
	    	switch (state){
	    		case 0:
	    		case 1:
	    			pub_move_base.publish(pose_kitchen_pick_cup);
	    		default:
	    			break;
	    	}

	    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scenario_1");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ScenarioExecutionNode node;
	ros::waitForShutdown();
	// ros::spin();
}