// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Marker tracking
#include <tf/transform_broadcaster.h>

class ArmToPoseNode {
	private:
	    ros::NodeHandle nh;
	    ros::Subscriber marker_location_sub;
	    geometry_msgs::PoseStamped goal_pose, marker_pose;
	    moveit::planning_interface::MoveGroupInterface group_arm_torso;
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	    moveit::planning_interface::MoveItErrorCode exec_success;
	    bool plan_success;
	    unsigned int success_tries;
		void getMarkerPosition(const geometry_msgs::PoseStamped& msg);
	    void reachGoalPose();
	public:
		ArmToPoseNode();
	    void reachPoseRelativeToMarker(float x, float y, float z, float R, float P, float Y);
	    geometry_msgs::PoseStamped getLastGoalPose();
	    void reachSetPose(geometry_msgs::PoseStamped goal);
	    void reachSetPose(float x, float y, float z, float R, float P, float Y);
};