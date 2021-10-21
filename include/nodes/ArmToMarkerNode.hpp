// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Marker tracking
#include <tf/transform_broadcaster.h>

class ArmToMarkerNode {
	private:
	    ros::NodeHandle nh;
	    ros::Subscriber marker_location_sub;
	    geometry_msgs::PoseStamped goal_pose;
	    moveit::planning_interface::MoveGroupInterface group_arm_torso;
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	    moveit::planning_interface::MoveItErrorCode exec_success;
	    bool plan_success;
	    unsigned int success_tries;
		void getMarkerPosition(const geometry_msgs::PoseStamped& msg);
	public:
		ArmToMarkerNode();
	    void reachPoseRelativeToMarker(float x, float y, float z, float R, float P, float Y);
};