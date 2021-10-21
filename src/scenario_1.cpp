// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// tiago_hospital_sim nodes
#include <nodes/DriveToPointNode.hpp>
#include <nodes/PerformMotionNode.hpp>
#include <nodes/ArmToMarkerNode.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scenario_1");
	ros::AsyncSpinner spinner(2);
	ros::Rate task_rate(1);
	PerformMotionNode motionNode;
	DriveToPointNode navigationNode;
	ArmToMarkerNode armMarkerNode;
	spinner.start();
	task_rate.sleep();
	// fast_home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	// drive to kitchen
	geometry_msgs::PoseStamped pose_kitchen_pick_cup;
	pose_kitchen_pick_cup.header.frame_id = "map";
	pose_kitchen_pick_cup.pose.position.x = 4.1;
	pose_kitchen_pick_cup.pose.position.y = -1;
	pose_kitchen_pick_cup.pose.orientation.z = 1;
	navigationNode.driveToPoint(pose_kitchen_pick_cup);
	task_rate.sleep();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// raise arm safely
	motionNode.performMotion("arm_prepare_grab");
	task_rate.sleep();
	// get arm close to marker location
	armMarkerNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// get arm to cup by marker location
	armMarkerNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// grab cup
	motionNode.performMotion("close_gripper");
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// drive to kitchen
	geometry_msgs::PoseStamped pose_s1_drop_cup;
	pose_s1_drop_cup.header.frame_id = "map";
	pose_s1_drop_cup.pose.position.x = 9;
	pose_s1_drop_cup.pose.position.y = -2;
	pose_s1_drop_cup.pose.orientation.z = 1;
	pose_s1_drop_cup.pose.orientation.w = -0.5;
	navigationNode.driveToPoint(pose_s1_drop_cup);
	task_rate.sleep();
	spinner.stop();
}