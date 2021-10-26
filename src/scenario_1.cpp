// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// tiago_hospital_sim nodes
#include <nodes/DriveToPointNode.hpp>
#include <nodes/PerformMotionNode.hpp>
#include <nodes/ArmToPoseNode.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scenario_1");
	ros::AsyncSpinner spinner(2);
	ros::Rate task_rate(1);
	PerformMotionNode motionNode;
	DriveToPointNode navigationNode;
	ArmToPoseNode armPoseNode;
	spinner.start();
	task_rate.sleep();
	// fast_home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	// drive to kitchen
	geometry_msgs::PoseStamped location_kitchen;
	location_kitchen.header.frame_id = "map";
	location_kitchen.pose.position.x = 4.1;
	location_kitchen.pose.position.y = -1;
	location_kitchen.pose.orientation.z = 1;
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// raise arm safely
	motionNode.performMotion("arm_prepare_grab");
	task_rate.sleep();
	// get arm close to marker location
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// get arm to cup by marker location
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// grab cup
	motionNode.performMotion("close_half");
	task_rate.sleep();
	// pick up cup
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// rotate
	location_kitchen.pose.orientation.w = 1;
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// lower hand to make robot see
	geometry_msgs::PoseStamped pose_hand_with_cup = armPoseNode.getLastGoalPose();
	pose_hand_with_cup.pose.position.x = 0.5;
	pose_hand_with_cup.pose.position.z -= 0.3;
	armPoseNode.reachSetPose(pose_hand_with_cup);
	task_rate.sleep();
	// drive to s1
	geometry_msgs::PoseStamped location_s1;
	location_s1.header.frame_id = "map";
	location_s1.pose.position.x = 9.1;
	location_s1.pose.position.y = -1.9;
	location_s1.pose.orientation.z = 1;
	location_s1.pose.orientation.w = -0.5;
	navigationNode.driveToPoint(location_s1);
	task_rate.sleep();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// move hand higher
	pose_hand_with_cup.pose.position.z += 0.3;
	armPoseNode.reachSetPose(pose_hand_with_cup);
	task_rate.sleep();
	// get arm close to marker location
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// get arm to cup by marker location
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// drop cup
	motionNode.performMotion("open_gripper");
	task_rate.sleep();
	// arm away
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// arm back the cup
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// grab cup
	motionNode.performMotion("close_half");
	task_rate.sleep();
	// arm away
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// s1 rotate door
	location_s1.pose.orientation.w = 0;
	navigationNode.driveToPoint(location_s1);
	task_rate.sleep();
	// arm home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	// drive to dispose place
	location_kitchen.pose.position.y = -2.4;
	location_kitchen.pose.orientation.w = -0.5;
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// hand up
	pose_hand_with_cup = armPoseNode.getLastGoalPose();
	pose_hand_with_cup.pose.position.x = 0.5;
	pose_hand_with_cup.pose.position.z += 0.3;
	armPoseNode.reachSetPose(pose_hand_with_cup);
	task_rate.sleep();
	// rotate to dispose place
	location_kitchen.pose.position.y = -2.75;
	location_kitchen.pose.orientation.w = -0.75;
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// hand forward
	pose_hand_with_cup = armPoseNode.getLastGoalPose();
	pose_hand_with_cup.pose.position.x = 1;
	armPoseNode.reachSetPose(pose_hand_with_cup);
	task_rate.sleep();
	// drop cup
	motionNode.performMotion("open_gripper");
	task_rate.sleep();
	// arm home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	// go back home
	geometry_msgs::PoseStamped location_lobby;
	location_lobby.header.frame_id = "map";
	location_lobby.pose.orientation.w = 1;
	navigationNode.driveToPoint(location_lobby);
	task_rate.sleep();
	// end node execution
	spinner.stop();
}