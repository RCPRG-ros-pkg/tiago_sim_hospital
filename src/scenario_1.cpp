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
	geometry_msgs::PoseStamped location_kitchen_pick_cup;
	location_kitchen_pick_cup.header.frame_id = "map";
	location_kitchen_pick_cup.pose.position.x = 4.1;
	location_kitchen_pick_cup.pose.position.y = -1;
	location_kitchen_pick_cup.pose.orientation.z = 1;
	navigationNode.driveToPoint(location_kitchen_pick_cup);
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
	motionNode.performMotion("close_gripper");
	task_rate.sleep();
	// pick up cup
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// rotate
	geometry_msgs::PoseStamped location_kitchen_look_door;
	location_kitchen_look_door.header.frame_id = "map";
	location_kitchen_look_door.pose.position.x = 4.1;
	location_kitchen_look_door.pose.position.y = -1;
	location_kitchen_look_door.pose.orientation.z = 1;
	location_kitchen_look_door.pose.orientation.w = 1;
	navigationNode.driveToPoint(location_kitchen_look_door);
	task_rate.sleep();
	// lower hand to make robot see
	geometry_msgs::PoseStamped pose_hand_with_cup = armPoseNode.getLastGoalPose();
	pose_hand_with_cup.pose.position.x = 0.6;
	pose_hand_with_cup.pose.position.z -= 0.4;
	armPoseNode.reachSetPose(pose_hand_with_cup);
	task_rate.sleep();
	// drive to s1
	geometry_msgs::PoseStamped location_s1_drop_cup;
	location_s1_drop_cup.header.frame_id = "map";
	location_s1_drop_cup.pose.position.x = 9;
	location_s1_drop_cup.pose.position.y = -1.6;
	location_s1_drop_cup.pose.orientation.z = 1;
	location_s1_drop_cup.pose.orientation.w = -0.5;
	navigationNode.driveToPoint(location_s1_drop_cup);
	task_rate.sleep();
	spinner.stop();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// // move hand higher
	// pose_hand_with_cup.pose.position.x = 0.8;
	// pose_hand_with_cup.pose.position.z += 0.4;
	// armPoseNode.reachSetPose(pose_hand_with_cup);
	// task_rate.sleep();
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
}