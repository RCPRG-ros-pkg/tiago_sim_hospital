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
	// kitchen location
	geometry_msgs::PoseStamped location_kitchen;
	location_kitchen.header.frame_id = "map";
	location_kitchen.pose.position.x = 0.6;
	location_kitchen.pose.position.y = -1;
	location_kitchen.pose.orientation.z = 1;
	// s1 location
	geometry_msgs::PoseStamped location_s1;
	location_s1.header.frame_id = "map";
	location_s1.pose.position.x = 5.71;
	location_s1.pose.position.y = -1.85;
	location_s1.pose.orientation.z = 1;
	location_s1.pose.orientation.w = -0.5;
	// charger location
	geometry_msgs::PoseStamped location_charger;
	location_charger.header.frame_id = "map";
	location_charger.pose.position.x = 0;
	location_charger.pose.position.y = 0;
	location_charger.pose.orientation.w = 1;
	// start
	spinner.start();
	task_rate.sleep();
	// fast_home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	armPoseNode.reachSetPose(0.425, 0.15, 1, 1.57, 0, 3.05);
	task_rate.sleep();
	motionNode.performMotion("open_gripper");
	task_rate.sleep();
	// drive to kitchen
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// raise arm safely
	armPoseNode.reachSetPose(0.6, 0.2, 1, 1.57, 0, 1.57);
	task_rate.sleep();
	armPoseNode.reachSetPose(0.6, 0.2, 1, 1.57, 0, 0);
	task_rate.sleep();
	// get arm close to marker location
	armPoseNode.reachPoseRelativeToMarker(-0.2, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// get arm to cup by marker location
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// grab cup
	motionNode.performMotion("close_gripper");
	task_rate.sleep();
	// pick up cup body
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// rotate hand to shelter cup
	armPoseNode.reachSetPose(0.6, 0.2, 1, 1.57, 0, 1.57);
	task_rate.sleep();
	armPoseNode.reachSetPose(0.425, 0.15, 1, 1.57, 0, 3.05);
	task_rate.sleep();
	// drive to s1
	navigationNode.driveToPoint(location_s1);
	task_rate.sleep();
	// look down for marker
	motionNode.performMotion("look_down");
	task_rate.sleep();
	// unshelter cup
	armPoseNode.reachSetPose(0.6, 0.2, 1, 1.57, 0, 1.57);
	task_rate.sleep();
	armPoseNode.reachSetPose(0.6, 0.2, 1, 1.57, 0, 0);
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
	// take back the cup
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.05, 1.57, 0, 0);
	task_rate.sleep();
	// grab cup
	motionNode.performMotion("close_half");
	task_rate.sleep();
	// arm away
	armPoseNode.reachPoseRelativeToMarker(-0.1, 0, 0.2, 1.57, 0, 0);
	task_rate.sleep();
	// look forward
	motionNode.performMotion("look_forward");
	task_rate.sleep();
	// rotate hand to shelter cup
	armPoseNode.reachSetPose(0.425, 0.15, 1, 1.57, 0, 3.05);
	task_rate.sleep();
	// drive to dispose place
	location_kitchen.pose.position.y = -2.4;
	location_kitchen.pose.orientation.w = -0.25;
	navigationNode.driveToPoint(location_kitchen);
	task_rate.sleep();
	// drop cup
	motionNode.performMotion("drop_item_in_front");
	task_rate.sleep();
	// arm home
	motionNode.performMotion("fast_home");
	task_rate.sleep();
	// go back home
	location_charger.pose.position.y = -0.2;
	navigationNode.driveToPoint(location_charger);
	task_rate.sleep();
	// end node execution
	spinner.stop();
}