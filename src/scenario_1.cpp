// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// // MoveIt! headers
// #include <moveit/move_group_interface/move_group_interface.h>

// // Play motion headers
// #include <actionlib/client/simple_action_client.h>
// #include <play_motion_msgs/PlayMotionAction.h>

// // Move base headers
// #include <move_base_msgs/MoveBaseActionResult.h>

// // Marker tracking
// #include <tf/transform_broadcaster.h>

// // C++ headers
// #include <string>

// tiago_hospital_sim nodes
#include <nodes/DriveToPointNode.hpp>
#include <nodes/PerformMotionNode.hpp>
#include <nodes/ArmToMarkerNode.hpp>

// class DriveToPointNode{
// 	private:
// 	    ros::NodeHandle nh;
// 	    ros::Publisher pub_move_base;
// 	    ros::Subscriber sub_move_result;
// 	    int seq, seq_received, move_status;
// 	    ros::Rate waitForResult;
// 	public:
// 	    DriveToPointNode(): nh("~"), waitForResult(1){
// 	    	seq = 0;
// 	    	seq_received = 1;
// 	    	move_status = 0;
// 	    	pub_move_base = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
// 	    	sub_move_result = nh.subscribe("/move_base/result", 1, &DriveToPointNode::getMoveResult, this);
// 	    }

// 	    void driveToPoint(geometry_msgs::PoseStamped goal){
// 	    	ROS_INFO("Driving to point: %f\t%f", goal.pose.position.x, goal.pose.position.y);
// 			pub_move_base.publish(goal);
// 			while (seq != seq_received && move_status != 3) {
// 				ROS_INFO("Driving...");
// 				waitForResult.sleep();
// 			}
// 			ROS_INFO("Point reached.");
// 			seq++;
// 	    }

// 	    void getMoveResult(const move_base_msgs::MoveBaseActionResult& msg){
// 	    	seq_received = msg.header.seq;
// 	    	move_status = msg.status.status;
// 	    }
// };

// class PerformMotionNode {
// 	private:
// 		ros::NodeHandle nh;
// 	    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> play_motion_client;
// 	    actionlib::SimpleClientGoalState state;
//     	play_motion_msgs::PlayMotionGoal goal;
//     	bool actionOk;
// 	public:
// 		PerformMotionNode(): nh("~"), play_motion_client("/play_motion", true), state(play_motion_client.getState()){
// 			play_motion_client.waitForServer();
// 			goal.skip_planning = false;
// 			goal.priority = 0;
// 		}

// 		void performMotion(std::string name) {
// 			goal.motion_name = name;
// 			actionOk = false;
// 			while (!actionOk){
// 				ROS_INFO_STREAM("Trying to perform motion: " << name);
// 				play_motion_client.sendGoal(goal);
// 				actionOk = play_motion_client.waitForResult(ros::Duration(60.0));

// 				state = play_motion_client.getState();
// 				if ( actionOk )
// 				{
// 					ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
// 				}
// 				else
// 				{
// 					ROS_ERROR_STREAM("Action failed with state: " << state.toString() << "\nRetrying...");
// 				}
// 			}
// 		}
// };

// class ArmToMarkerNode {
// 	private:
// 	    ros::NodeHandle nh;
// 	    ros::Subscriber marker_location_sub;
// 	    geometry_msgs::PoseStamped goal_pose;
// 	    moveit::planning_interface::MoveGroupInterface group_arm_torso;
// 	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// 	    moveit::planning_interface::MoveItErrorCode exec_success;
// 	    bool plan_success;
// 	    unsigned int success_tries;
// 	public:
// 		ArmToMarkerNode(): nh("~"), group_arm_torso("arm_torso") {
// 			group_arm_torso.setPlannerId("SBLkConfigDefault");
// 			group_arm_torso.setPoseReferenceFrame("base_footprint");
// 			group_arm_torso.setMaxVelocityScalingFactor(1.0);
// 			group_arm_torso.setPlanningTime(1.0);

// 			goal_pose.header.frame_id = "base_footprint";
// 			goal_pose.pose.orientation.x = 0;
// 			goal_pose.pose.orientation.y = 0;
// 			goal_pose.pose.orientation.z = 0;
// 			goal_pose.pose.orientation.w = 1;

// 			marker_location_sub = nh.subscribe("/aruco_single/pose", 1, &ArmToMarkerNode::getMarkerPosition, this);
// 		}

// 		void getMarkerPosition(const geometry_msgs::PoseStamped& msg) {
// 			goal_pose.pose.position.x = msg.pose.position.x;
// 			goal_pose.pose.position.y = msg.pose.position.y;
// 			goal_pose.pose.position.z = msg.pose.position.z;
// 	    }

// 	    void reachPoseRelativeToMarker(float x = 0, float y = 0, float z = 0, float R = 0, float P = 0, float Y = 0){
// 	    	goal_pose.pose.position.x += x;
// 	    	goal_pose.pose.position.y += y;
// 	    	goal_pose.pose.position.z += z;
// 			goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(R, P, Y);

// 			ROS_INFO("Goal for arm:\t%f\t%f\t%f\t%f\t%f\t%f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, R, P, Y);

// 			group_arm_torso.setPoseTarget(goal_pose);

// 			ROS_INFO_STREAM("Planning to move " <<
// 			              group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
// 			              group_arm_torso.getPlanningFrame());

// 			plan_success = false;
// 			success_tries = 0;

// 			while(!plan_success or !bool(exec_success)){
// 				success_tries++;
// 				if (success_tries > 5) {
// 					ROS_ERROR("Aborting motion to marker...");
// 					return;
// 				}

// 				group_arm_torso.setStartStateToCurrentState();

// 				plan_success = bool(group_arm_torso.plan(my_plan));

// 				if ( !plan_success ) {
// 					ROS_ERROR("No plan found");
// 					// continue;
// 				}

// 				ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

// 				// Execute the plan
// 				ros::Time start = ros::Time::now();

// 				exec_success = group_arm_torso.move();
// 				if (!bool(exec_success)) {
// 					ROS_ERROR("Error executing plan");
// 					// continue;
// 				}

// 				ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
// 			}

// 	    }
// };

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
	motionNode.performMotion("close_gripper_half");
	task_rate.sleep();
	spinner.stop();
}