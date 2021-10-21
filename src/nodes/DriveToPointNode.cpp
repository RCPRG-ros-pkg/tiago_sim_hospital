#include <nodes/DriveToPointNode.hpp>

DriveToPointNode::DriveToPointNode(): nh("~"), waitForResult(1){
	seq = 0;
	seq_received = 1;
	move_status = 0;
	pub_move_base = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	sub_move_result = nh.subscribe("/move_base/result", 1, &DriveToPointNode::getMoveResult, this);
}

void DriveToPointNode::driveToPoint(geometry_msgs::PoseStamped goal){
	ROS_INFO("Driving to point: %f\t%f", goal.pose.position.x, goal.pose.position.y);
	pub_move_base.publish(goal);
	while (seq != seq_received && move_status != 3) {
		ROS_INFO("Driving...");
		waitForResult.sleep();
	}
	ROS_INFO("Point reached.");
	seq++;
}

void DriveToPointNode::getMoveResult(const move_base_msgs::MoveBaseActionResult& msg){
	seq_received = msg.header.seq;
	move_status = msg.status.status;
}