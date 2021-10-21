#include <nodes/ArmToMarkerNode.hpp>

ArmToMarkerNode::ArmToMarkerNode(): nh("~"), group_arm_torso("arm_torso") {
	group_arm_torso.setPlannerId("SBLkConfigDefault");
	group_arm_torso.setPoseReferenceFrame("base_footprint");
	group_arm_torso.setMaxVelocityScalingFactor(1.0);
	group_arm_torso.setPlanningTime(1.0);

	goal_pose.header.frame_id = "base_footprint";

	marker_location_sub = nh.subscribe("/aruco_single/pose", 1, &ArmToMarkerNode::getMarkerPosition, this);
}

void ArmToMarkerNode::getMarkerPosition(const geometry_msgs::PoseStamped& msg) {
	goal_pose.pose.position.x = msg.pose.position.x;
	goal_pose.pose.position.y = msg.pose.position.y;
	goal_pose.pose.position.z = msg.pose.position.z;
}

void ArmToMarkerNode::reachPoseRelativeToMarker(float x = 0, float y = 0, float z = 0, float R = 0, float P = 0, float Y = 0){
	goal_pose.pose.position.x += x;
	goal_pose.pose.position.y += y;
	goal_pose.pose.position.z += z;
	goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(R, P, Y);

	ROS_INFO("Goal for arm:\t%f\t%f\t%f\t%f\t%f\t%f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, R, P, Y);

	group_arm_torso.setPoseTarget(goal_pose);

	ROS_INFO_STREAM("Planning to move " <<
	              group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
	              group_arm_torso.getPlanningFrame());

	plan_success = false;
	success_tries = 0;

	while(!plan_success or !bool(exec_success)){
		success_tries++;
		if (success_tries > 5) {
			ROS_ERROR("Aborting motion to marker...");
			return;
		}

		group_arm_torso.setStartStateToCurrentState();

		plan_success = bool(group_arm_torso.plan(my_plan));

		if ( !plan_success ) {
			ROS_ERROR("No plan found");
			// continue;
		}

		ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

		// Execute the plan
		ros::Time start = ros::Time::now();

		exec_success = group_arm_torso.move();
		if (!bool(exec_success)) {
			ROS_ERROR("Error executing plan");
			// continue;
		}

		ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
	}

}