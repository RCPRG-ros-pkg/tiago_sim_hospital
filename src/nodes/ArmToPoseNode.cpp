#include <nodes/ArmToPoseNode.hpp>

ArmToPoseNode::ArmToPoseNode(): nh("~"), group_arm_torso("arm_torso") {
	group_arm_torso.setPlannerId("SBLkConfigDefault");
	group_arm_torso.setPoseReferenceFrame("base_footprint");
	group_arm_torso.setMaxVelocityScalingFactor(1.0);
	group_arm_torso.setPlanningTime(1.0);

	goal_pose.header.frame_id = "base_footprint";
	marker_pose.header.frame_id = "base_footprint";

	marker_location_sub = nh.subscribe("/aruco_single/pose", 1, &ArmToPoseNode::getMarkerPosition, this);
	// ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
	// std::copy(group_arm_torso.getJointModelGroupNames().begin(), group_arm_torso.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

void ArmToPoseNode::getMarkerPosition(const geometry_msgs::PoseStamped& msg) {
	marker_pose.pose.position.x = msg.pose.position.x;
	marker_pose.pose.position.y = msg.pose.position.y;
	marker_pose.pose.position.z = msg.pose.position.z;
}

void ArmToPoseNode::reachPoseRelativeToMarker(float x = 0, float y = 0, float z = 0, float R = 0, float P = 0, float Y = 0){
	goal_pose.pose.position.x = marker_pose.pose.position.x + x;
	goal_pose.pose.position.y = marker_pose.pose.position.y + y;
	goal_pose.pose.position.z = marker_pose.pose.position.z + z;
	goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(R, P, Y);

	ROS_INFO("Goal for arm (XYZ,RPY):\t%f\t%f\t%f,\t%f\t%f\t%f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, R, P, Y);

	reachGoalPose();
}

geometry_msgs::PoseStamped ArmToPoseNode::getLastGoalPose() {
	return goal_pose;
}

void ArmToPoseNode::reachSetPose(geometry_msgs::PoseStamped goal){
	goal_pose = goal;
	ROS_INFO("Goal for arm(XYZ,QxQyQzQw):\t%f\t%f\t%f,\t%f\t%f\t%f\t%f", 
		goal_pose.pose.position.x, 
		goal_pose.pose.position.y, 
		goal_pose.pose.position.z,
		goal_pose.pose.orientation.x,
		goal_pose.pose.orientation.y,
		goal_pose.pose.orientation.z,
	    goal_pose.pose.orientation.w);
	reachGoalPose();
}

void ArmToPoseNode::reachGoalPose(){
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