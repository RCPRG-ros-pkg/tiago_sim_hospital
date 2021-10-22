#include <nodes/PerformMotionNode.hpp>

PerformMotionNode::PerformMotionNode(): nh("~"), play_motion_client("/play_motion", true), state(play_motion_client.getState()){
	play_motion_client.waitForServer();
	goal.skip_planning = false;
	goal.priority = 0;
}

void PerformMotionNode::performMotion(std::string name) {
	goal.motion_name = name;
	actionOk = 0;
	while (actionOk != 1){
		ROS_INFO_STREAM("Trying to perform motion: " << name);
		play_motion_client.sendGoal(goal);
		actionOk = play_motion_client.waitForResult(ros::Duration(60.0));

		state = play_motion_client.getState();
		if ( actionOk == 1 )
		{
			ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
		}
		else
		{
			ROS_ERROR_STREAM("Action failed with state: " << state.toString() << "\nRetrying...");
		}
	}
}