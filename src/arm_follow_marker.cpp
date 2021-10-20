// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

class ArmFollowMarkerNode{
  private:
    ros::NodeHandle nh;
    ros::Subscriber marker_location_sub;
    ros::Timer follow_marker_timer;
    geometry_msgs::PoseStamped goal_pose;
    moveit::planning_interface::MoveGroupInterface group_arm_torso;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode e;
    bool success;

  public:
    ArmFollowMarkerNode(): nh("~"), group_arm_torso("arm_torso"){
      group_arm_torso.setPlannerId("SBLkConfigDefault");
      group_arm_torso.setPoseReferenceFrame("base_footprint");
      group_arm_torso.setMaxVelocityScalingFactor(1.0);
      group_arm_torso.setPlanningTime(1.0);

      goal_pose.header.frame_id = "base_footprint";
      goal_pose.pose.orientation.x = 0;
      goal_pose.pose.orientation.y = 0;
      goal_pose.pose.orientation.z = 0;
      goal_pose.pose.orientation.w = 1;

      marker_location_sub = nh.subscribe("/aruco_single/pose", 1, &ArmFollowMarkerNode::getMarkerPosition, this);
      follow_marker_timer = nh.createTimer(ros::Duration(5), &ArmFollowMarkerNode::followMarkerCallback, this);
    }

    void getMarkerPosition(const geometry_msgs::PoseStamped& msg) {
      goal_pose.pose.position.x = msg.pose.position.x - 0.4;
      goal_pose.pose.position.y = msg.pose.position.y;
      goal_pose.pose.position.z = msg.pose.position.z;
      // goal_pose.pose.orientation.x = msg.pose.orientation.x;
      // goal_pose.pose.orientation.y = msg.pose.orientation.y;
      // goal_pose.pose.orientation.z = msg.pose.orientation.z;
      // goal_pose.pose.orientation.w = msg.pose.orientation.w;
      // ROS_INFO("%f\t%f\t%f\t", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    }

    void followMarkerCallback(const ros::TimerEvent& event) {
      ROS_INFO("Goal:\t%f\t%f\t%f\t", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
      // ROS_INFO("Test");
      group_arm_torso.setPoseTarget(goal_pose);

      ROS_INFO_STREAM("Planning to move " <<
                      group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                      group_arm_torso.getPlanningFrame());

      group_arm_torso.setStartStateToCurrentState();

      success = bool(group_arm_torso.plan(my_plan));

      if ( !success ) {
        ROS_ERROR("No plan found");
        return;
      }

      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();

      e = group_arm_torso.move();
      if (!bool(e)) {
        ROS_ERROR("Error executing plan");
        return;
      }

      ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_follow_marker");
  ros::AsyncSpinner spinner(2);
  ArmFollowMarkerNode node;
  spinner.start();
  ros::waitForShutdown();
}
