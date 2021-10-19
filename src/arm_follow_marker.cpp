/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
// #include <tf/transform_broadcaster.h>

// Std C++ headers
// #include <string>
// #include <vector>
// #include <map>

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

      ROS_INFO("1");
      // group_arm_torso.plan(my_plan);
      success = bool(group_arm_torso.plan(my_plan));
      ROS_INFO("2");

      if ( !success ) {
        ROS_INFO("2a");
        ROS_ERROR("No plan found");
        return;
      }

      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();

      ROS_INFO("3");
      e = group_arm_torso.move();
      ROS_INFO("4");
      if (!bool(e)) {
        ROS_INFO("4a");
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
