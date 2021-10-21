// ROS headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Move base headers
#include <move_base_msgs/MoveBaseActionResult.h>

class DriveToPointNode{
	private:
	    ros::NodeHandle nh;
	    ros::Publisher pub_move_base;
	    ros::Subscriber sub_move_result;
	    int seq, seq_received, move_status;
	    ros::Rate waitForResult;
	    void getMoveResult(const move_base_msgs::MoveBaseActionResult& msg);
	public:
	    DriveToPointNode();
	    void driveToPoint(geometry_msgs::PoseStamped goal);
};